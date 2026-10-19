#include "stdafx.h"
#include "rb_input.h"
#include "rb_state.h"
#include "rb_t5dr.h"

#include "Emu/Cell/PPUThread.h"
#include "Emu/Io/pad_types.h"

// GGPO (local fork in tekken-rollback/ggpo, branch tekken-rollback: MAX_PREDICTION_FRAMES 3, 56 byte inputs),
// built as a static library with the static CRT (ggpo/build_rb)
#include "../../../../ggpo/src/include/ggponet.h"
#pragma comment(lib, "GGPO.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "winmm.lib")

#include <array>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <string>
#include <thread>

LOG_CHANNEL(rb_net_log, "Netplay");

// Netplay with GGPO (rollback), configured by environment variables:
//   RB_NET=1                 enable
//   RB_NET_PLAYER=1|2        local player (port 0 or 1 of the game)
//   RB_NET_LOCAL_PORT        UDP port of this instance (default 7000)
//   RB_NET_REMOTE=ip:port    the other instance (default 127.0.0.1:7001)
//   RB_NET_DELAY=frames      input delay (default 3); GGPO rolls back at most 3 frames (fork), the rest waits
//   RB_NET_START=frame|+N    game frame where the session starts, "+N" = N frames after the sync point (fight start);
//                            empty = at the first snapshot (RB_SNAP_SAVE_AT, which can be "+N" as well)
//   RB_NET_INPUT=record      local input from the replay file (automatic test), otherwise the local controller
//                            (port 0 of this instance; the data of its previous read, i.e. one frame old)
//   ggpo.network.delay=ms    GGPO's own simulated network latency (send delay)
//
// The frame structure follows GGPO: at every main loop frame boundary ggpo_idle (rollbacks happen there: load
// a snapshot, then advance_frame callbacks re-simulate frames without rendering), add the local input, get the
// inputs of both players (ggpo_synchronize_input), run the frame, ggpo_advance_frame (saves the snapshot).
namespace rb::net
{
	namespace
	{
		constexpr int input_size = 4 + 24 * 2; // CellPadData len + button[0..23] (big endian, as the game sees it)

		struct config_t
		{
			bool enabled = false;
			int player = 1;
			u16 local_port = 7000;
			std::string remote_ip = "127.0.0.1";
			u16 remote_port = 7001;
			int delay = 3;
			u64 start = 0;
			u64 start_rel = umax;
			bool record_input = false;
		};

		const config_t& config()
		{
			static const config_t cfg = []()
			{
				config_t c;
				const auto env = [](const char* name) { const char* v = std::getenv(name); return std::string(v ? v : ""); };
				c.enabled = env("RB_NET") == "1";

				if (const auto v = env("RB_NET_PLAYER"); !v.empty()) c.player = std::atoi(v.c_str()) == 2 ? 2 : 1;
				if (const auto v = env("RB_NET_LOCAL_PORT"); !v.empty()) c.local_port = static_cast<u16>(std::atoi(v.c_str()));
				if (const auto v = env("RB_NET_DELAY"); !v.empty()) c.delay = std::atoi(v.c_str());
				if (const auto v = env("RB_NET_START"); !v.empty())
				{
					if (v[0] == '+') c.start_rel = std::strtoull(v.c_str() + 1, nullptr, 10);
					else c.start = std::strtoull(v.c_str(), nullptr, 10);
				}
				c.record_input = env("RB_NET_INPUT") == "record";

				if (const auto v = env("RB_NET_REMOTE"); !v.empty())
				{
					const auto colon = v.find(':');
					c.remote_ip = v.substr(0, colon);

					if (colon != std::string::npos)
					{
						c.remote_port = static_cast<u16>(std::atoi(v.c_str() + colon + 1));
					}
				}

				return c;
			}();

			return cfg;
		}

		GGPOSession* g_session = nullptr;
		GGPOPlayerHandle g_local = GGPO_INVALID_HANDLE;
		GGPOPlayerHandle g_remote = GGPO_INVALID_HANDLE;
		bool g_started = false;
		bool g_running = false;
		u64 g_start_frame = 0;
		int g_frames_ahead = 0;

		ppu_thread* g_ppu = nullptr;
		u32 g_root = 0;

		u8 g_inputs[2][input_size]{};
		bool g_inputs_valid = false;

		u64 g_frames = 0, g_resim_frames = 0, g_loads = 0, g_wait_ms = 0;

		// Desync check (Tekken 5 DR): each side keeps a fingerprint of the fight state (rb_t5dr.h fight_fingerprint) of
		// every frame, taken right before its task walk, and sends the one of a frame `check_lag` frames back inside its
		// input, in pad bytes the game never reads (button[16..19]). The receiver compares it with its own fingerprint of
		// that frame, which no rollback can change any more by then. To cost no extra rollbacks, the sent fingerprint
		// only changes on frames where the local buttons change anyway (GGPO predicts the previous input, so those frames
		// are mispredicted in any case); in between it stays the same.
		constexpr u32 check_lag = 30;
		constexpr int check_tag_byte = 4 + 16 * 2; // button[16], [17]: frame (u32, big endian)
		constexpr int check_fp_byte = 4 + 18 * 2;  // button[18], [19]: fingerprint
		struct fp_entry { u32 frame = 0; u32 fp = 0; };
		std::array<fp_entry, 256> g_fps{};
		u8 g_sent_payload[8]{};
		u16 g_last_local_keys = 0xffff;
		u32 g_last_checked = 0, g_checks = 0, g_desyncs = 0, g_first_desync = 0;

		void note_fingerprint(u32 frame)
		{
			g_fps[frame & 255] = {frame, t5dr::fight_fingerprint()};
		}

		u32 put_be32(u8* p, u32 v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = static_cast<u8>(v); return v; }
		u32 get_be32(const u8* p) { return (u32{p[0]} << 24) | (u32{p[1]} << 16) | (u32{p[2]} << 8) | p[3]; }

		void attach_check(u8* local, u32 frame)
		{
			const u16 keys = static_cast<u16>((local[4 + 2 * 2 + 1] << 8) | local[4 + 3 * 2 + 1]);

			if (keys != g_last_local_keys && frame > check_lag)
			{
				g_last_local_keys = keys;
				const fp_entry& e = g_fps[(frame - check_lag) & 255];

				if (e.frame == frame - check_lag && e.fp)
				{
					put_be32(g_sent_payload, e.frame);
					put_be32(g_sent_payload + 4, e.fp);
				}
			}

			std::memcpy(local + check_tag_byte, g_sent_payload, 4);
			std::memcpy(local + check_fp_byte, g_sent_payload + 4, 4);
		}

		void check_remote(const u8* remote)
		{
			const u32 frame = get_be32(remote + check_tag_byte);
			const u32 fp = get_be32(remote + check_fp_byte);

			if (!frame || frame == g_last_checked)
			{
				return;
			}

			const fp_entry& e = g_fps[frame & 255];

			if (e.frame != frame || !e.fp)
			{
				return;
			}

			g_last_checked = frame;
			g_checks++;

			if (e.fp != fp)
			{
				if (!g_desyncs++)
				{
					g_first_desync = frame;
				}

				if (g_desyncs <= 10)
				{
					rb_net_log.error("DESYNC at game frame %u: fight state 0x%08x here, 0x%08x on the other side (health here: P1 %u, P2 %u)",
						frame, e.fp, fp, t5dr::fight_health_of(0), t5dr::fight_health_of(1));
				}
			}
		}
		bool g_advance_pending = false; // T5DR: ggpo_advance_frame of the last live frame, done at the next frame boundary

		void pack(const CellPadData& pad, u8* out)
		{
			std::memcpy(out, &pad, input_size);
		}

		void sync_inputs()
		{
			int disconnect_flags = 0;

			if (const auto err = ggpo_synchronize_input(g_session, g_inputs, sizeof(g_inputs), &disconnect_flags); GGPO_SUCCEEDED(err))
			{
				g_inputs_valid = true;

				if (t5dr_active())
				{
					check_remote(g_inputs[2 - config().player]);
				}
			}
			else
			{
				rb_net_log.error("ggpo_synchronize_input failed: %d", static_cast<int>(err));
			}
		}

		bool __cdecl cb_begin_game(const char*)
		{
			return true;
		}

		bool __cdecl cb_save_game_state(unsigned char** buffer, int* len, int* checksum, int frame)
		{
			u32 seq[2]{};
			get_seq(seq);
			const u64 game_frame = g_start_frame + frame;

			if (seq[0] != game_frame)
			{
				rb_net_log.error("Save of GGPO frame %d: game frame %u, expected %u", frame, seq[0], game_frame);
			}

			state_net_save(game_frame, seq);

			auto* token = new u64[1]{game_frame};
			*buffer = reinterpret_cast<unsigned char*>(token);
			*len = sizeof(u64);
			*checksum = 0;
			return true;
		}

		bool __cdecl cb_load_game_state(unsigned char* buffer, int)
		{
			const u64 game_frame = *reinterpret_cast<u64*>(buffer);
			snapshot_info info{};

			if (!state_net_load(game_frame, info))
			{
				return false;
			}

			set_seq(info.seq);
			g_loads++;
			return true;
		}

		bool __cdecl cb_log_game_state(char*, unsigned char*, int)
		{
			return true;
		}

		void __cdecl cb_free_buffer(void* buffer)
		{
			delete[] static_cast<u64*>(buffer);
		}

		// Re-simulation of one frame inside a rollback (called from ggpo_idle). Tekken 5 DR: a GGPO frame is a whole main
		// loop iteration from one task walk to the next (snapshots are taken right before the task walk), so the
		// re-simulated frame also runs the loop code after its walk, flip included (t5dr_between_walks).
		bool __cdecl cb_advance_frame(int)
		{
			if (t5dr_active())
			{
				u32 seq[2]{};
				get_seq(seq);
				note_fingerprint(seq[0]);
			}

			sync_inputs();
			state_net_set_resim(true);
			begin_frame();
			run_task_walk(*g_ppu, g_root);

			if (t5dr_active())
			{
				t5dr_between_walks(*g_ppu);
			}

			state_net_set_resim(false);
			ggpo_advance_frame(g_session);
			g_resim_frames++;
			return true;
		}

		bool __cdecl cb_on_event(GGPOEvent* ev)
		{
			switch (ev->code)
			{
			case GGPO_EVENTCODE_CONNECTED_TO_PEER: rb_net_log.success("Connected to peer"); break;
			case GGPO_EVENTCODE_SYNCHRONIZING_WITH_PEER: break;
			case GGPO_EVENTCODE_SYNCHRONIZED_WITH_PEER: rb_net_log.success("Synchronized with peer"); break;
			case GGPO_EVENTCODE_RUNNING: g_running = true; rb_net_log.success("Session running"); break;
			case GGPO_EVENTCODE_DISCONNECTED_FROM_PEER: rb_net_log.error("Disconnected from peer"); break;
			case GGPO_EVENTCODE_TIMESYNC: g_frames_ahead = ev->u.timesync.frames_ahead; break;
			case GGPO_EVENTCODE_CONNECTION_INTERRUPTED: rb_net_log.warning("Connection interrupted"); break;
			case GGPO_EVENTCODE_CONNECTION_RESUMED: rb_net_log.warning("Connection resumed"); break;
			default: break;
			}

			return true;
		}

		bool start_session(u64 game_frame)
		{
			const config_t& cfg = config();

			GGPOSessionCallbacks cb{};
			cb.begin_game = cb_begin_game;
			cb.save_game_state = cb_save_game_state;
			cb.load_game_state = cb_load_game_state;
			cb.log_game_state = cb_log_game_state;
			cb.free_buffer = cb_free_buffer;
			cb.advance_frame = cb_advance_frame;
			cb.on_event = cb_on_event;

			if (const auto err = ggpo_start_session(&g_session, &cb, const_cast<char*>(t5dr_active() ? "tekken5dr" : "tekken6"), 2, input_size, cfg.local_port); !GGPO_SUCCEEDED(err))
			{
				rb_net_log.fatal("ggpo_start_session failed: %d", static_cast<int>(err));
				return false;
			}

			ggpo_set_disconnect_timeout(g_session, 5000);
			ggpo_set_disconnect_notify_start(g_session, 1000);

			for (int p = 1; p <= 2; p++)
			{
				GGPOPlayer player{};
				player.size = sizeof(player);
				player.player_num = p;
				GGPOPlayerHandle handle = GGPO_INVALID_HANDLE;

				if (p == cfg.player)
				{
					player.type = GGPO_PLAYERTYPE_LOCAL;
					ggpo_add_player(g_session, &player, &handle);
					g_local = handle;
				}
				else
				{
					player.type = GGPO_PLAYERTYPE_REMOTE;
					strcpy_s(player.u.remote.ip_address, cfg.remote_ip.c_str());
					player.u.remote.port = cfg.remote_port;
					ggpo_add_player(g_session, &player, &handle);
					g_remote = handle;
				}
			}

			ggpo_set_frame_delay(g_session, g_local, cfg.delay);
			g_start_frame = game_frame;
			rb_net_log.success("Session started at game frame %u: player %d, local port %u, remote %s:%u, input delay %d",
				game_frame, cfg.player, cfg.local_port, cfg.remote_ip, cfg.remote_port, cfg.delay);

			// Both instances wait here until the peer is connected and synchronized
			const auto wait_start = std::chrono::steady_clock::now();

			auto last_note = wait_start;

			while (!g_running && !g_ppu->is_stopped())
			{
				ggpo_idle(g_session, 10);

				if (std::chrono::steady_clock::now() - last_note > std::chrono::seconds(10))
				{
					last_note = std::chrono::steady_clock::now();
					rb_net_log.warning("Waiting for the other player (%s:%u)...", cfg.remote_ip, cfg.remote_port);
				}
			}

			rb_net_log.success("Peer ready after %u ms", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - wait_start).count());
			return true;
		}

		void local_input(u8* out)
		{
			const config_t& cfg = config();
			CellPadData pad{};
			u32 seq[2]{};
			get_seq(seq);
			const u32 port = cfg.player - 1;

			if (cfg.record_input)
			{
				// The input added now is used `delay` frames later: take the recorded input of that frame
				if (!recorded_input(port, seq[port] + cfg.delay, pad))
				{
					std::memset(&pad, 0, sizeof(pad));
				}
			}
			else if (!live_pad(pad))
			{
				std::memset(&pad, 0, sizeof(pad));
			}

			pack(pad, out);
		}
	}

	bool run_frame(ppu_thread& ppu, u32 root, u64 game_frame)
	{
		const config_t& cfg = config();

		if (!cfg.enabled)
		{
			return false;
		}

		g_ppu = &ppu;
		g_root = root;

		if (!g_started)
		{
			u64 start = cfg.start;

			if (cfg.start_rel != umax)
			{
				const u64 sync = sync_game_frame();
				start = sync == umax ? umax : sync + cfg.start_rel;
			}

			if (game_frame < start || !state_net_ready())
			{
				return false;
			}

			g_started = true;

			if (!start_session(game_frame))
			{
				return false;
			}
		}

		if (!g_session || !g_running)
		{
			return false;
		}

		// Tekken 5 DR: the previous live frame ends here, at the boundary after the main loop ran its code between the
		// walks: its snapshot (ggpo_advance_frame -> cb_save_game_state) is taken at the same point as the snapshots
		// of re-simulated frames and as GGPO's first one (at the first ggpo_add_local_input), right before a task walk
		if (g_advance_pending)
		{
			g_advance_pending = false;
			ggpo_advance_frame(g_session);
		}

		// Network, and rollbacks (load + re-simulated frames through cb_advance_frame)
		ggpo_idle(g_session, 0);

		// Time sync: this side is ahead of the other, give it time to catch up
		if (g_frames_ahead > 0)
		{
			std::this_thread::sleep_for(std::chrono::microseconds(16667));
			g_frames_ahead--;
			g_wait_ms += 16;
		}

		u8 local[input_size]{};
		local_input(local);

		if (t5dr_active())
		{
			u32 seq[2]{};
			get_seq(seq);

			// RB_NET_TEST_DESYNC=<game frame>: this side takes 10 health off player 1 at that frame (tests the desync check)
			static const u64 test_desync = std::strtoull(std::getenv("RB_NET_TEST_DESYNC") ? std::getenv("RB_NET_TEST_DESYNC") : "0", nullptr, 10);

			if (test_desync && seq[0] == test_desync && vm::check_addr<2>(t5dr::fight_health[0]))
			{
				const u16 hp = vm::read16(t5dr::fight_health[0]);
				vm::write16(t5dr::fight_health[0], static_cast<u16>(hp > 10 ? hp - 10 : 0));
				rb_net_log.warning("Test: player 1 health %u -> %u at game frame %u (RB_NET_TEST_DESYNC)", hp, vm::read16(t5dr::fight_health[0]), seq[0]);
			}

			note_fingerprint(seq[0]);
			attach_check(local, seq[0]);
		}

		// Too far ahead of the remote inputs (more than 3 predicted frames): wait for them
		const auto wait_start = std::chrono::steady_clock::now();

		while (!ppu.is_stopped())
		{
			const auto err = ggpo_add_local_input(g_session, g_local, local, input_size);

			if (GGPO_SUCCEEDED(err))
			{
				break;
			}

			if (err != GGPO_ERRORCODE_PREDICTION_THRESHOLD && err != GGPO_ERRORCODE_NOT_SYNCHRONIZED)
			{
				rb_net_log.error("ggpo_add_local_input failed: %d", static_cast<int>(err));
				break;
			}

			ggpo_idle(g_session, 1);
		}

		g_wait_ms += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - wait_start).count();

		sync_inputs();
		begin_frame();
		run_task_walk(ppu, root);

		if (t5dr_active())
		{
			g_advance_pending = true;
		}
		else
		{
			ggpo_advance_frame(g_session);
		}

		if (++g_frames % 600 == 0)
		{
			GGPONetworkStats stats{};
			ggpo_get_network_stats(g_session, g_remote, &stats);
			rb_net_log.success("Frame %u: %u frames, %u re-simulated frames in %u rollbacks, waited %u ms, ping %d ms, remote frames behind %d",
				game_frame, g_frames, g_resim_frames, g_loads, g_wait_ms, stats.network.ping, stats.timesync.remote_frames_behind);

			if (t5dr_active())
			{
				if (g_desyncs)
					rb_net_log.error("Desync check: %u checks, %u DIFFERENT (first at game frame %u)", g_checks, g_desyncs, g_first_desync);
				else
					rb_net_log.success("Desync check: %u checks, all equal", g_checks);
			}
		}

		return true;
	}

	bool pad_input(u32 port, CellPadData* data)
	{
		if (!g_inputs_valid || port >= 2)
		{
			return false;
		}

		std::memset(data, 0, sizeof(CellPadData));
		std::memcpy(data, g_inputs[port], input_size);
		return true;
	}
}
