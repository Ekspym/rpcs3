#include "stdafx.h"
#include "rb_input.h"
#include "rb_state.h"

#include "Emu/Cell/PPUThread.h"
#include "Emu/Io/pad_types.h"

// GGPO (local fork in tekken-rollback/ggpo, branch tekken-rollback: MAX_PREDICTION_FRAMES 3, 56 byte inputs),
// built as a static library with the static CRT (ggpo/build_rb)
#include "../../../../ggpo/src/include/ggponet.h"
#pragma comment(lib, "C:/t6r/ggpo/build_clean/lib/x64/Release/GGPO.lib")
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "winmm.lib")

#include <chrono>
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

		// Re-simulation of one frame inside a rollback (called from ggpo_idle)
		bool __cdecl cb_advance_frame(int)
		{
			sync_inputs();
			state_net_set_resim(true);
			begin_frame();
			run_task_walk(*g_ppu, g_root);
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

			if (const auto err = ggpo_start_session(&g_session, &cb, const_cast<char*>("tekken6"), 2, input_size, cfg.local_port); !GGPO_SUCCEEDED(err))
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
		ggpo_advance_frame(g_session);

		if (++g_frames % 600 == 0)
		{
			GGPONetworkStats stats{};
			ggpo_get_network_stats(g_session, g_remote, &stats);
			rb_net_log.success("Frame %u: %u frames, %u re-simulated frames in %u rollbacks, waited %u ms, ping %d ms, remote frames behind %d",
				game_frame, g_frames, g_resim_frames, g_loads, g_wait_ms, stats.network.ping, stats.timesync.remote_frames_behind);
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
