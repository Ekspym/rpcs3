#include "stdafx.h"
#include "rb_input.h"
#include "rb_state.h"
#include "rb_checksum.h"

#include "Emu/Cell/PPUThread.h"
#include "Emu/Cell/SPUThread.h"
#include "Emu/Cell/PPUFunction.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/IdManager.h"
#include "Emu/System.h"
#include "Emu/Memory/vm.h"
#include "Emu/Cell/lv2/sys_timer.h"
#include "Emu/Cell/lv2/sys_cond.h"
#include "Emu/Cell/lv2/sys_mutex.h"
#include "Emu/Cell/lv2/sys_ppu_thread.h"
#include "Emu/system_config.h"
#include "Utilities/File.h"
#include "Utilities/StrUtil.h"
#include "util/asm.hpp"

#include <array>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <thread>

LOG_CHANNEL(rb_online_log, "Online");

extern std::atomic<bool> g_rb_net_lag_active;

extern bool ppu_form_branch_to_code(u32 entry, u32 target);

// Rollback in the online mode of Tekken 6 (the game's own netcode over RPCN).
//
// The game keeps the inputs of both players in an input manager (*0x00BCC400, functions 0x004069dc..0x0040a148):
//   +0x08 active, +0x10 mode (1 = online), +0x14 local player (1/2),
//   per player p (i = p - 1): entries +0x1c + i*0x10 (0x708 x 0x4C, end pointer +0x20), flags +0x3c + i*0x10
//   (bit 1 present, bit 2 used), +0x58 + i*4 frames available, +0x60 + i*4 simulated frame,
//   +0x74 how far the local input may run ahead of the simulation (3).
// FUN_004073f4(mgr, player) returns the input of the simulated frame or 0 (the fight waits): the delay netcode.
// FUN_00407fd4(mgr, player, frame, entry) stores a received input and refuses frames that are already present.
//
//   RB_ONLINE_LEAD=n     local input lead (+0x74) instead of 3: the input delay of the online mode
//   RB_ONLINE_PREDICT=1  a missing remote input is predicted (the last one repeated, at most RB_ONLINE_MAX_PREDICT
//                        frames, default 6) instead of waiting. When the real input arrives and differs, the state of
//                        the main loop frame before that simulated frame is loaded and the frames are re-simulated
//                        without rendering. Needs the snapshot ring (RB_SNAP_RING=1 with RB_SNAP_RANGES etc.).
namespace rb::online
{
	static bool predict_game_mode();
	static bool run_receiver(ppu_thread& ppu);

	// RB_NET_TIMING=1: steady clock (us) when a remote frame's input was first stored and when it was first predicted
	static void rb_timing(const char* what, u32 frame)
	{
		static const bool on = std::getenv("RB_NET_TIMING") && *std::getenv("RB_NET_TIMING") == '1';

		if (on)
		{
			if (static std::atomic<u32> logged = 0; logged++ < 40000)
			{
				rb_online_log.notice("RB timing %s %u t=%llu", what, frame,
					std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
			}
		}
	}

	namespace
	{
		constexpr u32 mgr_pointer = 0x00BCC400;
		constexpr u32 entry_size = 0x4C;
		constexpr u32 ring = 0x708;
		constexpr u64 toc = 0x0096A8B0;

		struct config_t
		{
			s32 lead = -1;
			bool predict = false;
			u32 max_predict = 6;
		};

		const config_t& config()
		{
			static const config_t cfg = []()
			{
				config_t c;
				if (const char* v = std::getenv("RB_ONLINE_LEAD"); v && *v) c.lead = std::string_view(v) == "auto" ? -2 : std::atoi(v);
				if (const char* v = std::getenv("RB_ONLINE_PREDICT"); v && *v == '1') c.predict = true;
				if (const char* v = std::getenv("RB_ONLINE_MAX_PREDICT"); v && *v) c.max_predict = static_cast<u32>(std::atoi(v));
				return c;
			}();

			return cfg;
		}

		u32 manager()
		{
			const u32 mgr = vm::read32(mgr_pointer);

			if (!mgr || !vm::check_addr(mgr, vm::page_readable) || !vm::read32(mgr + 8) || vm::read32(mgr + 0x10) != 1)
			{
				return 0;
			}

			return mgr;
		}

		// The input manager also runs the online character select (lockstep); prediction and rollback only in the fight itself
		// (intro + rounds), stable for 2 seconds (counted in on_frame). Fight: 0x0097DBB4 == 1 and 0x00C03D44 == 0 (character
		// select: 0 and 14; found by comparing memory dumps, checked over several matches and stages). 0x00A77A10 changes with
		// the menus/loading (logged for diagnostics) but is not a fight marker.
		constexpr u32 scene_addr = 0x00A77A10;

		bool fight_markers()
		{
			return vm::read32(0x0097DBB4) == 1 && vm::read32(0x00C03D44) == 0;
		}
		u32 g_fight_frames = 0;

		// RB_ONLINE_STRESS_RING=1: rollbacks through the snapshot ring of the stress test (rb_state: a snapshot of every
		// frame boundary, restore at a frame boundary, re-simulation by the game's main loop) - the path that held hundreds
		// of rollbacks in the online fight - instead of the net ring (state_net_save/load)
		bool use_stress_ring()
		{
			static const bool on = std::getenv("RB_ONLINE_STRESS_RING") && *std::getenv("RB_ONLINE_STRESS_RING") == '1';
			return on;
		}

		bool ring_ready()
		{
			return use_stress_ring() ? state_stress_ready() : state_net_ready();
		}

		bool in_fight()
		{
			// Half a second of stable markers (was 2 s: lockstep with waits at every fight start, ANALYZA.md 39)
			return g_fight_frames >= 30;
		}

		std::atomic<u64> g_fight_start_ns{0}; // steady clock when the fight markers became stable (0 = no fight)

		u32 remote_player(u32 mgr)
		{
			return vm::read32(mgr + 0x14) == 1 ? 2 : 1;
		}

		using entry_t = std::array<u8, entry_size>;

		std::map<u32, entry_t> g_predicted; // remote frames simulated with a predicted input
		std::map<u32, u64> g_predicted_at;  // game frame where a remote frame was first predicted
		std::map<u32, std::pair<entry_t, u8>> g_local; // local input of a frame as first created (entry, flags): never changes
		u64 g_local_fixed = 0;              // local entries the game had changed after a rollback, written back
		std::atomic<u32> g_max_received{0};   // the highest remote frame contained in a received input message
		u64 g_unreceived_skipped = 0;
		std::map<u32, u64> g_remote_at;     // game frame when the real remote input of a frame was first stored
		u64 g_remote_kept = 0;              // real remote inputs received again with another value (first one kept)
		u32 g_resim_sim_limit = 0;          // remote simulated frame when the running rollback was requested
		u64 g_remote_waits = 0;             // calls that found the remote input of the next simulated frame missing
		u64 g_predict_waits = 0;            // frames that waited because the oldest prediction was too old
		std::map<u32, entry_t> g_remote;    // real remote inputs (simulated frame -> entry)
		u32 g_remote_last = 0;              // highest real remote frame + 1
		u32 g_mismatch_frame = umax;        // earliest remote frame that was mispredicted

		// Time synchronisation (RB_ONLINE_SYNC_TARGET, default 1 frame; 0 = off): the side that runs ahead of the other in
		// time keeps predicting the remote inputs (the user's test: 932 rollbacks of ~3 frames on one side, 18 on the other at
		// 15 ms ping). When the predicted frames (remote simulated - remote available, averaged) exceed the target, one frame
		// waits for the remote input instead of predicting (at most every 8th frame), like GGPO's frame advantage balancing.
		bool g_sync_hold = false;
		double g_sync_avg = 0.0;
		u64 g_sync_last_hold = 0, g_sync_holds = 0;
		u64 g_live_pred_count = 0, g_live_pred_depth = 0; // live predictions and their depth (frames beyond the available ones)
		u64 g_predictions = 0, g_mispredictions = 0, g_rollbacks = 0, g_resim_frames = 0;
		u64 g_dbg_get_remote = 0, g_dbg_get_missing = 0, g_dbg_store_remote = 0, g_dbg_store_local = 0;

		// Main loop frame (game frame) -> remote simulated frame at its start
		std::map<u64, u32> g_sim_at;

		// RB_ONLINE_HASH_OUT=<csv>: hash of the fight state at the start of every simulated frame (both instances must
		// write the same values; after a rollback the frame is written again, the last row counts)
		// Words of the player structures (0x00B2C000-0x00B2E400) and the fight coordinate system (0x00B07970-0x00B07A44)
		// that change during a fight and were equal in two offline runs with the same inputs but a different heap layout
		// (zapas3 m/n dumps at frames 4500/5500/6500): game values without pointers, comparable between the instances
		constexpr std::pair<u32, u32> s_hash_ranges[] =
		{
			{0x00B2C0E0u, 0x00B2C0F0u}, {0x00B2C0F4u, 0x00B2C0F8u}, {0x00B2C0FCu, 0x00B2C108u}, {0x00B2C10Cu, 0x00B2C128u},
			{0x00B2C140u, 0x00B2C144u}, {0x00B2C14Cu, 0x00B2C150u}, {0x00B2C154u, 0x00B2C158u}, {0x00B2C188u, 0x00B2C1E0u},
			{0x00B2C1ECu, 0x00B2C1F4u}, {0x00B2C210u, 0x00B2C218u}, {0x00B2C22Cu, 0x00B2C230u}, {0x00B2C23Cu, 0x00B2C240u},
			{0x00B2C28Cu, 0x00B2C294u}, {0x00B2C2A0u, 0x00B2C2A8u}, {0x00B2C2C8u, 0x00B2C2CCu}, {0x00B2C2D4u, 0x00B2C2D8u},
			{0x00B2C2DCu, 0x00B2C2E0u}, {0x00B2C304u, 0x00B2C310u}, {0x00B2C314u, 0x00B2C31Cu}, {0x00B2C33Cu, 0x00B2C340u},
			{0x00B2C360u, 0x00B2C36Cu}, {0x00B2C3D0u, 0x00B2C3D8u}, {0x00B2C468u, 0x00B2C46Cu}, {0x00B2C470u, 0x00B2C474u},
			{0x00B2C484u, 0x00B2C488u}, {0x00B2C6B4u, 0x00B2C6BCu}, {0x00B2C75Cu, 0x00B2C764u}, {0x00B2C768u, 0x00B2C76Cu},
			{0x00B2C790u, 0x00B2C798u}, {0x00B2C7B4u, 0x00B2C7B8u}, {0x00B2C7E0u, 0x00B2C7E4u}, {0x00B2C828u, 0x00B2C82Cu},
			{0x00B2C840u, 0x00B2C844u}, {0x00B2C850u, 0x00B2C85Cu}, {0x00B2C860u, 0x00B2C868u}, {0x00B2C870u, 0x00B2C874u},
			{0x00B2C878u, 0x00B2C87Cu}, {0x00B2C880u, 0x00B2C88Cu}, {0x00B2C894u, 0x00B2C8C4u}, {0x00B2C8F4u, 0x00B2C900u},
			{0x00B2C908u, 0x00B2C914u}, {0x00B2C91Cu, 0x00B2C928u}, {0x00B2C930u, 0x00B2C93Cu}, {0x00B2C944u, 0x00B2C950u},
			{0x00B2C958u, 0x00B2C964u}, {0x00B2C96Cu, 0x00B2C978u}, {0x00B2C980u, 0x00B2C98Cu}, {0x00B2C994u, 0x00B2C9A0u},
			{0x00B2C9A8u, 0x00B2C9B4u}, {0x00B2C9BCu, 0x00B2C9C8u}, {0x00B2C9D0u, 0x00B2C9DCu}, {0x00B2C9E4u, 0x00B2C9F0u},
			{0x00B2C9F8u, 0x00B2CA04u}, {0x00B2CA0Cu, 0x00B2CA18u}, {0x00B2CA1Cu, 0x00B2CA48u}, {0x00B2CA4Cu, 0x00B2CA8Cu},
			{0x00B2CA90u, 0x00B2CA9Cu}, {0x00B2CAA4u, 0x00B2CABCu}, {0x00B2CAC4u, 0x00B2CAC8u}, {0x00B2CB2Cu, 0x00B2CB44u},
			{0x00B2CB50u, 0x00B2CBBCu}, {0x00B2CBF4u, 0x00B2CC08u}, {0x00B2CC18u, 0x00B2CC2Cu}, {0x00B2CC44u, 0x00B2CC48u},
			{0x00B2CC5Cu, 0x00B2CC60u}, {0x00B2CC6Cu, 0x00B2CC74u}, {0x00B2CC80u, 0x00B2CC88u}, {0x00B2CC94u, 0x00B2CC9Cu},
			{0x00B2CCD0u, 0x00B2CCD4u}, {0x00B2CD20u, 0x00B2CD30u}, {0x00B2CD34u, 0x00B2CD38u}, {0x00B2CD3Cu, 0x00B2CD48u},
			{0x00B2CD4Cu, 0x00B2CD50u}, {0x00B2CD54u, 0x00B2CD58u}, {0x00B2CD5Cu, 0x00B2CD68u}, {0x00B2CD80u, 0x00B2CD84u},
			{0x00B2CD8Cu, 0x00B2CD98u}, {0x00B2CDC8u, 0x00B2CE20u}, {0x00B2CE2Cu, 0x00B2CE34u}, {0x00B2CE50u, 0x00B2CE58u},
			{0x00B2CE6Cu, 0x00B2CE70u}, {0x00B2CEB0u, 0x00B2CEB4u}, {0x00B2CECCu, 0x00B2CED8u}, {0x00B2CEE0u, 0x00B2CEE8u},
			{0x00B2CF08u, 0x00B2CF0Cu}, {0x00B2CF14u, 0x00B2CF18u}, {0x00B2CF1Cu, 0x00B2CF20u}, {0x00B2CF34u, 0x00B2CF38u},
			{0x00B2CF44u, 0x00B2CF4Cu}, {0x00B2CF50u, 0x00B2CF5Cu}, {0x00B2CF7Cu, 0x00B2CF80u}, {0x00B2CFA0u, 0x00B2CFACu},
			{0x00B2D010u, 0x00B2D018u}, {0x00B2D0BCu, 0x00B2D0C0u}, {0x00B2D0C4u, 0x00B2D0C8u}, {0x00B2D2F4u, 0x00B2D2FCu},
			{0x00B2D398u, 0x00B2D3A4u}, {0x00B2D3D0u, 0x00B2D3D8u}, {0x00B2D3F4u, 0x00B2D3F8u}, {0x00B2D420u, 0x00B2D424u},
			{0x00B2D454u, 0x00B2D458u}, {0x00B2D480u, 0x00B2D484u}, {0x00B2D490u, 0x00B2D49Cu}, {0x00B2D4A0u, 0x00B2D4A8u},
			{0x00B2D4B0u, 0x00B2D4D0u}, {0x00B2D4D4u, 0x00B2D504u}, {0x00B2D534u, 0x00B2D540u}, {0x00B2D548u, 0x00B2D554u},
			{0x00B2D55Cu, 0x00B2D568u}, {0x00B2D570u, 0x00B2D57Cu}, {0x00B2D584u, 0x00B2D590u}, {0x00B2D598u, 0x00B2D5A4u},
			{0x00B2D5ACu, 0x00B2D5B8u}, {0x00B2D5C0u, 0x00B2D5CCu}, {0x00B2D5D4u, 0x00B2D5E0u}, {0x00B2D5E8u, 0x00B2D5F4u},
			{0x00B2D5FCu, 0x00B2D608u}, {0x00B2D610u, 0x00B2D61Cu}, {0x00B2D624u, 0x00B2D630u}, {0x00B2D638u, 0x00B2D644u},
			{0x00B2D64Cu, 0x00B2D658u}, {0x00B2D65Cu, 0x00B2D668u}, {0x00B2D66Cu, 0x00B2D678u}, {0x00B2D67Cu, 0x00B2D688u},
			{0x00B2D68Cu, 0x00B2D698u}, {0x00B2D69Cu, 0x00B2D6A8u}, {0x00B2D6ACu, 0x00B2D6CCu}, {0x00B2D6D0u, 0x00B2D6DCu},
			{0x00B2D6E4u, 0x00B2D6FCu}, {0x00B2D704u, 0x00B2D708u}, {0x00B2D76Cu, 0x00B2D784u}, {0x00B2D790u, 0x00B2D7D8u},
			{0x00B2D7E4u, 0x00B2D804u}, {0x00B2D838u, 0x00B2D83Cu}, {0x00B2D840u, 0x00B2D848u}, {0x00B2D85Cu, 0x00B2D86Cu},
			{0x00B2D884u, 0x00B2D888u}, {0x00B2D89Cu, 0x00B2D8A0u}, {0x00B2D8B0u, 0x00B2D8B4u}, {0x00B2D8C4u, 0x00B2D8C8u},
			{0x00B2D8D4u, 0x00B2D8DCu}, {0x00B2D910u, 0x00B2D914u}, {0x00B07970u, 0x00B07974u}, {0x00B07978u, 0x00B0797Cu},
			{0x00B07990u, 0x00B07994u}, {0x00B07998u, 0x00B0799Cu}, {0x00B079A0u, 0x00B079A4u}, {0x00B079A8u, 0x00B079ACu},
			{0x00B079B0u, 0x00B079B4u}, {0x00B079B8u, 0x00B079BCu}, {0x00B079D0u, 0x00B079D4u}, {0x00B079D8u, 0x00B079DCu},
			{0x00B079F0u, 0x00B079FCu}, {0x00B07A00u, 0x00B07A0Cu}, {0x00B07A10u, 0x00B07A2Cu}, {0x00B07A3Cu, 0x00B07A44u}
		};

		// part 0: player structures (0x00B2xxxx), part 1: fight coordinate system / camera (0x00B079xx)
		u64 state_hash(u32 part = 0)
		{
			u64 h = 0xcbf29ce484222325;

			for (const auto& [b, e] : s_hash_ranges)
			{
				if ((b >= 0x00B2C000) != (part == 0))
				{
					continue;
				}

				for (u32 a = b; a < e; a += 4)
				{
					h = (h ^ vm::read32(a)) * 0x100000001b3;
				}
			}

			return h;
		}

		void write_hash(u32 sim, u64 game_frame, bool resim)
		{
			static fs::file out = []()
			{
				// UTF-8 path (std::getenv gives the ANSI code page: "Nová složka" would not open)
#ifdef _WIN32
				const wchar_t* wpath = _wgetenv(L"RB_ONLINE_HASH_OUT");
				const std::string path = wpath ? wchar_to_utf8(wpath) : std::string();
#else
				const std::string path = std::getenv("RB_ONLINE_HASH_OUT") ? std::getenv("RB_ONLINE_HASH_OUT") : "";
#endif
				fs::file f;

				if (!path.empty() && !f.open(path, fs::rewrite))
				{
					rb_online_log.error("Fight log '%s' cannot be created (%s)", path, fs::g_tls_error);
				}

				if (f)
				{
					const std::string header = "sim_frame,game_frame,resim,state_hash,camera_hash,sim_p1,sim_p2,p1pos,p2pos,in1,in2\n";
					f.write(header.data(), header.size());
				}

				return f;
			}();

			if (out)
			{
				const u32 mgr = manager();
				// Player positions (x, y, z floats; offline analysis: P1 0x00B2CA3C, P2 0x00B2D67C) as raw words
				const auto pos = [](u32 a)
				{
					return fmt::format("%08x%08x%08x", +vm::read32(a), +vm::read32(a + 4), +vm::read32(a + 8));
				};

				// Input entry of the last simulated frame of each player (FNV-1a of the 0x4C bytes, "-" if not present)
				const auto input = [&](u32 i) -> std::string
				{
					const u32 s = mgr ? +vm::read32(mgr + 0x60 + i * 4) : 0;
					const u32 entries = mgr ? +vm::read32(mgr + i * 0x10 + 0x1c) : 0;

					if (!s || !entries || !(vm::read8(vm::read32(mgr + i * 0x10 + 0x3c) + (s - 1) % ring) & 1))
					{
						return "-";
					}

					const u8* e = vm::get_super_ptr<u8>(entries + ((s - 1) % ring) * entry_size);
					u32 h = 0x811c9dc5;

					for (u32 k = 0; k < entry_size; k++)
					{
						h = (h ^ e[k]) * 0x01000193;
					}

					return fmt::format("%08x", h);
				};

				const std::string line = fmt::format("%u,%u,%u,%016x,%016x,%u,%u,%s,%s,%s,%s\n", sim, game_frame, resim ? 1 : 0, state_hash(0), state_hash(1),
					mgr ? static_cast<u32>(vm::read32(mgr + 0x60)) : 0u, mgr ? static_cast<u32>(vm::read32(mgr + 0x64)) : 0u, pos(0x00B2CA3C),
					pos(0x00B2D67C), input(0), input(1));
				out.write(line.data(), line.size());
			}
		}

		entry_t read_entry(u32 addr)
		{
			entry_t e{};
			std::memcpy(e.data(), vm::get_super_ptr<u8>(addr), entry_size);
			return e;
		}

		u32 slot_addr(u32 mgr, u32 i, u32 frame)
		{
			return vm::read32(mgr + i * 0x10 + 0x1c) + (frame % ring) * entry_size;
		}

		u32 flag_addr(u32 mgr, u32 i, u32 frame)
		{
			return vm::read32(mgr + i * 0x10 + 0x3c) + frame % ring;
		}

		// GGPO rule: a local input never changes once created (it may already be sent; o94: after a rollback the sender's
		// entries of already sent frames held other values, the peer simulated different inputs). New local entries are
		// recorded (not during a re-simulation), recorded ones are written back where the ring differs.
		void keep_local_inputs(u32 mgr, u32 li, bool resim)
		{
			if (!vm::read32(mgr + li * 0x10 + 0x1c) || !vm::read32(mgr + li * 0x10 + 0x3c))
			{
				return;
			}

			const u32 avail = vm::read32(mgr + 0x58 + li * 4);
			const u32 from = avail > 16 ? avail - 16 : 0;

			if (!resim)
			{
				for (u32 f = from; f < avail; f++)
				{
					if (!g_local.contains(f) && (vm::read8(flag_addr(mgr, li, f)) & 1))
					{
						g_local.emplace(f, std::pair{read_entry(slot_addr(mgr, li, f)), vm::read8(flag_addr(mgr, li, f))});
					}
				}
			}

			for (auto it = g_local.lower_bound(from); it != g_local.end() && it->first < avail; ++it)
			{
				const u8 fl = vm::read8(flag_addr(mgr, li, it->first));

				if (!(fl & 1) || read_entry(slot_addr(mgr, li, it->first)) != it->second.first)
				{
					std::memcpy(vm::get_super_ptr<u8>(slot_addr(mgr, li, it->first)), it->second.first.data(), entry_size);
					vm::write8(flag_addr(mgr, li, it->first), fl | 1);

					if (g_local_fixed++ < 30)
					{
						rb_online_log.warning("Local input of frame %u restored to its first value (available %u, resim %d)", it->first, avail, resim);
					}
				}
			}

			while (!g_local.empty() && g_local.begin()->first + 64 < avail)
			{
				g_local.erase(g_local.begin());
			}
		}

		// Copy of FUN_004073f4 with prediction of the remote input
		// Diagnostics: the distinct callers of the hooked input functions (RB_ONLINE_PHASE_LOG)
		void note_caller(const char* what, ppu_thread& ppu, u32 mgr)
		{
			static std::set<u64> seen;
			static const bool on = std::getenv("RB_ONLINE_PHASE_LOG") != nullptr;
			const u32 lr = static_cast<u32>(ppu.lr);

			if (on && seen.emplace((static_cast<u64>(what[0]) << 32) | lr).second)
			{
				rb_online_log.notice("Caller of %s: 0x%08x, mode %u, sim %u, lcg 0x%08x", what, lr, +vm::read32(mgr + 0x10), +vm::read32(mgr + 0x60),
					+vm::read32(0x00D142AC));
			}
		}

		u32 hle_get_input(ppu_thread& ppu, u32 mgr, u32 player)
		{
			note_caller("get_input", ppu, mgr);
			const u32 i = player - 1;

			if (!vm::read32(mgr + 8) || i >= 2)
			{
				return 0;
			}

			const u32 mode = vm::read32(mgr + 0x10);
			u32 sim = vm::read32(mgr + 0x60 + i * 4);
			const u32 slot = mode == 0 ? sim % 0x3c : mode == 1 ? sim % ring : sim;
			const u32 flags = vm::read32(mgr + i * 0x10 + 0x3c) + slot;
			const u32 entry = vm::read32(mgr + i * 0x10 + 0x1c) + slot * entry_size;
			u8 b = vm::read8(flags);

			if (mode == 1 && player == remote_player(mgr))
			{
				g_dbg_get_remote++;
				g_dbg_get_missing += !(b & 1);
			}

			if (b & 1)
			{
				vm::write8(flags, b | 2);
				return entry;
			}

			return 0;
		}

		// Copy of FUN_00407054(mgr, player): is the input of the simulated frame present? Both players' answers form
		// the "inputs ready" flag (mgr+0xc, FUN_00422960) that pauses the whole fight. A missing remote input is
		// predicted here (the previous one repeated), so the fight does not wait.
		u32 hle_input_available(ppu_thread& ppu, u32 mgr, u32 player)
		{
			note_caller("available", ppu, mgr);

			// The battle net task object (FUN_00423364 -> FUN_00422960, which keeps it in r31 while it calls this function):
			// send/receive state, message sequence numbers, input history rings (up to +0x10B4EC). Network state, not fight
			// state: restoring it made the peer drop the next packets (already seen sequence numbers), both sides stalled.
			if (static u32 last_session = 0; ppu.lr == 0x004229a0 || ppu.lr == 0x004229b8)
			{
				const u32 session = static_cast<u32>(ppu.gpr[31]);

				// RB_ONLINE_NET_EXCLUDE=0 keeps the object in the snapshot (diagnostics)
				if (session != last_session && vm::check_addr(session, vm::page_readable) &&
					!(std::getenv("RB_ONLINE_NET_EXCLUDE") && *std::getenv("RB_ONLINE_NET_EXCLUDE") == '0'))
				{
					last_session = session;
					// Without its first 0x60 bytes: the object is also a node of the game's task tree (+0x10 next, +0x44 flags,
					// +0x50 counter, +0x54/+0x58 its handle list), the tree must be restored with the rest (crash in the task walk 0x000ff584)
					state_exclude(session + 0x60, session + 0x9C + 2 * 0x85A28, "online battle net task object (network part)");
				}
			}
			const u32 i = player - 1;

			if (i >= 2)
			{
				return 0;
			}

			const u32 mode = vm::read32(mgr + 0x10);
			const u32 sim = vm::read32(mgr + 0x60 + i * 4);
			const u32 slot = mode == 0 ? sim % 0x3c : mode == 1 ? sim % ring : 0;
			const u32 flags = vm::read32(mgr + i * 0x10 + 0x3c) + slot;
			const u8 b = vm::read8(flags);
			const u32 available = vm::read32(mgr + 0x58 + i * 4);

			if (b & 1)
			{
				// The flags of the 0x708 frame ring are cleared only for the next frame of a store: beyond the available ones
				// a slot can still hold the present bit of frame sim - 0x708 (o101/o103: after 1800 frames of a round the
				// simulation ran on 30 s old inputs past available, desync and a stuck round end; ANALYZA.md 41)
				if (mode != 1 || sim < available || (player == remote_player(mgr) && g_predicted.contains(sim)))
				{
					return 1;
				}

				vm::write8(flags, b & ~1);

				if (static u32 logged = 0; logged++ < 20)
				{
					rb_online_log.warning("Stale input flag of player %u frame %u cleared (available %u)", player, sim, available);
				}
			}

			// Lead 0 (no input delay, like offline): the net task (FUN_00422960, these two calls) sets the "ready" flag before
			// the pad task adds the local input of this frame (FUN_00407c8c) and reads it (FUN_00185cc8). The local entry of
			// frame sim is missing at that moment but added in the same frame: reported available. (Otherwise the fight steps
			// only every second frame: ANALYZA.md section 31.)
			if (mode == 1 && player != remote_player(mgr) && available == sim && vm::read32(mgr + 0x74) == 0 &&
				(ppu.lr == 0x004229a0 || ppu.lr == 0x004229b8))
			{
				return 1;
			}

			// The oldest prediction not yet confirmed must stay within reach of the snapshot ring: a misprediction found later
			// could not be rolled back (desync). RB_ONLINE_MAX_PREDICT_AGE game frames (default 5: ring of 8, the rollback
			// happens one frame after the request, the re-simulation is at most 7 frames); older -> wait for the input
			static const u64 max_age = std::getenv("RB_ONLINE_MAX_PREDICT_AGE") ? std::strtoull(std::getenv("RB_ONLINE_MAX_PREDICT_AGE"), nullptr, 10) : 60;
			u32 seq[2]{};
			get_seq(seq);
			const bool too_old = !g_predicted.empty() && g_predicted_at.contains(g_predicted.begin()->first) &&
				seq[0] - g_predicted_at[g_predicted.begin()->first] >= max_age;

			// During a re-simulation the limits do not apply up to the frame where the rollback was requested (simulated once
			// already; beyond it the normal limits: o101 predicted 100 frames ahead without that); waiting there stalled
			// the re-simulation, no snapshot was saved meanwhile and the next rollback found no slot (o98, ANALYZA.md 37)
			const bool resim = is_resimulating();

			// RB_ONLINE_RECV_WAIT_US (default 0 = off): in a live fight frame whose remote input is the only one missing, wait up
			// to this long for it (running the game's receiver every 250 us) before predicting. At a low ping the input arrives
			// a few ms after this check: a short wait is cheaper than the rollback (v1.0 test, 15 ms RTT: 68 % of the remote
			// inputs were predicted, 20 % of them wrong)
			static const u64 recv_wait_us = std::getenv("RB_ONLINE_RECV_WAIT_US") ? std::strtoull(std::getenv("RB_ONLINE_RECV_WAIT_US"), nullptr, 10) : 0;

			if (recv_wait_us && !resim && mode == 1 && player == remote_player(mgr) && sim == available && in_fight() && predict_game_mode() &&
				(ppu.lr == 0x004229a0 || ppu.lr == 0x004229b8))
			{
				static u64 waited = 0, got = 0, total_us = 0;
				const auto start = std::chrono::steady_clock::now();
				bool arrived = false;
				waited++;

				while (std::chrono::steady_clock::now() - start < std::chrono::microseconds(recv_wait_us))
				{
					std::this_thread::sleep_for(std::chrono::microseconds(250));

					if (run_receiver(ppu) && vm::read32(mgr + 0x58 + i * 4) > sim && (vm::read8(flags) & 1))
					{
						arrived = true;
						break;
					}
				}

				total_us += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
				got += arrived;

				if (waited % 600 == 0)
				{
					rb_online_log.notice("Receive wait: %u of %u waits got the remote input, avg %u us", got, waited, total_us / waited);
				}

				if (arrived)
				{
					return 1;
				}
			}

			if (config().predict && mode == 1 && in_fight() && ring_ready() && !g_sim_at.empty() && player == remote_player(mgr) && sim > 0 &&
				((resim && sim <= g_resim_sim_limit) || (sim < available + config().max_predict && !too_old && !g_sync_hold && predict_game_mode())) && (vm::read8(flag_addr(mgr, i, sim - 1)) & 1))
			{
				const entry_t prev = read_entry(slot_addr(mgr, i, sim - 1));
				std::memcpy(vm::get_super_ptr<u8>(slot_addr(mgr, i, sim)), prev.data(), entry_size);
				vm::write8(flags, 1);
				g_predicted[sim] = prev;

				if (!g_predicted_at.contains(sim) && !resim)
				{
					g_live_pred_count++;
					g_live_pred_depth += sim + 1 - available;
					rb_timing("predict", sim);
				}

				if (!g_predicted_at.contains(sim) || !is_resimulating())
				{
					g_predicted_at[sim] = seq[0];
				}

				while (g_predicted_at.size() > 128)
				{
					g_predicted_at.erase(g_predicted_at.begin());
				}

				g_predictions++;
				return 1;
			}

			if (too_old)
			{
				g_predict_waits++;
			}

			if (mode == 1 && player == remote_player(mgr))
			{
				g_remote_waits++;
			}

			return 0;
		}

		// Copy of FUN_00407fd4; a predicted frame is compared with the real input and overwritten
		u32 hle_store_input(ppu_thread& ppu, u32 mgr, u32 player, u32 frame, u32 src)
		{
			note_caller("store_input", ppu, mgr);

			if (!vm::read32(mgr + 8) || !vm::read32(mgr + 0x10))
			{
				return 0;
			}

			const u32 next = frame + 1;
			const u32 i = player - 1;
			const u32 local = vm::read32(mgr + 0x14);

			if (!((local - 1u) > 1 || (i < 2 && player != local)))
			{
				return 0;
			}

			const u32 flags = flag_addr(mgr, i, frame);
			const entry_t real = read_entry(src);

			(player == remote_player(mgr) ? g_dbg_store_remote : g_dbg_store_local)++;

			// FUN_004219d8 copies the net history into the manager for every frame up to the history's last one, also slots that
			// were never written (zeros stored as real input, replaced by the next message: "Remote input of frame N changed",
			// extra rollbacks). Frames above the highest received one are not stored (ANALYZA.md 35).
			if (player == remote_player(mgr) && ppu.lr >= 0x004219d8 && ppu.lr < 0x00421bf0 && frame > g_max_received)
			{
				if (g_unreceived_skipped++ < 20)
				{
					rb_online_log.warning("Remote frame %u not stored: not received yet (highest received %u)", frame, +g_max_received);
				}

				return 0;
			}

			if (player == remote_player(mgr))
			{
				u32 now[2]{};
				get_seq(now);

				// Diagnostics: a real remote input received again with a different content (o94: the peer's sender put older
				// inputs onto newer frames while it re-simulated; it no longer sends then). The newest value wins (rollback).
				// Only within 300 game frames of the first one (a new round starts the frame numbers again).
				if (const auto it = g_remote.find(frame); it != g_remote.end() && it->second != real && g_remote_at.contains(frame) &&
					now[0] - g_remote_at[frame] < 300)
				{
					g_remote_kept++;

					if (static u32 logged = 0; logged++ < 50)
					{
						u32 seq[2]{};
						get_seq(seq);
						std::string diff;

						for (u32 k = 0; k < entry_size; k++)
						{
							if (it->second[k] != real[k])
							{
								fmt::append(diff, " +0x%x:%02x->%02x", k, it->second[k], real[k]);
							}
						}

						rb_online_log.error("Remote input of frame %u changed (sim %u, available %u, game frame %u):%s", frame,
							+vm::read32(mgr + 0x60 + i * 4), +vm::read32(mgr + 0x58 + i * 4), seq[0], diff);
					}
				}

				if (!g_remote.contains(frame) && !is_resimulating())
				{
					rb_timing("remote", frame);
				}

				if (!g_remote.contains(frame) || g_remote[frame] != real)
				{
					g_remote_at[frame] = now[0];
				}

				g_remote[frame] = real;
				g_remote_last = std::max(g_remote_last, next);

				while (!g_remote_at.empty() && g_remote_at.begin()->first + 600 < frame)
				{
					g_remote_at.erase(g_remote_at.begin());
				}

				// Old entries are not needed any more
				while (!g_remote.empty() && g_remote.begin()->first + 600 < frame)
				{
					g_remote.erase(g_remote.begin());
				}

				// A predicted frame (or one restored as predicted by a snapshot load) was already used by the simulation: the real
				// input replaces it and keeps its use flags (bit 2/4/8); a different input triggers the rollback. The available
				// counter must advance in every case, otherwise the prediction window runs out and both sides stall.
				const bool predicted = g_predicted.erase(frame) != 0;
				const bool present = vm::read8(flags) & 1;

				// Diagnostics around the first rollbacks: every received remote input
				if (static u32 logged = 0; g_rollbacks && logged < 300 && std::getenv("RB_ONLINE_PHASE_LOG"))
				{
					logged++;
					rb_online_log.notice("Store remote frame %u: predicted %d present %d flags 0x%x, available %u, sim %u, caller 0x%x", frame,
						predicted, present, +vm::read8(flags), +vm::read32(mgr + 0x58 + i * 4), +vm::read32(mgr + 0x60 + i * 4), +ppu.lr);
				}

				if (predicted || present)
				{
					// Only a frame the simulation already used needs a rollback; a prediction not used yet is just replaced
					// (o88: after a KO the simulation stops, late inputs of predicted frames caused an endless rollback loop)
					if (frame < vm::read32(mgr + 0x60 + i * 4) && read_entry(slot_addr(mgr, i, frame)) != real)
					{
						g_mismatch_frame = std::min(g_mismatch_frame, frame);
						g_mispredictions++;

						// Which bytes of the entry were mispredicted (offset histogram every 300 mispredictions; first 40 in full)
						{
							const entry_t used = read_entry(slot_addr(mgr, i, frame));
							static std::map<u32, u32> offsets;
							static u32 logged = 0;
							std::string text;

							for (u32 b = 0; b < entry_size; b++)
							{
								if (used[b] != real[b])
								{
									offsets[b]++;

									if (logged < 40)
									{
										fmt::append(text, " +0x%x:%02x->%02x", b, used[b], real[b]);
									}
								}
							}

							if (logged < 40)
							{
								logged++;
								rb_online_log.notice("Misprediction of frame %u:%s", frame, text);
							}

							if (g_mispredictions % 300 == 0)
							{
								std::string hist;

								for (const auto& [b, n] : offsets)
								{
									fmt::append(hist, " +0x%x:%u", b, n);
								}

								rb_online_log.notice("Mispredicted bytes (offset:count over %u):%s", g_mispredictions, hist);
							}
						}
					}

					std::memcpy(vm::get_super_ptr<u8>(slot_addr(mgr, i, frame)), real.data(), entry_size);
					vm::write8(flags, vm::read8(flags) | 1);

					if (next > vm::read32(mgr + 0x58 + i * 4))
					{
						vm::write32(mgr + 0x58 + i * 4, next);
					}

					return 1;
				}
			}

			if (vm::read8(flags) & 1)
			{
				return 0;
			}

			const u32 entries = vm::read32(mgr + i * 0x10 + 0x1c);
			const u32 end = vm::read32(mgr + i * 0x10 + 0x20);

			if (!entries || frame % ring >= (end - entries) / entry_size)
			{
				return 0;
			}

			std::memcpy(vm::get_super_ptr<u8>(entries + (frame % ring) * entry_size), real.data(), entry_size);
			vm::write8(flags, 1);

			// The next frame's flags are reset for a fresh entry, but not when it holds a prediction already in use
			if (player != remote_player(mgr) || !g_predicted.contains(next))
			{
				vm::write8(flag_addr(mgr, i, next), 0);
			}

			if (next > vm::read32(mgr + 0x58 + i * 4))
			{
				vm::write32(mgr + 0x58 + i * 4, next);
			}

			return 1;
		}

		// After a load: all real remote inputs of frames from `from` on are written again, predictions are dropped
		void reapply_remote(u32 mgr, u32 from)
		{
			const u32 i = remote_player(mgr) - 1;

			for (auto it = g_predicted.lower_bound(from); it != g_predicted.end();)
			{
				vm::write8(flag_addr(mgr, i, it->first), vm::read8(flag_addr(mgr, i, it->first)) & ~1);
				it = g_predicted.erase(it);
			}

			for (auto it = g_remote.lower_bound(from); it != g_remote.end(); ++it)
			{
				std::memcpy(vm::get_super_ptr<u8>(slot_addr(mgr, i, it->first)), it->second.data(), entry_size);
				vm::write8(flag_addr(mgr, i, it->first), vm::read8(flag_addr(mgr, i, it->first)) | 1);
			}

			if (g_remote_last > vm::read32(mgr + 0x58 + i * 4))
			{
				vm::write32(mgr + 0x58 + i * 4, g_remote_last);
			}
		}
	}

	// Game mode (FUN_0020c754: *(*0x00967cf4 + 0xc); 1/3/4/5 round, 6 KO, 10 after the KO / replay). Changes are logged.
	// RB_ONLINE_PREDICT_MODES (comma separated, default: every mode): prediction only in these modes, lockstep in the
	// others, so a skipped intro or replay (a remote input every prediction misses) never needs a rollback across the
	// scene change (user: crash when skipping the replay or the intro scenes)
	static bool predict_game_mode()
	{
		const u32 base = vm::read32(0x00967cf4);
		const u32 mode = base ? static_cast<u32>(vm::read32(base + 0xc)) : umax;


		static const std::set<u32> modes = []()
		{
			std::set<u32> m;

			if (const char* v = std::getenv("RB_ONLINE_PREDICT_MODES"); v && *v)
			{
				for (const auto& part : fmt::split(v, {","}))
				{
					m.insert(static_cast<u32>(std::atoi(part.c_str())));
				}
			}

			return m;
		}();

		return modes.empty() || modes.count(mode);
	}

	// Copy of FUN_0021bb94(job) (wait for a SPURS job: pending +0x304, launched +0x308, FUN_0021bb00 = still running, then
	// cellSpursEventFlagTryWait on job+0x200 every 100 us) with a time limit of 200 ms: a job pending in a restored state that
	// finished in the discarded timeline (or never runs again) would be waited for forever (o134: stage particle job after
	// a rollback). The game's own loop has no limit; a real job takes well below 20 ms.
	// Jobs whose pending wait was cancelled by a load (restored state, completion bit not set). Their next launches in the
	// re-simulation did not finish at times either (job chain state of the other timeline): 2 x 200 ms in one rollback
	// (o161: 414 / 419 ms). They are waited for at most 20 ms, in live frames too and for good (online 2026-10-08: a live
	// frame after a rollback waited the full 200 ms for job 0x00b80800, a 225 ms freeze; o269: forgotten once the job had
	// finished, it got stuck again at the next round start, 200 ms).
	std::set<u32> g_restore_cancelled_jobs;
	// Such a job that also timed out in the re-simulation: the next launch (the frame the rollback happened in, not a
	// re-simulated one) waited the full 200 ms and did not finish either (o189 228 ms, o209 233 ms rollbacks). Until it
	// finishes once, every wait for it is limited to 20 ms.
	std::set<u32> g_restore_timed_out_jobs;

	void hle_spurs_job_wait(ppu_thread& ppu, u32 job)
	{
		constexpr u64 toc = 0x0096A8B0;

		if (vm::read32(job + 0x304) == 0)
		{
			return;
		}

		vm::write32(job + 0x304, 0);

		if (vm::read32(job + 0x308) == 0)
		{
			return;
		}

		// The job's event flag (job+0x200, CellSpursEventFlag 128 bytes) is set by the SPU: restoring it from a snapshot left it
		// out of step with SPURS, the job never "finished" again (o142: every frame waited, the game crawled). It is kept
		// out of the snapshots, for every job the game waits for.
		if (static std::set<u32> jobs; jobs.insert(job).second)
		{
			state_exclude(job + 0x200, job + 0x280, "SPURS job event flag");

			// The SPURS instance of its job chain (CellSpurs, 0x2000 bytes, pointer at chain+0x78) is SPU scheduler state written
			// by cellSpurs calls on the main thread: restored by a load, the workload states no longer matched the SPUs and the
			// stage teardown hung in cellSpursJoinJobChain (r10, r12: black screen at the round end)
			if (const u32 chain = vm::read32(job + 0x180); chain && !(chain & 0x7f) && vm::check_addr(chain + 0x7c, vm::page_readable))
			{
				if (const u32 spurs = vm::read32(chain + 0x7c); spurs && !(spurs & 0x7f))
				{
					if (static std::set<u32> instances; instances.insert(spurs).second)
					{
						// CellSpurs is 0x1000 bytes, CellSpurs2 (32 workloads, flags1 +0x74 bit 0x40) 0x2000
						state_exclude(spurs, spurs + ((vm::read8(spurs + 0x74) & 0x40) ? 0x2000 : 0x1000), "SPURS instance");
					}

					// The job chain itself (CellSpursJobChain 0x110 bytes: pc, link registers, counters) is advanced by the SPUs:
					// restored, its pc went back while the SPURS instance (live) did not, the chain got stuck (r18: job 0x00b80800
					// "pending without completion" after every load from round 2 on, then the teardown did not shut it down).
					// RB_SPURS_CHAIN_EXCLUDE=0 keeps it in the snapshots (s12 desynced with it excluded, but without the instance).
					if (static const bool chain_ex = !std::getenv("RB_SPURS_CHAIN_EXCLUDE") || *std::getenv("RB_SPURS_CHAIN_EXCLUDE") != '0'; chain_ex)
					{
						if (static std::set<u32> chains; chains.insert(chain).second)
						{
							state_exclude(chain, chain + 0x110, "SPURS job chain");
						}
					}
				}
			}
		}

		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
		const u32 bits = static_cast<u32>(ppu.gpr[1]) + 0x70;

		ppu.gpr[3] = job;
		ppu.fast_call(0x0021bb00, toc);

		// A job that timed out twice is not waited for again until its event bit shows it finished (r14: a stuck job cost
		// 200 ms in every frame, ~5 fps for 80 s until the peer gave up with a communication error)
		static std::map<u32, u32> stuck_waits;

		if (static_cast<u32>(ppu.gpr[3]) == 0)
		{
			// Not running (finished): later launches of this job object are waited for normally again
			stuck_waits.erase(job);
			g_restore_timed_out_jobs.erase(job);
		}
		else if (stuck_waits[job] >= 2 && vm::read16(job + 0x200) == 0)
		{
			ppu.gpr[1] = old_r1;
			return;
		}

		if (static_cast<u32>(ppu.gpr[3]) != 0)
		{
			const auto start = std::chrono::steady_clock::now();
			const u64 limit_ms = g_restore_cancelled_jobs.contains(job) || g_restore_timed_out_jobs.contains(job) ? 20 : 200;
			vm::write16(bits, 1);

			while (!ppu.is_stopped())
			{
				ppu.gpr[3] = job + 0x200;
				ppu.gpr[4] = bits;
				ppu.gpr[5] = 1;
				ppu.fast_call(0x0075198c, toc);

				if (static_cast<u32>(ppu.gpr[3]) == 0)
				{
					stuck_waits[job] = 0;
					g_restore_timed_out_jobs.erase(job);
					break;
				}

				if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(limit_ms))
				{
					stuck_waits[job]++;

					if (g_restore_cancelled_jobs.contains(job))
					{
						g_restore_timed_out_jobs.insert(job);
					}

					if (static u32 logged = 0; logged++ < 50)
					{
						rb_online_log.warning("SPURS job 0x%08x did not finish within %u ms (a job of a discarded timeline), wait cancelled", job, limit_ms);
					}

					break;
				}

				sys_timer_usleep(ppu, 100);
			}
		}

		ppu.gpr[1] = old_r1;
	}

	// Copy of FUN_0021bce8(job) (stage teardown of a SPURS job object: wait for the job, cellSpursShutdownJobChain,
	// cellSpursJoinJobChain, detach the event flag, free the buffers) where the join only happens once the workload really
	// shut down. A job chain whose state went through rollbacks did not shut down at times and the join (libsre, LLE) waited
	// forever: black screen at the round end (r10, r12, r13). After 1 s the join is skipped (the workload slot stays
	// occupied, the chain does not run any more).
	void hle_spurs_job_teardown(ppu_thread& ppu, u32 job)
	{
		constexpr u64 toc = 0x0096A8B0;

		hle_spurs_job_wait(ppu, job);

		if (vm::read32(job + 0x14) == 0)
		{
			return;
		}

		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);

		const auto call = [&](u32 addr, u32 r3, u32 r4 = 0)
		{
			ppu.gpr[3] = r3;
			ppu.gpr[4] = r4;
			ppu.fast_call(addr, toc);
		};

		const u32 chain = vm::read32(job + 0x180);
		call(0x007519ac, chain); // cellSpursShutdownJobChain

		bool removable = true;

		if (chain && vm::check_addr(chain + 0x7c, vm::page_readable))
		{
			const u32 spurs = vm::read32(chain + 0x7c);
			const u32 wid = vm::read32(chain + 0x74);

			if (spurs && wid < 32 && vm::check_addr(spurs, vm::page_readable))
			{
				const u32 state_addr = spurs + (wid < 16 ? 0x80 : 0xD0) + (wid & 0xf);
				const auto start = std::chrono::steady_clock::now();

				// SPURS_WKL_STATE_SHUTTING_DOWN (3) until the SPUs left the workload, then REMOVABLE (4); joined only then
				while (vm::read8(state_addr) != 4 && vm::read8(state_addr) != 0 && !ppu.is_stopped())
				{
					if (std::chrono::steady_clock::now() - start > std::chrono::seconds(1))
					{
						removable = false;
						rb_online_log.error("SPURS job chain 0x%08x (job 0x%08x, workload %u) did not shut down within 1 s, not joined", chain, job, wid);
						break;
					}

					sys_timer_usleep(ppu, 500);
				}
			}
		}

		if (removable)
		{
			call(0x00751b4c, chain); // cellSpursJoinJobChain
		}

		call(0x0075184c, job + 0x200); // cellSpursEventFlagDetachLv2EventQueue

		for (const auto& [ptr, buf] : {std::pair{0x180u, 0x320u}, std::pair{0x184u, 0x330u}, std::pair{0x188u, 0x340u}, std::pair{0x308u, 0x350u}})
		{
			// Not shut down: the SPUs may still read the chain's buffers, they are leaked instead of freed
			if (vm::read32(job + ptr) != 0 && removable)
			{
				call(0x00139748, job + buf, 0);
			}

			if (ptr != 0x308)
			{
				vm::write32(job + ptr, 0);
			}
		}

		vm::write32(job + 0x14, 0);
		vm::write32(job + 0x308, 0);
		ppu.gpr[1] = old_r1;
	}

	// Copy of FUN_006b51dc(wrapper): destroys the ATRAC3plus decoder of a sound voice (FUN_006b4dc0: quit obj+0x218, signal
	// the cond obj+0x220, join the thread obj+0x210, free obj+0x24c, destroy the cond and mutex obj+0x21c; then vtable[0](obj),
	// FUN_006b2b40(obj)) with a bounded join. r18: a rolled back cellAdec state lost an AU-done, the decoder thread waited in
	// cellAtracDecode forever and the join stopped bnusCoreUpdateThread (all sound silent). A thread that does not exit within
	// 500 ms is left behind: object, mutex, cond and buffers leaked, its wrapper pointer goes to a private copy (the caller
	// zeroes and reuses the wrapper).
	void hle_at3p_decoder_destroy(ppu_thread& ppu, u32 wrapper)
	{
		constexpr u64 toc = 0x0096A8B0;
		const u32 obj = vm::read32(wrapper + 0x10);

		if (!obj)
		{
			return;
		}

		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
		const u32 sp = static_cast<u32>(ppu.gpr[1]);

		const u32 tid = static_cast<u32>(vm::read64(obj + 0x210));
		vm::write8(obj + 0x218, 1);
		sys_cond_signal(ppu, vm::read32(obj + 0x220));

		// Exited: a joinable thread after sys_ppu_thread_exit is a zombie until joined (or gone from IDM)
		const auto exited = [&]()
		{
			const auto r = idm::check<named_thread<ppu_thread>>(tid, [](named_thread<ppu_thread>& t)
			{
				const ppu_join_status s = t.joiner.load();
				return s == ppu_join_status::zombie || s == ppu_join_status::exited;
			});

			return !r || r.ret;
		};

		const auto start = std::chrono::steady_clock::now();
		bool done = exited();

		while (!done && !ppu.is_stopped() && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500))
		{
			sys_timer_usleep(ppu, 1000);
			done = exited();
		}

		if (done)
		{
			// The original sequence (the join returns at once)
			sys_ppu_thread_join(ppu, tid, vm::ptr<u64>::make(sp + 0x70));

			if (const u32 buf = vm::read32(obj + 0x24c))
			{
				ppu.gpr[3] = buf;
				ppu.fast_call(0x006b2b40, toc);
			}

			vm::write32(obj + 0x24c, 0);
			sys_cond_destroy(ppu, vm::read32(obj + 0x220));
			sys_mutex_destroy(ppu, vm::read32(obj + 0x21c));

			const u32 opd = vm::read32(vm::read32(obj)); // vtable[0]
			ppu.gpr[3] = obj;
			ppu.fast_call(vm::read32(opd), vm::read32(opd + 4));
			ppu.gpr[3] = obj;
			ppu.fast_call(0x006b2b40, toc);
		}
		else
		{
			// One 64 KiB block for at most 256 wrapper copies (outside the snapshot ranges)
			static u32 leaked = 0;
			static u32 pool = 0;

			if (!pool)
			{
				pool = vm::alloc(0x10000, vm::main);
			}

			if (const u32 copy = pool && leaked < 0x100 ? pool + leaked * 0x100 : 0)
			{
				std::memcpy(vm::base(copy), vm::base(wrapper), 0x100);
				vm::write32(copy + 0x10, obj);
				vm::write32(obj + 4, copy);
			}

			rb_online_log.error("AT3P decoder thread 0x%x (object 0x%08x) did not exit within 500 ms: left behind (%u so far)", tid, obj, ++leaked);
		}

		ppu.gpr[1] = old_r1;
	}

	// Copy of FUN_0069afa0 (stop a sound voice and wait until the sound library reports it stopped) with a time limit.
	// A rollback restores the game's voice table but not the sound library (asynchronous threads), so a voice can refer
	// to a sound the library already reused for a looping one, and the original waited for it forever (seen at the end of
	// a round: "Synching..." hang). 2 waits of 0x14d5 us (several voices are stopped at once, longer waits stalled the game
	// until the peer disconnected).
	void hle_voice_stop_wait(ppu_thread& ppu, u32 voice)
	{
		constexpr u64 toc = 0x0096A8B0;

		if (vm::read32(voice + 0x20) != 1)
		{
			return;
		}

		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);

		ppu.gpr[3] = vm::read32(voice + 4);
		ppu.fast_call(0x006b055c, toc);

		for (u32 n = 0; !ppu.is_stopped(); n++)
		{
			ppu.gpr[3] = vm::read32(voice + 4);
			ppu.gpr[4] = vm::read32(voice + 8);
			ppu.fast_call(0x006afd8c, toc);

			if (static_cast<u8>(ppu.gpr[3]) != 0)
			{
				break;
			}

			if (n == 2)
			{
				static u32 reported = 0;

				if (reported++ < 20)
				{
					rb_online_log.warning("Sound voice 0x%08x did not stop (sound 0x%08x id %u), not waiting any longer", voice,
						+vm::read32(voice + 4), +vm::read32(voice + 8));
				}

				break;
			}

			ppu.gpr[3] = 0x14d5;
			ppu.fast_call(0x0069de20, toc);
		}

		ppu.gpr[3] = voice;
		ppu.fast_call(0x0069a040, toc);
		ppu.gpr[1] = old_r1;
	}

	// ---- Side heap (RB_SIDE_HEAP=1) ----------------------------------------------------------------------------------
	// The game's heap H = *(0x00B02A70) (lwmutex H+0x28) is also used by the network and sound threads (sampled owner of
	// its lock during a fight: net_workerThread_1 most of the time). A rollback restores the heap's metadata while those
	// threads' allocations live on -> broken free lists, cyclic task / draw lists, crashes. Allocations of every thread
	// except the PPU main thread go to a separate region excluded from the snapshot instead. The game's allocator API
	// (ANALYZA.md section 8): alloc core FUN_0013cd90(H, handle, align_mask, size, type, cb) with the callers 0013d1dc,
	// 0013d224, 0013d25c; free core FUN_001395e4(H, handle, name) with the callers 001399b4, 00139748, 00139888, 001398a4,
	// 0013975c, 001397a4. Those callers are replaced: the main thread runs the core functions as before, a side heap
	// block is recognised by its descriptor address. Blocks get a fake 0x28 byte descriptor (+4 data, +8 align mask,
	// +0xc size, +0x10 type 0xc, +0x14 share count), raw blocks keep the descriptor pointer at data - 4.
	namespace side_heap
	{
		constexpr u64 toc = 0x0096A8B0;
		constexpr u32 region_size = 16 << 20;
		std::mutex g_mutex;
		u32 g_base = 0, g_top = 0;
		std::map<u32, std::vector<u32>> g_free;        // chunk size -> free chunks
		std::unordered_map<u32, u32> g_chunk_size;     // chunk (= descriptor) -> chunk size
		u64 g_allocs = 0, g_frees = 0, g_bytes = 0, g_double_frees = 0;
		// The side heap is outside the snapshots, the main thread is not: when the main thread frees a side block and a
		// rollback re-simulates that frame, the block is freed a second time. Freed chunks wait 1 s (> the deepest
		// re-simulation) before they are handed out again, and a free of a chunk that is not in use is ignored, so the
		// second free can neither break the free lists nor release a block another thread got in the meantime.
		std::unordered_set<u32> g_unused;              // free or waiting chunks
		std::deque<std::pair<u32, std::chrono::steady_clock::time_point>> g_waiting;

		// Diagnostics: calls per hook and thread (main / other)
		std::atomic<u64> g_calls[9][2]{};
		const char* const g_call_names[9]{"alloc_h1", "alloc_h", "alloc_raw", "free_raw", "free_h", "destruct", "destruct_del", "assign", "core_free_other"};

		void count(u32 hook, ppu_thread& ppu)
		{
			g_calls[hook][ppu.id == 0x1000000 ? 0 : 1]++;
		}

		void log_stats()
		{
			std::string out;

			for (u32 i = 0; i < 9; i++)
			{
				fmt::append(out, " %s=%u/%u", g_call_names[i], +g_calls[i][0], +g_calls[i][1]);
			}

			rb_online_log.notice("Side heap calls (main/other):%s; side blocks %u allocs %u frees, %u repeated frees ignored", out, g_allocs, g_frees, g_double_frees);
		}

		bool enabled()
		{
			static const bool on = std::getenv("RB_SIDE_HEAP") && *std::getenv("RB_SIDE_HEAP") == '1';
			return on;
		}

		u32 heap()
		{
			return vm::read32(vm::read32(0x00965d54));
		}

		bool is_side_desc(u32 desc)
		{
			return g_base > 1 && desc >= g_base && desc < g_base + region_size;
		}

		// Side heap only for other threads than the main thread and blocks up to 2 MiB (large ones are loader buffers);
		// when the side heap cannot serve a block, the game's heap does (as before)
		bool use_side(ppu_thread& ppu, u32 size)
		{
			return ppu.id != 0x1000000 && size <= (2u << 20) && g_base > 1;
		}

		// Fake descriptor + data block; returns the descriptor (0 when the region is full)
		u32 alloc(u32 align_mask, u32 size)
		{
			std::lock_guard lock(g_mutex);

			if (g_base <= 1)
			{
				return 0;
			}

			const u32 align = std::max<u32>(align_mask ? (align_mask & (0u - align_mask)) : 0x10, 0x10);
			const u32 need = 0x30 + align + size;
			u32 chunk_size = 0x40;

			while (chunk_size < need && chunk_size < 0x10000)
			{
				chunk_size <<= 1;
			}

			if (chunk_size < need)
			{
				chunk_size = utils::align<u32>(need, 0x10000);
			}

			u32 chunk = 0;

			for (const auto now = std::chrono::steady_clock::now(); !g_waiting.empty() && now - g_waiting.front().second > std::chrono::seconds(1);)
			{
				g_free[g_chunk_size[g_waiting.front().first]].push_back(g_waiting.front().first);
				g_waiting.pop_front();
			}

			if (auto& list = g_free[chunk_size]; !list.empty())
			{
				chunk = list.back();
				list.pop_back();
				g_unused.erase(chunk);
			}
			else if (g_top + chunk_size <= g_base + region_size)
			{
				chunk = g_top;
				g_top += chunk_size;
				g_chunk_size[chunk] = chunk_size;
			}
			else
			{
				static u32 reported = 0;

				if (reported++ < 10)
				{
					rb_online_log.error("Side heap full (allocation of %u bytes)", size);
				}

				return 0;
			}

			const u32 data = utils::align<u32>(chunk + 0x2c, align);
			std::memset(vm::get_super_ptr<u8>(chunk), 0, 0x28);
			vm::write32(chunk + 4, data);
			vm::write32(chunk + 8, align_mask);
			vm::write32(chunk + 0xc, size);
			vm::write32(chunk + 0x10, 0xc);
			vm::write32(data - 4, chunk);
			g_allocs++;
			g_bytes += chunk_size;

			if (g_allocs % 1000 == 1)
			{
				rb_online_log.notice("Side heap: %u allocations, %u frees, %u KiB in use, top 0x%08x", g_allocs, g_frees, g_bytes >> 10, g_top);
			}

			return chunk;
		}

		void free(u32 desc)
		{
			std::lock_guard lock(g_mutex);
			const auto found = g_chunk_size.find(desc);

			if (found == g_chunk_size.end())
			{
				rb_online_log.error("Side heap: free of an unknown block 0x%08x", desc);
				return;
			}

			if (!g_unused.insert(desc).second)
			{
				g_double_frees++;
				return;
			}

			g_waiting.emplace_back(desc, std::chrono::steady_clock::now());
			g_frees++;
			g_bytes -= found->second;
		}

		// Runs a game function with a fresh stack frame (0x100 bytes; scratch at r1 + 0x80)
		template <typename F>
		void with_frame(ppu_thread& ppu, F&& f)
		{
			const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
			const u64 old_r2 = ppu.gpr[2];
			ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
			vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
			f(static_cast<u32>(ppu.gpr[1]) + 0x80);
			ppu.gpr[1] = old_r1;
			ppu.gpr[2] = old_r2;
		}

		u64 call(ppu_thread& ppu, u32 func, std::initializer_list<u64> args)
		{
			u32 r = 3;

			for (const u64 a : args)
			{
				ppu.gpr[r++] = a;
			}

			ppu.fast_call(func, toc);
			return ppu.gpr[3];
		}

		void drop(u32 desc)
		{
			if (std::lock_guard lock(g_mutex); g_unused.contains(desc))
			{
				g_double_frees++;
				return;
			}

			if (const u32 shared = vm::read32(desc + 0x14))
			{
				vm::write32(desc + 0x14, shared - 1);
			}
			else
			{
				free(desc);
			}
		}

		// Release of a handle (free core semantics): side descriptor -> share count or free, else the core
		void release(ppu_thread& ppu, u32 handle, u32 name)
		{
			const u32 desc = vm::read32(handle + 4);

			if (is_side_desc(desc))
			{
				drop(desc);
				vm::write32(handle + 4, 0);
				return;
			}

			if (ppu.id != 0x1000000 && desc)
			{
				g_calls[8][1]++;
			}

			with_frame(ppu, [&](u32) { call(ppu, 0x001395e4, {heap(), handle, name}); });
		}

		u32 destructor_name()
		{
			return vm::read32(0x00965d7c);
		}

		u32 handle_vtable()
		{
			return vm::read32(0x00965d50);
		}
	}

	// FUN_0013d1dc(handle, align, size, a, b): handle+8 = 1, allocation into the handle
	void hle_heap_alloc_handle1(ppu_thread& ppu, u32 handle, u32 align, u32 size, u32 a, u32 b)
	{
		side_heap::count(0, ppu);
		vm::write32(handle + 8, 1);

		if (side_heap::use_side(ppu, size))
		{
			if (const u32 desc = side_heap::alloc(align, size))
			{
				vm::write32(handle + 4, desc);
				ppu.gpr[3] = 1;
				return;
			}
		}

		side_heap::with_frame(ppu, [&](u32) { side_heap::call(ppu, 0x0013cd90, {side_heap::heap(), handle, align, size, a, b}); });
	}

	// FUN_0013d224(handle, align, size, a, b)
	void hle_heap_alloc_handle(ppu_thread& ppu, u32 handle, u32 align, u32 size, u32 a, u32 b)
	{
		side_heap::count(1, ppu);
		if (side_heap::use_side(ppu, size))
		{
			if (const u32 desc = side_heap::alloc(align, size))
			{
				vm::write32(handle + 4, desc);
				ppu.gpr[3] = 1;
				return;
			}
		}

		side_heap::with_frame(ppu, [&](u32) { side_heap::call(ppu, 0x0013cd90, {side_heap::heap(), handle, align, size, a, b}); });
	}

	// FUN_0013d25c(H, align, size, type, flags, cb): raw block (data pointer, descriptor at data - 4)
	u32 hle_heap_alloc_raw(ppu_thread& ppu, u32 h, u32 align, u32 size, u32 type, u32 flags, u32 cb)
	{
		side_heap::count(2, ppu);
		if (side_heap::use_side(ppu, size))
		{
			if (const u32 desc = side_heap::alloc(align, size))
			{
				return vm::read32(desc + 4);
			}
		}

		{
			u32 result = 0;

			side_heap::with_frame(ppu, [&](u32 tmp)
			{
				vm::write32(tmp, side_heap::handle_vtable());
				vm::write32(tmp + 4, 0);
				vm::write32(tmp + 8, flags);

				// FUN_0013d25c: on success desc+0x14 is incremented and decremented again by the temporary handle's
				// destructor, so only the data pointer remains
				if (static_cast<u8>(side_heap::call(ppu, 0x0013cd90, {h, tmp, align, size, type, cb})) != 0)
				{
					result = vm::read32(vm::read32(tmp + 4) + 4);
				}
			});

			return result;
		}
	}

	// FUN_001399b4(H, ptr, name): raw free
	void hle_heap_free_raw(ppu_thread& ppu, u32 h, u32 ptr, u32 name)
	{
		side_heap::count(3, ppu);
		if (!ptr || !vm::read32(ptr - 4))
		{
			return;
		}

		const u32 desc = vm::read32(ptr - 4);

		if (side_heap::is_side_desc(desc))
		{
			side_heap::drop(desc);
			return;
		}

		side_heap::with_frame(ppu, [&](u32 tmp)
		{
			// Temporary handle {vtable, desc, flag}; flag by descriptor type (table at 0x00139a54)
			const u32 type = vm::read32(desc + 0x10);
			const u32 flag = type == 0 ? 1 : type == 8 ? 4 : type == 9 ? 5 : type == 10 ? 7 : type == 11 ? 8 : 0;
			vm::write32(tmp, side_heap::handle_vtable());
			vm::write32(tmp + 4, desc);
			vm::write32(tmp + 8, flag);
			side_heap::call(ppu, 0x001395e4, {h, tmp, name});
		});
	}

	// FUN_00139748(handle, name)
	void hle_heap_free_handle(ppu_thread& ppu, u32 handle, u32 name)
	{
		side_heap::count(4, ppu);
		side_heap::release(ppu, handle, name);
	}

	// FUN_00139888(handle) / FUN_001397a4(handle): destructors
	void hle_heap_destruct(ppu_thread& ppu, u32 handle)
	{
		side_heap::count(5, ppu);
		vm::write32(handle, side_heap::handle_vtable());
		side_heap::release(ppu, handle, side_heap::destructor_name());
	}

	// FUN_0013975c(handle): destructor + FUN_00137c8c(handle)
	void hle_heap_destruct_delete(ppu_thread& ppu, u32 handle)
	{
		side_heap::count(6, ppu);
		vm::write32(handle, side_heap::handle_vtable());
		side_heap::release(ppu, handle, side_heap::destructor_name());
		side_heap::with_frame(ppu, [&](u32) { side_heap::call(ppu, 0x00137c8c, {handle}); });
	}

	// FUN_001398a4(dst, src): handle assignment
	u32 hle_heap_assign(ppu_thread& ppu, u32 dst, u32 src)
	{
		side_heap::count(7, ppu);
		if (vm::read32(dst + 4) != 0 && vm::read32(src + 4) != vm::read32(dst + 4))
		{
			side_heap::release(ppu, dst, vm::read32(0x00965d80));
		}

		side_heap::with_frame(ppu, [&](u32) { side_heap::call(ppu, 0x0013760c, {side_heap::heap(), dst, src}); });
		return dst;
	}

	// ---- Sound outside the rollback --------------------------------------------------------------------------------
	// Copy of FUN_0068e108 (PLAY: FUN_00694ba8 prepares the voices and writes the handle to *out, then KeyOn each one).
	// A live start is recorded (frame, out = the sound object's handle field, bank r3, cue r4, handle). When a
	// re-simulated frame starts the same sound again, nothing is started and the recorded handle is written back: the
	// sound plays once and its object can still stop it (before, the object got handle -1 and looping sounds played
	// forever; the library's voices ran out: sound gone, the next match hung on the VS screen). Sounds of the discarded
	// timeline that the re-simulation did not start again are stopped when it ends; new ones play (a few frames late).
	struct sound_play_t
	{
		u64 frame;
		u32 out, bank, cue, handle;
		bool claimed;
		bool cosmetic = false; // started by work left out of re-simulated frames (drawing, effects): never stopped
		bool collected = false; // stopped by the leak cleanup
		bool rolled = false; // a snapshot load went back to or before its start frame (its game object was restored)
	};

	std::deque<sound_play_t> g_sound_plays;
	u64 g_sound_resim_first = umax;
	bool g_sound_was_resim = false;

	// RB_STRESS_SOUND=1 (offline stress only, without RB_ONLINE_PREDICT): the sound protections of the online fight run in the
	// offline stress test too (sound/mixer exclusions without the online manager, the HLE sound hooks, dedup and cleanup of
	// sounds across the stress rollbacks), so a stress run tests sound like the online fight. Off by default: online unchanged.
	bool stress_sound()
	{
		static const bool on = std::getenv("RB_STRESS_SOUND") && *std::getenv("RB_STRESS_SOUND") == '1' && !config().predict;
		return on;
	}

	// Frame and re-simulation state for the sound code: the frame of the main loop boundary, not the pad read position seq[0],
	// which the port 0 read advances at the start of the frame. A PLAY (after the read) in the last re-simulated frame L-1 saw
	// seq[0] = L and state_resimulating(L) = false: it counted as live, was not deduplicated and the sound started a second
	// time a few ms after the first (online: at every rollback; offline depth-1 stress: always). The other users of
	// is_resimulating() (input, saves) keep their behaviour.
	u32 sound_frame_no()
	{
		return static_cast<u32>(boundary_frame());
	}

	bool sound_resim()
	{
		return (use_stress_ring() || stress_sound()) ? state_resimulating(boundary_frame()) : is_resimulating();
	}

	// Sounds are recorded (dedup, discarded stop, leak cleanup) in the online fight, or in the offline stress with RB_STRESS_SOUND
	bool sound_recording_active()
	{
		return fight_active() || (stress_sound() && state_stress_ready());
	}

	bool sound_dedup()
	{
		static const bool on = !std::getenv("RB_ONLINE_SOUND_DEDUP") || *std::getenv("RB_ONLINE_SOUND_DEDUP") != '0';
		return on;
	}

	// Sound diagnostics (every 600 live frames): starts, failed starts (by code), starts skipped as already playing, stops of
	// discarded sounds, active voices of the library
	u64 g_snd_ok = 0, g_snd_fail = 0, g_snd_claimed = 0, g_snd_stopped = 0;
	u64 g_snd_skipped = 0; // re-simulated PLAYs not started (RB_SOUND_SIMPLE)

	// RB_SOUND_SEPARATE=1 (hybrid): the sound library is outside the rollback and the DLL's sound boundary replays the
	// library calls of re-simulated frames. The bookkeeping that guessed which sounds belonged to a discarded timeline
	// (stop after the re-simulation, leak cleanup) is off: it saw no PLAY in the re-simulation and cut correct sounds (o243:
	// 278 "discarded" stops). Sounds play on as if there was no rollback.
	// RB_ONLINE_SOUND_ROLLBACK (diagnostics): the sound library is rolled back like the game; read once (the sound
	// exclusions run every frame: getenv there cost ~1.7 % of a re-simulation, o250)
	bool sound_rollback_env()
	{
		static const bool on = std::getenv("RB_ONLINE_SOUND_ROLLBACK") != nullptr;
		return on;
	}

	bool sound_separate()
	{
		static const bool on = std::getenv("RB_SOUND_SEPARATE") && *std::getenv("RB_SOUND_SEPARATE") == '1';
		return on;
	}

	bool sound_simple()
	{
		// With RB_SOUND_SEPARATE the hybrid's sound boundary replays the calls of a re-simulated frame; a PLAY that still
		// reaches the library there belongs to the corrected timeline and must sound: default off then
		static const bool separate = std::getenv("RB_SOUND_SEPARATE") && *std::getenv("RB_SOUND_SEPARATE") == '1';
		static const bool on = std::getenv("RB_SOUND_SIMPLE") ? *std::getenv("RB_SOUND_SIMPLE") != '0' : !separate;
		return on;
	}
	std::map<s32, u64> g_snd_fail_codes;

	// RB_SOUND_PAGES=1 (diagnostics): snapshot pages changed by the sound library (PLAY, the per-frame library update):
	// its runtime state, which a rollback must not restore. Logged every 100 measured calls with change counts.
	struct sound_page_probe
	{
		std::vector<std::pair<u32, u64>> before;
		static inline std::map<u32, u32> changed;
		static inline u32 calls = 0;

		static bool on()
		{
			static const bool v = std::getenv("RB_SOUND_PAGES") && *std::getenv("RB_SOUND_PAGES") == '1';
			return v && calls < 2000;
		}

		static u64 hash(u32 page)
		{
			const u8* p = vm::get_super_ptr<u8>(page);
			u64 h = 0xcbf29ce484222325;

			for (u32 i = 0; i < 0x1000; i += 8)
			{
				u64 w;
				std::memcpy(&w, p + i, 8);
				h = (h ^ w) * 0x100000001b3;
			}

			return h;
		}

		sound_page_probe()
		{
			if (!on())
			{
				return;
			}

			for (const u32 page : state_saved_pages())
			{
				if (vm::check_addr(page, vm::page_readable))
				{
					before.emplace_back(page, hash(page));
				}
			}
		}

		~sound_page_probe()
		{
			if (before.empty())
			{
				return;
			}

			for (const auto& [page, h] : before)
			{
				if (vm::check_addr(page, vm::page_readable) && hash(page) != h)
				{
					changed[page]++;
				}
			}

			if (++calls % 100 == 0)
			{
				std::string text;
				u32 begin = 0, end = 0, peak = 0;

				for (const auto& [page, n] : changed)
				{
					if (page != end)
					{
						if (end)
						{
							fmt::append(text, " 0x%08x-0x%08x(%u)", begin, end, peak);
						}

						begin = page;
						peak = 0;
					}

					end = page + 0x1000;
					peak = std::max(peak, n);
				}

				if (end)
				{
					fmt::append(text, " 0x%08x-0x%08x(%u)", begin, end, peak);
				}

				rb_online_log.notice("Sound library pages in the snapshot (%u calls):%s", calls, text);
			}
		}
	};

	// Sound library state outside the rollback, learnt: during the first RB_SOUND_LEARN calls (default 300; 0 = off) of the
	// sound start (PLAY) and of the per-frame library update in live fight frames, every byte of the game heap pages of
	// the snapshot (0x36C00000-0x37600000) that the call changes is counted. Bytes changed in at least 5% of the calls
	// (library counters, the game's voice table, the bank runtime data, the handles of the sound objects) are excluded from
	// the snapshot: a rollback restored them while the library went on, the bank data got inconsistent and every start
	// failed ("KeyOn Failed" 0x80000000: silence; the next match hung on the VS screen waiting for the announcer).
	struct sound_learner
	{
		static inline std::map<u32, std::array<u16, 0x1000>> counts;
		static inline u32 calls = 0;
		static inline bool applied = false;
		static inline bool active = false;
		std::vector<std::pair<u32, std::array<u8, 0x1000>>> before;

		static u32 target()
		{
			static const u32 v = std::getenv("RB_SOUND_LEARN") ? static_cast<u32>(std::atoi(std::getenv("RB_SOUND_LEARN"))) : 0u; // off by default: it also caught bytes of other threads (input manager)
			return v;
		}

		void begin()
		{
			if (applied || active || !target() || !fight_active() || sound_resim())
			{
				return;
			}

			active = true;

			for (const u32 page : state_saved_pages())
			{
				if (page >= 0x36C00000 && page < 0x37600000 && vm::check_addr(page, vm::page_readable))
				{
					before.emplace_back(page, std::array<u8, 0x1000>{});
					std::memcpy(before.back().second.data(), vm::get_super_ptr<u8>(page), 0x1000);
				}
			}
		}

		void end()
		{
			if (before.empty())
			{
				active = false;
				return;
			}

			for (const auto& [page, data] : before)
			{
				const u8* now = vm::get_super_ptr<u8>(page);

				if (vm::check_addr(page, vm::page_readable) && std::memcmp(now, data.data(), 0x1000) != 0)
				{
					auto& c = counts[page];

					for (u32 b = 0; b < 0x1000; b++)
					{
						c[b] += now[b] != data[b];
					}
				}
			}

			before.clear();
			active = false;

			if (++calls >= target())
			{
				apply();
			}
		}

		static void apply()
		{
			applied = true;
			const u32 threshold = std::max<u32>(3, calls / 20);
			u32 ranges = 0, bytes = 0;

			for (const auto& [page, c] : counts)
			{
				for (u32 b = 0; b < 0x1000;)
				{
					if (c[b] < threshold)
					{
						b++;
						continue;
					}

					u32 e = b + 1;

					for (u32 last = b; e < 0x1000 && e - last <= 8; e++)
					{
						if (c[e] >= threshold)
						{
							last = e;
						}
					}

					while (e > b + 1 && c[e - 1] < threshold)
					{
						e--;
					}

					const u32 rb = (page + b) & ~3u, re = (page + e + 3) & ~3u;
					state_exclude(rb, re, "sound library state (learnt)");
					ranges++;
					bytes += re - rb;
					b = e;
				}
			}

			counts.clear();
			rb_online_log.success("Sound library state learnt from %u calls: %u bytes in %u ranges excluded from the snapshot", calls, bytes, ranges);
		}
	};

	sound_learner g_sound_learner;

	void sound_learn_begin()
	{
		state_file_foreign_scope(true);
		g_sound_learner.begin();
	}

	void sound_learn_end()
	{
		g_sound_learner.end();
		state_file_foreign_scope(false);
	}

	s32 hle_sound_play(ppu_thread& ppu)
	{
		constexpr u64 toc = 0x0096A8B0;
		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		const u32 bank = static_cast<u32>(ppu.gpr[3]);
		const u32 cue = static_cast<u32>(ppu.gpr[4]);
		const u32 out = vm::read32(old_r1 + 0x74);
		const u32 caller = static_cast<u32>(ppu.lr);
		u32 seq[2]{};
		get_seq(seq);
		seq[0] = sound_frame_no();
		const bool resim = sound_resim();

		if (resim && sound_dedup())
		{
			// The same sound of the discarded timeline is taken over when it started within RB_SOUND_MATCH_FRAMES (default 3)
			// frames of this one (the nearest): after a misprediction a hit lands a frame or two earlier or later, and
			// stopping the first one and starting it again cut the sound (heard as distortion of some effects)
			static const u64 tol = std::getenv("RB_SOUND_MATCH_FRAMES") ? std::strtoull(std::getenv("RB_SOUND_MATCH_FRAMES"), nullptr, 10) : 3;
			sound_play_t* best = nullptr;
			u64 best_d = umax;

			for (auto& p : g_sound_plays)
			{
				// The same sound object is preferred; a sound of another object (restored with another address) only counts
				// with a penalty
				if (!p.claimed && p.bank == bank && p.cue == cue)
				{
					const u64 d = (p.frame > seq[0] ? p.frame - seq[0] : seq[0] - p.frame) + (p.out == out ? 0 : 1);

					if (d <= tol + 1 && d < best_d)
					{
						best = &p;
						best_d = d;
					}
				}
			}

			if (best)
			{
				best->claimed = true;

				if (out)
				{
					vm::write32(out, best->handle);
				}

				g_snd_claimed++;
				return 0;
			}

			// RB_SOUND_SIMPLE (default on, 0 = off): sound ignores rollbacks. What played, plays on (never stopped after a
			// re-simulation); a re-simulated frame starts nothing new. A sound that only the corrected timeline has is not
			// heard (rare: a mispredicted hit); every cut or restart was heard as distortion.
			if (sound_simple())
			{
				g_snd_skipped++;
				return 0;
			}
		}

		std::optional<sound_page_probe> probe;

		if (!resim && sound_page_probe::on())
		{
			probe.emplace();
		}

		sound_learn_begin();

		const u64 a3 = ppu.gpr[3], a4 = ppu.gpr[4], a5 = ppu.gpr[5], a8 = ppu.gpr[8], a9 = ppu.gpr[9], a10 = ppu.gpr[10];
		const u32 sp = old_r1 - 0xb0;
		vm::write64(sp, old_r1);
		vm::write64(sp + 0x70, a10);
		vm::write64(sp + 0x78, out);
		vm::write64(sp + 0x80, sp + 0x90);
		vm::write32(sp + 0x90, 0);

		const u32 mgr = vm::read32(vm::read32(0x0097078c));
		ppu.gpr[1] = sp;
		ppu.gpr[3] = mgr;
		ppu.gpr[4] = a3;
		ppu.gpr[5] = a4;
		ppu.gpr[6] = a5;
		ppu.gpr[7] = a9;
		ppu.gpr[8] = a8;
		ppu.gpr[9] = a8;
		ppu.gpr[10] = a9;
		ppu.fast_call(0x00694ba8, toc);
		s32 ret = static_cast<s32>(ppu.gpr[3]);

		if (ret == 0)
		{
			const s32 count = static_cast<s32>(vm::read32(sp + 0x90));

			for (s32 i = 0; i < count; i++)
			{
				ppu.gpr[3] = vm::read32(vm::read32(0x0097078c));
				ppu.gpr[4] = i;
				ppu.fast_call(0x0068ff48, toc);
				ppu.gpr[3] = static_cast<u32>(ppu.gpr[3]);
				ppu.fast_call(0x0068de98, toc);
				ret = static_cast<s32>(ppu.gpr[3]);

				if (ret != 0)
				{
					break;
				}
			}
		}

		ppu.gpr[1] = old_r1;
		sound_learn_end();

		if (ret == 0)
		{
			g_snd_ok++;
		}
		else
		{
			g_snd_fail++;
			g_snd_fail_codes[ret]++;

			// Where is the data of the failing sound: bank pointer (BM + bank * 0x2008 + 0x108), the bank info table
			// (BM + 0x8030c) and whether the snapshot holds them (1 = saved/restored, 2 = excluded, 0 = not in the snapshot)
			if (static u32 logged = 0; logged++ < 20)
			{
				const u32 smgr = vm::read32(vm::read32(0x0097078c));
				const u32 bm = vm::read32(smgr + 0x1c38);
				const u32 bank_ptr = vm::read32(bm + static_cast<u32>(a3) * 0x2008 + 0x108);
				rb_online_log.warning("Sound start failed 0x%08x: bank %u cue %u, bank data 0x%08x (snapshot %d), bank infos 0x%08x (snapshot %d), game frame %u",
					static_cast<u32>(ret), static_cast<u32>(a3), static_cast<u32>(a4), bank_ptr, state_addr_saved(bank_ptr), bm + 0x8030c,
					state_addr_saved(bm + 0x8030c), seq[0]);

				// bnusCore voice allocator (global 0x00D66D60: count +0x80, voices *(+0xc0) x 0x2f0, allocated +0xfc, failed
				// allocations +0xf8; a voice is busy while +0x1a8 or +0x140 is set) and the nuSound2 voices holding a core
				// voice (T = *(0x00D65ABC), 0xC0 x 0x48 at T + 0x24014, core voice id at +4, -1 = none)
				constexpr u32 g = 0x00D66D60;
				const u32 n = vm::read32(g + 0x80), core = vm::read32(g + 0xc0), t = vm::read32(0x00D65ABC);
				u32 busy = 0, held = 0;

				for (u32 i = 0; core && i < n && i < 0x400; i++)
				{
					busy += vm::read32(core + i * 0x2f0 + 0x1a8) != 0 || vm::read32(core + i * 0x2f0 + 0x140) != 0;
				}

				for (u32 i = 0; t && i < 0xC0; i++)
				{
					held += vm::read32(t + 0x24014 + i * 0x48 + 4) != umax;
				}

				rb_online_log.warning("Sound core: %u voices, %u busy, allocated counter %d, failed allocations %u, nuSound2 voices holding one %u (core 0x%08x saved %d, T 0x%08x saved %d)",
					n, busy, static_cast<s32>(vm::read32(g + 0xfc)), +vm::read32(g + 0xf8), held, core, state_addr_saved(core), t, state_addr_saved(t + 0x24014));
			}
		}

		// RB_SOUND_TRACE=1: every PLAY of the fight with frame, bank/cue, object field, caller (first 400)
		if (static const bool trace = std::getenv("RB_SOUND_TRACE") && *std::getenv("RB_SOUND_TRACE") == '1'; trace && sound_recording_active())
		{
			if (static u32 traced = 0; traced++ < 400)
			{
				rb_online_log.notice("Sound PLAY %s: frame %u, bank %u, cue %u, out 0x%08x, cosmetic %d, ret %d, lr 0x%08x, task 0x%08x", resim ? "resim (unmatched)" : "live",
					seq[0], bank, cue, out, in_cosmetic_work(), ret, caller, current_task_method());
			}
		}

		if (ret == 0 && out && sound_dedup() && sound_recording_active())
		{
			g_sound_plays.push_back({seq[0], out, bank, cue, vm::read32(out), resim, in_cosmetic_work(), false});
		}

		return ret;
	}

	// After a snapshot load: a sound started by drawing work (left out of re-simulated frames) keeps playing in the library
	// (not rolled back), but the load reset its game object's handle field to "none" (-1): the object could not stop it and
	// started it once more in the next live frame, looping sounds leaked a voice per rollback until all 192 were busy
	// ("KeyOn Failed" 0x80000000, silence, the next match hung on the VS screen waiting for the announcer: r1/r2 tests).
	// The handle of every such sound started since the restored frame is written back into its object.
	void sound_after_load(u64 target)
	{
		if (!sound_dedup())
		{
			return;
		}

		u32 restored = 0;

		// The leak cleanup only touches sounds whose game object a load restored
		for (auto& p : g_sound_plays)
		{
			if (p.frame >= target)
			{
				p.rolled = true;
			}
		}

		for (const auto& p : g_sound_plays)
		{
			if (p.cosmetic && p.frame >= target && p.out && vm::check_addr(p.out, vm::page_readable | vm::page_writable) &&
				vm::read32(p.out) == 0xffffffffu)
			{
				vm::write32(p.out, p.handle);
				restored++;
			}
		}

		if (static u64 total = 0; restored && (total += restored) % 50 < restored)
		{
			rb_online_log.notice("Sound handles of drawing work restored after loads: %u", total);
		}
	}

	void stress_sound_after_load(u64 target)
	{
		if (stress_sound())
		{
			sound_after_load(target);
		}
	}

	// RB_SOUND_KEEP_RUNTIME=1: the playback runtime of the bnusCore voices (0x2f0 each, [v, v+0x150): position, decoder history,
	// envelope, SPU status; written by the SPU mixer) survives a snapshot load for voices that exist unchanged in both timelines
	// (same allocation, generation +0x1a4, valid +0x1ac and host request +0x1a8 before and after the load). The voices stay in
	// the snapshot (splitting a voice between restored and live parts crashed at the first rollback, RB_SOUND_EXCL_CORE_RUN);
	// only their playback progress is put back after the load, so playing sounds no longer jump back at every rollback.
	struct voice_runtime_t
	{
		u8 alloc;
		u32 gen, req, valid;
		std::array<u8, 0x150> data;
		// RB_SOUND_KEEP_PARAMS: the per-frame parameters the main thread sets (volume/pitch/pan/sends/pause)
		std::array<u8, 0x40> param_block; // +0x160..+0x19f (FUN_006b20f4 / 20ec / 2104 / 2134)
		u32 rate, pause;                  // +0x1bc sample rate, +0x1cc pause counter
		std::array<u8, 0x100> sends;      // +0x1d0..+0x2cf (0x20 per bus, FUN_006b17a8)
		u32 pitch_ratio;                  // +0x2d0 (FUN_006b17f4)
	};

	// nuSound2 voice state (0x48 at T+0x24014): binding to the core voice (+4 index, +8 generation), +0x20 active,
	// parameters +0x28 volume, +0x2c pitch, +0x38..+0x47 send slot/levels/pan
	struct nu_voice_params_t
	{
		u32 core_idx, core_gen, state;
		std::array<u8, 8> vol_pitch;  // +0x28..+0x2f
		std::array<u8, 0x10> sends;   // +0x38..+0x47
	};

	std::vector<voice_runtime_t> g_voice_rt;
	std::vector<nu_voice_params_t> g_nu_voice;
	bool g_voice_rt_taken = false;

	bool sound_keep_runtime()
	{
		static const bool on = std::getenv("RB_SOUND_KEEP_RUNTIME") && *std::getenv("RB_SOUND_KEEP_RUNTIME") == '1';
		return on;
	}

	// RB_SOUND_KEEP_PARAMS=1 (extends KEEP_RUNTIME, also enables it): a sound that plays in both timelines keeps all its
	// parameters across a load, not only its playback progress: a rollback changes nothing about such a sound. The KeyOn
	// handshake (+0x1a0..+0x1b4) and the callbacks are never copied (restored with the snapshot).
	bool sound_keep_params()
	{
		static const bool on = std::getenv("RB_SOUND_KEEP_PARAMS") && *std::getenv("RB_SOUND_KEEP_PARAMS") == '1';
		return on;
	}

	static bool core_voices(u32& core, u32& n)
	{
		constexpr u32 g = 0x00D66D60;
		core = vm::read32(g + 0xc0);
		n = vm::read32(g + 0x80);
		return core && n && n <= 0x400 && vm::check_addr(core, vm::page_readable | vm::page_writable, n * 0x2f0);
	}

	static u32 nu_voice_states()
	{
		const u32 t = vm::read32(0x00D65ABC);
		return t && vm::check_addr(t + 0x24014, vm::page_readable | vm::page_writable, 0xC0 * 0x48) ? t + 0x24014 : 0;
	}

	void sound_runtime_before_load()
	{
		g_voice_rt_taken = false;
		u32 core = 0, n = 0;

		if (!(sound_keep_runtime() || sound_keep_params()) || !core_voices(core, n))
		{
			return;
		}

		g_voice_rt.resize(n);

		for (u32 i = 0; i < n; i++)
		{
			const u32 v = core + i * 0x2f0;
			const u8* ptr = vm::get_super_ptr<u8>(v);
			voice_runtime_t& e = g_voice_rt[i];
			e.alloc = vm::read8(v + 0x1a0);
			e.gen = vm::read32(v + 0x1a4);
			e.req = vm::read32(v + 0x1a8);
			e.valid = vm::read32(v + 0x1ac);
			std::memcpy(e.data.data(), ptr, 0x150);

			if (sound_keep_params())
			{
				std::memcpy(e.param_block.data(), ptr + 0x160, 0x40);
				e.rate = vm::read32(v + 0x1bc);
				e.pause = vm::read32(v + 0x1cc);
				std::memcpy(e.sends.data(), ptr + 0x1d0, 0x100);
				e.pitch_ratio = vm::read32(v + 0x2d0);
			}
		}

		g_nu_voice.clear();

		if (const u32 states = sound_keep_params() ? nu_voice_states() : 0)
		{
			g_nu_voice.resize(0xC0);

			for (u32 i = 0; i < 0xC0; i++)
			{
				const u32 s = states + i * 0x48;
				const u8* ptr = vm::get_super_ptr<u8>(s);
				nu_voice_params_t& e = g_nu_voice[i];
				e.core_idx = vm::read32(s + 4);
				e.core_gen = vm::read32(s + 8);
				e.state = vm::read32(s + 0x20);
				std::memcpy(e.vol_pitch.data(), ptr + 0x28, 8);
				std::memcpy(e.sends.data(), ptr + 0x38, 0x10);
			}
		}

		g_voice_rt_taken = true;
	}

	void sound_runtime_after_load()
	{
		if (!g_voice_rt_taken)
		{
			return;
		}

		g_voice_rt_taken = false;
		u32 core = 0, n = 0;

		if (!core_voices(core, n) || n != g_voice_rt.size())
		{
			return;
		}

		u32 kept = 0, changed = 0, nu_kept = 0;
		const bool params = sound_keep_params();

		for (u32 i = 0; i < n; i++)
		{
			const u32 v = core + i * 0x2f0;
			const voice_runtime_t& e = g_voice_rt[i];

			if (!e.alloc || !e.valid)
			{
				continue;
			}

			if (vm::read8(v + 0x1a0) && vm::read32(v + 0x1a4) == e.gen && vm::read32(v + 0x1ac) == e.valid && vm::read32(v + 0x1a8) == e.req)
			{
				u8* ptr = vm::get_super_ptr<u8>(v);
				std::memcpy(ptr, e.data.data(), 0x150);

				if (params)
				{
					std::memcpy(ptr + 0x160, e.param_block.data(), 0x40);
					vm::write32(v + 0x1bc, e.rate);
					vm::write32(v + 0x1cc, e.pause);
					std::memcpy(ptr + 0x1d0, e.sends.data(), 0x100);
					vm::write32(v + 0x2d0, e.pitch_ratio);
				}

				kept++;
			}
			else
			{
				changed++;
			}
		}

		// nuSound2 voice states bound to the same core voice (index and generation) and active before and after
		if (const u32 states = params && g_nu_voice.size() == 0xC0 ? nu_voice_states() : 0)
		{
			for (u32 i = 0; i < 0xC0; i++)
			{
				const u32 s = states + i * 0x48;
				const nu_voice_params_t& e = g_nu_voice[i];

				if (e.state == 1 && e.core_idx != umax && vm::read32(s + 0x20) == 1 && vm::read32(s + 4) == e.core_idx && vm::read32(s + 8) == e.core_gen)
				{
					u8* ptr = vm::get_super_ptr<u8>(s);
					std::memcpy(ptr + 0x28, e.vol_pitch.data(), 8);
					std::memcpy(ptr + 0x38, e.sends.data(), 0x10);
					nu_kept++;
				}
			}
		}

		if (static u64 loads = 0, total_kept = 0, total_changed = 0; total_kept += kept, total_changed += changed, ++loads % 100 == 1)
		{
			rb_online_log.notice("Sound runtime kept across loads%s: this load %u voices kept (%u nuSound2 states), %u not (changed in the discarded frames); %u loads, %u kept, %u not in total",
				params ? " with parameters" : "", kept, nu_kept, changed, loads, total_kept, total_changed);
		}
	}

	// Once per main loop frame: game mode changes (with the phase counter *(*0x00967cf4 + 0x24)) and, in the KO replay
	// (mode 6, ends at counter 600 = 480 + 120 frames or when skipped), its progress every 60 frames
	static void game_mode_frame()
	{
		const u32 base = vm::read32(0x00967cf4);

		if (!base)
		{
			return;
		}

		const u32 mode = vm::read32(base + 0xc);
		const u32 counter = vm::read32(base + 0x24);

		if (static u32 last = 0xfffffffe; mode != last)
		{
			// Frames that waited for the remote input during the mode that ends (lockstep scenes: should stay near 0)
			static u64 waits_at = 0;
			rb_online_log.notice("Game mode %u -> %u (counter %u, rollbacks %u, re-simulating %d, remote input waits %u)", last, mode, counter, g_rollbacks,
				is_resimulating(), g_remote_waits - waits_at);
			waits_at = g_remote_waits;
			last = mode;
		}

		if (static u64 frames = 0; mode == 6 && !is_resimulating() && ++frames % 60 == 0)
		{
			rb_online_log.notice("Replay: counter %u / 600, rollbacks %u", counter, g_rollbacks);
		}
	}

	// Once per main loop frame: after a re-simulation, the unclaimed sounds of the discarded frames are stopped
	void sound_frame()
	{
		if (!sound_dedup())
		{
			return;
		}

		u32 seq[2]{};
		get_seq(seq);
		seq[0] = sound_frame_no();
		const bool resim = sound_resim();

		if (resim)
		{
			g_sound_resim_first = std::min<u64>(g_sound_resim_first, seq[0]);
		}
		else if (g_sound_was_resim)
		{
			u32 stopped = 0;

			for (auto it = g_sound_plays.begin(); it != g_sound_plays.end();)
			{
				// A sound whose game object still holds its handle after the re-simulation is alive in the corrected timeline:
				// the object saw it playing (the sound library is not rolled back) and did not start it again. Stopping it made
				// the next live frame start it once more: cut + restart of most hit sounds at every rollback = the distortion
				// (offline replay with identical inputs: 78 stopped, 65 started again). RB_SOUND_KEEP_HELD=0: old behaviour
				static const bool keep_held = !std::getenv("RB_SOUND_KEEP_HELD") || *std::getenv("RB_SOUND_KEEP_HELD") != '0';
				bool held = keep_held && it->out && vm::check_addr(it->out, vm::page_readable) && vm::read32(it->out) == it->handle;

				// Two sounds started into the same object field in the same frame: the second handle overwrote the first, and
				// the game never stopped the first one. While the object holds the second, the first stays too
				if (keep_held && !held && it->out && vm::check_addr(it->out, vm::page_readable))
				{
					const u32 now = vm::read32(it->out);

					for (const auto& q : g_sound_plays)
					{
						if (&q != &*it && q.out == it->out && q.handle == now && q.frame == it->frame)
						{
							held = true;
							break;
						}
					}
				}

				if (!sound_simple() && !sound_separate() && it->frame >= g_sound_resim_first && it->frame < seq[0] && !it->claimed && !it->cosmetic && !held)
				{
					auto& ppu = *get_current_cpu_thread<ppu_thread>();
					const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
					ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
					vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
					ppu.gpr[3] = it->handle;
					ppu.fast_call(0x0068dd38, 0x0096A8B0); // KeyOff(handle, 1)
					ppu.gpr[1] = old_r1;
					if (static u32 logged = 0; logged++ < 30)
					{
						rb_online_log.notice("Sound of a discarded frame stopped: frame %u, bank %u, cue %u, handle %u", it->frame, it->bank, it->cue, it->handle);
					}

					it = g_sound_plays.erase(it);
					stopped++;
					g_snd_stopped++;
				}
				else
				{
					it->claimed = false;
					++it;
				}
			}

			if (static u64 total = 0; stopped && (total += stopped) % 50 < stopped)
			{
				rb_online_log.notice("Sounds of discarded frames stopped: %u", total);
			}

			g_sound_resim_first = umax;
		}

		g_sound_was_resim = resim;

		if (static u64 frames = 0; !resim && ++frames % 600 == 0)
		{
			// Active voices: the library's voice table *(S+0x1c3c), 0xC0 entries of 0x168 bytes, each starting with its handle
			u32 active = 0;
			const u32 mgr_ptr = vm::read32(0x0097078c);
			const u32 snd = mgr_ptr ? static_cast<u32>(vm::read32(mgr_ptr)) : 0u;
			const u32 voices = snd && vm::check_addr(snd + 0x1c3c, vm::page_readable) ? static_cast<u32>(vm::read32(snd + 0x1c3c)) : 0u;

			if (voices && vm::check_addr(voices, vm::page_readable, 0xC0 * 0x168))
			{
				for (u32 v = 0; v < 0xC0; v++)
				{
					active += vm::read32(voices + v * 0x168) != 0;
				}
			}

			std::string codes;

			for (const auto& [c, n] : g_snd_fail_codes)
			{
				fmt::append(codes, " 0x%08x:%u", static_cast<u32>(c), n);
			}

			rb_online_log.notice("Sound: %u started, %u failed (%s), %u kept from the first pass, %u discarded stopped, %u of 192 voices active, core updates %u",
				g_snd_ok, g_snd_fail, codes, g_snd_claimed, g_snd_stopped, active, +vm::read32(0x00D66D60 + 0x90));

			// The sound core update thread stopped (its counter did not move for 600 frames): where it waits, the SPU
			// threads, and the mixer queues (first 0x80 bytes of *(0x00D66D60 + 0x3c / 0x44))
			if (static u32 last_updates = 0, dumps = 0; vm::read32(0x00D66D60 + 0x90) == last_updates && last_updates && dumps < 3)
			{
				dumps++;
				std::string out;

				idm::select<named_thread<ppu_thread>>([&](u32 id, named_thread<ppu_thread>& t)
				{
					const auto name = t.ppu_tname.load();

					if (name && (name->starts_with("bnus") || name->starts_with("CnuSound") || name->starts_with("HLE") || name->find("Spurs") != umax))
					{
						fmt::append(out, "\n  PPU 0x%x %s %s cia=0x%08x lr=0x%08x r3=0x%x%s", id, *name, (t.state & cpu_flag::wait) ? "WAIT" : "RUN ",
							t.cia, static_cast<u32>(t.lr), t.gpr[3], name->starts_with("bnusCoreUpdate") ? "\n" + t.dump_callstack() : std::string());
					}
				});

				idm::select<named_thread<spu_thread>>([&](u32 id, named_thread<spu_thread>& t)
				{
					fmt::append(out, "\n  SPU 0x%x %s pc=0x%05x state=0x%x", id, t.get_name(), t.pc, +t.state.load());
				});

				for (const u32 off : {0x3cu, 0x44u})
				{
					const u32 q = vm::read32(0x00D66D60 + off);
					fmt::append(out, "\n  queue +0x%x 0x%08x:", off, q);

					for (u32 o = 0; q && o < 0x80; o += 4)
					{
						fmt::append(out, " %08x", +vm::read32(q + o));
					}
				}

				rb_online_log.error("Sound core update thread stopped at %u updates:%s", last_updates, out);
			}
			else
			{
				last_updates = vm::read32(0x00D66D60 + 0x90);
			}

			// Diagnostics of voices that are not released: state (+0x20) histogram, the oldest voices' first 0x40 bytes
			if (active >= 24)
			{
				std::map<u32, u32> states;
				std::vector<std::pair<u32, u32>> by_handle;

				for (u32 v = 0; v < 0xC0; v++)
				{
					if (const u32 h = vm::read32(voices + v * 0x168))
					{
						states[vm::read32(voices + v * 0x168 + 0x20)]++;
						by_handle.emplace_back(h, voices + v * 0x168);
					}
				}

				std::sort(by_handle.begin(), by_handle.end());
				std::string hist, dump;

				for (const auto& [st, n] : states)
				{
					fmt::append(hist, " %u:%u", st, n);
				}

				for (usz i = 0; i < by_handle.size() && i < 3; i++)
				{
					fmt::append(dump, "\n  voice 0x%08x:", by_handle[i].second);

					for (u32 o = 0; o < 0x40; o += 4)
					{
						fmt::append(dump, " %08x", +vm::read32(by_handle[i].second + o));
					}
				}

				rb_online_log.notice("Sound voices: handle counter %u, states (+0x20)%s, oldest:%s", +vm::read32(snd + 4), hist, dump);
			}
		}

		// Leak cleanup (live frames, every 30): a sound playing for 4 s or more whose game object was rolled back to "no sound"
		// (handle -1) is a voice nobody can stop any more: stopped (not when the field was reused for something else). Short sounds end on their own before that.
		// Only sounds covered by a load (p.rolled): a -1 the game wrote itself (sound done, field reset or reused) is not a leak;
		// stopping those (~500 per fight, 70 % of all starts) cut long sounds such as the stage music.
		if (!resim && !sound_separate() && seq[0] % 30 == 0)
		{
			u32 cleaned = 0;

			// Is the voice of this handle still in the library's voice table (*(S+0x1c3c), 0xC0 x 0x168, handle at +0)?
			const auto voice_active = [](u32 handle)
			{
				const u32 mgr_ptr = vm::read32(0x0097078c);
				const u32 snd = mgr_ptr ? static_cast<u32>(vm::read32(mgr_ptr)) : 0u;
				const u32 voices = snd && vm::check_addr(snd + 0x1c3c, vm::page_readable) ? static_cast<u32>(vm::read32(snd + 0x1c3c)) : 0u;

				if (!voices || !vm::check_addr(voices, vm::page_readable, 0xC0 * 0x168))
				{
					return false;
				}

				for (u32 v = 0; v < 0xC0; v++)
				{
					if (vm::read32(voices + v * 0x168) == handle)
					{
						return true;
					}
				}

				return false;
			};

			for (auto& p : g_sound_plays)
			{
				if (!p.collected && p.frame + 240 <= seq[0] && p.out && vm::check_addr(p.out, vm::page_readable) && vm::read32(p.out) == 0xffffffffu)
				{
					if (!p.rolled)
					{
						// Not a leak; logged once per bank/cue while still playing (the old cleanup stopped these)
						if (static std::set<u64> seen; seen.size() < 30 && voice_active(p.handle) && seen.insert(u64{p.bank} << 32 | p.cue).second)
						{
							rb_online_log.notice("Sound kept playing without an owner handle (not rolled back): bank %u, cue %u, handle %u, started at frame %u",
								p.bank, p.cue, p.handle, p.frame);
						}

						p.collected = true;
						continue;
					}

					if (static u32 logged = 0; logged < 30 && voice_active(p.handle))
					{
						logged++;
						rb_online_log.notice("Sound without an owner stopped while playing: bank %u, cue %u, handle %u, started at frame %u",
							p.bank, p.cue, p.handle, p.frame);
					}

					p.collected = true;
					auto& ppu = *get_current_cpu_thread<ppu_thread>();
					const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
					ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
					vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
					ppu.gpr[3] = p.handle;
					ppu.fast_call(0x0068dd38, 0x0096A8B0); // KeyOff(handle, 1); nothing happens for a voice that ended
					ppu.gpr[1] = old_r1;
					cleaned++;
				}
			}

			if (static u64 total = 0; cleaned && (total += cleaned) % 100 < cleaned)
			{
				rb_online_log.notice("Sounds without an owner stopped: %u", total);
			}
		}

		while (!g_sound_plays.empty() && (g_sound_plays.front().frame + 1200 < seq[0] || g_sound_plays.front().frame > seq[0] + 1200))
		{
			g_sound_plays.pop_front();
		}
	}

	// Copy of FUN_0069effc(bm, bank, out) (sound bank lookup, ANALYZA.md section 15): during a re-simulation it fails for
	// the sound start path (FUN_00694ba8, return address 0x00694c70), so sounds of re-simulated frames are not started a
	// second time (PLAY fails cleanly: the game's sound object keeps handle -1, running voices are untouched). The sound
	// manager is not part of the snapshot, so this does not change the restored game state.
	u32 hle_sound_bank_lookup(ppu_thread& ppu, u32 bm, u32 bank, u32 out)
	{
		if (ppu.lr == 0x00694c70 && sound_resim() && !sound_dedup())
		{
			static u64 muted = 0;

			if (muted++ % 500 == 0)
			{
				rb_online_log.notice("Sound starts muted during re-simulation: %u", muted);
			}

			return 0x80000020;
		}

		if (!out)
		{
			return 0x80000024;
		}

		u32 r = 0;
		side_heap::with_frame(ppu, [&](u32) { r = static_cast<u32>(side_heap::call(ppu, 0x0069eeb4, {bm, bank})); });

		if (r)
		{
			return r;
		}

		const u32 bank_ptr = vm::read32(bm + bank * 0x2008 + 0x108);

		if (!bank_ptr)
		{
			return 0x80000020;
		}

		vm::write32(out, bank_ptr);
		return 0;
	}

	// Copy of FUN_0042371c(msg, stream): deserializer of the game's input message (byte, count, u32, mask byte, then
	// count entries of 6 words; flag of entry i at msg+0x5c+i*0x4c, words at msg+0x14+i*0x4c). The message object is a
	// stack buffer of the caller (FUN_00422478, 0x1d4 bytes = at most 6 entries); a damaged message with a larger count
	// overwrote the main thread stack (o81: access violation at 0x0042377c after ~1800 rollbacks). Larger counts are
	// dropped (count 0 = empty message; the inputs are sent redundantly in a window of 6 frames).
	u32 hle_input_message_read(ppu_thread& ppu, u32 msg, u32 stream)
	{
		constexpr u32 read_u8 = 0x00405fd0, read_u32 = 0x00405ef8, max_entries = 6;
		u32 head[4]{};

		side_heap::with_frame(ppu, [&](u32)
		{
			head[0] = static_cast<u32>(side_heap::call(ppu, read_u8, {stream})) & 0xff;
			head[1] = static_cast<u32>(side_heap::call(ppu, read_u8, {stream})) & 0xff;
			head[2] = static_cast<u32>(side_heap::call(ppu, read_u32, {stream}));
			head[3] = static_cast<u32>(side_heap::call(ppu, read_u8, {stream})) & 0xff;
		});

		u32 count = head[1];

		if (count > max_entries)
		{
			static u64 dropped = 0;
			rb_online_log.error("Damaged input message dropped (count %u, #%u)", count, ++dropped);
			count = 0;
		}

		vm::write8(msg + 0xc, static_cast<u8>(head[0]));
		vm::write8(msg + 0xd, static_cast<u8>(count));
		vm::write32(msg + 0x10, head[2]);

		// The highest remote frame really received (hdr = the message's last frame); a new round (round byte bit 0) starts again
		if (static u8 last_round = 0xff; count && (head[0] & 1) != last_round)
		{
			last_round = head[0] & 1;
			g_max_received = head[2];
		}
		else if (count)
		{
			g_max_received = std::max<u32>(g_max_received, head[2]);
		}

		for (u32 i = 0; i < count; i++)
		{
			vm::write32(msg + 0x5c + i * 0x4c, (head[3] >> i) & 1);
		}

		for (u32 i = 0; i < count; i++)
		{
			for (u32 k = 0; k < 6; k++)
			{
				u32 v = 0;
				side_heap::with_frame(ppu, [&](u32) { v = static_cast<u32>(side_heap::call(ppu, read_u32, {stream})); });
				vm::write32(msg + 0x14 + i * 0x4c + k * 4, v);
			}
		}

		return 1;
	}

	// Copy of FUN_005bb268 (ShaderContext: set a parameter by name; walks ctx+0x14, next = node+4, FUN_005bd918(node, name,
	// value) < 0 = not this one) with cycle protection: after a rollback the list can be cyclic (o105: the main thread looped
	// forever, ANALYZA.md 43). A node seen again ends the list there (graphics only).
	u32 shader_list_set(ppu_thread& ppu, u32 ctx, u32 list, u32 name, u32 value)
	{
		std::vector<u32> seen;
		u32 prev = 0;

		for (u32 node = vm::read32(ctx + list); node; node = vm::read32(node + 4))
		{
			if (seen.size() >= 4096 || std::find(seen.begin(), seen.end(), node) != seen.end() || !vm::check_addr(node, vm::page_readable))
			{
				if (static u32 logged = 0; logged++ < 20)
				{
					rb_online_log.error("Shader parameter list of context 0x%08x is cyclic or broken at node 0x%08x (%u nodes): cut", ctx, node, seen.size());
				}

				vm::write32(prev ? prev + 4 : ctx + list, 0);
				return 0x80004005;
			}

			seen.push_back(node);
			s32 r = 0;
			side_heap::with_frame(ppu, [&](u32) { r = static_cast<s32>(side_heap::call(ppu, 0x005bd918, {node, name, value})); });

			if (r >= 0)
			{
				return 0;
			}

			prev = node;
		}

		return 0x80004005;
	}

	u32 hle_shader_param_set(ppu_thread& ppu, u32 ctx, u32 name, u32 value)
	{
		return shader_list_set(ppu, ctx, 0x14, name, value);
	}

	// FUN_005bb0e8: the same for the second list (ctx+0x18)
	u32 hle_shader_param_set_b(ppu_thread& ppu, u32 ctx, u32 name, u32 value)
	{
		return shader_list_set(ppu, ctx, 0x18, name, value);
	}

	const u32 s_shader_param_set_index = REG_HIDDEN_FUNC(hle_shader_param_set);
	const u32 s_shader_param_set_b_index = REG_HIDDEN_FUNC(hle_shader_param_set_b);
	const u32 s_input_message_read_index = REG_HIDDEN_FUNC(hle_input_message_read);
	const u32 s_sound_bank_lookup_index = REG_HIDDEN_FUNC(hle_sound_bank_lookup);
	const u32 s_heap_alloc_handle1_index = REG_HIDDEN_FUNC(hle_heap_alloc_handle1);
	const u32 s_heap_alloc_handle_index = REG_HIDDEN_FUNC(hle_heap_alloc_handle);
	const u32 s_heap_alloc_raw_index = REG_HIDDEN_FUNC(hle_heap_alloc_raw);
	const u32 s_heap_free_raw_index = REG_HIDDEN_FUNC(hle_heap_free_raw);
	const u32 s_heap_free_handle_index = REG_HIDDEN_FUNC(hle_heap_free_handle);
	const u32 s_heap_destruct_index = REG_HIDDEN_FUNC(hle_heap_destruct);
	const u32 s_heap_destruct_delete_index = REG_HIDDEN_FUNC(hle_heap_destruct_delete);
	const u32 s_heap_assign_index = REG_HIDDEN_FUNC(hle_heap_assign);

	const u32 s_voice_stop_index = REG_HIDDEN_FUNC(hle_voice_stop_wait);
	const u32 s_sound_play_index = REG_HIDDEN_FUNC(hle_sound_play);
	const u32 s_spurs_job_wait_index = REG_HIDDEN_FUNC(hle_spurs_job_wait);
	const u32 s_spurs_job_teardown_index = REG_HIDDEN_FUNC(hle_spurs_job_teardown);
	const u32 s_at3p_decoder_destroy_index = REG_HIDDEN_FUNC(hle_at3p_decoder_destroy);
	const u32 s_get_input_index = REG_HIDDEN_FUNC(hle_get_input);
	const u32 s_available_index = REG_HIDDEN_FUNC(hle_input_available);
	const u32 s_store_input_index = REG_HIDDEN_FUNC(hle_store_input);

	void install_hooks(std::vector<u32>& applied)
	{
		if (side_heap::enabled())
		{
			// The region is reserved at load time (vm::alloc may not be called from a PPU thread inside an HLE function)
			side_heap::g_base = vm::alloc(side_heap::region_size, vm::main, 0x100000);

			if (side_heap::g_base)
			{
				side_heap::g_top = side_heap::g_base;
				state_exclude(side_heap::g_base, side_heap::g_base + side_heap::region_size, "side heap (network and sound thread allocations)");
				rb_online_log.success("Side heap at 0x%08x (%u MiB)", side_heap::g_base, side_heap::region_size >> 20);
			}
			else
			{
				rb_online_log.error("Side heap: no guest memory for the region");
				side_heap::g_base = 1;
			}

			for (const auto& [addr, index] : {std::pair{0x0013d1dcu, s_heap_alloc_handle1_index}, std::pair{0x0013d224u, s_heap_alloc_handle_index},
				std::pair{0x0013d25cu, s_heap_alloc_raw_index}, std::pair{0x001399b4u, s_heap_free_raw_index},
				std::pair{0x00139748u, s_heap_free_handle_index}, std::pair{0x00139888u, s_heap_destruct_index},
				std::pair{0x001397a4u, s_heap_destruct_index}, std::pair{0x0013975cu, s_heap_destruct_delete_index},
				std::pair{0x001398a4u, s_heap_assign_index}})
			{
				const u32 target = g_fxo->get<ppu_function_manager>().func_addr(index, true);

				if (target && ppu_form_branch_to_code(addr, target))
				{
					applied.push_back(addr);
					rb_online_log.success("Side heap hook 0x%08x -> HLE 0x%08x", addr, target);
				}
			}
		}

		if (!config().predict)
		{
			if (!stress_sound())
			{
				return;
			}

			// RB_STRESS_SOUND: only the sound hooks (no online input / SPURS job hooks)
			for (const auto& [addr, index] : {std::pair{0x0069afa0u, s_voice_stop_index}, std::pair{0x0069effcu, s_sound_bank_lookup_index},
				std::pair{0x0068e108u, s_sound_play_index}, std::pair{0x006b51dcu, s_at3p_decoder_destroy_index}})
			{
				const u32 target = g_fxo->get<ppu_function_manager>().func_addr(index, true);

				if (target && ppu_form_branch_to_code(addr, target))
				{
					applied.push_back(addr);
					rb_online_log.success("Stress sound hook 0x%08x -> HLE 0x%08x", addr, target);
				}
			}

			return;
		}

		for (const auto& [addr, index] : {std::pair{0x004073f4u, s_get_input_index}, std::pair{0x00407fd4u, s_store_input_index},
			std::pair{0x00407054u, s_available_index}, std::pair{0x0069afa0u, s_voice_stop_index}, std::pair{0x0069effcu, s_sound_bank_lookup_index}, std::pair{0x0068e108u, s_sound_play_index}, std::pair{0x0021bb94u, s_spurs_job_wait_index},
			std::pair{0x0021bce8u, s_spurs_job_teardown_index}, std::pair{0x006b51dcu, s_at3p_decoder_destroy_index}, std::pair{0x0042371cu, s_input_message_read_index}, std::pair{0x005bb268u, s_shader_param_set_index}, std::pair{0x005bb0e8u, s_shader_param_set_b_index}})
		{
			const u32 target = g_fxo->get<ppu_function_manager>().func_addr(index, true);

			if (target && ppu_form_branch_to_code(addr, target))
			{
				applied.push_back(addr);
				rb_online_log.success("Online prediction hook 0x%08x -> HLE 0x%08x", addr, target);
			}
		}
	}

	bool resimulating = false;
	u64 g_resim_until = 0; // game frame where the re-simulation reaches the frame of the rollback again

	bool fight_active()
	{
		return config().predict && in_fight() && ring_ready();
	}

	bool mute_stream(u64 opened_ns)
	{
		// With RB_SOUND_SEPARATE the music stream is not rolled back: not muted by default
		static const bool on = std::getenv("RB_MUTE_STAGE_MUSIC") ? *std::getenv("RB_MUTE_STAGE_MUSIC") != '0' : !sound_separate();
		const u64 start = g_fight_start_ns;
		return on && config().predict && start && opened_ns < start;
	}

	bool is_resimulating()
	{
		if (use_stress_ring() || stress_sound())
		{
			u32 seq[2]{};
			get_seq(seq);
			return state_resimulating(seq[0]);
		}

		return resimulating;
	}

	// SPURS jobs launched in one frame are waited for in the next one (FUN_0021bb94: job pending flag +0x304, event flag
	// at +0x200 of the job object, +0xc bit 0: really kicked). A snapshot taken while a job runs restores "pending, flag not
	// set" after the job already finished (the game then waited forever) and can hold half written job results. Before
	// saving, the job's completion is awaited passively: its event bit (CellSpursEventFlag.events, u16 at +0x200) is
	// polled with sys_timer_usleep (a real PPU sleep: busy waiting kept the SPURS handler thread from running). The game's
	// own wait consumes the bit later as usual. If a job does not finish within 20 ms the boundary is not saved.
	// (Calling the game's wait function here interfered with the game.) Job objects (callers of FUN_0021bb94):
	// *0x009680ac + 0 / 0x380 / 0x700, *0x009682ec + 0x80, *0x00968324 + 0x4e80.
	// max_wait_us: a re-simulated boundary waits at most 1 ms (20 ms per boundary made 0.5-0.9 s spikes in o121)
	static bool spurs_job_pending(ppu_thread& ppu, u32 max_wait_us = 20000)
	{
		for (const auto& [ptr, offset] : {std::pair{0x009680acu, 0u}, std::pair{0x009680acu, 0x380u}, std::pair{0x009680acu, 0x700u},
			std::pair{0x009682ecu, 0x80u}, std::pair{0x00968324u, 0x4e80u}})
		{
			const u32 base = vm::read32(ptr);
			const u32 obj = base + offset;

			// A job that did not finish twice in a row is not waited for again until it finishes (the user's KO: every
			// boundary waited 20 ms for a job that never ran, frames took 22-24 ms and no snapshot was saved)
			static std::map<u32, u32> stuck;

			if (base && vm::check_addr(obj, vm::page_readable) && vm::check_addr(obj + 0x308, vm::page_readable) &&
				vm::read32(obj + 0x304) && vm::read32(obj + 0x308) && vm::read32(obj + 0x14))
			{
				if (vm::read16(obj + 0x200) != 0)
				{
					stuck[obj] = 0;
					continue;
				}

				if (stuck[obj] >= 2)
				{
					continue;
				}

				const auto t0 = std::chrono::steady_clock::now();

				while (vm::read16(obj + 0x200) == 0)
				{
					if (ppu.is_stopped() || std::chrono::steady_clock::now() - t0 > std::chrono::microseconds(max_wait_us))
					{
						if (max_wait_us >= 20000 && ++stuck[obj] == 2)
						{
							rb_online_log.warning("SPURS job 0x%08x did not finish twice, not waited for any more until it does", obj);
						}

						return true;
					}

					sys_timer_usleep(ppu, 50);
				}

				stuck[obj] = 0;
			}
			else
			{
				stuck[obj] = 0;
			}
		}

		return false;
	}

	// Bytes of the game's network layer on pages shared with game state (the network threads write the receive queues,
	// sequence numbers, etc. on the same heap pages as e.g. the input manager): learnt while the main thread saves a
	// snapshot (it does not run game code then, so every byte that changes meanwhile was written by another thread), and
	// kept at their current value when a snapshot is loaded. Pages: RB_SNAP_FOREIGN=2 (first written by net_* threads).
	struct net_page
	{
		std::array<u8, 4096> mask{};
		std::array<u8, 4096> buf{};
		u32 bytes = 0;
	};

	static std::map<u32, net_page> g_net_pages;

	static void net_pages_before_save()
	{
		for (const u32 addr : state_foreign_pages())
		{
			g_net_pages.try_emplace(addr);
		}

		for (auto& [addr, p] : g_net_pages)
		{
			if (vm::check_addr(addr, vm::page_readable))
			{
				std::memcpy(p.buf.data(), vm::get_super_ptr<u8>(addr), 4096);
			}
		}
	}

	static void net_pages_after_save()
	{
		for (auto& [addr, p] : g_net_pages)
		{
			if (!vm::check_addr(addr, vm::page_readable))
			{
				continue;
			}

			const u8* cur = vm::get_super_ptr<u8>(addr);

			for (u32 i = 0; i < 4096; i++)
			{
				if (cur[i] != p.buf[i] && !p.mask[i])
				{
					p.mask[i] = 1;
					p.bytes++;
				}
			}
		}
	}

	// Longest re-simulation in game frames (RB_ONLINE_MAX_RESIM, default 6): longer ones crashed in the game's graphics code
	// (per frame rotating buffers wrapped around); the offline rollback was verified with up to 3
	static u64 max_resim()
	{
		static const u64 v = std::getenv("RB_ONLINE_MAX_RESIM") ? std::strtoull(std::getenv("RB_ONLINE_MAX_RESIM"), nullptr, 10) : 20;
		return v;
	}

	// Diagnostics (RB_ONLINE_TASK_CHECK=1): the game's task tree (M = *0x00A77C34: walk chain from M+4 via +0x10, 29 list
	// heads at M+0x70[i] with next +4, handle list of every node at +0x58 with next +0xc) - every node or handle that is
	// not restored by the snapshot (outside the saved pages or excluded) is logged once
	static void check_task_tree(const char* when)
	{
		static const bool on = std::getenv("RB_ONLINE_TASK_CHECK") != nullptr;

		if (!on)
		{
			return;
		}

		static std::set<u32> reported;
		const u32 m = vm::read32(0x00A77C34);

		const auto check = [&](u32 addr, const char* what, u32 owner)
		{
			if (!addr || !vm::check_addr(addr, vm::page_readable))
			{
				return false;
			}

			if (const int saved = state_addr_saved(addr); saved != 1 && reported.size() < 200 && reported.insert(addr).second)
			{
				rb_online_log.error("Task check (%s): %s 0x%08x (owner 0x%08x) is %s", when, what, addr, owner, saved == 2 ? "EXCLUDED" : "NOT SAVED");
			}

			return true;
		};

		if (!check(m, "task manager", 0))
		{
			return;
		}

		std::set<u32> nodes;

		for (u32 node = vm::read32(m + 4), n = 0; node && n < 5000 && check(node, "task node", m); node = vm::read32(node + 0x10), n++)
		{
			if (!nodes.insert(node).second)
			{
				break;
			}
		}

		for (u32 i = 0; i < 29; i++)
		{
			const u32 head = m + 0x70;
			const u32 first = vm::read32(vm::read32(head) + i * 4);

			for (u32 node = first, n = 0; node && n < 5000 && check(node, "listed node", head); node = vm::read32(node + 4), n++)
			{
				if (node == first && n)
				{
					break;
				}
			}
		}

		for (const u32 node : nodes)
		{
			for (u32 h = vm::read32(node + 0x58), n = 0; h && h != node + 0x58 && n < 1000 && check(h, "handle", node); h = vm::read32(h + 0xc), n++)
			{
			}
		}
	}

	// Saves the frame boundary only when the remote simulated frame changed since the last saved one
	static void save_if_advanced(ppu_thread& ppu, u32 mgr, u32 ri, u64 game_frame, const u32 (&seq)[2])
	{
		const u32 sim = vm::read32(mgr + 0x60 + ri * 4);

		// Diagnostics: RB_ONLINE_SAVE_EVERY=1 saves every frame boundary (like the offline stress test),
		// RB_ONLINE_SPURS_WAIT=0 saves without waiting for running SPURS jobs
		static const bool save_every = std::getenv("RB_ONLINE_SAVE_EVERY") && *std::getenv("RB_ONLINE_SAVE_EVERY") == '1';
		static const bool spurs_wait = !(std::getenv("RB_ONLINE_SPURS_WAIT") && *std::getenv("RB_ONLINE_SPURS_WAIT") == '0');

		if (!save_every && !g_sim_at.empty() && g_sim_at.rbegin()->second == sim)
		{
			return;
		}

		// The wait for the SPU job is limited (RB_ONLINE_SPURS_WAIT_US, default 20000; o123: 2000 skipped too many saves,
		// rollbacks got deeper)
		static const u32 wait_us = std::getenv("RB_ONLINE_SPURS_WAIT_US") ? std::atoi(std::getenv("RB_ONLINE_SPURS_WAIT_US")) : 20000;
		const auto wait_start = std::chrono::steady_clock::now();
		const bool pending = spurs_wait && spurs_job_pending(ppu, wait_us);
		{
			static u64 n = 0, total_us = 0, max_us = 0;
			const u64 us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - wait_start).count();
			total_us += us;
			max_us = std::max(max_us, us);

			if (++n % 600 == 0)
			{
				rb_online_log.notice("SPURS wait before saves: avg %u us, max %u us (600 live boundaries)", total_us / 600, max_us);
				total_us = max_us = 0;
			}
		}

		if (pending)
		{
			static u64 skipped = 0;

			if (++skipped % 100 == 1)
			{
				rb_online_log.notice("Frame boundary not saved: a SPURS job is running (%u so far)", skipped);
			}

			return;
		}

		g_sim_at[game_frame] = sim;

		while (g_sim_at.size() > 600)
		{
			g_sim_at.erase(g_sim_at.begin());
		}

		net_pages_before_save();
		state_net_save(game_frame, seq);
		net_pages_after_save();

		if (static u32 n = 0; ++n % 60 == 0)
		{
			check_task_tree("save");
		}
	}

	// Diagnostics RB_LWMUTEX_SAMPLE=<ptr>:<offset>: a host thread samples the owner thread id of the lwmutex at
	// *(ptr) + offset (e.g. the game's heap lock 0x00B02A70:0x28) for 60 s from the fight on and logs who holds it
	static void start_lwmutex_sampler()
	{
		static bool started = false;
		const char* v = std::getenv("RB_LWMUTEX_SAMPLE");

		if (started || !v || !*v)
		{
			return;
		}

		started = true;
		const auto parts = fmt::split(v, {":"});
		const u32 ptr = static_cast<u32>(std::strtoul(parts[0].c_str(), nullptr, 16));
		const u32 off = parts.size() > 1 ? static_cast<u32>(std::strtoul(parts[1].c_str(), nullptr, 16)) : 0;
		const u32 addr = vm::read32(ptr) + off;
		rb_online_log.notice("Sampling the owner of lwmutex 0x%08x", addr);

		std::thread([addr]()
		{
			std::map<u32, u64> counts;
			u64 samples = 0;
			const auto start = std::chrono::steady_clock::now();

			while (std::chrono::steady_clock::now() - start < std::chrono::seconds(60) && !Emu.IsStopped())
			{
				const u32 owner = *vm::get_super_ptr<be_t<u32>>(addr);
				samples++;

				if (owner != 0xffffffffu && owner != 0)
				{
					counts[owner]++;
				}

				std::this_thread::yield();

				if (static auto last = std::chrono::steady_clock::now(); std::chrono::steady_clock::now() - last > std::chrono::seconds(5))
				{
					last = std::chrono::steady_clock::now();
					std::string part;
					for (const auto& [id, c] : counts)
					{
						std::string name = "?";

						if (const auto t = idm::get_unlocked<named_thread<ppu_thread>>(id))
						{
							if (const auto n = t->ppu_tname.load())
							{
								name = *n;
							}
						}

						fmt::append(part, " 0x%x(%s)=%u", id, name, c);
					}
					rb_online_log.error("lwmutex 0x%08x owners so far (%u samples):%s", addr, samples, part);
				}
			}

			std::string out;

			for (const auto& [id, c] : counts)
			{
				std::string name = "?";

				if (const auto t = idm::get_unlocked<named_thread<ppu_thread>>(id))
				{
					if (const auto n = t->ppu_tname.load())
					{
						name = *n;
					}
				}

				fmt::append(out, " 0x%x(%s)=%u", id, name, c);
			}

			rb_online_log.error("lwmutex 0x%08x held in %u of %u samples:%s", addr, [&]() { u64 t = 0; for (auto& [i, c] : counts) t += c; return t; }(), samples, out);
		}).detach();
	}

	// Called at every main loop frame boundary (rb_frame_step), before the frame runs
	// RB_ONLINE_RESIM_DUMP=N (with RB_ONLINE_STRESS): the first N artificial rollbacks dump the memory at the frame of the
	// request and again when the re-simulation reaches it (same inputs: every difference is state the snapshot misses;
	// tools\missing_state.py). RB_ONLINE_RESIM_DUMP_PREFIX, default records\resim_dump
	static const u32 resim_dumps = std::getenv("RB_ONLINE_RESIM_DUMP") ? std::atoi(std::getenv("RB_ONLINE_RESIM_DUMP")) : 0;
	static const std::string resim_dump_prefix = std::getenv("RB_ONLINE_RESIM_DUMP_PREFIX") ? std::getenv("RB_ONLINE_RESIM_DUMP_PREFIX") :
		"records/resim_dump";
	static u32 resim_dumped = 0;
	static u64 resim_dump_frame = umax;

	// The rollback request for g_mismatch_frame (performed by rb_state at the next frame boundary)
	static void request_rollback(u32 mgr, u32 ri, u64 game_frame)
	{
		if (resim_dumped < resim_dumps && resim_dump_frame == umax && std::getenv("RB_ONLINE_STRESS") && g_predicted.empty())
		{
			resim_dumped++;
			resim_dump_frame = game_frame;
			dump_memory_now(resim_dump_prefix, game_frame);
			rb_online_log.success("Re-simulation dump at game frame %u (first pass)", game_frame);
		}

		const u32 bad = g_mismatch_frame;
		g_mismatch_frame = umax;
		u64 target = umax;

		for (auto it = g_sim_at.rbegin(); it != g_sim_at.rend(); ++it)
		{
			if (it->second <= bad && it->first <= game_frame)
			{
				target = it->first;
				break;
			}
		}

		// Depth statistics: detection delay (simulated frames past the mispredicted one) vs. snapshot distance
		if (target != umax)
		{
			static u64 n = 0, detect = 0, depth = 0, extra = 0;
			const u32 sim_now = vm::read32(mgr + 0x60 + ri * 4);
			const u64 d = game_frame + 1 - target;
			const u64 needed = sim_now > bad ? sim_now - bad : 0;
			n++, detect += needed, depth += d, extra += d > needed ? d - needed : 0;

			if (n % 300 == 0)
			{
				rb_online_log.notice("Rollback depth (300 rollbacks): avg %.2f game frames, misprediction found %.2f simulated frames late, %.2f frames before it (snapshot distance)",
					depth / 300.0, detect / 300.0, extra / 300.0);
				detect = depth = extra = 0;
			}
		}

		// The restore happens at the next boundary (game_frame + 1); the ring holds 8 frames
		if (target != umax && game_frame + 1 - target <= 120)
		{
			struct saved_local
			{
				u32 available = 0, sent = 0;
				std::vector<std::pair<u32, entry_t>> entries;
			};

			const auto local = std::make_shared<saved_local>();
			const u32 li = 1 - ri;

			// Before the restore: local inputs (not rolled back, already sent) and the network bytes (see net_page)
			auto before = [=]()
			{
				// The remote frame simulated before the restore: the re-simulation predicts freely up to it
				g_resim_sim_limit = vm::read32(mgr + 0x60 + ri * 4);

				local->available = vm::read32(mgr + 0x58 + li * 4);
				local->sent = vm::read32(mgr + 0x68 + li * 4);

				for (u32 f = local->available > 32 ? local->available - 32 : 0; f < local->available; f++)
				{
					if (vm::read8(flag_addr(mgr, li, f)) & 1)
					{
						local->entries.emplace_back(f, read_entry(slot_addr(mgr, li, f)));
					}
				}

				for (auto& [addr, page] : g_net_pages)
				{
					if (vm::check_addr(addr, vm::page_readable))
					{
						std::memcpy(page.buf.data(), vm::get_super_ptr<u8>(addr), 4096);
					}
				}
			};

			auto after = [=]()
			{
				for (auto& [addr, page] : g_net_pages)
				{
					if (page.bytes && vm::check_addr(addr, vm::page_readable))
					{
						u8* cur = vm::get_super_ptr<u8>(addr);

						for (u32 i = 0; i < 4096; i++)
						{
							if (page.mask[i])
							{
								cur[i] = page.buf[i];
							}
						}
					}
				}

				// Only the present bit: 0x2 (read) would let FUN_00406a04 advance the local sim without a read (ANALYZA.md 32)
				for (const auto& [f, e] : local->entries)
				{
					std::memcpy(vm::get_super_ptr<u8>(slot_addr(mgr, li, f)), e.data(), entry_size);
					vm::write8(flag_addr(mgr, li, f), vm::read8(flag_addr(mgr, li, f)) | 1);
				}

				// Nothing is sent during the re-simulation (every local input up to here was sent before the rollback; the
				// sender's window during the re-simulation put older inputs onto newer frames: o94)
				vm::write32(mgr + 0x68 + li * 4, vm::read32(mgr + 0x58 + li * 4));

				vm::write32(mgr + 0x58 + li * 4, std::max<u32>(vm::read32(mgr + 0x58 + li * 4), local->available));
				vm::write32(mgr + 0x68 + li * 4, std::max<u32>(vm::read32(mgr + 0x68 + li * 4), local->sent));
				reapply_remote(mgr, vm::read32(mgr + 0x60 + ri * 4));
				sound_after_load(target);
				g_rollbacks++;
				g_sim_at.erase(g_sim_at.upper_bound(target), g_sim_at.end());

				// A SPURS job pending in the restored state whose completion bit is not set: it finished in the
				// discarded timeline (or never runs again), the game's wait (FUN_0021bb94) would never end. The
				// pending flag is cleared; the job's results are graphics data of one frame.
				for (const auto& [ptr, offset] : {std::pair{0x009680acu, 0u}, std::pair{0x009680acu, 0x380u}, std::pair{0x009680acu, 0x700u},
					std::pair{0x009682ecu, 0x80u}, std::pair{0x00968324u, 0x4e80u}})
				{
					const u32 base = vm::read32(ptr);
					const u32 obj = base + offset;

					if (base && vm::check_addr(obj + 0x308, vm::page_readable) && vm::read32(obj + 0x304) && vm::read32(obj + 0x308) &&
						vm::read32(obj + 0x14) && vm::read16(obj + 0x200) == 0)
					{
						vm::write32(obj + 0x304, 0);
						g_restore_cancelled_jobs.insert(obj);

						if (static u32 logged = 0; logged++ < 500)
						{
							rb_online_log.warning("Restored state: SPURS job 0x%08x pending without completion, wait cancelled (snapshot %u, sim %u)", obj,
								target, +vm::read32(mgr + 0x60 + ri * 4));
						}
					}
				}
			};

			// The re-simulation is limited in simulated frames (a stall makes several game frames per simulated frame)
			state_request_rollback(target, std::move(before), std::move(after), 120, bad, bad > max_resim() ? bad - max_resim() : 0);
		}
		else
		{
			rb_online_log.error("Misprediction of remote frame %u cannot be rolled back (snapshot %u, game frame %u)", bad, target, game_frame);
		}
		}

	// Runs the game's receiver FUN_00422478(net) once (the session must be connected); its call counters (+0x88 statistics,
	// +0x64 disconnect timeout) are restored, so an extra call does not shorten the timeout. Returns whether it ran
	static bool run_receiver(ppu_thread& ppu)
	{
		const u32 net = net_manager_object();

		if (!net || vm::read32(net + 0x5c) == 0)
		{
			return false;
		}

		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
		constexpr u64 toc = 0x0096A8B0;

		ppu.fast_call(0x00425f04, toc);
		const u32 session = static_cast<u32>(ppu.gpr[3]);
		bool ok = false;

		if (session)
		{
			ppu.gpr[3] = session;
			ppu.fast_call(0x004240c0, toc);

			if (static_cast<u32>(ppu.gpr[3]))
			{
				ppu.gpr[3] = session;
				ppu.fast_call(0x00424148, toc);
				const u32 state = static_cast<u32>(ppu.gpr[3]);
				ok = state && vm::read32(state + 0xc) == 1;
			}
		}

		if (ok)
		{
			const u32 c88 = vm::read32(net + 0x88), c64 = vm::read32(net + 0x64);
			ppu.gpr[3] = net;
			ppu.fast_call(0x00422478, toc);
			vm::write32(net + 0x88, c88);
			vm::write32(net + 0x64, c64);
		}

		ppu.gpr[1] = old_r1;

		return ok;
	}

	// RB_ONLINE_EARLY_RECV (default on, 0 = off): at the end of a live frame the game's receiver FUN_00422478(manager) runs
	// once more, so the remote inputs that arrived during the frame's vblank wait are stored now and a misprediction is
	// rolled back at the next boundary instead of one frame later (each rollback was one frame deeper than needed). The
	// receiver counts calls (+0x88 statistics, +0x64 = 100: disconnect timeout > 299, reset when both sides are ready):
	// both are restored, so the extra call does not shorten the timeout.
	void end_of_frame(ppu_thread& ppu)
	{
		static const bool on = !std::getenv("RB_ONLINE_EARLY_RECV") || *std::getenv("RB_ONLINE_EARLY_RECV") != '0';
		const u32 mgr = manager();

		if (!on || !config().predict || !mgr || !in_fight() || !use_stress_ring() || !state_stress_ready())
		{
			return;
		}

		const u32 net = net_manager_object();

		if (!net || vm::read32(net + 0x5c) == 0)
		{
			return;
		}

		const bool ok = run_receiver(ppu);

		if (ok && g_mismatch_frame != umax)
		{
			u32 seq[2]{};
			get_seq(seq);
			request_rollback(mgr, remote_player(mgr) - 1, seq[0]);

			if (static u64 n = 0; n++ == 0)
			{
				rb_online_log.success("Early receive: misprediction found at the end of the frame, rollback at the next boundary");
			}
		}
	}

	// Sound library, mixer and (optional) lower layers outside the snapshot; called every online frame and, with
	// RB_STRESS_SOUND, every frame of the offline stress (no online manager there)
	static void sound_exclusions()
	{
		// The sound manager (*(*0x0097078c), voice table 0xC0 x 0x48 at +0x24014) is not rolled back: sound is not part of the
		// fight, and restoring the game's voice table without the sound library (asynchronous threads) left voices that
		// never stop. A re-simulated frame can start a sound once more (a few frames of audio).
		if (static u32 sound_manager = 0; vm::read32(0x0097078c) && vm::read32(vm::read32(0x0097078c)) != sound_manager)
		{
			sound_manager = vm::read32(vm::read32(0x0097078c));

			if (vm::check_addr(sound_manager, vm::page_readable) && !sound_rollback_env())
			{
				// The whole nuSound2 library state stays out of the rollback (the user: sound must not be rolled back): the
				// manager S (handle counter +4, bank manager +0x1c38, voice table +0x1c3c, voices +0x24014: 0x28000 bytes), the
				// bank manager (bank infos at +0x8030c) and the voice table (0xC0 x 0x168). Restoring them while the sound
				// thread used them corrupted the bank infos: "KeyOn Failed" (0x80000000) for more and more sounds, the sound
				// went silent even in the lobby (r1 test).
				state_exclude(sound_manager, sound_manager + 0x28000, "sound library manager");
				const u32 banks = vm::read32(sound_manager + 0x1c38);
				const u32 voices = vm::read32(sound_manager + 0x1c3c);

				if (banks && vm::check_addr(banks, vm::page_readable))
				{
					state_exclude(banks, banks + 0x81000, "sound library bank manager");
				}

				if (voices && vm::check_addr(voices, vm::page_readable))
				{
					state_exclude(voices, voices + 0xC0 * 0x168, "sound library voice table");
				}

				rb_online_log.success("Sound library outside the rollback: manager 0x%08x, banks 0x%08x, voices 0x%08x", sound_manager, banks, voices);
			}
		}

		// The lower layers of the sound library, written by KeyOn on the main thread (so their pages were saved and restored)
		// and advanced by the sound threads: restored, a voice stayed "playing" forever (KeyOff waits timed out, voices were
		// never freed, 0x1f voices per cue -> "KeyOn Failed" 0x80000000, silence, the VS screen waited for the announcer: r14).
		// - nuSound2 voice states: T = *(0x00D65AB8 + 4), 0xC0 x 0x48 bytes at T + 0x24014 (FUN_0069e568), queue at T + 0x24010
		// - bnusCore (global 0x00D66D60, not in the snapshot): voices *(+0xc0), *(+0x80) x 0x2f0 bytes; list *(+0xd8), *(+0xd0) x 4
		// The sound mixer runs on an SPU: bnusCoreUpdateThread (FUN_006b3d2c, arg 0x00D66D60) pushes "STAT" to the SPURS
		// queue *(+0x3c) and waits on the queue *(+0x44) for "END ". Restoring the queues (CellSyncLFQueue, 0x80 bytes, buffer
		// *(+0x1c) of size +0x10 x depth +0x14) or their taskset (m_eaSignal +0x74, CellSpursTaskset 0x1900 bytes) from a
		// snapshot left the update thread waiting forever: all sound went silent shortly after the first rollbacks (s20).
		if (!sound_rollback_env())
		{
			static std::set<u32> done;
			constexpr u32 core_thread_arg = 0x00D66D60;

			// The queues (+0x3c push, +0x44 pop) and the other SPURS objects of the mixer (+0x30..+0x48) were allocated in one
			// block next to the core voice list (*(+0xd8), *(+0xd0) x 4), e.g. 0x373fedc0..0x373ffa8c (s22): the whole block
			// stays out of the snapshots (the layout of the LLE queue objects did not match CellSyncLFQueue, s21)
			u32 lo = umax, hi = 0;

			for (u32 off = 0x30; off <= 0x48; off += 4)
			{
				const u32 ptr = vm::read32(core_thread_arg + off);

				const u32 anchor = vm::read32(core_thread_arg + 0x44);

				if (ptr >= 0x10000000 && !(ptr & 0xf) && ptr + 0x4000 > anchor && ptr < anchor + 0x4000 && vm::check_addr(ptr, vm::page_readable))
				{
					lo = std::min(lo, ptr & ~0x7fu);
					hi = std::max(hi, ptr + 0x180);
				}
			}

			const u32 list = vm::read32(core_thread_arg + 0xd8);
			const u32 list_n = vm::read32(core_thread_arg + 0xd0);

			if (list && list_n <= 0x1000 && list >= lo && list < hi + 0x1000)
			{
				hi = std::max(hi, list + list_n * 4);
			}

			// The mixer's SPURS taskset (the queues' signal pointer, +0x64 in the LLE queue layout, e.g. 0x36fa4a80): restored,
			// the mixer task was never woken again (s24: all SPURS kernels idle, the update thread waiting for "END ")
			if (const u32 q = vm::read32(core_thread_arg + 0x44); q && vm::check_addr(q + 0x64, vm::page_readable))
			{
				const u32 taskset = vm::read32(q + 0x64);

				if (taskset && !(taskset & 0x7f) && vm::check_addr(taskset, vm::page_readable) && done.insert(taskset).second)
				{
					state_exclude(taskset, taskset + 0x1900, "sound mixer SPURS taskset");
					rb_online_log.success("Sound mixer SPURS taskset outside the rollback: 0x%08x (saved %d)", taskset, state_addr_saved(taskset));
				}
			}

			if (lo < hi && hi - lo <= 0x8000 && done.insert(lo).second)
			{
				state_exclude(lo, hi, "sound mixer SPURS objects");
				rb_online_log.success("Sound mixer SPURS objects outside the rollback: 0x%08x-0x%08x (saved %d)", lo, hi, state_addr_saved(vm::read32(core_thread_arg + 0x44)));
			}
		}

		// RB_SOUND_SEPARATE=1: the rest of the sound library outside the rollback as one unit (statics, group T, core voices,
		// side blocks, the libc mspace of the sound/network threads); see recomp_t6/fix_separate.py
		if (!sound_rollback_env() && sound_separate())
		{
			static std::set<u64> done;
			const auto exclude_once = [](u32 from, u32 to, const char* what)
			{
				if (from && to > from && vm::check_addr(from, vm::page_readable) && done.insert((u64{from} << 32) | to).second)
				{
					state_exclude(from, to, what);
					rb_online_log.success("Sound separate: %s 0x%08x-0x%08x outside the rollback", what, from, to);
				}
			};

			// Size of a heap block from its descriptor (raw blocks: descriptor at data - 4, desc+4 = data, desc+0xc = size)
			const auto block_end = [](u32 data, u32 fallback) -> u32
			{
				if (!vm::check_addr(data - 4, vm::page_readable))
				{
					return data + fallback;
				}

				const u32 desc = vm::read32(data - 4);

				if (desc && vm::check_addr(desc, vm::page_readable, 0x10) && vm::read32(desc + 4) == data)
				{
					const u32 size = vm::read32(desc + 0xc);

					if (size && size <= 0x100000)
					{
						return data + size;
					}
				}

				return data + fallback;
			};

			exclude_once(0x00D64EA0, 0x00D66000, "sound statics");
			// The libc mspace also holds the NP pool (sceNpInit, 0x20000 at 0x300005d6) and other blocks: only with
			// RB_SOUND_SEPARATE_MSPACE=1 (o233 crashed with it: SPURS kernel null read right after the first rollbacks)
			if (static const bool mspace = std::getenv("RB_SOUND_SEPARATE_MSPACE") && *std::getenv("RB_SOUND_SEPARATE_MSPACE") == '1'; mspace)
			{
				exclude_once(0x30000000, 0x30100000, "libc mspace (sound and network threads)");
			}

			if (const u32 t = vm::read32(0x00D65ABC))
			{
				exclude_once(t, t + 0x24014 + 0xC0 * 0x48, "sound group T (head, queue, voice states)");
			}

			constexpr u32 g = 0x00D66D60;
			const u32 core = vm::read32(g + 0xc0);
			const u32 n = vm::read32(g + 0x80);

			if (core && n && n <= 0x400)
			{
				exclude_once(core, core + n * 0x2f0, "sound core voices");
			}

			if (const u32 list = vm::read32(g + 0xd8), list_n = vm::read32(g + 0xd0); list && list_n && list_n <= 0x1000)
			{
				exclude_once(list, list + list_n * 4, "sound core voice list");
			}

			// Every heap block the bnusCore global points to (hy25: voices 0x23400, mixer work 0xc000 / 0x2000 / 0x280 / 0x200,
			// voice list 0x34c, ...): the SPU mixer and the update thread write them; restored next to live voices, the mixer
			// read a null decoder pointer (+0x1b4) and the SPURS kernel crashed (o234, o237)
			for (u32 off = 0; off < 0x200; off += 4)
			{
				const u32 v = vm::read32(g + off);

				if (v >= 0x10000000 && v < 0x40000000 && vm::check_addr(v - 4, vm::page_readable))
				{
					const u32 desc = vm::read32(v - 4);

					if (desc && vm::check_addr(desc, vm::page_readable, 0x10) && vm::read32(desc + 4) == v)
					{
						if (const u32 size = vm::read32(desc + 0xc); size && size <= 0x100000)
						{
							exclude_once(v, v + size, "sound core block (bnusCore global)");
						}
					}
				}
			}

			// Side blocks of the library (pointer scan hy24; fallback = the extent written in the page map)
			if (const u32 b = vm::read32(0x00D64C74); b >= 0x10000000)
			{
				exclude_once(b, block_end(b, 0x20), "sound block *(0x00D64C74)");
			}

			if (const u32 sp = vm::read32(0x0097078c); sp && vm::read32(sp))
			{
				const u32 s_mgr = vm::read32(sp);

				for (const u32 off : {0x1c40u, 0x1c44u, 0x1c48u})
				{
					if (const u32 b = vm::read32(s_mgr + off); b >= 0x10000000)
					{
						exclude_once(b, block_end(b, 0x80), "sound block *(S + 0x1c4x)");
					}
				}
			}
		}

		// RB_SOUND_EXCL_LOWER=1 (r16: with both excluded, sounds failed already in the fight; under investigation)
		if (!sound_rollback_env() && []{ static const bool v = std::getenv("RB_SOUND_EXCL_LOWER") && *std::getenv("RB_SOUND_EXCL_LOWER") == '1'; return v; }())
		{
			static u32 last_t = 0, last_core = 0;
			const u32 t = vm::read32(0x00D65ABC);
			constexpr u32 g = 0x00D66D60;
			const u32 core = vm::read32(g + 0xc0);

			if (t && t != last_t && vm::check_addr(t, vm::page_readable))
			{
				last_t = t;
				state_exclude(t, t + 0x24014 + 0xC0 * 0x48, "sound library voice states");
				rb_online_log.success("Sound voice states outside the rollback: 0x%08x", t);
			}

			if (core && core != last_core && vm::check_addr(core, vm::page_readable))
			{
				last_core = core;
				const u32 n = vm::read32(g + 0x80);
				const u32 list = vm::read32(g + 0xd8);
				const u32 list_n = vm::read32(g + 0xd0);

				if (n && n <= 0x400)
				{
					state_exclude(core, core + n * 0x2f0, "sound core voices");
				}

				if (list && list_n && list_n <= 0x1000 && vm::check_addr(list, vm::page_readable))
				{
					state_exclude(list, list + list_n * 4, "sound core voice list");
				}

				rb_online_log.success("Sound core outside the rollback: %u voices at 0x%08x, list 0x%08x (%u)", n, core, list, list_n);
			}
		}

		// Staged exclusions of the lower sound layers (analysis_sound_rollback_distortion.md). The upper layer (manager, banks, voice
		// table) is already outside the rollback and KeyOn is deduplicated; restoring the lower layers rewound every playing voice
		// on each load (all sounds distorted at every rollback). RB_SOUND_EXCL_LOWER also excluded the whole nuSound2 group head
		// (T+0..0x24010) and froze the mixer (o171); these stages leave it alone.
		if (!sound_rollback_env())
		{
			const auto env_on = [](const char* name) { const char* v = std::getenv(name); return v && *v == '1'; };
			static const bool core_run = env_on("RB_SOUND_EXCL_CORE_RUN");
			static const bool core_all = env_on("RB_SOUND_EXCL_CORE_ALL");
			static const bool voice_states = env_on("RB_SOUND_EXCL_VOICE_STATES");
			constexpr u32 g = 0x00D66D60;

			// (1) RB_SOUND_EXCL_CORE_RUN: per bnusCore voice (0x2f0) only the SPU mixer runtime [v, v+0x150): playback
			// position, decoder history, envelope and the SPU status (+0x140 state, +0x144 flags, +0x148 parameter). The PPU
			// parameters and the KeyOn handshake (+0x160..) stay in the snapshot.
			// (2) RB_SOUND_EXCL_CORE_ALL: the whole core voices and the voice list *(g+0xd8) (instead of (1))
			if (static u32 last_core = 0; core_run || core_all)
			{
				const u32 core = vm::read32(g + 0xc0);
				const u32 n = vm::read32(g + 0x80);

				if (core && core != last_core && n && n <= 0x400 && vm::check_addr(core, vm::page_readable, n * 0x2f0))
				{
					last_core = core;

					if (core_all)
					{
						state_exclude(core, core + n * 0x2f0, "sound core voices (whole)");
						const u32 list = vm::read32(g + 0xd8);
						const u32 list_n = vm::read32(g + 0xd0);

						if (list && list_n && list_n <= 0x1000 && vm::check_addr(list, vm::page_readable))
						{
							state_exclude(list, list + list_n * 4, "sound core voice list");
						}

						rb_online_log.success("Sound stage CORE_ALL: %u core voices at 0x%08x outside the rollback (list 0x%08x)", n, core, list);
					}
					else
					{
						for (u32 i = 0; i < n; i++)
						{
							state_exclude(core + i * 0x2f0, core + i * 0x2f0 + 0x150, "sound core voice runtime");
						}

						rb_online_log.success("Sound stage CORE_RUN: runtime [+0, +0x150) of %u core voices at 0x%08x outside the rollback", n, core);
					}
				}
			}

			// (3) RB_SOUND_EXCL_VOICE_STATES: the nuSound2 voice states (0xC0 x 0x48 at T+0x24014: core voice binding +4/+8,
			// state +0x20, params) - without the group head T+0..0x24010
			if (static u32 last_t = 0; voice_states)
			{
				const u32 t = vm::read32(0x00D65ABC);

				if (t && t != last_t && vm::check_addr(t + 0x24014, vm::page_readable, 0xC0 * 0x48))
				{
					last_t = t;
					state_exclude(t + 0x24014, t + 0x24014 + 0xC0 * 0x48, "sound voice states (without the group head)");
					rb_online_log.success("Sound stage VOICE_STATES: 0x%08x-0x%08x outside the rollback", t + 0x24014, t + 0x24014 + 0xC0 * 0x48);
				}
			}
		}
	}

	void on_frame(ppu_thread& ppu, u32 root)
	{

		const config_t& cfg = config();
		const u32 mgr = manager();

		game_mode_frame();
		sound_frame();

		// Frame pacing of the live fight frames (main loop boundary to boundary): how many exceed 17 / 20 / 33 ms, every 600
		if (static std::chrono::steady_clock::time_point last{}; mgr && fight_active() && !is_resimulating())
		{
			static u64 n = 0, over17 = 0, over20 = 0, over33 = 0, max_us = 0, total_n = 0, total20 = 0;
			const auto now = std::chrono::steady_clock::now();
			const u64 us = last.time_since_epoch().count() ? std::chrono::duration_cast<std::chrono::microseconds>(now - last).count() : 0;
			last = now;

			if (us && us < 1000000)
			{
				n++;
				over17 += us > 17000;
				over20 += us > 20000;
				over33 += us > 33300;
				max_us = std::max(max_us, us);

				if (n == 600)
				{
					total_n += n;
					total20 += over20;
					rb_online_log.notice("Frame pacing (600 live fight frames): >17 ms %u, >20 ms %u, >33 ms %u, max %u us; all fights: %.1f %% over 20 ms",
						over17, over20, over33, max_us, 100.0 * total20 / total_n);
					n = over17 = over20 = over33 = max_us = 0;
				}
			}
		}
		else if (!fight_active())
		{
			last = {};
		}

		if (mgr && fight_markers() && !is_resimulating())
		{
			rb_timing("frame", vm::read32(mgr + 0x60 + (2 - remote_player(mgr)) * 4));
		}
		g_fight_frames = mgr && fight_markers() ? g_fight_frames + 1 : 0;

		// Time synchronisation (see g_sync_hold): measured in live fight frames only
		{
			static const double target = std::getenv("RB_ONLINE_SYNC_TARGET") ? std::atof(std::getenv("RB_ONLINE_SYNC_TARGET")) : 1.0;
			u32 sq[2]{};
			get_seq(sq);
			g_sync_hold = false;

			if (target > 0 && mgr && in_fight() && predict_game_mode() && !state_resimulating(sq[0]))
			{
				const u32 r = remote_player(mgr) - 1;
				const u32 rsim = vm::read32(mgr + 0x60 + r * 4);
				const u32 ravail = vm::read32(mgr + 0x58 + r * 4);
				const double ahead = rsim > ravail ? static_cast<double>(rsim - ravail) : 0.0;
				g_sync_avg += (ahead - g_sync_avg) / 30.0;

				if (g_sync_avg > target + 0.75 && sq[0] - g_sync_last_hold >= 8)
				{
					g_sync_hold = true;
					g_sync_last_hold = sq[0];
					g_sync_holds++;
				}

				// RB_ONLINE_SYNC_PRED (default 0 = off; 0.5 tested: fewer but deeper rollbacks at 34 ms, o205/o206): the share of live frames that predicted the remote input, with the
				// prediction depth. Predicting about one frame in most frames = this side runs a whole frame ahead of the other
				// (at a low ping the input of the frame arrives just after it was needed: o203, 15 ms RTT, the input was sent
				// at 0.7 ms and needed at 0.65 ms into the frame, 7.6 ms on the way). One frame waits for it and both sides line
				// up. Deeper predictions (high ping) are not a frame advantage: left to the auto input delay
				static const double pred_target = std::getenv("RB_ONLINE_SYNC_PRED") ? std::atof(std::getenv("RB_ONLINE_SYNC_PRED")) : 0.0;
				static u64 last_count = 0, last_depth = 0;
				static double pred_share = 0.0, pred_depth = 0.0;
				const u64 dc = g_live_pred_count - last_count, dd = g_live_pred_depth - last_depth;
				last_count = g_live_pred_count;
				last_depth = g_live_pred_depth;
				pred_share += ((dc ? 1.0 : 0.0) - pred_share) / 60.0;

				if (dc)
				{
					pred_depth += (static_cast<double>(dd) / static_cast<double>(dc) - pred_depth) / 30.0;
				}

				if (pred_target > 0 && !g_sync_hold && pred_share > pred_target && pred_depth < 1.5 && sq[0] - g_sync_last_hold >= 60)
				{
					g_sync_hold = true;
					g_sync_last_hold = sq[0];
					g_sync_holds++;
					pred_share = 0.0;

					if (static u32 logged = 0; logged++ < 30)
					{
						rb_online_log.notice("Time sync: predicting in most frames by ~%.1f frame: one frame waits for the remote input", pred_depth);
					}
				}

				if (static u64 n = 0; ++n % 600 == 0)
				{
					rb_online_log.notice("Time sync: %.2f frames predicted on average, %u frames waited so far (target %.1f), live frames predicting %.0f %%", g_sync_avg, g_sync_holds, target, pred_share * 100);
				}
			}
			else if (!mgr || !in_fight())
			{
				g_sync_avg = 0.0;
			}
		}

		// RB_FAST_MENU=1 (tests): until the fight runs, the game runs at RB_FAST_MENU_RATE vblanks per second (the game counts
		// its frames by vblank, so boot, menus and the character select go faster) and without the simulated network lag
		if (static const bool fast_menu = std::getenv("RB_FAST_MENU") && *std::getenv("RB_FAST_MENU") == '1'; fast_menu)
		{
			static const s64 rate = std::getenv("RB_FAST_MENU_RATE") ? std::atoi(std::getenv("RB_FAST_MENU_RATE")) : 180;
			const bool fight_now = in_fight() || (mgr && fight_markers());

			if (g_cfg.video.vblank_rate != (fight_now ? 60 : rate))
			{
				g_cfg.video.vblank_rate.set(fight_now ? 60 : rate);
				g_rb_net_lag_active = fight_now;
				rb_online_log.notice("Fast menus: vblank rate %d, network lag %s", fight_now ? 60 : rate, fight_now ? "on" : "off");
			}
		}

		if (static u32 last_scene = umax; vm::read32(scene_addr) != last_scene)
		{
			last_scene = vm::read32(scene_addr);
			rb_online_log.notice("Scene %u (input manager 0x%08x, sim %u)", last_scene, mgr, mgr ? +vm::read32(mgr + 0x60) : 0);
		}

		if (!mgr)
		{
			if (stress_sound())
			{
				sound_exclusions();
			}

			return;
		}

		// The online fight runs: it is the sync point of the snapshot module (the offline one, the game LCG, is not used)
		const bool fight = in_fight();

		if (fight && !g_fight_start_ns)
		{
			g_fight_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		}
		else if (!fight)
		{
			g_fight_start_ns = 0;
		}

		if (fight)
		{
			force_sync_point("online fight");
			start_lwmutex_sampler();
		}

		// RB_ONLINE_LEAD=auto: the input delay starts at 1 (the user's choice: delay 1 + rollback of at most RB_ONLINE_MAX_PREDICT
		// frames up to ~100 ms ping; lead 0 needs a larger prediction window) and follows the input deficit (see below).
		static s32 auto_lead = 1;
		const s32 lead = cfg.lead == -2 ? auto_lead : cfg.lead;

		// A new input manager (online character select starts): logged once (tools\online_match.ps1 waits for it)
		if (static u32 last_mgr = 0; mgr != last_mgr)
		{
			last_mgr = mgr;
			rb_online_log.success("Online input manager 0x%08x (game's input lead %u)", mgr, +vm::read32(mgr + 0x74));
		}

		// Intro, KO, replay (game modes without prediction) run in lockstep: with the fight's short input delay the game
		// waited for the remote input almost every frame at ~100 ms ping (the scenes stuttered). There the delay covers the
		// latency instead (inputs only skip the scene): RB_ONLINE_LOCKSTEP_LEAD (default auto: starts at 4, +1 when the 90th
		// percentile of the input deficit over 120 frames is above 0, -1 after two windows with it at -2 or lower).
		static s32 lockstep_lead = []()
		{
			const char* v = std::getenv("RB_ONLINE_LOCKSTEP_LEAD");
			return v && *v && std::string_view(v) != "auto" ? std::atoi(v) : -2;
		}();
		static s32 lockstep_auto = 4;
		const bool lockstep_scene = cfg.predict && lead >= 0 && lockstep_lead != 0 && !predict_game_mode();
		const s32 want_lead = !lockstep_scene ? lead : std::max(lead, lockstep_lead == -2 ? lockstep_auto : lockstep_lead);

		// Only in the fight (menus and character select keep the game's value)
		if (want_lead >= 0 && fight_markers() && vm::read32(mgr + 0x74) != static_cast<u32>(want_lead))
		{
			rb_online_log.success("Online input lead %u -> %d%s", +vm::read32(mgr + 0x74), want_lead, lockstep_scene ? " (lockstep scene)" : "");
			vm::write32(mgr + 0x74, want_lead);
		}

		if (lockstep_scene && lockstep_lead == -2 && fight_markers() && !is_resimulating())
		{
			static std::vector<s32> deficits;
			static u32 low_windows = 0;
			static u32 last_sim_ls = 0;
			static u64 last_waits = 0;
			const u32 rp = remote_player(mgr) - 1;
			const u32 sim_ls = vm::read32(mgr + 0x60 + rp * 4);
			const bool waited = g_remote_waits != last_waits;
			last_waits = g_remote_waits;

			if (sim_ls != last_sim_ls || waited)
			{
				const s32 d = static_cast<s32>(vm::read32(mgr + 0x60 + (1 - rp) * 4) + 1) - static_cast<s32>(vm::read32(mgr + 0x58 + rp * 4));

				if (d > -30 && d < 30)
				{
					deficits.push_back(d);
				}
			}

			last_sim_ls = sim_ls;

			if (deficits.size() == 120)
			{
				std::sort(deficits.begin(), deficits.end());
				const s32 p90 = deficits[108];
				const s32 old = lockstep_auto;

				if (p90 > 0)
				{
					lockstep_auto = std::min(12, lockstep_auto + 1);
					low_windows = 0;
				}
				else if (p90 <= -2 && ++low_windows >= 2)
				{
					lockstep_auto = std::max(1, lockstep_auto - 1);
					low_windows = 0;
				}
				else if (p90 > -2)
				{
					low_windows = 0;
				}

				if (lockstep_auto != old)
				{
					rb_online_log.success("Online: lockstep scenes: input deficit 90th percentile %d: input delay %d -> %d", p90, old, lockstep_auto);
				}

				deficits.clear();
			}
		}

		const u32 ri = remote_player(mgr) - 1;

		// New round / new fight: the input manager starts again at frame 0
		static u32 last_sim = 0;
		const u32 sim_now = vm::read32(mgr + 0x60 + ri * 4);

		// (a rollback also lowers it, by at most RB_ONLINE_MAX_RESIM: only a drop to the first frames counts)
		if (sim_now + 30 < last_sim && sim_now < 10)
		{
			rb_online_log.notice("Online: simulated frames restarted (%u -> %u), prediction state cleared", last_sim, sim_now);
			g_predicted.clear();
			g_remote.clear();
			g_remote_at.clear();
			g_local.clear();
			g_remote_last = 0;
			g_mismatch_frame = umax;
			g_sim_at.clear();
		}

		last_sim = sim_now;

		// Resend after a stall: the game sends inputs only when it has a new local frame (6 frame window). A rollback restores
		// the receive queue of the game's network layer while its network thread keeps writing into it, so messages that
		// arrived in between can be lost; when both sides then wait for each other nothing is sent any more. While the
		// remote simulated frame does not advance, the local "sent" counter is moved back every 10 frames so that the
		// last 6 local inputs go out again (the peer ignores the ones it already has).
		if (fight)
		{
			static u32 stall_sim = 0;
			static u32 stall_frames = 0;
			stall_frames = sim_now == stall_sim ? stall_frames + 1 : 0;

			if (cfg.lead == -2 && !is_resimulating() && cfg.predict && in_fight() && predict_game_mode() && ring_ready())
			{
				// Deficit of a frame = (local sim + 1) - remote available: the game waits when it exceeds the prediction
				// window, every frame of input delay lowers it by one (ANALYZA.md 38). Raised at once by the 90th
				// percentile's excess over 300 frames; lowered by one after 600 frames with the percentile 2 below the window.
				// Only frames where the fight runs (simulation advanced or waited for the remote input; not the round end).
				static std::vector<s32> deficits;
				static u32 low_windows = 0;
				static u64 last_remote_waits = 0;
				const bool waited = g_remote_waits != last_remote_waits;
				last_remote_waits = g_remote_waits;

				if (sim_now != stall_sim || waited)
				{
					const u32 li = 1 - ri;
					const s32 d = static_cast<s32>(vm::read32(mgr + 0x60 + li * 4) + 1) - static_cast<s32>(vm::read32(mgr + 0x58 + ri * 4));

					// Not the round end / start (the counters jump)
					if (d > -30 && d < 30)
					{
						deficits.push_back(d);
					}
				}

				if (deficits.size() == 300)
				{
					std::sort(deficits.begin(), deficits.end());
					// Raised by one when most frames (median) would wait; a jitter tail (90th percentile) is covered by the
					// rollback (the user's choice: delay 1 up to ~100 ms ping)
					const s32 p50 = deficits[150];
					const s32 p90 = deficits[270];
					// Compared with the rollback depth the delay is meant to keep (the user: rollback up to ~100 ms ping = 3
					// frames, above that only a higher delay; RB_ONLINE_AUTO_DEPTH), not with the prediction window (5 in
					// play_online.bat: the delay rose only at ~200 ms and rollbacks went 5 frames deep). The window stays the
					// reserve for jitter.
					static const s32 depth = []()
					{
						const char* v = std::getenv("RB_ONLINE_AUTO_DEPTH");
						return v && *v ? std::max(1, std::atoi(v)) : 3;
					}();
					const s32 window = std::min(depth, static_cast<s32>(cfg.max_predict));
					const s32 old = auto_lead;

					if (p50 > window)
					{
						auto_lead = std::min(8, auto_lead + 1);
						low_windows = 0;
					}
					else if (p90 <= window - 2 && ++low_windows >= 2)
					{
						auto_lead = std::max(1, auto_lead - 1);
						low_windows = 0;
					}
					else if (p90 > window - 2)
					{
						low_windows = 0;
					}

					if (auto_lead != old)
					{
						rb_online_log.success("Online: input deficit median %d / 90th percentile %d, target rollback depth %d: input delay %d -> %d", p50, p90, window, old, auto_lead);
					}

					deficits.clear();
				}
			}

			stall_sim = sim_now;

			if (stall_frames && stall_frames % 10 == 0)
			{
				const u32 li = 1 - ri;
				const u32 available = vm::read32(mgr + 0x58 + li * 4);
				const u32 resend_from = available > 1 ? available - 1 : 0; // window [avail-6, avail-1]: the newest 6 in one message

				if (vm::read32(mgr + 0x68 + li * 4) > resend_from)
				{
					vm::write32(mgr + 0x68 + li * 4, resend_from);
				}

				static u32 logged = 0;

				if (logged++ < 30)
				{
					rb_online_log.warning("Stalled at remote simulated frame %u for %u frames: resending local inputs from %u", sim_now, stall_frames, resend_from);
				}
			}
		}

		u32 seq[2]{};
		get_seq(seq);
		u64 game_frame = seq[0];

		if (resimulating)
		{
			if (game_frame < g_resim_until && fight && !ppu.is_stopped())
			{
				write_hash(vm::read32(mgr + 0x60 + ri * 4), game_frame, true);
				save_if_advanced(ppu, mgr, ri, game_frame, seq);
				state_net_set_resim(true);
				g_resim_frames++;
				return;
			}

			resimulating = false;
			state_net_set_resim(false);

			if (game_frame != g_resim_until)
			{
				rb_online_log.error("Re-simulation ended at game frame %u instead of %u", game_frame, g_resim_until);
			}
		}

		// Snapshots of the stress ring wait for running SPURS jobs (see spurs_job_pending)
		if (static bool registered = false; !registered && cfg.predict && use_stress_ring())
		{
			registered = true;
			// Only frame boundaries where the remote simulated frame advanced are saved: frames the game spends waiting for
			// the peer would push the needed snapshots out of the ring (at 100 ms latency the rollback target was often
			// older than 8 frames -> mispredictions that could not be corrected -> desync)
			state_set_save_tag([]() -> u64
			{
				const u32 m = manager();
				return m ? +vm::read32(m + 0x60 + (remote_player(m) - 1) * 4) : umax;
			});

			state_set_spu_busy_check([]()
			{
				const auto ppu = get_current_cpu_thread<ppu_thread>();
				return ppu && spurs_job_pending(*ppu, 0);
			});

			state_set_save_check([]()
			{
				static u32 last_sim = umax;

				// No snapshot at the boundaries of a re-simulation: the re-simulated frames
				// do not wait for their SPURS jobs (RB_FAST_RESIM), a state saved there can hold unfinished job data
				// (o84/o85/o86: a second rollback to such a snapshot desynced). A second rollback uses an older snapshot.
				// The first boundary after the re-simulation is a normal live frame and is saved (o89: skipping it too left
				// the ring without new snapshots when rollbacks came every few frames); its SPURS jobs are awaited below.
				// Exception (RB_ONLINE_RESIM_SAVE, default on): a re-simulated boundary whose state used only real remote inputs
				// (remote sim <= the first predicted frame and <= the highest real one) is final; saved after its SPURS jobs
				// finished (checked below). Without it the next rollback had to go before the previous window (depth ~5.5
				// instead of ~3, ANALYZA.md 47).
				if (is_resimulating())
				{
					static const bool resim_save = !std::getenv("RB_ONLINE_RESIM_SAVE") || *std::getenv("RB_ONLINE_RESIM_SAVE") != '0';
					const u32 m = manager();

					if (!resim_save || !m)
					{
						return false;
					}

					const u32 rsim = vm::read32(m + 0x60 + (remote_player(m) - 1) * 4);

					static u64 not_final = 0, spu_busy = 0, saved = 0;
					const auto report = []()
					{
						if ((not_final + spu_busy + saved) % 300 == 0)
						{
							rb_online_log.notice("Re-simulated boundaries: %u saved, %u not final (predicted inputs), %u SPU job still running", saved, not_final, spu_busy);
						}
					};

					// The last final boundary: the remote simulation reached the first frame that is not confirmed (a predicted
					// one, or beyond the highest real input). Only this one is saved: one save per re-simulation (each costs
					// ~0.9 ms; o136 saved every final boundary), and it is the snapshot the next rollback of a chain needs
					u64 limit = g_remote_last;

					if (!g_predicted.empty())
					{
						limit = std::min<u64>(limit, g_predicted.begin()->first);
					}

					if (g_mismatch_frame != umax)
					{
						limit = std::min<u64>(limit, g_mismatch_frame);
					}

					if (rsim != limit || rsim == last_sim)
					{
						not_final++;
						report();
						return false;
					}

					// Up to 4 ms for the SPU job of the re-simulated frame (1 ms skipped most saves: chains of rollbacks to the
					// same old snapshot, 11-18 frames deep in o135)
					const auto ppu = get_current_cpu_thread<ppu_thread>();
					static const u32 resim_wait_us = std::getenv("RB_ONLINE_RESIM_SPURS_WAIT_US") ? std::atoi(std::getenv("RB_ONLINE_RESIM_SPURS_WAIT_US")) : 4000;

					if (ppu && spurs_job_pending(*ppu, resim_wait_us))
					{
						spu_busy++;
						report();
						return false;
					}

					saved++;
					report();
					last_sim = rsim;
					return true;
				}

				if (const u32 m = manager())
				{
					const u32 sim = vm::read32(m + 0x60 + (remote_player(m) - 1) * 4);

					// A frame where the remote simulation did not advance (waiting) is saved too: rb_state overwrites the slot of
					// the same simulated frame, so the ring keeps one snapshot per simulated frame (the latest), and a rollback
					// no longer re-simulates all the waiting frames before it (o133: chains of 12-19 frame rollbacks)
					static const bool save_waiting = !std::getenv("RB_ONLINE_SAVE_WAITING") || *std::getenv("RB_ONLINE_SAVE_WAITING") != '0';

					if (sim == last_sim && !save_waiting)
					{
						return false;
					}

					const auto ppu = get_current_cpu_thread<ppu_thread>();

					if (ppu && spurs_job_pending(*ppu))
					{
						return false;
					}

					last_sim = sim;
					return true;
				}

				const auto ppu = get_current_cpu_thread<ppu_thread>();
				return !ppu || !spurs_job_pending(*ppu);
			});
		}

		if (cfg.predict && fight && use_stress_ring() && state_stress_ready())
		{
			const bool resim_now = state_resimulating(game_frame);
			const u32 sim = vm::read32(mgr + 0x60 + ri * 4);

			// RB_ONLINE_STRESS=N (diagnostics): artificial rollbacks of 2 simulated frames with the correct inputs
			if (static const u32 stress = std::getenv("RB_ONLINE_STRESS") ? std::atoi(std::getenv("RB_ONLINE_STRESS")) : 0; stress && !resim_now)
			{
				static u64 n = 0;

				if (++n % stress == 0 && g_mismatch_frame == umax && sim > 2)
				{
					g_mismatch_frame = sim - 2;
				}
			}


			if (resim_now && game_frame == resim_dump_frame)
			{
				dump_memory_now(resim_dump_prefix, game_frame);
				rb_online_log.success("Re-simulation dump at game frame %u (pass 1)", game_frame);
				resim_dump_frame = umax;
			}

			// A misprediction: rollback request (performed by rb_state at the next frame boundary); not during a
			// re-simulation (the request waits until the current frame is reached again)
			if (g_mismatch_frame != umax && !resim_now)
			{
				request_rollback(mgr, ri, game_frame);
			}

			keep_local_inputs(mgr, 1 - ri, resim_now);

			if (resim_now)
			{
				vm::write32(mgr + 0x68 + (1 - ri) * 4, vm::read32(mgr + 0x58 + (1 - ri) * 4));
			}
			g_resim_frames += resim_now;
			g_sim_at[game_frame] = sim;

			while (g_sim_at.size() > 600)
			{
				g_sim_at.erase(g_sim_at.begin());
			}

			// Local player check (by simulated frame; the game frame of a simulated frame differs when the first pass waited):
			// a re-simulated frame must put the local player where the first pass did, unless the
			// corrected remote input touched him (hit, push). Every correction is counted, large ones are logged.
			{
				static std::map<u64, std::array<f32, 3>> local_pos;
				static u64 corrections = 0, large = 0;
				const u32 pa = ri == 0 ? 0x00B2D67C : 0x00B2CA3C;
				const std::array<f32, 3> p{std::bit_cast<f32>(+vm::read32(pa)), std::bit_cast<f32>(+vm::read32(pa + 4)), std::bit_cast<f32>(+vm::read32(pa + 8))};

				// Only frames where the simulation advanced (a frame waiting for the remote input repeats the same sim)
				static u32 prev_sim = umax;
				const bool advanced = sim != prev_sim;
				prev_sim = sim;

				if (!advanced)
				{
				}
				else if (!resim_now)
				{
					local_pos[sim] = p;

					while (local_pos.size() > 64)
					{
						local_pos.erase(local_pos.begin());
					}
				}
				else if (const auto it = local_pos.find(sim); it != local_pos.end())
				{
					const f32 d = std::hypot(p[0] - it->second[0], p[1] - it->second[1], p[2] - it->second[2]);
					it->second = p;

					if (d > 0.01f)
					{
						corrections++;

						if (d > 20.f && ++large <= 200)
						{
							rb_online_log.error("Local player moved by %.1f in the re-simulation of game frame %u (sim %u, %u corrections, %u large)",
								d, game_frame, sim, corrections, large);
						}
					}
				}
			}

			write_hash(sim, game_frame, resim_now);
		}
		else if (cfg.predict && fight && state_net_ready())
		{
			// RB_ONLINE_STRESS=N: every N frames an artificial rollback of RB_ONLINE_STRESS_DEPTH (2) simulated frames with
			// the correct inputs (tests that the snapshots of the online fight are complete, independent of predictions)
			if (static const u32 stress = std::getenv("RB_ONLINE_STRESS") ? std::atoi(std::getenv("RB_ONLINE_STRESS")) : 0; stress)
			{
				static const u32 depth = std::getenv("RB_ONLINE_STRESS_DEPTH") ? std::atoi(std::getenv("RB_ONLINE_STRESS_DEPTH")) : 2;
				static u64 n = 0;
				const u32 sim = vm::read32(mgr + 0x60 + ri * 4);

				if (++n % stress == 0 && g_mismatch_frame == umax && sim > depth)
				{
					g_mismatch_frame = sim - depth;
				}
			}

			// Rollback: the last frame boundary whose remote simulated frame is not after the mispredicted one
			if (g_mismatch_frame != umax)
			{
				const u32 bad = g_mismatch_frame;
				g_mismatch_frame = umax;

				u64 target = umax;

				for (auto it = g_sim_at.rbegin(); it != g_sim_at.rend(); ++it)
				{
					if (it->second <= bad && it->first < game_frame)
					{
						target = it->first;
						break;
					}
				}

				snapshot_info info{};

				// Snapshots exist only for frames where the simulation advanced (save_if_advanced), so the ring covers simulated
				// frames, not frames the game spent waiting; the re-simulation length is still bounded (host time)
				// The local inputs are not rolled back: they were already sent to the peer, and the re-simulation would generate
				// different ones (its frames wait less than the original ones). Entries, present bits, available and sent
				// counters of the local player are kept as they are now.
				const u32 li = 1 - ri;
				const u32 local_available = vm::read32(mgr + 0x58 + li * 4);
				const u32 local_sent = vm::read32(mgr + 0x68 + li * 4);
				std::vector<std::pair<u32, entry_t>> local_entries;

				for (u32 f = local_available > 32 ? local_available - 32 : 0; f < local_available; f++)
				{
					if (vm::read8(flag_addr(mgr, li, f)) & 1)
					{
						local_entries.emplace_back(f, read_entry(slot_addr(mgr, li, f)));
					}
				}

				// Current content of the network bytes (see net_page)
				for (auto& [addr, p] : g_net_pages)
				{
					if (vm::check_addr(addr, vm::page_readable))
					{
						std::memcpy(p.buf.data(), vm::get_super_ptr<u8>(addr), 4096);
					}
				}

				if (target != umax && game_frame - target <= max_resim() && state_net_load(target, info))
				{
					set_seq(info.seq);

					for (auto& [addr, p] : g_net_pages)
					{
						if (p.bytes && vm::check_addr(addr, vm::page_readable))
						{
							u8* cur = vm::get_super_ptr<u8>(addr);

							for (u32 i = 0; i < 4096; i++)
							{
								if (p.mask[i])
								{
									cur[i] = p.buf[i];
								}
							}
						}
					}

					// Diagnostics: SPURS jobs pending in the loaded state (see settle_spurs_jobs)
					if (static u32 logged = 0; logged < 40)
					{
						for (const auto& [ptr, offset] : {std::pair{0x009680acu, 0u}, std::pair{0x009680acu, 0x380u}, std::pair{0x009680acu, 0x700u},
							std::pair{0x009682ecu, 0x80u}, std::pair{0x00968324u, 0x4e80u}})
						{
							const u32 obj = vm::read32(ptr) + offset;

							if (vm::read32(ptr) && vm::read32(obj + 0x304))
							{
								logged++;
								rb_online_log.notice("Loaded state of game frame %u: SPURS job 0x%08x pending (+0x308 %u)", target, obj, +vm::read32(obj + 0x308));
							}
						}
					}

					for (const auto& [f, e] : local_entries)
					{
						std::memcpy(vm::get_super_ptr<u8>(slot_addr(mgr, li, f)), e.data(), entry_size);
						vm::write8(flag_addr(mgr, li, f), vm::read8(flag_addr(mgr, li, f)) | 1);
					}

					vm::write32(mgr + 0x58 + li * 4, std::max<u32>(vm::read32(mgr + 0x58 + li * 4), local_available));
					vm::write32(mgr + 0x68 + li * 4, std::max<u32>(vm::read32(mgr + 0x68 + li * 4), local_sent));
					reapply_remote(mgr, vm::read32(mgr + 0x60 + ri * 4));
					g_rollbacks++;
					g_sim_at.erase(g_sim_at.upper_bound(target), g_sim_at.end());

					// Re-simulation up to the current frame: the game's own main loop runs the frames again (like the offline
					// rollback: frame end, frame allocator swap, draw requests - a loop of task walks alone skipped those and
					// broke the draw lists / shader references / allocator), without rendering (RB_FAST_RESIM) and with the
					// recorded local pad input. on_frame only saves the boundaries until the current frame is reached.
					resimulating = true;
					g_resim_until = game_frame;
					state_net_set_resim(true);
					begin_frame();
					return;
				}
				else
				{
					rb_online_log.error("Misprediction of remote frame %u cannot be rolled back (snapshot %u, game frame %u)", bad, target, game_frame);
				}
			}

			write_hash(vm::read32(mgr + 0x60 + ri * 4), game_frame, false);
			save_if_advanced(ppu, mgr, ri, game_frame, seq);
		}

		static u64 frames = 0;

		sound_exclusions();

		if (std::getenv("RB_ONLINE_PHASE_LOG") && frames % 60 == 0)
		{
			rb_online_log.notice("Phase: sim %u lcg 0x%08x scene %u c03d44 %u 97dbb4 %u 972440 %u", +vm::read32(mgr + 0x60 + ri * 4),
				+vm::read32(0x00D142AC), +vm::read32(0x00A77A10), +vm::read32(0x00C03D44), +vm::read32(0x0097DBB4), +vm::read32(0x00972440));
		}

		if (frames % 600 == 599 && !g_net_pages.empty())
		{
			u32 bytes = 0;

			for (const auto& [addr, p] : g_net_pages)
			{
				bytes += p.bytes;
			}

			rb_online_log.notice("Network bytes kept across loads: %u on %u pages", bytes, g_net_pages.size());
		}

		if (side_heap::enabled() && frames % 600 == 300)
		{
			side_heap::log_stats();
		}

		if (++frames % 600 == 0)
		{
			rb_online_log.notice("Online: game frame %u, remote simulated %u, local available %u, remote available %u, predictions %u, "
				"mispredicted %u, rollbacks %u (%u re-simulated frames), prediction waits %u; get remote %u (missing %u), stored remote %u local %u, remote last %u",
				game_frame, +vm::read32(mgr + 0x60 + ri * 4), +vm::read32(mgr + 0x58 + (1 - ri) * 4), +vm::read32(mgr + 0x58 + ri * 4),
				g_predictions, g_mispredictions, g_rollbacks, g_resim_frames, g_predict_waits, g_dbg_get_remote, g_dbg_get_missing, g_dbg_store_remote,
				g_dbg_store_local, g_remote_last);
		}
	}

	void sound_boundary_state(u64& frame, u64& loads, bool& resim, bool& active)
	{
		static const bool on = std::getenv("RB_SOUND_SEPARATE") && *std::getenv("RB_SOUND_SEPARATE") == '1';
		frame = sound_frame_no();
		loads = state_load_count();
		resim = sound_resim();
		active = on && sound_recording_active();
	}
}
