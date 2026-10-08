#include "stdafx.h"
#include "rb_input.h"
#include "rb_checksum.h"
#include "rb_state.h"

#include "Emu/Cell/timers.hpp"
#include "Emu/Cell/PPUThread.h"
#include "Emu/Cell/PPUFunction.h"
#include "Emu/Cell/PPUModule.h"
#include "Emu/System.h"
#include "Emu/RSX/RSXThread.h"
#include "Emu/system_config.h"
#include "Emu/IdManager.h"
#include "Emu/Io/pad_types.h"
#include "Emu/Memory/vm.h"
#include "Utilities/File.h"
#include "Utilities/StrUtil.h"

#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <optional>
#include <array>
#include <set>
#include <thread>

#include <atomic>
#include <cstdlib>
#include <mutex>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

LOG_CHANNEL(rb_log, "Rollback");

extern bool ppu_form_branch_to_code(u32 entry, u32 target);

namespace rb
{
	namespace
	{
		enum class mode_t { off, record, replay };

		constexpr char file_magic[4] = {'R', 'B', 'I', 'N'};
		constexpr u32 file_version = 1;
		constexpr u32 max_ports = 2;

		// Marker record: port = sync_marker_port + p, seq = read position of port p at the sync point
		constexpr u32 sync_marker_port = 0xFFFF0000;

		// One cellPadGetData result exactly as the game received it (CellPadData kept in guest byte order)
		struct pad_record
		{
			u64 frame;
			u32 port;
			u32 seq; // index of the call on this port
			u8 data[sizeof(CellPadData)];
		};

		struct state_t
		{
			mode_t mode = mode_t::off;
			fs::file input;
			fs::file stats;

			std::vector<pad_record> replay[max_ports];
			u32 seq[max_ports]{};
			u64 frame_mismatches = 0;
			bool replay_exhausted[max_ports]{};

			u64 predicted_reads = 0;

			u32 sync_addr = 0;
			u32 sync_frame = 0; // RB_SYNC_FRAME: sync point at this port 0 read (recordings without the LCG seed)
			bool synced = false;
			s64 recorded_sync_seq[max_ports]{-1, -1};
		};

		state_t g_state;
		std::once_flag g_init;

		std::atomic<u64> g_frame{0};
		std::atomic<u64> g_game_frame{0};
		u64 g_sync_frame = umax; // game frame (port 0 reads) of the sync point
		CellPadData g_live_pad{};
		bool g_live_pad_valid = false;
		u64 g_port0_reads = 0; // main thread only
		std::atomic<u32> g_reads[max_ports]{};
		std::atomic<u32> g_reset_flip{0};

		// Virtual timebase of the main thread (RB_VIRTUAL_TB)
		constexpr u64 vtb_per_frame = 79'800'000 / 60;
		constexpr u64 vtb_step = 800; // 10 us per read; reads that wait for time to pass still progress
		u64 g_vtb_base = 0;           // timebase of game frame 0
		u64 g_vtb_frame = 0;          // game frame of the current main thread frame
		u64 g_vtb_offset = 0;         // advance inside the current frame
		bool g_vtb_active = false;

		std::string get_env(const char* name)
		{
			const char* value = std::getenv(name);
			return value ? value : "";
		}

		void load_replay(const std::string& path)
		{
			fs::file f(path);

			if (!f)
			{
				rb_log.error("Replay file '%s' cannot be opened, input passes through", path);
				g_state.mode = mode_t::off;
				return;
			}

			char magic[4]{};
			u32 version = 0;

			if (f.read(magic, sizeof(magic)) != sizeof(magic) || std::memcmp(magic, file_magic, sizeof(magic)) != 0 ||
				f.read(&version, sizeof(version)) != sizeof(version) || version != file_version)
			{
				rb_log.error("Replay file '%s' has an unknown format", path);
				g_state.mode = mode_t::off;
				return;
			}

			pad_record rec;

			while (f.read(&rec, sizeof(rec)) == sizeof(rec))
			{
				if (rec.port < max_ports)
				{
					g_state.replay[rec.port].push_back(rec);
				}
				else if (rec.port - sync_marker_port < max_ports)
				{
					g_state.recorded_sync_seq[rec.port - sync_marker_port] = rec.seq;
				}
			}

			if (g_state.recorded_sync_seq[0] >= 0)
			{
				rb_log.success("Replay sync point: port 0 read %d, port 1 read %d", g_state.recorded_sync_seq[0], g_state.recorded_sync_seq[1]);
			}

			rb_log.success("Replay loaded from '%s': port 0: %u calls, port 1: %u calls", path, g_state.replay[0].size(), g_state.replay[1].size());
		}

		void init()
		{
			const std::string mode = get_env("RB_MODE");
			std::string path = get_env("RB_FILE");
			const std::string stats_path = get_env("RB_STATS");
			g_state.sync_addr = static_cast<u32>(std::strtoul(get_env("RB_SYNC_ADDR").c_str(), nullptr, 16));
			g_state.sync_frame = static_cast<u32>(std::strtoul(get_env("RB_SYNC_FRAME").c_str(), nullptr, 10));

			// RB_NET_DUMP=1: every UDP packet of the game in the log (sys_net_dump trace) plus a marker per main loop frame
			if (get_env("RB_NET_DUMP") == "1")
			{
				logs::set_level("sys_net_dump", logs::level::trace);
			}

			if (path.empty())
			{
				path = fs::get_config_dir() + "rb_input.bin";
			}

			if (mode == "record")
			{
				g_state.mode = mode_t::record;

				if (!g_state.input.open(path, fs::rewrite))
				{
					rb_log.error("Recording file '%s' cannot be created", path);
					g_state.mode = mode_t::off;
				}
				else
				{
					g_state.input.write(file_magic, sizeof(file_magic));
					g_state.input.write(&file_version, sizeof(file_version));
					rb_log.success("Recording pad input to '%s'", path);
				}
			}
			else if (mode == "replay")
			{
				g_state.mode = mode_t::replay;
				load_replay(path);
			}

			if (!stats_path.empty())
			{
				if (g_state.stats.open(stats_path, fs::rewrite))
				{
					const std::string header = "frame,time_us,pad0_reads,pad1_reads,reset_flip_calls\n";
					g_state.stats.write(header.data(), header.size());
					rb_log.success("Writing frame statistics to '%s'", stats_path);
				}
				else
				{
					rb_log.error("Statistics file '%s' cannot be created", stats_path);
				}
			}
		}

		void ensure_init()
		{
			std::call_once(g_init, init);
		}

		// Called once, on the first port 0 read after the sync value became non-zero
		void check_sync_point()
		{
			g_state.synced = true;

			if (g_state.mode == mode_t::record)
			{
				for (u32 p = 0; p < max_ports; p++)
				{
					pad_record rec{};
					rec.frame = g_frame.load();
					rec.port = sync_marker_port + p;
					rec.seq = g_state.seq[p];
					g_state.input.write(&rec, sizeof(rec));
				}

				rb_log.success("Sync point recorded: port 0 read %u, port 1 read %u", g_state.seq[0], g_state.seq[1]);
			}
			else if (g_state.mode == mode_t::replay && g_state.recorded_sync_seq[0] >= 0)
			{
				rb_log.success("Sync point reached at port 0 read %u (recorded %d), port 1 read %u (recorded %d): realigning",
					g_state.seq[0], g_state.recorded_sync_seq[0], g_state.seq[1], g_state.recorded_sync_seq[1]);

				for (u32 p = 0; p < max_ports; p++)
				{
					g_state.seq[p] = static_cast<u32>(g_state.recorded_sync_seq[p]);
				}
			}

			g_sync_frame = g_state.seq[0];
			rb_log.success("Sync point (fight start) at game frame %u", g_sync_frame);
		}
	}

	u64 frame()
	{
		return g_frame.load();
	}

	u64 game_frame()
	{
		return g_game_frame.load();
	}

	bool get_fixed_time(s64& sec, s64& nsec)
	{
		static const s64 base = []() -> s64
		{
			const char* value = std::getenv("RB_FIXED_TIME");
			return value && *value ? std::strtoll(value, nullptr, 10) : -1;
		}();

		if (base < 0)
		{
			return false;
		}

		// 60 game frames per second of guest wall clock
		const u64 f = g_game_frame.load();
		sec = base + static_cast<s64>(f / 60);
		nsec = static_cast<s64>(f % 60) * 1'000'000'000 / 60;
		return true;
	}

	bool get_frame_system_time(u32 lr, u64& usec)
	{
		static const std::vector<u32> callers = []()
		{
			std::vector<u32> out;
			const char* value = std::getenv("RB_FRAME_TIME_LR");

			for (const std::string& item : fmt::split(value ? value : "", {","}))
			{
				out.push_back(static_cast<u32>(std::strtoul(item.c_str(), nullptr, 16)));
				rb_log.success("sys_time_get_system_time is frame based for caller LR 0x%x", out.back());
			}

			return out;
		}();

		if (std::find(callers.begin(), callers.end(), lr) == callers.end())
		{
			return false;
		}

		// Arbitrary start so the value looks like a running system clock
		usec = 1'000'000'000 + g_game_frame.load() * 1'000'000 / 60;
		return true;
	}

	// Scripted input: lines "<button>[+<button>...] <frames>" or "wait <frames>" appended to the RB_PAD_CMD file are
	// executed in order on port 0 (one entry per pad read). Buttons: up down left right cross circle square triangle
	// start select l1 r1 l2 r2. A released frame follows every press, so repeated presses are recognized.
	namespace
	{
		struct pad_cmd
		{
			u16 digital1 = 0, digital2 = 0;
			u32 frames = 0;
		};

		std::deque<pad_cmd> g_pad_cmds;

		void apply_pad_commands(CellPadData* data)
		{
			static const std::string path = get_env("RB_PAD_CMD");

			if (path.empty())
			{
				return;
			}

			// New commands (the file is emptied after reading)
			if (fs::file f{path, fs::read + fs::write}; f && f.size())
			{
				std::string text(f.size(), '\0');
				f.read(text.data(), text.size());
				f.trunc(0);

				for (const std::string& line : fmt::split(text, {"\n", "\r"}))
				{
					const auto parts = fmt::split(line, {" "});

					if (parts.size() != 2)
					{
						continue;
					}

					// "dump <n>": full memory dump now (RB_PAD_DUMP_PREFIX, default records\pad_dump)
					if (parts[0] == "dump")
					{
						std::string prefix = get_env("RB_PAD_DUMP_PREFIX");
						dump_memory_now(prefix.empty() ? path + "_dump" : prefix, g_state.seq[0]);
						continue;
					}

					pad_cmd cmd;
					cmd.frames = static_cast<u32>(std::strtoul(parts[1].c_str(), nullptr, 10));

					for (const std::string& b : fmt::split(parts[0], {"+"}))
					{
						if (b == "select") cmd.digital1 |= 0x01;
						else if (b == "start") cmd.digital1 |= 0x08;
						else if (b == "up") cmd.digital1 |= 0x10;
						else if (b == "right") cmd.digital1 |= 0x20;
						else if (b == "down") cmd.digital1 |= 0x40;
						else if (b == "left") cmd.digital1 |= 0x80;
						else if (b == "l2") cmd.digital2 |= 0x01;
						else if (b == "r2") cmd.digital2 |= 0x02;
						else if (b == "l1") cmd.digital2 |= 0x04;
						else if (b == "r1") cmd.digital2 |= 0x08;
						else if (b == "triangle") cmd.digital2 |= 0x10;
						else if (b == "circle") cmd.digital2 |= 0x20;
						else if (b == "cross") cmd.digital2 |= 0x40;
						else if (b == "square") cmd.digital2 |= 0x80;
					}

					g_pad_cmds.push_back(cmd);

					// Release after a press
					if (cmd.digital1 || cmd.digital2)
					{
						g_pad_cmds.push_back(pad_cmd{0, 0, 2});
					}
				}
			}

			if (g_pad_cmds.empty())
			{
				return;
			}

			pad_cmd& cmd = g_pad_cmds.front();

			// A connected pad with centered sticks and the scripted buttons
			data->len = 24;
			data->button[CELL_PAD_BTN_OFFSET_DIGITAL1] = cmd.digital1;
			data->button[CELL_PAD_BTN_OFFSET_DIGITAL2] = cmd.digital2;

			for (u32 i = CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X; i <= CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y; i++)
			{
				data->button[i] = 0x80;
			}

			if (cmd.frames <= 1)
			{
				g_pad_cmds.pop_front();
			}
			else
			{
				cmd.frames--;
			}
		}
	}

	void on_pad_get_data_impl(u32 port_no, CellPadData* data);

	// Hybrid frame differential (rb_native.cpp): a frame runs twice from the same state; the second run must read the
	// same controller data as the first (scripted commands are consumed per read)
	namespace native
	{
		int g_pad_mode = 0; // 1 = record the reads, 2 = replay them
		std::vector<std::pair<u32, CellPadData>> g_pad_reads;
		usz g_pad_pos = 0;

		void pad_record()
		{
			g_pad_mode = 1;
			g_pad_reads.clear();
		}

		void pad_replay()
		{
			g_pad_mode = 2;
			g_pad_pos = 0;
		}

		void pad_off()
		{
			g_pad_mode = 0;
		}
	}

	void on_pad_get_data(u32 port_no, CellPadData* data)
	{
		if (native::g_pad_mode == 2)
		{
			if (native::g_pad_pos < native::g_pad_reads.size())
			{
				std::memcpy(data, &native::g_pad_reads[native::g_pad_pos++].second, sizeof(CellPadData));
			}

			return;
		}

		on_pad_get_data_impl(port_no, data);

		if (native::g_pad_mode == 1)
		{
			native::g_pad_reads.emplace_back(port_no, *data);
		}
	}

	void on_pad_get_data_impl(u32 port_no, CellPadData* data)
	{
		ensure_init();

		if (port_no >= max_ports)
		{
			return;
		}

		g_reads[port_no]++;

		if (port_no == 0)
		{
			g_port0_reads++;
		}

		if (port_no == 0 && !g_state.synced && ((g_state.sync_frame && g_state.seq[0] >= g_state.sync_frame) ||
			(g_state.sync_addr && vm::check_addr<4>(g_state.sync_addr, vm::page_readable) && *vm::get_super_ptr<u32>(g_state.sync_addr) != 0u)))
		{
			check_sync_point();
		}

		const u32 seq = g_state.seq[port_no]++;
		const u64 frame = g_frame.load();

		// Online rollback: port 0 (the local player) of re-simulated frames gets the data of the original frame
		static std::map<u32, CellPadData> local_record;

		if (port_no == 0 && online::is_resimulating())
		{
			if (auto found = local_record.find(seq); found != local_record.end())
			{
				std::memcpy(data, &found->second, sizeof(CellPadData));
			}

			return;
		}

		// RB_PAD_CMD=<file>: scripted button presses on port 0 (menu navigation of the test instances)
		if (port_no == 0)
		{
			apply_pad_commands(data);
			std::memcpy(&local_record[seq], data, sizeof(CellPadData));

			while (local_record.size() > 240)
			{
				local_record.erase(local_record.begin());
			}
		}

		// The local controller (port 0 of this instance) before any replacement: netplay live input
		if (port_no == 0)
		{
			std::memcpy(&g_live_pad, data, sizeof(CellPadData));
			g_live_pad_valid = true;
		}

		// Netplay: inputs of both players come from GGPO (any mode)
		if (net::pad_input(port_no, data))
		{
			return;
		}

		switch (g_state.mode)
		{
		case mode_t::record:
		{
			pad_record rec{};
			rec.frame = frame;
			rec.port = port_no;
			rec.seq = seq;
			std::memcpy(rec.data, data, sizeof(CellPadData));
			g_state.input.write(&rec, sizeof(rec));
			break;
		}
		case mode_t::replay:
		{
			const auto& records = g_state.replay[port_no];

			if (seq >= records.size())
			{
				if (!g_state.replay_exhausted[port_no])
				{
					g_state.replay_exhausted[port_no] = true;
					rb_log.warning("Replay for port %u ended at frame %u, input passes through", port_no, frame);
				}

				break;
			}

			u32 use_seq = seq;

			// Frames that will be rolled back later get a predicted port 1 input (repeat of the last one)
			if (u64 predicted_from; port_no == 1 && g_state.seq[0] && state_is_speculative(g_state.seq[0] - 1, predicted_from))
			{
				const u64 back = (g_state.seq[0] - 1) - predicted_from;
				use_seq = seq >= back ? static_cast<u32>(seq - back) : seq;

				if (g_state.predicted_reads++ % 300 == 0)
				{
					rb_log.success("Prediction: port 1 frame %u uses the input of frame %u (%u predicted reads so far)",
						g_state.seq[0] - 1, predicted_from, g_state.predicted_reads);
				}
			}

			const pad_record& rec = records[use_seq];

			if (use_seq == seq && rec.frame != frame && g_state.frame_mismatches++ < 20)
			{
				rb_log.warning("Replay port %u call %u: recorded in frame %u, now in frame %u", port_no, seq, rec.frame, frame);
			}

			std::memcpy(data, rec.data, sizeof(CellPadData));
			break;
		}
		case mode_t::off:
			break;
		}
	}

	// Saving/restoring at the pad read happens in the middle of a frame: the game's task loop keeps its current
	// position in registers and on the stack, which a restore cannot rewind (crashes in the task system when a
	// task is created/removed inside the rollback window). The real frame boundary is the main loop
	// FUN_0010204c: `while (running) FUN_00100250(root)`. FUN_00100250 is replaced by rb_frame_step, which
	// handles the state first and then runs the original task walk:
	//     node = root->first (+0x14); do node = FUN_000ff344(node, 1); while (node != root + 4);
	constexpr u32 tekken_task_walk = 0x00100250;  // FUN_00100250, called only from the main loop
	constexpr u32 tekken_task_step = 0x000ff344;  // FUN_000ff344(task, 1) -> next task
	constexpr u64 tekken_toc = 0x0096A8B0;

	bool g_frame_hook_installed = false;

	void rb_frame_step(ppu_thread& ppu, u32 root);
	const u32 s_frame_step_index = REG_HIDDEN_FUNC(rb_frame_step);

	// RB_ALLOC_LOG=1: copy of the per-frame render allocator FUN_005c0728(alloc, size, u32* out) -> overflow,
	// logging every overflow (an overflow sets RequestManager+4, the buffers are then not swapped at frame end)
	constexpr u32 tekken_frame_alloc = 0x005c0728;

	u32 rb_frame_alloc(ppu_thread& ppu, u32 alloc, u64 size, u32 out)
	{
		const u32 ptr = vm::read32(alloc + 0x14);
		const u32 next = static_cast<u32>(ptr + ((size + 0x7f) & 0xffffff80u));
		const u32 end = vm::read32(alloc + 0x18);
		const bool overflow = end <= next;

		if (overflow)
		{
			const u32 start = vm::read32(alloc + vm::read32(alloc + 0x10) * 4 + 8);
			vm::write32(alloc + 0x14, start);
			vm::write32(out, start);

			static u32 reported = 0;

			if (reported++ < 50)
			{
				rb_log.error("Frame allocator overflow: alloc 0x%08x size 0x%llx ptr 0x%08x end 0x%08x lr 0x%08x game frame %u",
					alloc, size, ptr, end, static_cast<u32>(ppu.lr), g_state.seq[0]);
			}
		}
		else
		{
			std::memset(vm::get_super_ptr<u8>(ptr & ~0x7fu), 0, 0x80); // dcbz
			vm::write32(out, ptr);
			vm::write32(alloc + 0x14, next);
		}

		return overflow;
	}

	const u32 s_frame_alloc_index = REG_HIDDEN_FUNC(rb_frame_alloc);

	// RB_ALLOC_LOG=1 also replaces FUN_005c110c(RequestManager) (frame end: swap of the frame buffers when +4 == 0)
	// with a copy that logs every call: FUN_005c4e68(TOC 0x0096f240 value), FUN_005c06c0(rm+8), FUN_005c0fe4(rm)
	constexpr u32 tekken_frame_swap = 0x005c110c;

	u32 rb_frame_swap(ppu_thread& ppu, u32 rm)
	{
		const u32 flag = vm::read32(rm + 4);
		static u32 logged = 0;

		if (logged < 400 && g_state.synced && g_state.seq[0] >= 4540)
		{
			logged++;
			rb_log.notice("Frame end: game frame %u rm+4=%u idx=%u ptr=0x%08x", g_state.seq[0], flag, +vm::read32(rm + 0x18), +vm::read32(rm + 0x1c));
		}

		if (flag != 0)
		{
			return 0x80004005;
		}

		vm::write32(rm + 0x58, vm::read32(rm + 0x54));
		vm::write32(rm + 0x4c, vm::read32(rm + 0x48));
		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
		ppu.gpr[3] = vm::read32(0x0096f240);
		ppu.fast_call(0x005c4e68, tekken_toc);
		ppu.gpr[3] = rm + 8;
		ppu.fast_call(0x005c06c0, tekken_toc);
		ppu.gpr[3] = rm;
		ppu.fast_call(0x005c0fe4, tekken_toc);
		ppu.gpr[1] = old_r1;
		return 0;
	}

	const u32 s_frame_swap_index = REG_HIDDEN_FUNC(rb_frame_swap);

	// RB_FAST_RESIM=1: FUN_005b39e0(param) = FUN_00600698(0x00D2CA98, *(0x00D2CA00 + 0x138), param) is the end of the
	// main thread frame: wait until the draw thread is idle, swap the per-frame buffers (FUN_005c110c), hand the
	// frame to the draw thread and wait for its progress (vblank paced). Re-simulated frames only swap the buffers:
	// nothing is drawn, nothing waits, so they run at CPU speed. The draw side (frame sync object 0x00D2CA98, not
	// in the snapshot) does not see them at all. Returns the lag (missed vblanks), 0 for re-simulated frames.
	constexpr u32 tekken_frame_end = 0x005b39e0;
	u64 g_boundary_frame = 0; // game frame of the current main loop iteration (after a possible restore)
	// Main loop root of the current frame (FUN_00100250 argument): the global task list runs from *(root + 0x14) to the
	// sentinel root + 4 through +0x10 (next); +0x14 prev, +0x18 parent, +0x1c next sibling, +0x20 prev sibling, +0x24 first child
	u32 g_task_root = 0;

	u64 boundary_frame()
	{
		return g_boundary_frame;
	}

	// RB_DEFER_HANDOFF: the live frame end waiting for the next boundary (see rb_frame_step)
	bool g_defer_pending = false;
	u64 g_defer_param = 0;
	u32 g_defer_req = 0;
	u64 g_defer_run = 0, g_defer_dropped = 0;
	u64 g_handoff_us = 0; // time in the deferred hand-off since the last read (RB_SLOW_FRAMES)

	bool defer_handoff()
	{
		static const bool on = get_env("RB_DEFER_HANDOFF") == "1";
		return on;
	}

	u32 rb_frame_end(ppu_thread& ppu, u64 param)
	{
		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
		u32 result = 0;

		// RB_CATCHUP=1: in the online fight the game keeps 59.94 simulated frames per second of wall clock time. When a
		// frame took too long (a rollback re-simulated several frames inside it), the next live frames (at most 2 in a
		// row) are not drawn and do not wait for vblank, like re-simulated frames: the simulation catches up, the
		// display skips frames (ANALYZA.md 49; the game's own catch-up would run several frames in one main loop iteration).
		static const bool catchup = get_env("RB_CATCHUP") == "1";
		static std::chrono::steady_clock::time_point catchup_start{};
		static u64 catchup_frames = 0, catchup_skipped = 0;
		static u32 catchup_row = 0;
		bool skip_draw = false;

		if (catchup && !state_resimulating(g_boundary_frame))
		{
			if (!online::fight_active())
			{
				catchup_start = {};
			}
			else
			{
				const auto now = std::chrono::steady_clock::now();

				if (catchup_start == std::chrono::steady_clock::time_point{})
				{
					catchup_start = now;
					catchup_frames = 0;
				}

				const double due = std::chrono::duration<double>(now - catchup_start).count() * 59.94;
				const double debt = due - static_cast<double>(catchup_frames);

				if (debt > 30)
				{
					// A pause (stall, round transition): start counting again
					catchup_start = now;
					catchup_frames = 0;
				}
				else if (debt >= 1.0 && catchup_row < 2)
				{
					skip_draw = true;
				}

				catchup_frames++;
				catchup_row = skip_draw ? catchup_row + 1 : 0;

				if (skip_draw && ++catchup_skipped % 600 == 1)
				{
					rb_log.notice("Catch-up: %u live frames not drawn so far (debt %.1f frames)", catchup_skipped, debt);
				}
			}
		}

		// RB_RESIM_DISCARD (default on, 0 = off): a re-simulated frame's draw requests are discarded (FUN_005c10b4: reset of
		// the frame allocator, nothing handed to the draw thread) instead of swapped (FUN_005c110c -> FUN_005c4e68), so the
		// draw thread gets no work and the next boundary does not wait for it (o146: ~3 ms per re-simulated frame)
		static const bool discard = get_env("RB_RESIM_DISCARD") != "0";

		if (state_resimulating(g_boundary_frame) && discard)
		{
			ppu.gpr[3] = 0x00C89800; // NU::Draw::RequestManager
			ppu.fast_call(0x005c10b4, tekken_toc);
		}
		else if (state_resimulating(g_boundary_frame) || skip_draw)
		{
			ppu.gpr[3] = 0x00C89800; // NU::Draw::RequestManager
			ppu.fast_call(0x005c110c, tekken_toc);
		}
		else if (defer_handoff() && online::fight_active())
		{
			// Run at the next boundary (after the save / a rollback); the lag result is only used with param 0
			g_defer_pending = true;
			g_defer_param = param;
			g_defer_req = vm::read32(0x00D2CA00 + 0x138);
		}
		else
		{
			ppu.gpr[3] = 0x00D2CA98;
			ppu.gpr[4] = vm::read32(0x00D2CA00 + 0x138);
			ppu.gpr[5] = param;
			ppu.fast_call(0x00600698, tekken_toc);
			result = static_cast<u32>(ppu.gpr[3]);
		}

		ppu.gpr[1] = old_r1;
		return result;
	}

	const u32 s_frame_end_index = REG_HIDDEN_FUNC(rb_frame_end);

	// RB_TASK_PROFILE=1: copy of the sub task walk FUN_000ff510(owner, first, end) that measures every task
	// (exclusive time: nested walks are subtracted from their parent task), grouped by the task method (+0x30)
	constexpr u32 tekken_subtask_walk = 0x000ff510;

	struct task_time
	{
		u64 ns = 0, calls = 0;
	};

	std::map<u32, task_time> g_task_prof[2];
	u64 g_task_prof_frames[2]{};
	std::vector<u64> g_task_child_ns; // per nesting level: time spent in nested tasks

	// RB_RESIM_SKIP=<hex vtables, comma separated>: in a re-simulated frame these tasks (pure rendering: draw tree, HUD,
	// effects; ANALYZA.md 46) are not run in their steady state (step method 0x0092CC00 = vtable slot 6): the walk
	// continues as if the method returned 0 (FUN_000ff344: flags |= 8, +0x50 counter, next node; flag 0x80 = past the
	// subtree via FUN_000ff2b0). Returns the next node, 0 = run the task normally.
	const std::set<u32>& resim_skip_set()
	{
		static const std::set<u32> skip = []()
		{
			std::set<u32> s;

			for (const std::string& item : fmt::split(get_env("RB_RESIM_SKIP"), {","}))
			{
				if (!item.empty())
				{
					s.insert(static_cast<u32>(std::strtoul(item.c_str(), nullptr, 16)));
				}
			}

			return s;
		}();

		return skip;
	}

	// Task listed in RB_RESIM_SKIP (by the vtable of its object)
	bool resim_skip_listed(u32 node)
	{
		const auto& skip = resim_skip_set();
		return !skip.empty() && skip.contains(vm::read32(node + vm::read32(node + 0x34)));
	}

	// Cosmetic random numbers: the work left out of re-simulated frames (draw pass, RB_RESIM_SKIP tasks: crowd, snow,
	// particles) uses the same generators as the fight logic (CRT rand: TLS r13-0x7000..r13-0x6f24 of the main thread;
	// the game's LCG at 0x00D142AC). Leaving it out would change the logic's random numbers. So it always runs with its
	// own generator state (kept outside guest memory): the logic's state is put aside before and restored after, and the
	// logic never sees the cosmetic draws, live or re-simulated - the same rule on both PCs keeps them in sync.
	class cosmetic_rng_scope
	{
	public:
		static inline u32 s_depth = 0;

	private:
		static inline bool s_init = false;
		static inline std::array<u8, 0xdc> s_rand{};
		static inline u32 s_lcg = 0;

		std::array<u8, 0xdc> m_logic{};
		u32 m_lcg = 0;
		u32 m_base = 0;
		bool m_active = false;

	public:
		explicit cosmetic_rng_scope(ppu_thread& ppu)
		{
			m_base = static_cast<u32>(ppu.gpr[13]) - 0x7000;

			// From the fight's sync point on (no rollbacks before it; the menus keep the original behaviour)
			if (s_depth++ || !g_state.synced || !vm::check_addr(m_base, vm::page_readable | vm::page_writable, 0xdc))
			{
				return;
			}

			m_active = true;
			u8* tls = vm::get_super_ptr<u8>(m_base);
			std::memcpy(m_logic.data(), tls, 0xdc);
			m_lcg = vm::read32(0x00D142AC);

			if (!s_init)
			{
				s_init = true;
				s_rand = m_logic;
				s_lcg = m_lcg ^ 0x5bd1e995;
			}

			std::memcpy(tls, s_rand.data(), 0xdc);
			vm::write32(0x00D142AC, s_lcg);
		}

		~cosmetic_rng_scope()
		{
			s_depth--;

			if (!m_active)
			{
				return;
			}

			u8* tls = vm::get_super_ptr<u8>(m_base);
			std::memcpy(s_rand.data(), tls, 0xdc);
			s_lcg = vm::read32(0x00D142AC);
			std::memcpy(tls, m_logic.data(), 0xdc);
			vm::write32(0x00D142AC, m_lcg);
		}
	};

	// RB_ONLINE_EARLY_SEND (default on, 0 = off): the local input of a live frame is sent right after tcb_device_input_t
	// (vtable 0x00909658) added it, instead of by the network manager tnOnlineBattle_Manager (vtable 0x0091E138, sender
	// FUN_00422ce0) in its own turn - one frame less input latency when the manager runs before the input task
	// (ANALYZA.md 48). The sender is idempotent (it sends the window of the last <= 6 local frames) and reads only local
	// inputs; never during a re-simulation (no network side effects).
	u32 g_net_mgr_node = 0, g_net_mgr_serial = 0;

	// The network manager object (tnOnlineBattle_Manager) seen in the task walk, 0 if gone
	u32 net_manager_object()
	{
		if (!g_net_mgr_node || vm::read32(g_net_mgr_node + 0xc) != g_net_mgr_serial)
		{
			return 0;
		}

		const u32 mgr = g_net_mgr_node + vm::read32(g_net_mgr_node + 0x34);
		return vm::read32(mgr) == 0x0091E138u ? mgr : 0;
	}

	void after_task_step(ppu_thread& ppu, u32 node, u32 vtable, bool resim)
	{
		static const bool on = get_env("RB_ONLINE_EARLY_SEND") != "0";

		if (!on)
		{
			return;
		}

		if (vtable == 0x0091E138u)
		{
			g_net_mgr_node = node;
			g_net_mgr_serial = vm::read32(node + 0xc);
			return;
		}

		if (vtable != 0x00909658u || resim || !g_net_mgr_node || vm::read32(g_net_mgr_node + 0xc) != g_net_mgr_serial)
		{
			return;
		}

		const u32 mgr = g_net_mgr_node + vm::read32(g_net_mgr_node + 0x34);

		if (vm::read32(mgr) != 0x0091E138u || vm::read32(mgr + 0x5c) == 0)
		{
			return;
		}

		ppu.fast_call(0x00425f04, tekken_toc);
		const u32 session = static_cast<u32>(ppu.gpr[3]);

		if (!session)
		{
			return;
		}

		ppu.gpr[3] = session;
		ppu.fast_call(0x004240c0, tekken_toc);

		if (!static_cast<u32>(ppu.gpr[3]))
		{
			return;
		}

		ppu.gpr[3] = session;
		ppu.fast_call(0x00424148, tekken_toc);
		const u32 state = static_cast<u32>(ppu.gpr[3]);

		if (state && vm::read32(state + 0xc) == 1)
		{
			ppu.gpr[3] = mgr;
			ppu.fast_call(0x00422ce0, tekken_toc);

			if (static u64 n = 0; n++ == 0)
			{
				rb_log.success("Early send: local input sent right after the input task (manager 0x%08x)", mgr);
			}
		}
	}

	bool in_cosmetic_work()
	{
		return cosmetic_rng_scope::s_depth != 0;
	}

	u32 try_skip_task(ppu_thread& ppu, u32 node, bool resim)
	{
		const auto& skip = resim_skip_set();

		if (!resim || skip.empty())
		{
			return 0;
		}

		const u32 obj = node + vm::read32(node + 0x34);

		if (!skip.contains(vm::read32(obj)))
		{
			return 0;
		}

		// Steady state: the generic step (descriptor 0x0092CC00) or, for the draw root, its state FUN_001060d8 (+0x60)
		const u32 step = vm::read32(node + 0x30);

		if (static u32 logged = 0; logged < 8)
		{
			logged++;
			rb_log.notice("Resim skip candidate: node 0x%08x vtable 0x%08x step 0x%08x obj+0x60 0x%08x flags 0x%x", node, vm::read32(obj), step,
				vm::read32(obj + 0x60), vm::read32(node + 0x44));
		}

		// Steady state: the generic step (descriptor 0x0092CC00 -> FUN_00017534, or 0x00932BE8 -> FUN_000ff1a0 = "call the
		// object's update vtable+0x18", the steady step of e.g. CINetMatching, tcb_owwindow_t, tcb_stage_particle_t: without it
		// these listed tasks were never skipped, o209), the draw root's state FUN_001060d8 (descriptor 0x00933148), or the
		// sound device's state FUN_002c69d8 (sound library update per frame; the library is not rolled back, re-simulated
		// frames must not advance it)
		const u32 state = vm::read32(obj + 0x60);
		const bool steady = step == 0x0092CC00u || step == 0x00932BE8u || state == 0x00933148u ||
			(!(state & 1) && state && vm::check_addr(state, vm::page_readable) && vm::read32(state) == 0x002c69d8u);

		if (!steady)
		{
			return 0;
		}

		const u32 flags = vm::read32(node + 0x44);
		vm::write32(node + 0x44, flags | 8);
		vm::write32(node + 0x50, vm::read32(node + 0x50) + 1);

		if (!(flags & 0x80))
		{
			return vm::read32(node + 0x10);
		}

		ppu.gpr[3] = node;
		ppu.fast_call(0x000ff2b0, tekken_toc);
		return vm::read32(static_cast<u32>(ppu.gpr[3]) + 0x10);
	}

	u32 g_task_method_now = 0; // task method (vtable) whose step runs now (diagnostics)

	u32 current_task_method()
	{
		return g_task_method_now;
	}

	void rb_subtask_walk(ppu_thread& ppu, u64 owner, u32 node, u32 end)
	{
		// A task list that leads back into a subtree whose walk is still running recurses until the guest stack is gone
		// (online 2026-10-04: access violation at the stack bottom 0xd0000fa0 under FUN_00176ac0 -> FUN_001001c0).
		// Such a walk is skipped instead when the guest stack is almost used up. (The same subtree can legitimately be
		// walked again from a nested main loop frame during a transition, so a repeated start node is not a cycle.)
		if (static_cast<u32>(ppu.gpr[1]) < ppu.stack_addr + 0x4000)
		{
			if (static u32 logged = 0; logged++ < 20)
			{
				rb_log.error("Sub task walk from node 0x%08x to 0x%08x skipped: guest stack almost full (r1 0x%08x, resim %d, LR 0x%08x)", node, end,
					static_cast<u32>(ppu.gpr[1]), state_resimulating(g_boundary_frame), static_cast<u32>(ppu.lr));
			}

			return;
		}

		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);

		ppu.gpr[3] = owner;
		ppu.fast_call(0x000ff4e8, tekken_toc);
		const u32 mask = static_cast<u32>(ppu.gpr[3]);
		const bool resim = state_resimulating(g_boundary_frame);
		u32 prev = 0;

		while (!ppu.is_stopped())
		{
			u32 cur = node;

			while (true)
			{
				u32 flags = vm::read32(cur + 0x44);

				if (cur != prev)
				{
					flags = ((flags >> 4) << 4) | (flags & 7);
					vm::write32(cur + 0x44, flags);
				}

				prev = cur;

				if ((mask & flags) == 0)
				{
					break;
				}

				cur = vm::read32(cur + 0x10);

				if (cur == end)
				{
					ppu.gpr[1] = old_r1;
					return;
				}
			}

			if (const u32 next = try_skip_task(ppu, cur, resim))
			{
				node = next;

				if (node == end)
				{
					break;
				}

				continue;
			}

			const u32 method = vm::read32(cur + vm::read32(cur + 0x34)); // vtable of the task object
			g_task_child_ns.push_back(0);
			const auto t0 = std::chrono::steady_clock::now();

			{
				std::optional<cosmetic_rng_scope> cosmetic;

				if (resim_skip_listed(cur))
				{
					cosmetic.emplace(ppu);
				}

				const bool sound_device = method == 0x00910798u && !resim;

				if (sound_device)
				{
					online::sound_learn_begin();
				}

				const u32 outer_method = g_task_method_now;
				g_task_method_now = method;
				ppu.gpr[3] = cur;
				ppu.gpr[4] = 1;

				// Hybrid: nested task steps in the lifted code too (RB_NATIVE_SUBTASK=0: in RPCS3)
				static const bool native_subtask = get_env("RB_NATIVE_SUBTASK") != "0";

				if (!native_subtask || !native::call(ppu, tekken_task_step))
				{
					ppu.fast_call(tekken_task_step, tekken_toc);
				}

				g_task_method_now = outer_method;

				if (sound_device)
				{
					online::sound_learn_end();
				}
			}

			node = static_cast<u32>(ppu.gpr[3]);
			after_task_step(ppu, cur, method, resim);

			const u64 total = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
			const u64 nested = g_task_child_ns.back();
			g_task_child_ns.pop_back();

			if (!g_task_child_ns.empty())
			{
				g_task_child_ns.back() += total;
			}

			if (g_state.synced)
			{
				auto& t = g_task_prof[resim][method];
				t.ns += total - std::min(total, nested);
				t.calls++;
			}

			if (node == end)
			{
				break;
			}
		}

		ppu.gpr[1] = old_r1;
	}

	const u32 s_subtask_walk_index = REG_HIDDEN_FUNC(rb_subtask_walk);

	// RB_RESIM_FILM=0 (with RB_RESIM_DRAWROOT=1): the match recording ("film", ~2.6 ms of every re-simulated frame in o118)
	// is not recorded in re-simulated frames and stays outside the rollback: its memory is copied before a snapshot is
	// loaded and written back after it, so it keeps the live frames' records and a consistent ring (only the KO replay
	// reads it; it may show a predicted motion for a few frames). Memory: the film module's static block (TOC 0x00969744,
	// up to +0xaa10), the film object (*(block+4), 0x40 bytes) and its ring (object 0x40 bytes, frame headers +0x28 with
	// count*0x30 bytes, data buffer +0x2c with +0x34 bytes; FUN_0029ccc8 / FUN_0029d100).
	bool film_kept()
	{
		static const bool kept = get_env("RB_RESIM_DRAWROOT") == "1" && get_env("RB_RESIM_FILM") == "0";
		return kept;
	}

	std::vector<std::pair<u32, std::vector<u8>>> g_film_copy;

	void film_save()
	{
		g_film_copy.clear();

		if (!film_kept())
		{
			return;
		}

		// Only the parts on pages the snapshot restores (the data buffer is ~9.7 MB, mostly untouched between rollbacks)
		const auto add = [](u32 addr, u32 size)
		{
			if (!addr || !size || size >= 0x2000000 || !vm::check_addr(addr, vm::page_readable, size))
			{
				return;
			}

			for (u32 page = addr & ~0xfffu; page < addr + size; page += 0x1000)
			{
				if (state_addr_saved(page) != 1)
				{
					continue;
				}

				const u32 begin = std::max(page, addr);
				const u32 end = std::min(page + 0x1000, addr + size);
				const u8* src = vm::get_super_ptr<u8>(begin);

				if (!g_film_copy.empty() && g_film_copy.back().first + g_film_copy.back().second.size() == begin)
				{
					g_film_copy.back().second.insert(g_film_copy.back().second.end(), src, src + (end - begin));
				}
				else
				{
					g_film_copy.emplace_back(begin, std::vector<u8>(src, src + (end - begin)));
				}
			}
		};

		const u32 block = vm::read32(0x00969744);

		if (!block || !vm::check_addr(block, vm::page_readable, 0xaa10))
		{
			return;
		}

		add(block, 0xaa10);
		const u32 film = vm::read32(block + 4);

		if (!film || !vm::check_addr(film, vm::page_readable, 0x40))
		{
			return;
		}

		add(film, 0x40);
		const u32 ring = vm::read32(film);

		if (!ring || !vm::check_addr(ring, vm::page_readable, 0x40))
		{
			return;
		}

		add(ring, 0x40);
		add(vm::read32(ring + 0x28), vm::read32(ring) * 0x30);
		add(vm::read32(ring + 0x2c), vm::read32(ring + 0x34));

		if (static bool logged = false; !logged)
		{
			logged = true;
			usz total = 0;

			for (const auto& [a, data] : g_film_copy)
			{
				total += data.size();
			}

			rb_log.success("Film kept outside the rollback: block 0x%08x, film 0x%08x, ring 0x%08x (%u frames), data 0x%08x+0x%x, %u KiB copied at the first rollback",
				block, film, ring, vm::read32(ring), vm::read32(ring + 0x2c), vm::read32(ring + 0x34), total / 1024);
		}
	}

	// RB_FILM_UNDO=1: the film data ring (the match recording, ~9.7 MB, ~31 KiB per frame written linearly) stays out of the
	// snapshot and the write tracking (it cost a write fault, an add to all slots and a cooling cycle for ~8 pages every frame).
	// The ring object, frame headers and the film block stay in the snapshot, so a load returns the write offset and headers;
	// the re-simulation records the frames again from there. What the discarded frames wrote is undone from an undo log:
	// at every boundary (live and re-simulated, after a possible load) the bytes the frame may write, [wp, wp + window) with
	// the wrap, are copied; a load to frame S writes the entries of frames >= S back, newest first.
	// Ring (ghidra/film_ring.c): +0x2c data, +0x30 write offset (16 aligned; FUN_0029d100 restarts at 0 when a record does
	// not fit), +0x34 data size.
	struct film_undo_entry
	{
		u64 frame;
		u32 wp;
		u32 window;
		std::vector<u8> head; // [wp, min(wp + window, size))
		std::vector<u8> wrap; // [0, window) when wp + window > size
	};

	std::deque<film_undo_entry> g_film_undo;
	u32 g_film_data = 0, g_film_size = 0;
	u32 g_film_window = 0x10000;
	bool g_film_undo_broken = false; // a frame wrote more than the window: older entries cannot restore it

	bool film_undo_on()
	{
		static const bool on = get_env("RB_FILM_UNDO") == "1";
		return on;
	}

	u32 film_ring()
	{
		const u32 block = vm::read32(0x00969744);

		if (!block || !vm::check_addr(block, vm::page_readable, 8))
		{
			return 0;
		}

		const u32 film = vm::read32(block + 4);

		if (!film || !vm::check_addr(film, vm::page_readable, 4))
		{
			return 0;
		}

		const u32 ring = vm::read32(film);
		return ring && vm::check_addr(ring, vm::page_readable, 0x40) ? ring : 0;
	}

	// Main thread, every boundary after the snapshot module (load / save) of the frame that is about to run
	void film_undo_boundary(u64 frame)
	{
		if (!film_undo_on())
		{
			return;
		}

		const u32 ring = film_ring();
		const u32 data = ring ? static_cast<u32>(vm::read32(ring + 0x2c)) : 0;
		const u32 size = ring ? static_cast<u32>(vm::read32(ring + 0x34)) : 0;
		const bool valid = data && size && size < 0x2000000 && vm::check_addr(data, vm::page_readable | vm::page_writable, size);

		// The buffer follows the film (a new match allocates a new one)
		if (!valid || data != g_film_data || size != g_film_size)
		{
			if (g_film_data)
			{
				state_untracked_range(g_film_data, g_film_data + g_film_size, false, "film data");
			}

			g_film_undo.clear();
			g_film_undo_broken = false;
			g_film_data = valid ? data : 0;
			g_film_size = valid ? size : 0;

			if (!valid)
			{
				return;
			}

			state_untracked_range(data, data + size, true, "film data");
			rb_log.success("Film undo: data 0x%08x+0x%x out of the snapshot (ring 0x%08x)", data, size, ring);
		}

		const u32 wp = vm::read32(ring + 0x30);

		if (wp >= size)
		{
			return;
		}

		// Bytes written by the previous frame (with the wrap); the window grows with what frames really write
		if (!g_film_undo.empty() && g_film_undo.back().frame + 1 == frame)
		{
			const film_undo_entry& prev = g_film_undo.back();
			const u32 written = wp >= prev.wp ? wp - prev.wp : (size - prev.wp) + wp;

			if (written > prev.window)
			{
				if (static u32 logged = 0; logged++ < 20)
				{
					rb_log.error("Film undo: frame %u wrote 0x%x bytes, more than the window 0x%x (a rollback over it cannot restore the film exactly)",
						prev.frame, written, prev.window);
				}

				g_film_undo_broken = true;
			}

			while (g_film_window < std::min<u32>(written * 2, 0x100000))
			{
				g_film_window *= 2;
			}
		}

		// A frame that runs again (after a load) replaces its entry and those after it
		while (!g_film_undo.empty() && g_film_undo.back().frame >= frame)
		{
			g_film_undo.pop_back();
		}

		film_undo_entry e{frame, wp, g_film_window, {}, {}};
		const u8* base = vm::get_super_ptr<u8>(data);
		const u32 head = std::min<u32>(g_film_window, size - wp);
		e.head.assign(base + wp, base + wp + head);

		if (head < g_film_window)
		{
			const u32 w = std::min<u32>(g_film_window, size);
			e.wrap.assign(base, base + w);
		}

		g_film_undo.push_back(std::move(e));

		while (g_film_undo.size() > 64)
		{
			g_film_undo.pop_front();
		}
	}

	// Right after a snapshot load to frame `target` (main thread)
	void film_undo_after_load(u64 target)
	{
		if (!film_undo_on() || !g_film_data)
		{
			return;
		}

		u8* base = vm::get_super_ptr<u8>(g_film_data);
		u32 undone = 0;

		while (!g_film_undo.empty() && g_film_undo.back().frame >= target)
		{
			const film_undo_entry& e = g_film_undo.back();

			// Wrap part first: the head part of the same entry is older in the ring order
			if (!e.wrap.empty())
			{
				std::memcpy(base, e.wrap.data(), e.wrap.size());
			}

			std::memcpy(base + e.wp, e.head.data(), e.head.size());
			g_film_undo.pop_back();
			undone++;
		}

		if (g_film_undo_broken)
		{
			if (static u32 logged = 0; logged++ < 20)
			{
				rb_log.error("Film undo: load to frame %u after a frame that wrote more than the window; the film may hold records of the discarded frames", target);
			}

			g_film_undo_broken = false;
		}

		if (static u64 loads = 0; ++loads % 500 == 1)
		{
			rb_log.notice("Film undo: load to frame %u undid %u frames (window 0x%x, %u entries)", target, undone, g_film_window, g_film_undo.size());
		}
	}

	void film_restore()
	{
		for (const auto& [addr, data] : g_film_copy)
		{
			std::memcpy(vm::get_super_ptr<u8>(addr), data.data(), data.size());
			note_write(addr, static_cast<u32>(data.size()));
		}

		g_film_copy.clear();
	}

	// RB_RESIM_DRAWROOT=1: HLE copy of FUN_000ffdec(mgr, list, sorted, filter, lo, hi), the draw pass (list idx of
	// mgr+0x70, 0xc bytes each; item vtable [0] = priority, [1] = draw(item, ctx)). In a re-simulated frame only the item
	// whose draw records the match ("film", FUN_00106e8c -> FUN_00104d50; the KO replay reads it, a missing record
	// crashed at the KO in stress pe) runs; everything else is drawing of a frame nobody sees (~1.9 ms).
	constexpr u32 tekken_draw_list = 0x000ffdec;
	std::map<u32, std::pair<u64, u64>> g_draw_item_prof; // draw method code -> (ns, calls), live frames

	void call_opd(ppu_thread& ppu, u32 opd)
	{
		ppu.fast_call(vm::read32(opd), vm::read32(opd + 4));
	}

	// Game function of the draw root: in RPCS3 (the drawing code calls cellGcm constantly; in the hybrid each of those
	// calls is a host transition: the lifted draw tree was slower, o262 live tasks 8.3 vs 6.4 ms)
	void call_game(ppu_thread& ppu, u32 addr)
	{
		ppu.fast_call(addr, tekken_toc);
	}

	u32 g_last_game_mode = umax; // game mode (FUN_0020c754) seen by the last re-simulated draw pass

	u32 last_game_mode()
	{
		return g_last_game_mode;
	}

	// RB_RESIM_FILM_ONLY (default on, 0 = off): in a re-simulated frame the drawing main item only records the match
	bool film_only()
	{
		static const bool on = get_env("RB_RESIM_FILM_ONLY") != "0";
		return on;
	}

	void rb_draw_list(ppu_thread& ppu, u32 mgr, u32 idx, u32 sorted, u32 filter, u32 lo, u32 hi)
	{
		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);

		static const bool profile = get_env("RB_TASK_PROFILE") == "1";
		bool resim = state_resimulating(g_boundary_frame);

		// From the KO on (game mode FUN_0020c754() >= 6: KO camera, replay) every item is drawn in re-simulated frames too:
		// items left out there left stale state for the KO scene (o128: crash at 0x005c9fa0 at a KO)
		if (resim)
		{
			ppu.fast_call(0x0020c754, tekken_toc);
			const u32 mode = static_cast<u32>(ppu.gpr[3]);
			g_last_game_mode = mode;

			if (static std::set<u32> seen; seen.size() < 16 && seen.insert(mode).second)
			{
				rb_log.notice("Draw pass in a re-simulated frame: game mode %u", mode);
			}

			// RB_RESIM_FULL_DRAW_MODES: game modes drawn in full in re-simulated frames (default: every mode >= 6; o140 with
			// only 6: desync at the next round start, mode 10 needs it too)
			static const std::set<u32> full_modes = []()
			{
				std::set<u32> m;

				for (const std::string& item : fmt::split(get_env("RB_RESIM_FULL_DRAW_MODES"), {","}))
				{
					if (!item.empty())
					{
						m.insert(static_cast<u32>(std::atoi(item.c_str())));
					}
				}

				return m;
			}();

			if (full_modes.empty() ? mode >= 6 : full_modes.contains(mode))
			{
				resim = false;
			}
		}

		const u32 ctx = vm::read32(mgr + 0x48) & 0x50005400u;
		const u32 list = vm::read32(mgr + 0x70) + idx * 0xc;
		static u64 part_ns[2][4]{}; // sort, begin, loop, end (draw root pass only: the list of 0x00A77C34)
		const bool main_pass = mgr == vm::read32(0x00A77C34);
		auto tp = std::chrono::steady_clock::now();
		const auto part = [&](u32 i)
		{
			const auto now = std::chrono::steady_clock::now();
			if (main_pass)
				part_ns[resim][i] += std::chrono::duration_cast<std::chrono::nanoseconds>(now - tp).count();
			tp = now;
		};

		if (sorted)
		{
			ppu.gpr[3] = mgr;
			ppu.gpr[4] = list;
			ppu.fast_call(0x000ffab4, tekken_toc);
		}

		part(0);
		ppu.gpr[3] = mgr;
		ppu.fast_call(0x001518c8, tekken_toc);
		part(1);

		const u32 off = sorted ? 4 : 8;

		for (u32 it = vm::read32(list + off); it != list && !ppu.is_stopped();)
		{
			const u32 next = vm::read32(it + off);
			const u32 vt = vm::read32(it);

			// The priority only matters with the filter (the draw root pass has none); not asked in a re-simulated frame
			u32 prio = 0;

			if (!resim || (filter & 0xff))
			{
				ppu.gpr[3] = it;
				call_opd(ppu, vm::read32(vt));
				prio = static_cast<u32>(ppu.gpr[3]);
			}

			if ((filter & 0xff) == 0 || (lo <= prio && prio <= hi))
			{
				const u32 draw = vm::read32(vt + 4);
				// The generic item draw FUN_001002dc calls the class's draw method (vtable +0x20); the match recording's
				// class has descriptor 0x00933180 (FUN_00106e8c -> FUN_00104d50) there
				const u32 class_draw = static_cast<u32>(vm::read32(vt + 0x20));
				const u32 draw_code = static_cast<u32>(vm::read32(draw));
				const u32 code = draw_code == 0x001002dcu ? class_draw : draw_code;

				if (resim && class_draw == 0x00933180u && draw_code == 0x001002dcu && film_only())
				{
					// Copy of the generic item draw FUN_001002dc(item, ctx) whose class draw (FUN_00106e8c: camera, scene,
					// then the match recording FUN_00104d50) is replaced by the recording alone
					const u32 flags = vm::read32(it + 0x44);
					u32 rest = flags & 0xf;
					bool drawn = false;

					if ((ctx & flags) == 0)
					{
						ppu.gpr[3] = it;
						call_opd(ppu, vm::read32(vt + 0x1c));
						const bool visible = static_cast<u8>(ppu.gpr[3]) != 0;
						const u32 f = vm::read32(it + 0x44);

						if (visible && (f & 2) && (f & 0x20))
						{
							ppu.gpr[3] = it;
							ppu.fast_call(0x00104d50, tekken_toc);
							rest = vm::read32(it + 0x44) | 0x10;
							vm::write32(it + 0x44, rest);
							drawn = true;
						}
						else
						{
							rest = f & 0xf;
							vm::write32(it + 0x44, f & ~0x10u);
						}
					}
					else
					{
						vm::write32(it + 0x44, flags & ~0x10u);
					}

					if (rest & 4)
					{
						ppu.gpr[3] = it;
						ppu.fast_call(0x000ff1ec, tekken_toc);
					}

					static_cast<void>(drawn);
				}
				else if (!resim || class_draw == 0x00933180u)
				{
					const auto t0 = std::chrono::steady_clock::now();
					ppu.gpr[3] = it;
					ppu.gpr[4] = ctx;
					call_opd(ppu, draw);

					if (profile && g_state.synced && main_pass)
					{
						auto& e = g_draw_item_prof[code + resim];
						e.first += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
						e.second++;
					}
				}
			}

			it = next;
		}

		part(2);
		ppu.gpr[3] = mgr;
		ppu.fast_call(0x001518a4, tekken_toc);
		part(3);

		if (profile && g_state.synced && main_pass)
		{
			static u64 passes = 0;

			if (++passes % 500 == 0)
			{
				for (u32 r = 0; r < 2; r++)
				{
					rb_log.notice("Draw pass parts (%s, sum over 500 passes): sort %u us, begin %u us, loop %u us, end %u us", r ? "re-simulated" : "live",
						part_ns[r][0] / 1000, part_ns[r][1] / 1000, part_ns[r][2] / 1000, part_ns[r][3] / 1000);
				}

				std::memset(part_ns, 0, sizeof(part_ns));
				std::vector<std::pair<u32, std::pair<u64, u64>>> sorted_items(g_draw_item_prof.begin(), g_draw_item_prof.end());
				std::sort(sorted_items.begin(), sorted_items.end(), [](const auto& a, const auto& b) { return a.second.first > b.second.first; });
				std::string text;

				for (usz i = 0; i < std::min<usz>(sorted_items.size(), 15); i++)
				{
					fmt::append(text, " [0x%08x %u us %.1f calls]", sorted_items[i].first, sorted_items[i].second.first / 500 / 1000, sorted_items[i].second.second / 500.0);
				}

				rb_log.notice("Draw pass items (class draw descriptor, +1 = re-simulated; per pass):%s", text);
				g_draw_item_prof.clear();
			}
		}

		ppu.gpr[1] = old_r1;
	}

	const u32 s_draw_list_index = REG_HIDDEN_FUNC(rb_draw_list);

	// RB_RESIM_DRAWROOT=1: HLE copy of FUN_001060d8, the steady state of tcb_drawroot_t (vtable 0x00908E40; ANALYZA.md 50).
	// In a re-simulated frame only the draw subtree (FUN_001001c0: scene, models, effects, HUD) is left out; the
	// match recording ("film" FUN_00104d50, read by the KO replay) always runs. Skipping the whole draw root froze the KO (o116).
	constexpr u32 tekken_drawroot_state = 0x001060d8;

	// Host time of the draw root state per part (RB_TASK_PROFILE=1): [resim][subtree, film, empty scene, calls]
	u64 g_drawroot_prof[2][4]{};

	u32 rb_drawroot_state(ppu_thread& ppu, u32 self)
	{
		const bool prof_resim = state_resimulating(g_boundary_frame);
		auto t = std::chrono::steady_clock::now();
		const auto lap = [&](u32 part)
		{
			const auto now = std::chrono::steady_clock::now();
			g_drawroot_prof[prof_resim][part] += std::chrono::duration_cast<std::chrono::nanoseconds>(now - t).count();
			t = now;
		};
		g_drawroot_prof[prof_resim][3]++;
		const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
		vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
		const u32 mask = vm::read32(0x0083D85C);

		vm::write32(self + 0xdc, 1);

		if (!state_resimulating(g_boundary_frame))
		{
			ppu.gpr[3] = self;
			call_game(ppu, 0x001001c0);
		}

		lap(0);

		// Weak reference (object, serial number): valid while the object still has that serial and is active
		const auto weak = [&](u32 ref, u32 off) -> u32
		{
			const u32 o = vm::read32(ref + off);
			return (o && vm::read32(o + 0xc) == vm::read32(ref + off + 4) && (vm::read32(o + 0x44) & mask)) ? o : 0;
		};

		const u32 scene = weak(0x00A78C44, 0x10);

		// (the original reads address 0x10c without a scene as well)
		// Normal fight frames take this branch: FUN_000ffff8 = the draw pass (~1.9 ms, stress pc); rb_draw_list runs only
		// its match recording item in a re-simulated frame.
		if (vm::read32(scene + 0x10c) == 0)
		{
			{
				// Draw items left out of re-simulated frames: their random numbers must not reach the logic (see above)
				cosmetic_rng_scope cosmetic(ppu);
				ppu.gpr[3] = vm::read32(0x00A77C34);
				ppu.gpr[4] = 0;
				ppu.gpr[5] = 1;
				call_game(ppu, 0x000ffff8);
			}

			lap(2);
		}
		else if (!(film_kept() && state_resimulating(g_boundary_frame)))
		{
			ppu.gpr[3] = weak(0x00A78C60, 0x2c);
			call_game(ppu, 0x00104d50);
			lap(1);
		}

		ppu.gpr[1] = old_r1;
		return 0;
	}

	const u32 s_drawroot_state_index = REG_HIDDEN_FUNC(rb_drawroot_state);

	void frame_boundary()
	{
		if (!g_state.synced)
		{
			return;
		}

		// RB_SLOW_FRAMES="from:every:ms": artificial host delay at the frame boundary, to test whether slow frames
		// (snapshot cost) change the simulation
		static const auto slow = []() -> std::array<u64, 3>
		{
			const char* v = std::getenv("RB_SLOW_FRAMES");
			std::array<u64, 3> r{};
			if (v && std::sscanf(v, "%llu:%llu:%llu", &r[0], &r[1], &r[2]) != 3) r = {};
			return r;
		}();

		if (slow[2] && g_state.seq[0] >= slow[0] && (g_state.seq[0] - slow[0]) % std::max<u64>(slow[1], 1) == 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(slow[2]));
		}

		if (snapshot_info restored; state_on_frame_start(g_state.seq[0], g_state.seq, restored))
		{
			g_state.seq[0] = restored.seq[0];
			g_state.seq[1] = restored.seq[1];
		}

		g_boundary_frame = g_state.seq[0];
		film_undo_boundary(g_boundary_frame);

		// RB_NO_LIMIT=1: frame limiter off from the sync point (how fast can the game run frames?)
		static const bool no_limit = get_env("RB_NO_LIMIT") == "1";

		if (no_limit && !g_disable_frame_limit)
		{
			g_disable_frame_limit = true;
		}

		// RB_SYNC_VBLANK=<Hz>: vblank rate from the sync point (measures how fast the fight can be simulated)
		static const u32 sync_vblank = static_cast<u32>(std::strtoul(get_env("RB_SYNC_VBLANK").c_str(), nullptr, 10));

		if (sync_vblank && g_cfg.video.vblank_rate != sync_vblank)
		{
			g_cfg.video.vblank_rate.set(sync_vblank);
			rb_log.success("Vblank rate set to %u Hz at game frame %u", sync_vblank, g_state.seq[0]);
		}

		// Host time per game frame, averaged over 300 frames
		{
			static auto last = std::chrono::steady_clock::now();
			static u64 last_frame = g_state.seq[0];

			if (g_state.seq[0] >= last_frame + 300)
			{
				const auto now = std::chrono::steady_clock::now();
				const u64 us = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count();
				rb_log.notice("Frame time: %u us per game frame (frames %u-%u)", us / (g_state.seq[0] - last_frame), last_frame, g_state.seq[0]);
				last = now;
				last_frame = g_state.seq[0];
			}
		}

		static const bool virtual_tb = get_env("RB_VIRTUAL_TB") == "1";

		if (virtual_tb)
		{
			if (!g_vtb_active)
			{
				g_vtb_base = get_timebased_time() - g_state.seq[0] * vtb_per_frame;
				g_vtb_active = true;
				rb_log.success("Virtual timebase for the main thread from game frame %u", g_state.seq[0]);
			}

			g_vtb_frame = g_state.seq[0];
			g_vtb_offset = 0;
		}
	}

	u64 guest_timebase()
	{
		if (g_vtb_active)
		{
			if (const auto ppu = cpu_thread::get_current<ppu_thread>(); ppu && ppu->id == 0x1000000)
			{
				// Stays below the next frame even if the thread reads the timebase very often
				g_vtb_offset += g_vtb_offset + vtb_step < vtb_per_frame ? vtb_step : 0;
				return g_vtb_base + g_vtb_frame * vtb_per_frame + g_vtb_offset;
			}
		}

		return get_timebased_time();
	}

	// Watchdog (started at the sync point): when the game frame does not advance for 15 s, logs the state of all
	// PPU threads and the main thread call stack, to see what a hang is waiting for
	void start_watchdog()
	{
		static bool started = false;

		if (std::exchange(started, true))
		{
			return;
		}

		std::thread([]()
		{
			u64 last = umax;
			auto since = std::chrono::steady_clock::now();
			bool reported = false;

			while (Emu.IsRunning() || Emu.IsPaused())
			{
				std::this_thread::sleep_for(std::chrono::seconds(1));
				const u64 now_frame = g_game_frame.load();

				if (now_frame != last)
				{
					last = now_frame;
					since = std::chrono::steady_clock::now();
					reported = false;
					continue;
				}

				if (reported || std::chrono::steady_clock::now() - since < std::chrono::seconds(15))
				{
					continue;
				}

				reported = true;
				rb_log.error("Watchdog: no game frame progress for 15 s at game frame %u", now_frame);

				// RB_WATCHDOG_MEM="addr:size,...": memory dumped on a hang
				const char* mem = std::getenv("RB_WATCHDOG_MEM");

				for (const std::string& item : fmt::split(mem ? mem : "", {","}))
				{
					const auto parts = fmt::split(item, {":"});

					if (parts.size() != 2)
					{
						continue;
					}

					const u32 addr = static_cast<u32>(std::strtoul(parts[0].c_str(), nullptr, 16)) & ~15u;
					const u32 size = static_cast<u32>(std::strtoul(parts[1].c_str(), nullptr, 16));
					std::string out;

					for (u32 a = addr; a < addr + size; a += 16)
					{
						fmt::append(out, "\n%08x:", a);

						for (u32 w = 0; w < 16; w += 4)
						{
							if (vm::check_addr<4>(a + w, vm::page_readable))
								fmt::append(out, " %08x", +*vm::get_super_ptr<u32>(a + w));
							else
								out += " --------";
						}
					}

					rb_log.error("Watchdog: memory at 0x%08x:%s", addr, out);
				}

				idm::select<named_thread<ppu_thread>>([&](u32 id, named_thread<ppu_thread>& t)
				{
					const auto name = t.ppu_tname.load();
					rb_log.error("Watchdog: 0x%x %-32s %s cia=0x%08x lr=0x%08x r3=0x%x", id, name ? *name : "?",
						(t.state & cpu_flag::wait) ? "WAIT" : "RUN ", t.cia, static_cast<u32>(t.lr), t.gpr[3]);

					if (name && *name == "main_thread")
					{
						rb_log.error("Watchdog: main thread call stack:\n%s", t.dump_callstack());
					}
				});
			}
		}).detach();
	}

	void rb_frame_step(ppu_thread& ppu, u32 root)
	{
		if (g_state.synced)
		{
			start_watchdog();
		}

		static const bool net_dump = get_env("RB_NET_DUMP") == "1";

		if (net_dump)
		{
			static u64 loop_frames = 0;
			rb_log.notice("Main loop frame %u (pad reads %u/%u)", loop_frames++, g_state.seq[0], g_state.seq[1]);
		}

		// The game can run a nested frame loop from inside a task (a transition): its frame boundaries are not frame
		// boundaries of the rollback (no snapshot, no rollback with the outer frame's stack still in use)
		static u32 depth = 0;

		if (depth)
		{
			static u32 logged = 0;

			if (logged++ < 10)
			{
				rb_log.warning("Nested main loop frame (depth %u) at game frame %u", depth, g_state.seq[0]);
			}

			run_task_walk(ppu, root);
			return;
		}

		depth++;
		g_task_root = root;
		const auto t_start = std::chrono::steady_clock::now();

		// RB_SLOW_FRAMES: parts of the current live interval (closed at the next live frame start)
		struct slow_parts { u64 boundary = 0, handoff = 0, live_tasks = 0, resim_tasks = 0, resim_frames = 0, loads = 0; };
		static slow_parts parts{};
		static std::chrono::steady_clock::time_point live_start{};
		static const bool slow_diag = get_env("RB_SLOW_FRAMES") == "1";
		const bool resim_iteration = state_resimulating(g_state.seq[0]);

		if (slow_diag && !resim_iteration)
		{
			if (live_start.time_since_epoch().count() && online::fight_active())
			{
				const u64 us = std::chrono::duration_cast<std::chrono::microseconds>(t_start - live_start).count();
				static u64 n = 0, slow = 0;
				n++;

				if (us > 20000 && us < 1000000)
				{
					slow++;

					if (static u32 logged = 0; logged++ < 300)
					{
						rb_log.notice("Slow live interval %u us: boundaries %u, hand-off %u, live tasks %u, re-simulated %u frames %u us, loads %u (%u of %u slow)",
							us, parts.boundary, parts.handoff, parts.live_tasks, parts.resim_frames, parts.resim_tasks, parts.loads, slow, n);
					}
				}
			}

			live_start = t_start;
			parts = {};
		}
		const u64 loads_before = state_load_count();
		u64 diag_before[static_cast<int>(diag_t::count)];

		for (int k = 0; k < static_cast<int>(diag_t::count); k++)
		{
			diag_before[k] = state_diag_us(static_cast<diag_t>(k));
		}

		frame_boundary();
		const auto t_state = std::chrono::steady_clock::now();
		online::on_frame(ppu, root);

		// RB_DEFER_HANDOFF: the previous live frame's frame end, now that its state is saved; dropped when this boundary
		// loaded a snapshot (that frame was mispredicted and is not shown)
		if (g_defer_pending)
		{
			g_defer_pending = false;

			if (state_load_count() == loads_before)
			{
				const u32 old_r1 = static_cast<u32>(ppu.gpr[1]);
				ppu.gpr[1] = (old_r1 - 0x100) & ~0xfu;
				vm::write32(static_cast<u32>(ppu.gpr[1]), old_r1);
				ppu.gpr[3] = 0x00D2CA98;
				ppu.gpr[4] = g_defer_req;
				ppu.gpr[5] = g_defer_param;
				const auto h0 = std::chrono::steady_clock::now();
				ppu.fast_call(0x00600698, tekken_toc);
				g_handoff_us += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - h0).count();
				ppu.gpr[1] = old_r1;
				g_defer_run++;
			}
			else if (++g_defer_dropped % 300 == 1)
			{
				rb_log.notice("Deferred hand-off: %u frames not shown after a rollback (%u handed off)", g_defer_dropped, g_defer_run);
			}
		}

		const auto t_boundary = std::chrono::steady_clock::now();
		const auto t_walk_start = std::chrono::steady_clock::now();

		// Stutter statistics: wall clock time between consecutive live frames after the sync point (a rollback's
		// re-simulated frames are part of the live frame interval they delay)
		if (g_state.synced && !state_resimulating(g_state.seq[0]))
		{
			static auto prev = std::chrono::steady_clock::now();
			static u64 n = 0, over20 = 0, over33 = 0, max_us = 0;
			const auto now = std::chrono::steady_clock::now();
			const u64 us = std::chrono::duration_cast<std::chrono::microseconds>(now - prev).count();
			prev = now;

			if (us < 1000000)
			{
				n++;
				over20 += us > 20000;
				over33 += us > 33400;
				max_us = std::max(max_us, us);
			}

			if (n >= 600)
			{
				rb_log.notice("Live frame intervals: %u frames, %u over 20 ms, %u over 33 ms, max %u us", n, over20, over33, max_us);
				n = over20 = over33 = max_us = 0;
			}
		}

		// Netplay runs the frame itself (GGPO input exchange, rollbacks with re-simulated frames)
		if (!net::run_frame(ppu, root, g_state.seq[0]))
		{
			run_task_walk(ppu, root);
		}

		if (slow_diag)
		{
			const auto now = std::chrono::steady_clock::now();
			const u64 walk = std::chrono::duration_cast<std::chrono::microseconds>(now - t_walk_start).count();
			parts.handoff += g_handoff_us;
			parts.boundary += std::chrono::duration_cast<std::chrono::microseconds>(t_walk_start - t_start).count() - g_handoff_us;
			g_handoff_us = 0;
			parts.loads += state_load_count() != loads_before;

			if (state_resimulating(g_state.seq[0]))
			{
				parts.resim_tasks += walk;
				parts.resim_frames++;
			}
			else
			{
				parts.live_tasks += walk;
			}
		}

		// Messages that arrived during this frame (its vblank wait) are received now, not in the next frame's tasks:
		// a misprediction is found one frame earlier and rolled back at the next boundary
		if (g_state.synced && !state_resimulating(g_state.seq[0]))
		{
			online::end_of_frame(ppu);
		}

		// Re-simulated frame budget: boundary (load / saves, online bookkeeping) vs. the frame's tasks
		if (g_state.synced && state_resimulating(g_state.seq[0]))
		{
			static u64 n = 0, b_total = 0, w_total = 0, st_total = 0, st_max = 0;
			static u64 diag_total[static_cast<int>(diag_t::count)]{};
			const auto now = std::chrono::steady_clock::now();
			const u64 st = std::chrono::duration_cast<std::chrono::microseconds>(t_state - t_start).count();

			// The parts of the snapshot module (and online bookkeeping) in this boundary
			for (int k = 0; k < static_cast<int>(diag_t::count); k++)
			{
				diag_total[k] += state_diag_us(static_cast<diag_t>(k)) - diag_before[k];
			}

			b_total += std::chrono::duration_cast<std::chrono::microseconds>(t_boundary - t_start).count();
			w_total += std::chrono::duration_cast<std::chrono::microseconds>(now - t_boundary).count();
			st_total += st;
			st_max = std::max(st_max, st);

			if (++n % 1000 == 0)
			{
				rb_log.notice("Re-simulated frame parts: boundary avg %u us (snapshot module %u us, max %u us; online %u us), tasks avg %u us (1000 frames)",
					b_total / 1000, st_total / 1000, st_max, (b_total - st_total) / 1000, w_total / 1000);
				rb_log.notice("Re-simulated boundary parts (avg per frame): save %u us, load %u us, draw thread wait %u us, SPURS job wait %u us",
					diag_total[0] / 1000, diag_total[1] / 1000, diag_total[2] / 1000, diag_total[3] / 1000);
				b_total = w_total = st_total = st_max = 0;
				std::fill(std::begin(diag_total), std::end(diag_total), 0);
			}
		}

		// Live frame budget: boundary (snapshot save, online bookkeeping) vs. the frame's tasks (incl. the frame end wait)
		if (g_state.synced && !state_resimulating(g_state.seq[0]))
		{
			static u64 n = 0, b_total = 0, b_max = 0, w_total = 0, w_max = 0;
			const auto now = std::chrono::steady_clock::now();
			const u64 b = std::chrono::duration_cast<std::chrono::microseconds>(t_boundary - t_start).count();
			const u64 w = std::chrono::duration_cast<std::chrono::microseconds>(now - t_boundary).count();
			b_total += b, b_max = std::max(b_max, b), w_total += w, w_max = std::max(w_max, w);

			if (++n % 600 == 0)
			{
				rb_log.notice("Live frame parts: boundary avg %u us max %u us, tasks avg %u us max %u us (600 frames)", b_total / 600, b_max, w_total / 600, w_max);
				b_total = b_max = w_total = w_max = 0;
			}
		}

		depth--;
	}

	bool live_pad(CellPadData& out)
	{
		std::memcpy(&out, &g_live_pad, sizeof(CellPadData));
		return g_live_pad_valid;
	}

	void force_sync_point(const char* reason)
	{
		if (!g_state.synced)
		{
			g_state.synced = true;
			g_sync_frame = g_state.seq[0];
			rb_log.success("Sync point (%s) at game frame %u", reason, g_sync_frame);
		}
	}

	u64 sync_game_frame()
	{
		return g_sync_frame;
	}

	void begin_frame()
	{
		g_boundary_frame = g_state.seq[0];

		if (g_vtb_active)
		{
			g_vtb_frame = g_state.seq[0];
			g_vtb_offset = 0;
		}
	}

	void get_seq(u32 (&seq)[2])
	{
		seq[0] = g_state.seq[0];
		seq[1] = g_state.seq[1];
	}

	void set_seq(const u32 (&seq)[2])
	{
		g_state.seq[0] = seq[0];
		g_state.seq[1] = seq[1];
	}

	bool recorded_input(u32 port, u32 seq, CellPadData& out)
	{
		if (port >= max_ports || seq >= g_state.replay[port].size())
		{
			return false;
		}

		std::memcpy(&out, g_state.replay[port][seq].data, sizeof(CellPadData));
		return true;
	}

	// One frame of the main loop: the task walk of FUN_00100250 (see rb_frame_step)
	void run_task_walk(ppu_thread& ppu, u32 root)
	{
		// Own stack frame for the calls (FUN_000ff344 saves its LR into the caller frame)
		const u64 old_r1 = ppu.gpr[1];
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfull;
		vm::write32(static_cast<u32>(ppu.gpr[1]), static_cast<u32>(old_r1));

		u32 node = vm::read32(root + 0x14);

		// RB_TASK_PROFILE=1: exclusive host time per task method (+0x30 of the task node), normal and re-simulated
		// frames; nested tasks are measured by rb_subtask_walk
		static const bool profile = get_env("RB_TASK_PROFILE") == "1";
		const bool resim = state_resimulating(g_boundary_frame);

		do
		{
			if (const u32 next = try_skip_task(ppu, node, resim))
			{
				node = next;
				continue;
			}

			const u32 task_vtable = vm::read32(node + vm::read32(node + 0x34));
			const u32 method = profile ? task_vtable : 0u;
			g_task_child_ns.push_back(0);
			const auto t0 = std::chrono::steady_clock::now();

			{
				std::optional<cosmetic_rng_scope> cosmetic;

				if (resim_skip_listed(node))
				{
					cosmetic.emplace(ppu);
				}

				// The sound device task updates the sound library every frame: learnt as library state (rb_online)
				const bool sound_device = task_vtable == 0x00910798u && !resim;

				if (sound_device)
				{
					online::sound_learn_begin();
				}

				ppu.gpr[3] = node;
				ppu.gpr[4] = 1;

				// RB_NATIVE: the task step runs in the lifted C++ code (hybrid), otherwise in RPCS3.
				// RB_NATIVE_EXCLUDE=<vtable>,...: these tasks stay in RPCS3 (to find a task whose lifted code diverges)
				static const std::set<u32> native_excluded = []()
				{
					std::set<u32> r;
					for (const auto& v : fmt::split(get_env("RB_NATIVE_EXCLUDE"), {","}))
					{
						r.insert(static_cast<u32>(std::stoul(v, nullptr, 16)));
					}
					return r;
				}();

				if (static std::set<u32> seen; seen.insert(task_vtable).second)
				{
					rb_log.notice("Native: task vtable 0x%08x first run (%s)", task_vtable, native_excluded.contains(task_vtable) ? "RPCS3" : "native");
				}

				if (native_excluded.contains(task_vtable) || !native::call(ppu, tekken_task_step))
				{
					ppu.fast_call(tekken_task_step, tekken_toc);
				}

				if (sound_device)
				{
					online::sound_learn_end();
				}
			}

			const u32 next_node = static_cast<u32>(ppu.gpr[3]);
			after_task_step(ppu, node, task_vtable, resim);
			ppu.gpr[3] = next_node;

			const u64 total = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
			const u64 nested = g_task_child_ns.back();
			g_task_child_ns.pop_back();

			if (profile && g_state.synced)
			{
				auto& t = g_task_prof[resim][method];
				t.ns += total - std::min(total, nested);
				t.calls++;
			}

			node = static_cast<u32>(ppu.gpr[3]);
		}
		while (node != root + 4 && !ppu.is_stopped());

		if (profile && g_state.synced && ++g_task_prof_frames[resim] % 500 == 0)
		{
			std::vector<std::pair<u32, task_time>> sorted(g_task_prof[resim].begin(), g_task_prof[resim].end());
			std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.second.ns > b.second.ns; });
			u64 total = 0;

			for (const auto& [m, t] : sorted)
			{
				total += t.ns;
			}

			std::string text;

			for (usz i = 0; i < std::min<usz>(sorted.size(), 25); i++)
			{
				fmt::append(text, " [0x%08x %u us %.1f calls]", sorted[i].first, sorted[i].second.ns / 500 / 1000, sorted[i].second.calls / 500.0);
			}

			rb_log.notice("Task profile (%s frames, %u us/frame in tasks, %u methods):%s", resim ? "re-simulated" : "normal", total / 500 / 1000, sorted.size(), text);

			if (const u64 calls = g_drawroot_prof[resim][3])
			{
				rb_log.notice("Draw root state (%s, %u calls): subtree %u us, film %u us, draw pass %u us per call", resim ? "re-simulated" : "normal", calls,
					g_drawroot_prof[resim][0] / calls / 1000, g_drawroot_prof[resim][1] / calls / 1000, g_drawroot_prof[resim][2] / calls / 1000);
				std::memset(g_drawroot_prof[resim], 0, sizeof(g_drawroot_prof[resim]));
			}
			g_task_prof[resim].clear();
		}

		ppu.gpr[1] = old_r1;
	}

	void task_check(u64 game_frame, const char* when)
	{
		static const u64 from = []() -> u64
		{
			const std::string v = get_env("RB_TASK_CHECK");
			return v.empty() ? umax : std::strtoull(v.c_str(), nullptr, 10);
		}();

		if (game_frame < from || !g_task_root)
		{
			return;
		}

		static u32 reported = 0;

		if (reported >= 3)
		{
			return;
		}

		const auto rd = [](u32 addr) -> u32
		{
			return addr && vm::check_addr(addr, vm::page_readable, 4) ? static_cast<u32>(vm::read32(addr)) : 0u;
		};

		const auto describe = [&](const char* what, u32 n) -> std::string
		{
			if (!n || !vm::check_addr(n, vm::page_readable, 0x38))
			{
				return fmt::format(" %s 0x%08x (unreadable)", what, n);
			}

			return fmt::format(" %s 0x%08x [vt 0x%08x obj-vt 0x%08x next 0x%08x prev 0x%08x parent 0x%08x nsib 0x%08x psib 0x%08x child 0x%08x saved %d]",
				what, n, rd(n), rd(n + rd(n + 0x34)), rd(n + 0x10), rd(n + 0x14), rd(n + 0x18), rd(n + 0x1c), rd(n + 0x20), rd(n + 0x24), state_addr_saved(n));
		};

		const auto violation = [&](const char* what, u32 n)
		{
			reported++;
			const u32 next = rd(n + 0x10), prev = rd(n + 0x14);
			rb_log.error("Task check %s, game frame %u (re-simulating %d): %s at node 0x%08x;%s%s%s%s", when, game_frame,
				state_resimulating(game_frame), what, n, describe("node", n).c_str(), describe("next", next).c_str(), describe("prev", prev).c_str(),
				describe("parent", rd(n + 0x18)).c_str());
		};

		const u32 sentinel = g_task_root + 4;
		u32 n = rd(g_task_root + 0x14);
		u32 count = 0;

		for (; n != sentinel; count++)
		{
			if (count >= 20000)
			{
				violation("list longer than 20000 nodes (cycle?)", n);
				return;
			}

			if (!n || !vm::check_addr(n, vm::page_readable, 0x28))
			{
				violation("unreadable node in the list", n);
				return;
			}

			const u32 next = rd(n + 0x10);

			if (!next)
			{
				violation("null next before the end of the list", n);
				return;
			}

			if (rd(next + 0x14) != n)
			{
				violation("next->prev != node", n);
				return;
			}

			n = next;
		}

		// FUN_000ff2b0(wall contents bank): the last node of the bank's subtree, where FUN_000ff914 inserts (crash at 5051 in zed1)
		constexpr u32 wall_bank = 0x10388900;

		if (rd(wall_bank) == 0x00912c68u)
		{
			u32 p = wall_bank, pred = 0;

			for (u32 up = 0; up < 64 && p; up++)
			{
				if (const u32 sib = rd(p + 0x1c))
				{
					pred = rd(sib + 0x14);
					break;
				}

				if (!rd(p + 0x18))
				{
					pred = rd(p + 0x14);
					break;
				}

				p = rd(p + 0x18);
			}

			if (!pred || !rd(pred + 0x10))
			{
				violation(fmt::format("insert point of the wall bank 0x%08x (FUN_000ff2b0 = 0x%08x, from 0x%08x) has no next", wall_bank, pred, p).c_str(), pred);
				return;
			}
		}

		if (static u64 ok = 0; ++ok % 600 == 1)
		{
			rb_log.notice("Task check %s, game frame %u: %u nodes OK", when, game_frame, count);
		}
	}

	void install_frame_hook()
	{
		// The hook is installed while the executable is loaded (install_static_hooks), before PPU compilation:
		// a function table change at run time is not seen by LLVM compiled direct calls
	}

	bool relaunch_pending()
	{
		return g_frame_hook_installed;
	}

	bool relaunch_for_boot(const std::string& path)
	{
		if (!g_frame_hook_installed)
		{
			return false;
		}

#ifdef _WIN32
		wchar_t exe[MAX_PATH]{};

		if (!GetModuleFileNameW(nullptr, exe, MAX_PATH))
		{
			rb_log.error("Relaunch: executable path unknown (%u)", GetLastError());
			return false;
		}

		// The new process overwrites RPCS3.log: this one's log is kept as RPCS3_prev_<time>.log (diagnostics of what happened before)
		{
			const std::string dir = fs::get_log_dir();
			const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::tm tm{};
			localtime_s(&tm, &now);
			const std::string copy = fmt::format("%sRPCS3_prev_%04d%02d%02d_%02d%02d%02d.log", dir, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

			if (!fs::copy_file(dir + "RPCS3.log", copy, true))
			{
				rb_log.error("Relaunch: log copy '%s' failed (%s)", copy, fs::g_tls_error);
			}
		}

		// The environment (RB_* variables of hrat_online.bat) is inherited
		std::wstring cmd = L"\"" + std::wstring(exe) + L"\" \"" + utf8_to_wchar(path) + L"\"";
		STARTUPINFOW si{};
		si.cb = sizeof(si);
		PROCESS_INFORMATION pi{};

		if (!CreateProcessW(exe, cmd.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &si, &pi))
		{
			rb_log.error("Relaunch: CreateProcess failed (%u)", GetLastError());
			return false;
		}

		CloseHandle(pi.hThread);
		CloseHandle(pi.hProcess);
		rb_log.success("Second boot in this process: started a fresh RPCS3 (pid %u) for '%s', quitting", pi.dwProcessId, path);
		return true;
#else
		rb_log.error("Second boot in this process: restart RPCS3 before playing again");
		return false;
#endif
	}

	void install_static_hooks(std::vector<u32>& applied)
	{
		const char* v = std::getenv("RB_FRAME_HOOK");

		if ((v && *v == '0') || Emu.GetTitleID() != "BLUS30359")
		{
			return;
		}

		// HLE function stub of rb_frame_step (code address inside the fake OPD array of HLE functions)
		const u32 target = g_fxo->get<ppu_function_manager>().func_addr(s_frame_step_index, true);

		if (target && ppu_form_branch_to_code(tekken_task_walk, target))
		{
			applied.push_back(tekken_task_walk);
			g_frame_hook_installed = true;
			rb_log.success("Frame hook: 0x%08x (task walk of the main loop) -> HLE 0x%08x", tekken_task_walk, target);
		}
		else
		{
			rb_log.error("Frame hook could not be installed (target 0x%08x)", target);
		}

		// Also for RB_RESIM_SKIP: the draw tree runs as a sub task walk
		if (const char* tp = std::getenv("RB_TASK_PROFILE"); (tp && *tp == '1') || !get_env("RB_RESIM_SKIP").empty())
		{
			const u32 walk_target = g_fxo->get<ppu_function_manager>().func_addr(s_subtask_walk_index, true);

			if (walk_target && ppu_form_branch_to_code(tekken_subtask_walk, walk_target))
			{
				applied.push_back(tekken_subtask_walk);
				rb_log.success("Task profiler: sub task walk 0x%08x -> HLE 0x%08x", tekken_subtask_walk, walk_target);
			}
		}

		if (get_env("RB_RESIM_DRAWROOT") == "1")
		{
			const u32 draw_list_target = g_fxo->get<ppu_function_manager>().func_addr(s_draw_list_index, true);

			if (draw_list_target && ppu_form_branch_to_code(tekken_draw_list, draw_list_target))
			{
				applied.push_back(tekken_draw_list);
				rb_log.success("Draw pass: 0x%08x -> HLE 0x%08x (re-simulated frames: match recording only)", tekken_draw_list, draw_list_target);
			}

			const u32 drawroot_target = g_fxo->get<ppu_function_manager>().func_addr(s_drawroot_state_index, true);

			if (drawroot_target && ppu_form_branch_to_code(tekken_drawroot_state, drawroot_target))
			{
				applied.push_back(tekken_drawroot_state);
				rb_log.success("Re-simulation without the draw subtree: 0x%08x -> HLE 0x%08x", tekken_drawroot_state, drawroot_target);
			}
		}

		if (const char* fr = std::getenv("RB_FAST_RESIM"); fr && *fr == '1')
		{
			const u32 end_target = g_fxo->get<ppu_function_manager>().func_addr(s_frame_end_index, true);

			if (end_target && ppu_form_branch_to_code(tekken_frame_end, end_target))
			{
				applied.push_back(tekken_frame_end);
				rb_log.success("Fast resimulation: frame end 0x%08x -> HLE 0x%08x", tekken_frame_end, end_target);
			}
		}

		if (const char* al = std::getenv("RB_ALLOC_LOG"); al && *al == '1')
		{
			const u32 alloc_target = g_fxo->get<ppu_function_manager>().func_addr(s_frame_alloc_index, true);

			if (alloc_target && ppu_form_branch_to_code(tekken_frame_alloc, alloc_target))
			{
				applied.push_back(tekken_frame_alloc);
				rb_log.success("Frame allocator hook: 0x%08x -> HLE 0x%08x", tekken_frame_alloc, alloc_target);
			}

			const u32 swap_target = g_fxo->get<ppu_function_manager>().func_addr(s_frame_swap_index, true);

			if (swap_target && ppu_form_branch_to_code(tekken_frame_swap, swap_target))
			{
				applied.push_back(tekken_frame_swap);
				rb_log.success("Frame swap hook: 0x%08x -> HLE 0x%08x", tekken_frame_swap, swap_target);
			}
		}

		online::install_hooks(applied);

		// RB_NO_FRAMESKIP=1: the main frame function FUN_00101ae0 stores the number of missed vblanks reported by
		// the draw sync object (0x00D2CA98, not part of the snapshot) into +0x114 and the game then skips rendering
		// of the next frames. A rollback makes a frame late, the skip then leaves the frame allocator unswapped,
		// it overflows and render data gets overwritten (crash ~4687). Pacing is ours: the lag is always 0.
		if (const char* nf = std::getenv("RB_NO_FRAMESKIP"); nf && *nf == '1')
		{
			constexpr u32 lag_check = 0x00101b94; // ble cr7, 0x00101ba4  ->  b 0x00101ba4

			if (vm::read32(lag_check) == 0x409d0010u)
			{
				vm::write32(lag_check, 0x48000010u);
				applied.push_back(lag_check);
				rb_log.success("Frame skip disabled (0x%08x)", lag_check);
			}
			else
			{
				rb_log.error("Frame skip patch: unexpected instruction 0x%08x at 0x%08x", +vm::read32(lag_check), lag_check);
			}
		}
	}

	void before_pad_get_data(u32 port_no)
	{
		ensure_init();

		// Frame boundary: the state test may restore memory and rewind the input positions.
		// The sync point is checked here as well, so a realignment happens before the snapshot logic.
		if (port_no != 0 || !g_state.synced)
		{
			return;
		}

		// Diagnostics (RB_DIAG_THREADS=1): state of all PPU threads at the pad read, for a few fight frames
		static const bool diag_threads = []() { const char* v = std::getenv("RB_DIAG_THREADS"); return v && *v == '1'; }();

		if (diag_threads && g_state.recorded_sync_seq[0] >= 0)
		{
			const u32 rel = g_state.seq[0] - static_cast<u32>(g_state.recorded_sync_seq[0]);

			// Sample main and draw thread for 2 s from a separate host thread (does not block the game):
			// where is the main thread when the draw thread is blocked in a system call (idle)?
			if (rel == 300)
			{
				std::thread([]()
				{
					std::map<std::pair<u32, u32>, u32> idle; // (draw cia, main lr) -> samples
					std::map<u32, u32> draw_blocked;          // draw cia -> samples (syscall region)
					u32 samples = 0;
					const auto start = std::chrono::steady_clock::now();

					while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
					{
						u32 draw_cia = 0, main_lr = 0, main_cia = 0;
						bool draw_wait = false;

						idm::select<named_thread<ppu_thread>>([&](u32, named_thread<ppu_thread>& t)
						{
							const auto name = t.ppu_tname.load();

							if (!name)
							{
								return;
							}

							if (*name == "NU::Draw::RequestManager")
							{
								draw_cia = t.cia;
								draw_wait = !!(t.state & cpu_flag::wait);
							}
							else if (*name == "main_thread")
							{
								main_cia = t.cia;
								main_lr = static_cast<u32>(t.lr);
							}
						});

						samples++;

						// System library code (liblv2/libsysmodule) lives above 0x00d80000
						if (draw_wait && draw_cia >= 0x00d80000)
						{
							draw_blocked[draw_cia]++;
							idle[{draw_cia, main_cia < 0x00d80000 ? main_cia : main_lr}]++;
						}

						std::this_thread::yield();
					}

					std::string out;
					for (const auto& [cia, n] : draw_blocked)
					{
						fmt::append(out, " 0x%08x:%u", cia, n);
					}

					rb_log.success("Draw idle sampler: %u samples, draw blocked in syscalls:%s", samples, out);

					for (const auto& [key, n] : idle)
					{
						if (n >= 20)
						{
							rb_log.success("Draw idle sampler: draw 0x%08x while main at 0x%08x: %u", key.first, key.second, n);
						}
					}
				}).detach();
			}

			// Sample the draw thread for up to 20 ms: where does it block (idle) after finishing a frame?
			if (rel >= 200 && rel < 210)
			{
				std::map<u32, u32> seen; // cia -> samples while blocked
				const auto start = std::chrono::steady_clock::now();

				while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(20))
				{
					idm::select<named_thread<ppu_thread>>([&](u32, named_thread<ppu_thread>& t)
					{
						const auto name = t.ppu_tname.load();

						if (name && *name == "NU::Draw::RequestManager" && (t.state & cpu_flag::wait))
						{
							seen[t.cia]++;
						}
					});

					std::this_thread::sleep_for(std::chrono::microseconds(100));
				}

				std::string out;
				for (const auto& [cia, n] : seen)
				{
					fmt::append(out, " 0x%08x:%u", cia, n);
				}

				rb_log.success("Draw thread blocked @fight+%u (cia:samples):%s", rel, out);
			}

			if ((rel >= 100 && rel < 106) || (rel >= 400 && rel < 403))
			{
				idm::select<named_thread<ppu_thread>>([&](u32 id, named_thread<ppu_thread>& t)
				{
					const auto name = t.ppu_tname.load();
					const bool waiting = !!(t.state & cpu_flag::wait);
					rb_log.success("Threads @fight+%u: 0x%x %-32s %s cia=0x%08x lr=0x%08x", rel, id, name ? *name : "?",
						waiting ? "WAIT" : "RUN ", t.cia, static_cast<u32>(t.lr));
				});
			}
		}

		// Once, 100 frames into the fight: log the guest call stack of the pad read (to find the main loop)
		if (static bool logged = false; !logged && g_state.recorded_sync_seq[0] >= 0 &&
			g_state.seq[0] == static_cast<u32>(g_state.recorded_sync_seq[0]) + 100)
		{
			logged = true;

			if (const auto ppu = get_current_cpu_thread<ppu_thread>())
			{
				rb_log.success("Call stack of the pad read (LR 0x%x):", ppu->lr);

				for (const auto& [addr, sp] : ppu->dump_callstack_list())
				{
					rb_log.success("    0x%08x (sp 0x%08x)", addr, sp);
				}
			}
		}

		install_frame_hook();

		// Without the frame hook (RB_FRAME_HOOK=0) the state is saved/restored at the pad read (mid-frame)
		if (!g_frame_hook_installed)
		{
			frame_boundary();
		}
	}

	// RB_TRACE_FRAMES="a-b,c-d": all log channels at trace level during these game frames, with a marker per frame
	void trace_window(u64 game_frame)
	{
		static const std::vector<std::pair<u64, u64>> windows = []()
		{
			std::vector<std::pair<u64, u64>> out;
			const char* v = std::getenv("RB_TRACE_FRAMES");

			for (const std::string& item : fmt::split(v ? v : "", {","}))
			{
				const auto parts = fmt::split(item, {"-"});

				if (parts.size() == 2)
				{
					out.emplace_back(std::strtoull(parts[0].c_str(), nullptr, 10), std::strtoull(parts[1].c_str(), nullptr, 10));
				}
			}

			return out;
		}();

		static std::map<std::string, logs::level> saved;

		bool inside = false;
		for (const auto& [a, b] : windows)
		{
			inside |= game_frame >= a && game_frame <= b;
		}

		if (inside && saved.empty())
		{
			for (const std::string& ch : logs::get_channels())
			{
				saved[ch] = logs::get_level(ch);
				logs::set_level(ch, logs::level::trace);
			}
		}
		else if (!inside && !saved.empty())
		{
			for (const auto& [ch, lvl] : saved)
			{
				logs::set_level(ch, lvl);
			}

			saved.clear();
		}

		if (inside)
		{
			rb_log.success("=== TRACE game frame %u ===", game_frame);
		}
	}

	void after_pad_get_data(u32 port_no)
	{
		// The first pad read of a game frame is the frame boundary
		static u64 last_reads = 0;

		// Count real reads, not seq: seq does not advance when cellPadGetData failed early,
		// and it can jump back when the replay realigns at the sync point
		if (port_no == 0 && g_port0_reads != last_reads)
		{
			last_reads = g_port0_reads;
			g_game_frame = g_state.seq[0];
			trace_window(g_state.seq[0] - 1);
			checksum_on_frame(g_state.seq[0] - 1);
		}
	}

	void on_reset_flip_status()
	{
		g_reset_flip++;
	}

	void on_guest_flip()
	{
		ensure_init();

		const u64 frame = g_frame++;

		if (g_state.stats)
		{
			const std::string line = fmt::format("%u,%u,%u,%u,%u\n", frame, get_system_time(), g_reads[0].exchange(0), g_reads[1].exchange(0), g_reset_flip.exchange(0));
			g_state.stats.write(line.data(), line.size());
		}
	}
}
