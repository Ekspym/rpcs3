#include "stdafx.h"
#include "rb_state.h"

#include "rb_input.h"
#include "Emu/Memory/vm_locking.h"
#include "Emu/CPU/CPUThread.h"
#include "Emu/Cell/PPUThread.h"
#include "Emu/Cell/lv2/sys_timer.h"
#include "Emu/Cell/lv2/sys_lwmutex.h"
#include "Emu/IdManager.h"
#include "Emu/Memory/vm.h"
#include "Emu/RSX/RSXThread.h"
#include "Emu/RSX/Capture/rsx_capture.h"
#include "Utilities/StrUtil.h"
#include "util/vm.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <thread>
#include <vector>

LOG_CHANNEL(rb_log, "Rollback");

extern std::atomic<u64> g_audio_dump_frames; // AudioDumper.cpp

namespace rb
{
	std::atomic<u64>& state_diag_us(diag_t what)
	{
		static std::atomic<u64> us[static_cast<int>(diag_t::count)]{};
		return us[static_cast<int>(what)];
	}

	static u64 diag_now_us()
	{
		return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	diag_scope::diag_scope(diag_t w) : what(w), t0(diag_now_us()) {}

	diag_scope::~diag_scope()
	{
		state_diag_us(what) += diag_now_us() - t0;
	}

	namespace
	{
		struct range_t
		{
			u32 begin;
			u32 end;
		};

		std::vector<range_t> g_ranges;

		// RB_SNAP_EXCLUDE: ranges inside RB_SNAP_RANGES that are never saved or restored
		std::vector<range_t> g_excluded;

		// Ranges always saved whole and ranges reduced by calibration to their changing pages
		std::vector<range_t> g_fixed_ranges;
		std::vector<range_t> g_cal_ranges;
		std::vector<u8> g_buffer;
		snapshot_info g_saved{};
		bool g_has_snapshot = false;
		u64 g_save_at = umax;
		u64 g_save_rel = umax;   // RB_SNAP_SAVE_AT=+N
		u64 g_cal_rel = umax;    // RB_SNAP_CALIBRATE_FROM=+N
		u64 g_restore_at = umax;
		u64 g_restore_rel = umax; // RB_SNAP_RESTORE_AT=+N
		std::once_flag g_init;

		// RB_SNAP_CALIBRATE_FROM: from this game frame until the save, every page of RB_SNAP_RANGES is
		// hashed each frame; only the pages that changed form the snapshot. Heap layout differs between
		// sessions, so the page set must be found in the same session.
		u64 g_calibrate_from = umax;
		std::vector<u32> g_cal_pages;  // candidate page addresses
		std::vector<u64> g_cal_hashes; // last hash of each candidate
		std::vector<u8> g_cal_dirty;   // changed at least once
		std::vector<u8> g_cal_cooled;  // how many times the page was cooled down (cooling hysteresis)
		u64 g_cal_frames = 0;
		bool g_cal_done = false;
		bool g_cal_continuous = false; // RB_SNAP_CONT_CAL=1: keep calibrating during the stress test

		// Content of every candidate page at calibration start. A page that was never seen changing still has
		// this content, so when it starts changing later, older ring snapshots are completed with it instead of
		// throwing the whole ring away
		std::vector<u8> g_cal_shadow;
		std::unordered_map<u32, u32> g_cal_index; // page address -> candidate index
		// Write tracking (RB_SNAP_TRACK=1): state per guest page
		constexpr u8 track_none = 0, track_clean = 1, track_dirty = 2;
		std::unique_ptr<std::atomic<u8>[]> g_track;
		bool g_track_enabled = false;
		std::atomic<bool> g_track_active = false;
		std::mutex g_track_queue_mutex;
		std::vector<u32> g_track_queue; // pages that became dirty since the last calibration step

		// Pages first written (after tracking started) by another thread than the PPU main thread (network and sound worker
		// threads, HLE/host threads) or behind the tracking's back (host writes found by the sweep): state of asynchronous
		// threads that are not rolled back, so restoring the page would desynchronise them from it (lost network packets,
		// sounds that never stop). They are removed from the snapshot (off by default, RB_SNAP_FOREIGN=1: the network threads also write the pages of the input manager and of a game memory pool, excluding them broke the rollback). Only the PPU threads
		// named in RB_SNAP_FOREIGN_THREADS (name prefixes, default "net_") count: the sound threads write pages that also hold
		// game heap objects, excluding those crashed the game (pool free list restored without its nodes).
		std::unique_ptr<std::atomic<u8>[]> g_foreign;
		std::mutex g_foreign_mutex;
		std::vector<u32> g_foreign_queue; // pages to remove from the snapshot (main thread)
		// RB_SNAP_FOREIGN=2: the pages are only reported (state_foreign_pages), they stay in the snapshot; the online module
		// keeps the bytes the other threads write live across a load
		bool g_foreign_track_only = false;
		// RB_SNAP_FOREIGN=3: only pages first written by the named threads (RB_SNAP_FOREIGN_THREADS) are excluded, not host
		// writes or unnamed threads (SPU job results are game state): e.g. the file loader threads, whose data (sound banks
		// of the round, loaded while the fight runs) a rollback restored to the content before the load
		bool g_foreign_named_only = false;
		std::vector<u32> g_foreign_tracked;

		// state_foreign_window: ranges where a write missed by the tracking makes the page foreign in every mode
		std::mutex g_foreign_window_mutex;
		std::vector<std::pair<u32, u32>> g_foreign_windows;

		bool in_foreign_window(u32 addr)
		{
			std::lock_guard lock(g_foreign_window_mutex);
			return std::any_of(g_foreign_windows.begin(), g_foreign_windows.end(), [&](const auto& w) { return addr >= w.first && addr < w.second; });
		}

		void mark_foreign(u32 page, const std::string& writer)
		{
			// Never in the game heap arena (0x10090000-0x120D0000, H+0x40): its pages also hold fight objects. A fios load into
			// a heap block made them foreign and the next rounds desynced (o172)
			if (page >= (0x10090000 >> 12) && page < (0x120D0000 >> 12))
			{
				return;
			}

			if (!g_foreign || g_foreign[page].exchange(1))
			{
				return;
			}

			{
				std::lock_guard lock(g_foreign_mutex);
				(g_foreign_track_only ? g_foreign_tracked : g_foreign_queue).push_back(page << 12);
			}

			static std::atomic<u32> reported = 0;

			if (reported++ < 64)
			{
				rb_log.notice("Snapshot: page 0x%08x written by %s, %s", page << 12, writer, g_foreign_track_only ? "tracked" : "excluded (not rolled back)");
			}
		}

		// Page protection only for pages the game still has mapped (a page freed by the game, e.g. sys_memory_free after
		// a round, made VirtualProtect fail -> fatal error)
		void protect_pages(u32 addr, u32 size, utils::protection prot)
		{
			for (u32 page = addr; page < addr + size; page += 0x1000)
			{
				if (vm::check_addr(page, vm::page_readable))
				{
					utils::memory_protect(vm::g_base_addr + page, 0x1000, prot);
				}
			}
		}

		void track_mark_dirty(u32 page)
		{
			std::lock_guard lock(g_track_queue_mutex);
			g_track_queue.push_back(page << 12);
		}

		std::vector<std::string> g_new_pages;
		std::vector<u32> g_new_dirty_pages; // pages found changing in the last calibration step // first pages found changing in the last calibration step (log)

		// Stress test (RB_SNAP_STRESS_EVERY): after the first save, the state is saved every frame into a ring
		// and every N frames the game is rolled back by RB_SNAP_DEPTH frames, like GGPO would do it
		constexpr u32 ring_size = 16; // online: 8 was too short when a remote input came ~18 simulated frames late (o137)
		u64 g_stress_every = 0;

		// Rollback requested by the online module (state_request_rollback)
		u64 g_request_to = umax;
		u64 g_request_max_depth = umax;
		u64 g_refused_rollbacks = 0; // the deepest allowed re-simulation (game frames) of a requested rollback
		std::function<void()> g_request_before, g_request_after;
		std::function<bool()> g_spu_busy; // online module: a game SPU job is running (non-blocking)

		// RB_SNAP_NO_IDLE=1: bytes of snapshot pages written by other threads (draw thread, network) are learnt while the
		// main thread waits for the draw thread (first RB_SNAP_NO_IDLE_LEARN boundaries without a running SPU job, default
		// 600) and excluded from the snapshot; afterwards saves and loads no longer wait for the draw thread (~4 ms per
		// frame with one instance, ~10 ms with two on one PC), the game's logic and drawing overlap again
		bool g_idle_learned = false;
		u64 g_request_failed = 0;
		std::function<bool()> g_save_check;
		u64 g_saves_skipped = 0;
		u64 g_depth = 3;

		// RB_SNAP_STRESS_PATTERN="interval:depth,...": cycled; the next rollback happens `interval` frames after
		// the previous rollback point and goes back `depth` frames (real netplay mixes 1, 2 and 3 frame rollbacks)
		std::vector<std::pair<u64, u64>> g_pattern;
		usz g_pattern_idx = 0;
		u64 g_next_rollback = umax;

		u64 current_depth()
		{
			return g_pattern.empty() ? g_depth : g_pattern[g_pattern_idx].second;
		}
		std::vector<u8> g_ring[ring_size];
		snapshot_info g_ring_info[ring_size]{};
		u64 g_ring_tag[ring_size]{};           // online mode: the remote simulated frame at the save (state_set_save_tag)
		std::function<u64()> g_save_tag;
		u64 g_request_max_tag = umax, g_request_min_tag = 0;
		bool g_ring_valid[ring_size]{};
		u64 g_last_rollback_from = 0;
		u64 g_rollbacks = 0;
		u64 g_save_us_total = 0, g_save_us_max = 0, g_saves = 0;
		std::chrono::steady_clock::time_point g_resim_start{};

		// Netplay (RB_NET=1): GGPO decides when to save and load; the snapshot module only provides the ring
		bool g_net_mode = false;
		bool g_net_resim = false;
		u32 g_resim_depth = 0;
		u64 g_resim_us_total = 0, g_resim_us_max = 0, g_resims = 0;
		u64 g_load_us_total = 0, g_load_us_max = 0;
		u64 g_wait_us_total = 0, g_wait_us_max = 0;

		u64 hash_page(const u8* data)
		{
			const u64* w = reinterpret_cast<const u64*>(data);
			u64 h0 = 0x9E3779B97F4A7C15, h1 = 0xC2B2AE3D27D4EB4F;

			for (u32 i = 0; i < 4096 / 8; i += 2)
			{
				h0 = (h0 ^ w[i]) * 0x100000001B3;
				h1 = (h1 ^ w[i + 1]) * 0x100000001B3;
			}

			return h0 ^ (h1 * 0x9E3779B97F4A7C15);
		}


		std::vector<u32> g_fifo_blocks;
		u32 g_fifo_shift = 20;          // block size of the FIFO exclusion: 1 MiB (Tekken 6), 64 KiB for Tekken 5 DR
		std::vector<u32> g_fifo_seen;   // T5DR: blocks the RSX GET/PUT pointers were seen in since the sync point

		// T5DR renders on its main thread from several command buffers in the RSX mapped memory, which also holds game
		// objects (second rewind test 2026-10-19: excluding all of it crashed on a game object at 0x5076c8c0 right after
		// the restore). The blocks the RSX actually reads commands from are learned instead: GET and PUT are sampled at
		// every frame boundary and key read from the sync point to the first save, and all blocks between them.
		void note_fifo_block(u32 ea)
		{
			if (ea != umax && std::find(g_fifo_seen.begin(), g_fifo_seen.end(), ea >> g_fifo_shift) == g_fifo_seen.end())
			{
				g_fifo_seen.push_back(ea >> g_fifo_shift);
				rb_log.notice("Snapshot: RSX command block 0x%08x-0x%08x seen", (ea >> g_fifo_shift) << g_fifo_shift, ((ea >> g_fifo_shift) + 1) << g_fifo_shift);
			}
		}

		void sample_fifo()
		{
			const auto render = rsx::get_current_renderer();

			if (!render || !render->ctrl)
			{
				return;
			}

			const u32 get = render->iomap_table.get_addr(render->ctrl->get);
			const u32 put = render->iomap_table.get_addr(render->ctrl->put);
			note_fifo_block(get);
			note_fifo_block(put);

			// Commands between GET and PUT are still to be executed
			if (get != umax && put != umax && get < put && put - get < 0x400000)
			{
				for (u32 a = get; a < put; a += 1u << g_fifo_shift)
				{
					note_fifo_block(a);
				}
			}
		}


		// Render state that must not be rewound (Tekken 5 DR, which renders on its main thread): RSX keeps executing the
		// command buffers while the game is rolled back, so restoring them, or the game's GCM context (begin/end/current/
		// callback) that points into them, makes the RSX run stale commands (first T5DR rewind test 2026-10-19:
		// nv406e::semaphore_acquire time-outs right after the restore). Excluded: the GCM context found in the saved ranges,
		// its command buffer and the 64 KiB blocks the RSX GET/PUT pointers visit (sample_fifo). RB_SNAP_EXCLUDE_IO=1 leaves
		// all RSX mapped main memory alone instead (crashes T5DR: game objects live there too).
		void exclude_render_state()
		{
			const char* v = std::getenv("RB_SNAP_EXCLUDE_IO");
			const bool exclude_io = v && *v == '1';

			if (!exclude_io && !t5dr_active())
			{
				return;
			}

			if (t5dr_active())
			{
				g_fifo_shift = 16;
			}

			const auto render = rsx::get_current_renderer();

			if (!render || !render->ctrl)
			{
				rb_log.error("Snapshot: RSX not ready, render state not excluded");
				return;
			}

			std::vector<u32> blocks;

			for (u32 i = 0; i < render->iomap_table.ea.size(); i++)
			{
				const u32 ea = render->iomap_table.ea[i];

				if (ea != umax)
				{
					blocks.push_back(ea & ~0xfffffu);
				}
			}

			std::sort(blocks.begin(), blocks.end());

			for (usz i = 0; i < blocks.size();)
			{
				usz j = i + 1;

				while (j < blocks.size() && blocks[j] == blocks[j - 1] + 0x100000)
				{
					j++;
				}

				if (exclude_io)
				{
					g_excluded.push_back({blocks[i], blocks[j - 1] + 0x100000});
					rb_log.success("Snapshot excludes 0x%08x-0x%08x (RSX mapped main memory)", blocks[i], blocks[j - 1] + 0x100000);
				}
				else
				{
					rb_log.notice("RSX mapped main memory 0x%08x-0x%08x (only its command blocks are left out)", blocks[i], blocks[j - 1] + 0x100000);
				}

				i = j;
			}

			const u32 put = render->iomap_table.get_addr(render->ctrl->put);
			u32 found = 0;

			for (const range_t& r : g_ranges)
			{
				for (u32 a = r.begin; a + 16 <= r.end && found < 8; a += 4)
				{
					if (!vm::check_addr<16>(a, vm::page_readable))
					{
						a = (a | 0xfff) - 3;
						continue;
					}

					const u32 begin = vm::read32(a), end = vm::read32(a + 4), current = vm::read32(a + 8);

					const u32 callback = vm::read32(a + 12);

					// A real context: callback is a function descriptor in the game's data (or an HLE one), not render data
					if (begin < end && end - begin <= 0x4000000 && current >= begin && current <= end && current >= put && current - put < 0x100000 &&
						callback && !(callback & 3) && callback < 0x50000000 && vm::check_addr<8>(callback, vm::page_readable) &&
						std::any_of(blocks.begin(), blocks.end(), [&](u32 b) { return begin - b < 0x100000; }))
					{
						g_excluded.push_back({a, a + 16});
						rb_log.success("Snapshot excludes the GCM context at 0x%08x (begin 0x%08x end 0x%08x current 0x%08x callback 0x%08x, RSX put 0x%08x)",
							a, begin, end, current, callback, put);

						// Its whole command buffer is RSX territory
						for (u32 b = begin; b < end; b += 1u << g_fifo_shift)
						{
							note_fifo_block(b);
						}

						note_fifo_block(end - 1);
						found++;
					}
				}
			}

			if (!found)
			{
				rb_log.error("Snapshot: no GCM context found near RSX put 0x%08x", put);
			}
		}

		void init()
		{
			const char* list = std::getenv("RB_SNAP_RANGES");
			const char* save_at = std::getenv("RB_SNAP_SAVE_AT");
			const char* restore_at = std::getenv("RB_SNAP_RESTORE_AT");

			// A range prefixed with '~' is calibrated (only its pages that change are kept), others are always saved whole
			for (std::string item : fmt::split(list ? list : "", {","}))
			{
				const bool calibrated = !item.empty() && item[0] == '~';

				if (calibrated)
				{
					item.erase(0, 1);
				}

				const auto parts = fmt::split(item, {"-"});

				if (parts.size() == 2)
				{
					const u32 begin = static_cast<u32>(std::strtoul(parts[0].c_str(), nullptr, 16));
					const u32 end = static_cast<u32>(std::strtoul(parts[1].c_str(), nullptr, 16));

					if (end > begin)
					{
						g_ranges.push_back({begin, end});
						(calibrated ? g_cal_ranges : g_fixed_ranges).push_back({begin, end});
					}
				}
			}

			const char* exclude = std::getenv("RB_SNAP_EXCLUDE");

			for (const std::string& item : fmt::split(exclude ? exclude : "", {","}))
			{
				const auto parts = fmt::split(item, {"-"});

				if (parts.size() == 2)
				{
					g_excluded.push_back({static_cast<u32>(std::strtoul(parts[0].c_str(), nullptr, 16)), static_cast<u32>(std::strtoul(parts[1].c_str(), nullptr, 16))});
					rb_log.success("Snapshot excludes 0x%08x-0x%08x", g_excluded.back().begin, g_excluded.back().end);
				}
			}

			if (g_ranges.empty() || !save_at || !*save_at || !restore_at || !*restore_at)
			{
				g_ranges.clear();
				return;
			}

			// "+N": N game frames after the sync point (fight start), for live play where menus take different time
			if (*save_at == '+')
			{
				g_save_rel = std::strtoull(save_at + 1, nullptr, 10);
			}
			else
			{
				g_save_at = std::strtoull(save_at, nullptr, 10);
			}

			if (*restore_at == '+')
			{
				g_restore_rel = std::strtoull(restore_at + 1, nullptr, 10);
			}
			else
			{
				g_restore_at = std::strtoull(restore_at, nullptr, 10);
			}

			if (const char* stress = std::getenv("RB_SNAP_STRESS_EVERY"); stress && *stress)
			{
				g_stress_every = std::strtoull(stress, nullptr, 10);
			}

			if (const char* depth = std::getenv("RB_SNAP_DEPTH"); depth && *depth)
			{
				g_depth = std::clamp<u64>(std::strtoull(depth, nullptr, 10), 1, ring_size - 1);
			}

			if (const char* pattern = std::getenv("RB_SNAP_STRESS_PATTERN"); pattern && *pattern)
			{
				for (const std::string& item : fmt::split(pattern, {","}))
				{
					const auto parts = fmt::split(item, {":"});

					if (parts.size() == 2)
					{
						const u64 interval = std::max<u64>(std::strtoull(parts[0].c_str(), nullptr, 10), 1);
						const u64 depth = std::clamp<u64>(std::strtoull(parts[1].c_str(), nullptr, 10), 1, ring_size - 1);
						g_pattern.emplace_back(interval, depth);
					}
				}

				if (!g_pattern.empty())
				{
					g_stress_every = g_pattern[0].first;
					rb_log.success("Snapshot stress test: pattern of %u (interval:depth) entries", g_pattern.size());
				}
			}

			if (g_stress_every)
			{
				g_next_rollback = g_save_at + (g_pattern.empty() ? g_stress_every : g_pattern[0].first);
				rb_log.success("Snapshot stress test: save every frame, roll back %u frames every %u frames (first at %u)", current_depth(), g_stress_every, g_next_rollback);
			}

			if (const char* cont = std::getenv("RB_SNAP_CONT_CAL"); cont && *cont == '1')
			{
				g_cal_continuous = true;
				rb_log.success("Snapshot calibration continues during the stress test");
			}

			// RB_SNAP_RING=1: the snapshot ring without stress test or GGPO (online mode of the game, rb_online.cpp)
			if (const char* net = std::getenv("RB_NET"); net && *net == '1')
			{
				g_net_mode = true;
			}

			if (const char* ring = std::getenv("RB_SNAP_RING"); ring && *ring == '1')
			{
				g_net_mode = true;
			}

			if (const char* track = std::getenv("RB_SNAP_TRACK"); track && *track == '1')
			{
				g_track = std::make_unique<std::atomic<u8>[]>(1u << 20);

				if (const char* f = std::getenv("RB_SNAP_FOREIGN"); f && (*f == '1' || *f == '2' || *f == '3'))
				{
					g_foreign = std::make_unique<std::atomic<u8>[]>(1u << 20);
					g_foreign_track_only = *f == '2';
					g_foreign_named_only = *f == '3';
				}
				g_track_enabled = true;
				rb_log.success("Snapshot: write tracking of calibrated pages");
			}

			if (const char* cal = std::getenv("RB_SNAP_CALIBRATE_FROM"); cal && *cal)
			{
				if (*cal == '+')
				{
					g_cal_rel = std::strtoull(cal + 1, nullptr, 10);
				}
				else
				{
					g_calibrate_from = std::strtoull(cal, nullptr, 10);
					rb_log.success("Snapshot calibration from game frame %u", g_calibrate_from);
				}
			}

			exclude_render_state();

			usz total = 0;
			for (const range_t& r : g_ranges)
			{
				total += r.end - r.begin;
				rb_log.success("Snapshot range 0x%08x-0x%08x (%u KiB)", r.begin, r.end, (r.end - r.begin) >> 10);
			}

			g_buffer.resize(total);
			rb_log.success("Snapshot test: save at game frame %u, restore at %u, %u KiB in total", g_save_at, g_restore_at, total >> 10);
		}

		// The RSX command buffer must not be rewound: RSX keeps consuming it and dies with
		// "Dead FIFO commands queue state". The whole IO mapped area (117 MiB in Tekken 6) also holds
		// game objects, so only the 1 MiB blocks around the FIFO GET and PUT pointers are excluded.
		void update_fifo_blocks()
		{
			g_fifo_blocks = g_fifo_seen;
			const auto render = rsx::get_current_renderer();

			if (!render || !render->ctrl)
			{
				return;
			}

			for (const u32 offs : {+render->ctrl->get, +render->ctrl->put})
			{
				const u32 ea = render->iomap_table.get_addr(offs);

				if (ea != umax && std::find(g_fifo_blocks.begin(), g_fifo_blocks.end(), ea >> g_fifo_shift) == g_fifo_blocks.end())
				{
					g_fifo_blocks.push_back(ea >> g_fifo_shift);
					rb_log.notice("Snapshot: RSX FIFO offset 0x%x -> 0x%08x, block left untouched", offs, ea);
				}
			}
		}

		bool is_rsx_io(u32 addr)
		{
			for (const range_t& r : g_excluded)
			{
				if (addr >= r.begin && addr < r.end)
				{
					return true;
				}
			}

			return std::find(g_fifo_blocks.begin(), g_fifo_blocks.end(), addr >> g_fifo_shift) != g_fifo_blocks.end();
		}

		// Lightweight mutexes keep their lock state (owner, waiters) in guest memory, the kernel keeps the
		// matching sleep queue. Restoring a sys_lwmutex_t brings back a stale owner -> the game hangs in
		// sys_lwmutex_lock (seen with the resource manager lwmutex at 0x00D60F80). All live lwmutex control
		// structures are therefore excluded from save/restore. (sys_mutex/sys_cond are kernel objects.)
		std::vector<range_t> g_sync_excluded;
		u64 g_layout_gen = 1; // changes whenever the snapshot layout (ranges, exclusions) changes

		void update_sync_exclusions()
		{
			const usz old_count = g_sync_excluded.size();
			g_sync_excluded.clear();

			idm::select<lv2_obj, lv2_lwmutex>([&](u32, lv2_lwmutex& m)
			{
				const u32 addr = m.control.addr();
				g_sync_excluded.push_back({addr, addr + 0x20});
			});

			if (g_sync_excluded.size() != old_count)
			{
				rb_log.notice("Snapshot: %u lwmutex control structures excluded", g_sync_excluded.size());
			}

			static std::vector<range_t> previous;

			if (previous.size() != g_sync_excluded.size() || !std::equal(previous.begin(), previous.end(), g_sync_excluded.begin(),
				[](const range_t& a, const range_t& b) { return a.begin == b.begin && a.end == b.end; }))
			{
				previous = g_sync_excluded;
				g_layout_gen++;
			}
		}

		// The copy itself is split over several threads (RB_SNAP_THREADS, default 4 including the main thread):
		// a snapshot is ~20 MB and a single memcpy is limited to roughly 10 GB/s
		struct copy_piece
		{
			u8* dst;
			const u8* src;
			u32 size;
		};

		std::vector<copy_piece> g_pieces;

		// RB_SNAP_LOAD_CMP (default on, 0 = off): a load compares each piece with guest memory first and writes only the pieces
		// that differ (reading is cheaper than writing the cache lines back); counters for the copy timing log
		bool g_pieces_cmp = false;
		std::atomic<u64> g_cmp_same_bytes{0}, g_cmp_diff_bytes{0};

		void copy_one(const copy_piece& p)
		{
			if (g_pieces_cmp)
			{
				if (std::memcmp(p.dst, p.src, p.size) == 0)
				{
					g_cmp_same_bytes.fetch_add(p.size, std::memory_order_relaxed);
					return;
				}

				g_cmp_diff_bytes.fetch_add(p.size, std::memory_order_relaxed);
			}

			std::memcpy(p.dst, p.src, p.size);
		}

		void add_piece(u8* dst, const u8* src, u32 size)
		{
			// Neighbouring pieces (consecutive pages) are merged, big ones split into 256 KiB parts
			if (!g_pieces.empty())
			{
				copy_piece& last = g_pieces.back();

				if (last.dst + last.size == dst && last.src + last.size == src && last.size + size <= 0x40000)
				{
					last.size += size;
					return;
				}
			}

			g_pieces.push_back({dst, src, size});
		}

		struct copy_pool
		{
			std::atomic<u32> generation{0};
			std::atomic<usz> next{0};
			std::atomic<u32> finished{0};
			u32 workers = 0;

			void work()
			{
				for (usz i; (i = next.fetch_add(1)) < g_pieces.size();)
				{
					copy_one(g_pieces[i]);
				}
			}

			void start()
			{
				const char* v = std::getenv("RB_SNAP_THREADS");
				const u32 total = v && *v ? std::max<u32>(1, static_cast<u32>(std::strtoul(v, nullptr, 10))) : 4;
				workers = total - 1;

				for (u32 t = 0; t < workers; t++)
				{
					std::thread([this]()
					{
						u32 seen = 0;

						while (true)
						{
							generation.wait(seen);
							seen = generation.load();
							work();
							finished.fetch_add(1);
							finished.notify_one();
						}
					}).detach();
				}

				rb_log.notice("Snapshot copy: %u threads", total);
			}

			void run()
			{
				next = 0;
				finished = 0;
				generation.fetch_add(1);
				generation.notify_all();
				work();

				for (u32 f; (f = finished.load()) < workers;)
				{
					finished.wait(f);
				}
			}
		};

		copy_pool g_pool;

		void run_pieces()
		{
			static std::once_flag started;
			std::call_once(started, []() { g_pool.start(); });

			if (g_pieces.size() < 16 || !g_pool.workers)
			{
				for (const copy_piece& p : g_pieces)
				{
					copy_one(p);
				}

				return;
			}

			g_pool.run();
		}

		// Copies between guest memory and the buffer; pages that are not allocated or that RSX can
		// access are skipped
		// Guest pieces of the snapshot (guest address, offset in the buffer, size). Computing them walks every page
		// and every exclusion, so they are cached and rebuilt only when the layout changes (calibration added pages,
		// lwmutex set or RSX FIFO blocks changed).
		struct segment
		{
			u32 guest;
			u32 offset;
			u32 size;
		};

		std::vector<segment> g_segments;
		u64 g_segments_gen = 0;
		usz g_skipped_bytes = 0;

		void rebuild_segments()
		{
			g_segments.clear();
			g_skipped_bytes = 0;
			u32 offset = 0;

			for (const range_t& r : g_ranges)
			{
				for (u32 addr = r.begin; addr < r.end;)
				{
					const u32 page_end = std::min<u32>((addr & ~0xfffu) + 0x1000, r.end);
					const u32 size = page_end - addr;

					if (std::find(g_fifo_blocks.begin(), g_fifo_blocks.end(), addr >> g_fifo_shift) != g_fifo_blocks.end())
					{
						g_skipped_bytes += size;
					}
					else if (vm::check_addr(addr, vm::page_readable))
					{
						// Byte precise exclusion: RB_SNAP_EXCLUDE ranges can be smaller than a page (e.g. the
						// main/draw thread handshake inside NU::Draw::RequestManager). An earlier page-level
						// check restored such a flag without the matching sys_cond_signal -> deadlock.
						for (u32 pos = addr; pos < page_end;)
						{
							u32 piece_end = page_end;
							bool excluded = false;

							for (const auto* list : {&g_excluded, &g_sync_excluded})
							{
								for (const range_t& ex : *list)
								{
									if (pos >= ex.begin && pos < ex.end)
									{
										excluded = true;
										piece_end = std::min(piece_end, ex.end);
									}
									else if (ex.begin > pos && ex.begin < piece_end)
									{
										piece_end = ex.begin;
									}
								}
							}

							if (excluded)
							{
								g_skipped_bytes += piece_end - pos;
							}
							else if (!g_segments.empty() && g_segments.back().guest + g_segments.back().size == pos &&
								g_segments.back().offset + g_segments.back().size == offset + (pos - addr))
							{
								g_segments.back().size += piece_end - pos;
							}
							else
							{
								g_segments.push_back({pos, offset + (pos - addr), piece_end - pos});
							}

							pos = piece_end;
						}
					}

					offset += size;
					addr = page_end;
				}
			}

			g_segments_gen = g_layout_gen;
		}

		// Copies between guest memory and the buffer; pages that are not allocated or that RSX can
		// access are skipped.
		// Known problem: other game threads (the draw thread "NU::Draw::RequestManager") keep running while
		// memory is copied; a rollback under it crashed it (reading 0x110a at 0x006bbd70).
		// cpu_thread::suspend_all is not usable here: slow (7.5 FPS) and it deadlocked after 3 rollbacks.
		template <bool Save>
		void copy_ranges(u8* buf, bool update_fifo = Save)
		{
			update_sync_exclusions();

			// Saving and restoring must skip the same blocks: keep the ones found at the first save
			if (update_fifo)
			{
				update_fifo_blocks();
				g_layout_gen++;
			}

			if (g_segments_gen != g_layout_gen)
			{
				rebuild_segments();
			}

			g_pieces.clear();

			for (const segment& seg : g_segments)
			{
				u8* guest = vm::get_super_ptr<u8>(seg.guest);

				for (u32 done = 0; done < seg.size; done += 0x40000)
				{
					const u32 n = std::min<u32>(seg.size - done, 0x40000);

					if constexpr (Save)
					{
						g_pieces.push_back({buf + seg.offset + done, guest + done, n});
					}
					else
					{
						g_pieces.push_back({guest + done, buf + seg.offset + done, n});
					}
				}
			}

			run_pieces();

			static bool reported = false;

			if (g_skipped_bytes && !reported)
			{
				reported = true;
				rb_log.notice("Snapshot: %u KiB excluded (RSX FIFO blocks, RB_SNAP_EXCLUDE) left untouched", g_skipped_bytes >> 10);
			}
		}

		// The game's draw thread (NU::Draw::RequestManager) renders frame N-1 while the main thread runs
		// frame N and is practically never idle. Before a restore the main thread yields (lv2 sleep, so the
		// draw thread is scheduled) until the next guest flip: the draw thread has then finished the frame it
		// was working on and does not hold pointers into memory that the restore changes.
		void wait_for_flip()
		{
			// RB_SNAP_WAIT_FLIP=0 disables the wait (not needed when no render data is restored)
			// T5DR: off by default (render state is not restored, and re-simulated frames are not flipped at all)
			static const bool enabled = []() { const char* v = std::getenv("RB_SNAP_WAIT_FLIP"); return v && *v ? *v != '0' : !t5dr_active(); }();
			const auto ppu = get_current_cpu_thread<ppu_thread>();

			if (!enabled || !ppu)
			{
				return;
			}

			const u64 flips = frame();
			const auto start = std::chrono::steady_clock::now();

			while (frame() == flips && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(50))
			{
				sys_timer_usleep(*ppu, 200);
			}
		}

		// RB_SNAP_IDLE_WAIT=<address of a u64>: before a restore, wait (lv2 sleep) until the value is 0.
		// Tekken 6: NU::Draw::RequestManager (0x00C89800) + 0xA8 = request state, 0 = draw thread finished the
		// previous frame. The main thread waits for the same condition later in the frame anyway (FUN_005c0944),
		// so this only moves the wait earlier; render data restored after it is not in use by the draw thread.
		u64 g_wait_us_idle_total = 0, g_wait_us_idle_max = 0;
		bool g_wait_rsx_ok = false; // this wait may end once the draw thread only waits for the GPU (RB_RSX_HOLD load)

		// for_save: the wait before a save (every live frame). RB_DRAW_IDLE_RSX=2 applies the RSX criterion below only to
		// these: a save does not change guest memory, so the RSX still reading the render buffers does not matter; a load
		// keeps the full wait (RB_DRAW_IDLE_RSX=1 for loads too gave visual artifacts online, v1.2.1)
		void wait_for_draw_idle(bool for_save = false)
		{
			const diag_scope diag(diag_t::draw_wait);

			static const u32 addr = []() -> u32
			{
				const char* v = std::getenv("RB_SNAP_IDLE_WAIT");
				return v && *v ? static_cast<u32>(std::strtoul(v, nullptr, 16)) : 0;
			}();

			const auto ppu = get_current_cpu_thread<ppu_thread>();

			if (!addr || !ppu || !vm::check_addr<8>(addr, vm::page_readable))
			{
				return;
			}

			const auto start = std::chrono::steady_clock::now();

			// RB_DRAW_IDLE_RSX=1: the draw thread counts as idle once it is blocked at 0x005fa650 (FUN_005fa558: it queued an
			// RSX semaphore after the frame's commands and waits for the GPU): its work on the frame is done, only the RSX
			// still reads the render buffers (o248: every wait over 2 ms had the thread there). The request state +0xA8 is
			// cleared only after that GPU wait (~7-12 ms per frame of the waits before).
			static const char idle_rsx_mode = std::getenv("RB_DRAW_IDLE_RSX") ? *std::getenv("RB_DRAW_IDLE_RSX") : '0';
			const bool idle_rsx = idle_rsx_mode == '1' || (idle_rsx_mode == '2' && for_save) || g_wait_rsx_ok;
			static u32 draw_tid = 0;

			const auto draw_waits_for_rsx = [&]() -> bool
			{
				if (!idle_rsx)
				{
					return false;
				}

				if (!draw_tid)
				{
					idm::select<named_thread<ppu_thread>>([&](u32 id, named_thread<ppu_thread>& t)
					{
						const auto name = t.ppu_tname.load();

						if (name && name->starts_with("NU::Draw"))
						{
							draw_tid = id;
						}
					});
				}

				bool at = false;

				if (draw_tid)
				{
					idm::check<named_thread<ppu_thread>>(draw_tid, [&](named_thread<ppu_thread>& t)
					{
						at = (t.state & cpu_flag::wait) && t.cia == 0x005fa650;
					});
				}

				return at;
			};

			// RB_DRAW_WAIT_DIAG=1: where the draw thread is while the wait lasts over 2 ms (histogram of cia / wait state,
			// logged every 300 samples)
			static const bool wait_diag = std::getenv("RB_DRAW_WAIT_DIAG") && *std::getenv("RB_DRAW_WAIT_DIAG") == '1';
			bool sampled = false;

			while (*vm::get_super_ptr<u64>(addr) != 0u && !draw_waits_for_rsx() && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100))
			{
				if (wait_diag && !sampled && std::chrono::steady_clock::now() - start > std::chrono::milliseconds(2))
				{
					sampled = true;
					static std::map<std::string, u32> hist;
					static u32 samples = 0;

					idm::select<named_thread<ppu_thread>>([&](u32, named_thread<ppu_thread>& t)
					{
						const auto name = t.ppu_tname.load();

						if (name && name->starts_with("NU::Draw"))
						{
							hist[fmt::format("%s cia=0x%08x lr=0x%08x", (t.state & cpu_flag::wait) ? "WAIT" : "RUN", t.cia, static_cast<u32>(t.lr))]++;
						}
					});

					if (++samples % 300 == 0)
					{
						std::string text;

						for (const auto& [k, n] : hist)
						{
							fmt::append(text, "\n  %4u %s", n, k);
						}

						rb_log.notice("Draw thread during the idle wait (%u samples, value 0x%x):%s", samples, +*vm::get_super_ptr<u64>(addr), text);
						hist.clear();
					}
				}

				sys_timer_usleep(*ppu, 100);
			}

			const u64 us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
			g_wait_us_idle_total += us;
			g_wait_us_idle_max = std::max(g_wait_us_idle_max, us);

			if (static u64 n = 0; ++n % 600 == 0)
			{
				rb_log.notice("Draw thread idle wait: avg %u us, max %u us (600 waits), saves avg %u us", g_wait_us_idle_total / 600, g_wait_us_idle_max,
					g_save_us_total / std::max<u64>(g_saves, 1));
				g_wait_us_idle_total = g_wait_us_idle_max = 0;
			}

			if (g_rollbacks && g_last_rollback_from && state_resimulating(game_frame()))
			{
				static u64 rn = 0, rtotal = 0;
				rtotal += us;

				if (++rn % 600 == 0)
				{
					rb_log.notice("Draw thread idle wait in re-simulated frames: avg %u us (600 waits)", rtotal / 600);
					rtotal = 0;
				}
			}

			if (*vm::get_super_ptr<u64>(addr) != 0u && !draw_waits_for_rsx())
			{
				rb_log.error("Draw idle wait timed out (0x%08x = %u)", addr, +*vm::get_super_ptr<u64>(addr));
			}
		}

		u64 elapsed_us(std::chrono::steady_clock::time_point start)
		{
			return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
		}

		// Returns the number of pages that changed for the first time
		usz calibrate_step()
		{
			usz new_dirty = 0;
			g_new_pages.clear();
			g_new_dirty_pages.clear();

			if (g_cal_pages.empty())
			{
				// Without '~' ranges everything is calibrated (older behaviour)
				for (const range_t& r : g_cal_ranges.empty() ? g_ranges : g_cal_ranges)
				{
					for (u32 addr = r.begin & ~0xfffu; addr < r.end; addr += 0x1000)
					{
						const bool excluded = std::any_of(g_excluded.begin(), g_excluded.end(), [&](const range_t& ex)
						{
							return ex.begin <= addr && addr + 0x1000 <= ex.end;
						});

						if (!excluded && !is_rsx_io(addr) && vm::check_addr(addr, vm::page_readable))
						{
							g_cal_pages.push_back(addr);
						}
					}
				}

				g_cal_hashes.resize(g_cal_pages.size());
				g_cal_dirty.assign(g_cal_pages.size(), 0);
				g_cal_cooled.assign(g_cal_pages.size(), 0);
				g_cal_shadow.assign(g_cal_pages.size() * 0x1000ull, 0);

				// Write tracking first, then the shadow copy: a write in between is seen as a write
				if (g_track_enabled)
				{
					for (usz i = 0; i < g_cal_pages.size();)
					{
						usz j = i + 1;

						while (j < g_cal_pages.size() && g_cal_pages[j] == g_cal_pages[j - 1] + 0x1000)
						{
							j++;
						}

						for (usz k = i; k < j; k++)
						{
							g_track[g_cal_pages[k] >> 12] = track_clean;
						}

						protect_pages(g_cal_pages[i], static_cast<u32>((j - i) * 0x1000), utils::protection::ro);
						i = j;
					}

					g_track_active = true;
				}

				for (usz i = 0; i < g_cal_pages.size(); i++)
				{
					g_cal_index[g_cal_pages[i]] = static_cast<u32>(i);
					std::memcpy(g_cal_shadow.data() + i * 0x1000, vm::get_super_ptr<u8>(g_cal_pages[i]), 0x1000);
				}
				rb_log.success("Snapshot calibration: %u candidate pages%s", g_cal_pages.size(), g_track_enabled ? " (write tracking)" : "");
			}

			// A page is dirty once its content at a frame boundary differs from the shadow copy (taken at the first
			// step). Until then every snapshot would have held exactly the shadow content. Dirty pages are part of the
			// snapshot and are not checked again; memcmp against the shadow is much faster than hashing.
			const auto check_start = std::chrono::steady_clock::now();

			// With write tracking (RB_SNAP_TRACK=1) the first write to a clean page is caught (access violation on the
			// read-only page, or rb::note_write from the RPCS3 paths that write through the unprotected view), so only
			// the flags are checked. A slow memcmp sweep over 1/32 of the clean pages per frame verifies the tracking.
			static usz sweep_pos = 0;
			// RB_SNAP_SWEEP_ALL=1: verify every clean page every frame (diagnostics)
			static const bool sweep_all = []() { const char* v = std::getenv("RB_SNAP_SWEEP_ALL"); return v && *v == '1'; }();
			// RB_SNAP_SWEEP_LIVE_ONLY (default on, 0 = off): the safety-net sweep runs only at live boundaries, not at the
			// re-simulated saves (the tracking catches PPU writes; the sweep cost ~60-130 us per boundary with ~37k candidates)
			static const bool sweep_live_only = []() { const char* v = std::getenv("RB_SNAP_SWEEP_LIVE_ONLY"); return !v || *v != '0'; }();
			const bool sweep_now = !(sweep_live_only && !sweep_all && state_resimulating(game_frame()));
			const usz sweep_count = !g_track_active || !sweep_now ? 0 : sweep_all ? g_cal_pages.size() : g_cal_pages.size() / 256 + 1;

			const auto mark_dirty = [&](usz i)
			{
				if (!g_cal_dirty[i])
				{
					const u32 addr = g_cal_pages[i];
					g_cal_dirty[i] = 1;
					g_new_dirty_pages.push_back(addr);
					new_dirty++;
					if (g_new_pages.size() < 64) g_new_pages.push_back(fmt::format("0x%x", addr));
				}
			};

			if (g_track_active && g_cal_frames)
			{
				// Pages reported by the write tracking since the last step
				std::vector<u32> queue;
				{
					std::lock_guard lock(g_track_queue_mutex);
					queue.swap(g_track_queue);
				}

				for (const u32 addr : queue)
				{
					if (const auto found = g_cal_index.find(addr); found != g_cal_index.end())
					{
						mark_dirty(found->second);
					}
				}

				// Safety net: a few clean pages per frame are compared with their shadow
				for (usz k = 0; k < sweep_count; k++)
				{
					const usz i = (sweep_pos + k) % g_cal_pages.size();
					const u32 addr = g_cal_pages[i];

					if (!g_cal_dirty[i] && g_track[addr >> 12] != track_dirty && vm::check_addr(addr, vm::page_readable) &&
						std::memcmp(vm::get_super_ptr<u8>(addr), g_cal_shadow.data() + i * 0x1000, 0x1000) != 0)
					{
						static u32 missed = 0;

						if (missed++ < 100)
						{
							rb_log.error("Write tracking missed a write to page 0x%08x (game frame %u, track state %u)", addr, game_frame(), +g_track[addr >> 12].load());
						}

						// (not in the tracking-only mode: these are mostly SPU job results, game state)
						if (!g_foreign_track_only && !g_foreign_named_only)
						{
							mark_foreign(addr >> 12, "a host write (missed by the tracking)");
						}
						else if (!g_foreign_track_only && in_foreign_window(addr))
						{
							// The game's AT3P stream ring next to a cellAdec instance (written by the sound SPU at audio rate):
							// restored by a rollback, stale audio was played at every rollback (distorted hits) and the stream's
							// producer/consumer could disagree (stage music stopped)
							mark_foreign(addr >> 12, "a non-PPU writer next to a cellAdec instance (audio stream buffer)");
						}
						mark_dirty(i);
					}
				}
			}
			else
			{
				for (usz i = 0; g_cal_frames && i < g_cal_pages.size(); i++)
				{
					const u32 addr = g_cal_pages[i];

					if (!g_cal_dirty[i] && (!vm::check_addr(addr, vm::page_readable) ||
						std::memcmp(vm::get_super_ptr<u8>(addr), g_cal_shadow.data() + i * 0x1000, 0x1000) != 0))
					{
						mark_dirty(i);
					}
				}
			}

			if (sweep_count)
			{
				sweep_pos = (sweep_pos + sweep_count) % g_cal_pages.size();
			}

			// Time of the continuous check (logged every 600 calls)
			static u64 calls = 0, check_us = 0;
			check_us += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - check_start).count();

			if (++calls % 600 == 0)
			{
				rb_log.notice("Calibration check: %u candidate pages, %u dirty, avg %u us", g_cal_pages.size(),
					std::count(g_cal_dirty.begin(), g_cal_dirty.end(), u8{1}), check_us / 600);
				check_us = 0;
			}

			g_cal_frames++;
			return new_dirty;
		}

		// Converts a snapshot saved with `old_ranges` to the current layout (g_ranges); parts that the old
		// layout did not have are taken from the calibration shadow copy
		std::vector<u8> remap_snapshot(const std::vector<range_t>& old_ranges, const std::vector<u8>& old_buf, usz new_size)
		{
			struct old_piece { u32 begin, end; usz offset; };
			std::vector<old_piece> pieces;
			usz off = 0;

			for (const range_t& r : old_ranges)
			{
				pieces.push_back({r.begin, r.end, off});
				off += r.end - r.begin;
			}

			std::sort(pieces.begin(), pieces.end(), [](const old_piece& a, const old_piece& b) { return a.begin < b.begin; });

			std::vector<u8> out(new_size);
			usz pos = 0;

			for (const range_t& r : g_ranges)
			{
				for (u32 addr = r.begin; addr < r.end;)
				{
					const u32 end = std::min<u32>((addr & ~0xfffu) + 0x1000, r.end);
					auto it = std::upper_bound(pieces.begin(), pieces.end(), addr, [](u32 a, const old_piece& p) { return a < p.begin; });

					if (it != pieces.begin() && end <= std::prev(it)->end)
					{
						const old_piece& p = *std::prev(it);
						std::memcpy(out.data() + pos, old_buf.data() + p.offset + (addr - p.begin), end - addr);
					}
					else if (auto found = g_cal_index.find(addr & ~0xfffu); found != g_cal_index.end())
					{
						std::memcpy(out.data() + pos, g_cal_shadow.data() + found->second * 0x1000ull + (addr & 0xfff), end - addr);
					}

					pos += end - addr;
					addr = end;
				}
			}

			return out;
		}

		// Replaces the snapshot ranges with the pages that changed during calibration
		void finish_calibration()
		{
			std::vector<range_t> ranges;

			if (!g_cal_ranges.empty())
			{
				ranges = g_fixed_ranges;
			}

			for (usz i = 0; i < g_cal_pages.size(); i++)
			{
				if (!g_cal_dirty[i])
				{
					continue;
				}

				const u32 addr = g_cal_pages[i];

				if (!ranges.empty() && ranges.back().end == addr)
				{
					ranges.back().end = addr + 0x1000;
				}
				else
				{
					ranges.push_back({addr, addr + 0x1000});
				}
			}

			usz total = 0;
			for (const range_t& r : ranges)
			{
				total += r.end - r.begin;
			}

			rb_log.success("Snapshot calibration over %u frames: %u of %u pages changed, %u ranges, %u KiB",
				g_cal_frames, total >> 12, g_cal_pages.size(), ranges.size(), total >> 10);

			// Size per 16 MiB region (first calibration and then every 500 frames)
			if (static u64 last_report = 0; !last_report || g_cal_frames >= last_report + 500)
			{
				last_report = g_cal_frames;
				std::map<u32, usz> per_region;

				for (const range_t& r : ranges)
				{
					per_region[r.begin >> 24] += r.end - r.begin;
				}

				std::string text;

				for (const auto& [region, size] : per_region)
				{
					fmt::append(text, " 0x%02x: %u KiB;", region, size >> 10);
				}

				rb_log.notice("Snapshot size per region:%s", text);

				if (last_report == g_cal_frames && g_cal_frames == 0 + last_report)
				{
					std::string list;

					for (const range_t& r : ranges)
					{
						fmt::append(list, "%x-%x,", r.begin, r.end);
					}

					rb_log.notice("Snapshot ranges: %s", list);
				}
			}

			g_ranges = std::move(ranges);
			g_buffer.resize(total);
			g_cal_done = true;
			g_layout_gen++;
		}
	}

	bool state_is_speculative(u64 game_frame, u64& predicted_from)
	{
		static const bool predict = []()
		{
			const char* v = std::getenv("RB_SNAP_PREDICT");
			return v && *v == '1';
		}();

		if (!predict || !g_stress_every || !g_has_snapshot || game_frame <= g_save_at)
		{
			return false;
		}

		// Frames inside the window of the next scheduled rollback are simulated with a prediction first
		const u64 next = g_next_rollback;
		const u64 window_begin = next - current_depth();

		if (game_frame >= window_begin && game_frame < next && window_begin > g_save_at)
		{
			predicted_from = window_begin - 1;
			return true;
		}

		return false;
	}

	// Ring of the stress test, stored per page: every page of the snapshot has one block with a copy for each ring
	// slot. Pages that start or stop changing are added/removed individually (no copy of the whole ring).
	struct page_unit
	{
		u32 page;
		u8* data; // ring_size copies of the page
		std::vector<std::pair<u16, u16>> pieces; // (offset, size) inside the page that are saved/restored
	};

	struct page_store
	{
		static constexpr usz block_size = ring_size * 0x1000ull;

		std::vector<page_unit> units;
		std::unordered_map<u32, u32> index;
		std::vector<std::unique_ptr<u8[]>> slabs;
		std::vector<u8*> free_blocks;
		u64 pieces_gen = 0;

		u8* alloc()
		{
			if (free_blocks.empty())
			{
				slabs.emplace_back(new u8[block_size * 64]);

				for (usz i = 0; i < 64; i++)
				{
					free_blocks.push_back(slabs.back().get() + i * block_size);
				}
			}

			u8* block = free_blocks.back();
			free_blocks.pop_back();
			return block;
		}

		// Parts of a page that are saved: fixed ranges and calibrated pages, minus RB_SNAP_EXCLUDE, lwmutexes,
		// RSX FIFO blocks and unmapped memory
		static std::vector<std::pair<u16, u16>> pieces_of(u32 page)
		{
			std::vector<std::pair<u16, u16>> out;

			if (!vm::check_addr(page, vm::page_readable) || std::find(g_fifo_blocks.begin(), g_fifo_blocks.end(), page >> g_fifo_shift) != g_fifo_blocks.end())
			{
				return out;
			}

			std::array<bool, 0x1000> mask{};
			const auto mark = [&](const range_t& r, bool value)
			{
				const u32 b = std::max(r.begin, page), e = std::min(r.end, page + 0x1000);

				for (u32 a = b; a < e; a++)
				{
					mask[a - page] = value;
				}
			};

			for (const range_t& r : g_fixed_ranges)
			{
				mark(r, true);
			}

			if (g_cal_index.count(page))
			{
				mark({page, page + 0x1000}, true);
			}

			for (const auto* list : {&g_excluded, &g_sync_excluded})
			{
				for (const range_t& r : *list)
				{
					mark(r, false);
				}
			}

			for (u32 i = 0; i < 0x1000;)
			{
				if (!mask[i])
				{
					i++;
					continue;
				}

				u32 j = i;

				while (j < 0x1000 && mask[j])
				{
					j++;
				}

				out.emplace_back(static_cast<u16>(i), static_cast<u16>(j - i));
				i = j;
			}

			return out;
		}

		void add(u32 page, const u8* init)
		{
			if (index.count(page) || (g_foreign && !g_foreign_track_only && g_foreign[page >> 12]))
			{
				return;
			}

			u8* data = alloc();

			for (u32 s = 0; s < ring_size; s++)
			{
				std::memcpy(data + s * 0x1000ull, init, 0x1000);
			}

			index[page] = static_cast<u32>(units.size());
			units.push_back({page, data, pieces_of(page)});
		}

		void remove(u32 page)
		{
			const auto found = index.find(page);

			if (found == index.end())
			{
				return;
			}

			const u32 idx = found->second;
			free_blocks.push_back(units[idx].data);
			index.erase(found);

			if (idx != units.size() - 1)
			{
				units[idx] = std::move(units.back());
				index[units[idx].page] = idx;
			}

			units.pop_back();
		}

		void refresh()
		{
			// lwmutex set or FIFO blocks changed: recompute the saved parts of every page
			if (pieces_gen != g_layout_gen)
			{
				for (page_unit& u : units)
				{
					u.pieces = pieces_of(u.page);
				}

				pieces_gen = g_layout_gen;
			}
		}

		// Load only: pages left as they are (RB_RSX_HOLD: the pages the RSX reads)
		const std::unordered_map<u32, u64>* skip_pages = nullptr;

		template <bool Save>
		void copy(u32 slot)
		{
			const diag_scope diag(Save ? diag_t::save : diag_t::load);

			// No page may be unmapped while the copy threads use it (r7: the file loader thread freed a heap page between the
			// check below and the copy, the copy threads crashed reading it); vm::dealloc takes the same lock
			// (the lock requires the caller's cpu_flag::wait, as for any thread that may block on it)
			struct wait_flag
			{
				cpu_thread* cpu = cpu_thread::get_current();
				wait_flag() { if (cpu) cpu->state += cpu_flag::wait; }
				~wait_flag() { if (cpu) { cpu->state -= cpu_flag::wait; cpu->check_state(); } }
			} wait;

			// RB_SNAP_COPY_TIMING (default on, 0 = off): host time of the copy parts (lock + layout, building the pieces, the copy),
			// logged every 600 saves / loads
			static const bool timing = []() { const char* v = std::getenv("RB_SNAP_COPY_TIMING"); return !v || *v != '0'; }();
			static const bool load_cmp = []() { const char* v = std::getenv("RB_SNAP_LOAD_CMP"); return !v || *v != '0'; }();
			const auto t0 = std::chrono::steady_clock::now();

			vm::writer_lock vm_lock;

			update_sync_exclusions();
			refresh();
			const auto t1 = std::chrono::steady_clock::now();
			g_pieces.clear();

			for (const page_unit& u : units)
			{
				// A page the game freed meanwhile (sys_memory_free at a round transition): reading it crashed the copy threads
				if (!vm::check_addr(u.page, vm::page_readable))
				{
					continue;
				}

				if constexpr (!Save)
				{
					if (skip_pages && skip_pages->count(u.page))
					{
						continue;
					}
				}

				u8* guest = vm::get_super_ptr<u8>(u.page);
				u8* saved = u.data + slot * 0x1000ull;

				for (const auto& [off, size] : u.pieces)
				{
					if constexpr (Save)
					{
						g_pieces.push_back({saved + off, guest + off, size});
					}
					else
					{
						g_pieces.push_back({guest + off, saved + off, size});
					}
				}
			}

			const auto t2 = std::chrono::steady_clock::now();
			g_pieces_cmp = !Save && load_cmp;
			run_pieces();
			g_pieces_cmp = false;

			if (timing)
			{
				const auto t3 = std::chrono::steady_clock::now();
				const auto us = [](auto a, auto b) { return static_cast<u64>(std::chrono::duration_cast<std::chrono::microseconds>(b - a).count()); };

				struct stat_t { u64 n = 0, lock = 0, build = 0, run = 0, lock_max = 0, build_max = 0, run_max = 0, pieces = 0, bytes = 0; };
				static stat_t stats[2];
				stat_t& st = stats[Save];
				const u64 l = us(t0, t1), b = us(t1, t2), r = us(t2, t3);
				st.n++;
				st.lock += l, st.build += b, st.run += r;
				st.lock_max = std::max(st.lock_max, l), st.build_max = std::max(st.build_max, b), st.run_max = std::max(st.run_max, r);
				st.pieces += g_pieces.size();

				for (const copy_piece& p : g_pieces)
				{
					st.bytes += p.size;
				}

				if (st.n == 600)
				{
					std::string cmp;

					if (!Save && load_cmp)
					{
						const u64 same = g_cmp_same_bytes.exchange(0), diff = g_cmp_diff_bytes.exchange(0);
						cmp = fmt::format(", compare: %u%% unchanged (%u KiB skipped, %u KiB written per load)", same + diff ? same * 100 / (same + diff) : 0,
							same / 600 / 1024, diff / 600 / 1024);
					}

					rb_log.notice("Snapshot copy timing (%s, 600 calls): lock+layout avg %u us max %u, pieces avg %u us max %u, copy avg %u us max %u; %u pieces, %u KiB per call%s",
						Save ? "save" : "load", st.lock / 600, st.lock_max, st.build / 600, st.build_max, st.run / 600, st.run_max, st.pieces / 600, st.bytes / 600 / 1024, cmp);
					st = {};
				}
			}
		}

		usz bytes() const
		{
			return units.size() * 0x1000ull;
		}
	};

	static page_store g_store;

	// RB_RSX_READS=1 (diagnostics): before every online load, which snapshot pages the RSX reads (rsx_capture.cpp collects
	// them per draw) the load would change - those tear the frame the RSX may still be drawing when the load does not
	// wait for it (RB_DRAW_IDLE_RSX=1). Logged every 300 loads as ranges: loads that changed the range / draws reading it.
	// Second part: RSX-read pages (in or out of the snapshot) written between the load and the end of the re-simulation -
	// the re-simulated frames write them while the RSX may still read them
	static std::map<u32, u64> g_rsx_before;
	static std::map<u32, u32> g_rsx_read_pages_all;
	static bool g_rsx_guard_pending = false;
	static std::map<u32, u32> g_rsx_resim_changed;

	static u64 rsx_page_hash(u32 page)
	{
		const u64* p = reinterpret_cast<const u64*>(vm::get_super_ptr<u8>(page));
		u64 h = 0xcbf29ce484222325;

		for (u32 i = 0; i < 512; i++)
		{
			h = (h ^ p[i]) * 0x100000001b3;
		}

		return h;
	}

	// RB_RSX_READS=2: the RSX-read pages of the snapshot are made inaccessible from the load to the end of the
	// re-simulation; every first access (thread, code address, read / write) is counted - what the re-simulation needs
	// of them or writes into them (a load that leaves them to the RSX must know)
	static std::atomic<bool> g_guard_active{false};
	static std::mutex g_guard_mutex;
	static std::set<u32> g_guard_pages;
	static std::map<std::string, u32> g_guard_hits;
	static std::map<u32, u32> g_guard_page_hits;
	static u64 g_guard_total = 0;

	static void guard_release(u32 page)
	{
		if (vm::check_addr(page, vm::page_readable))
		{
			const bool clean = g_track_active && g_track && g_track[page >> 12].load() == track_clean;
			utils::memory_protect(vm::g_base_addr + page, 0x1000, clean ? utils::protection::ro : utils::protection::rw);
		}
	}

	static void guard_end()
	{
		std::lock_guard lock(g_guard_mutex);

		for (const u32 page : g_guard_pages)
		{
			guard_release(page);
		}

		g_guard_pages.clear();
		g_guard_active = false;
	}

	static bool hold_fault(u32 addr, bool is_writing);

	bool on_guard_fault(u32 addr, bool is_writing)
	{
		if (hold_fault(addr, is_writing))
		{
			return true;
		}

		if (!g_guard_active)
		{
			return false;
		}

		std::lock_guard lock(g_guard_mutex);
		const u32 page = addr & ~0xfffu;

		if (!g_guard_pages.erase(page))
		{
			return false;
		}

		std::string who;
		const auto cpu = cpu_thread::get_current();

		if (cpu && cpu->get_class() == thread_class::ppu)
		{
			const auto& ppu = *static_cast<ppu_thread*>(cpu);
			const auto name = ppu.ppu_tname.load();
			who = fmt::format("%s cia 0x%08x lr 0x%08x", cpu->id == 0x1000000 ? "main" : name ? name->c_str() : "ppu", ppu.cia, static_cast<u32>(ppu.lr));
		}
		else
		{
			who = std::string(thread_ctrl::get_name());
		}

		g_guard_hits[fmt::format("%s %s", is_writing ? "W" : "R", who)]++;
		g_guard_page_hits[page]++;
		guard_release(page);
		return true;
	}

	static void rsx_reads_resim_end()
	{
		guard_end();
		for (const auto& [page, h] : g_rsx_before)
		{
			if (vm::check_addr(page, vm::page_readable) && rsx_page_hash(page) != h)
			{
				g_rsx_resim_changed[page]++;
			}
		}

		g_rsx_before.clear();
	}

	// RB_RSX_HOLD=1: online rollback without waiting for the RSX (players' logs 2026-10-10: the wait before the load was
	// ~11 ms per rollback = nearly every rollback dropped a frame). The draw thread is done with the frame once it waits
	// for the GPU (FUN_005fa558, wait_for_draw_idle RSX criterion); what the RSX still reads of the snapshot are render
	// outputs (skinned vertex data etc., ~340 pages changed per load, tests fd1-fd3) that the fight logic never touches
	// in a re-simulation (guard test fd3: 0 accesses of the main thread in 300 rollbacks). So:
	// - the pages the RSX read in the last 2 s (rsx_capture.cpp, sampled) are not restored by an online load;
	// - while the RSX is still busy after the load they are write-protected: a writer (SPU job of a re-simulated frame,
	//   ~3 % of the rollbacks) waits in the fault handler until the RSX is done; the hold ends with the re-simulation.
	// The set is dropped at every game mode change (round transitions free and reuse memory).
	static const bool g_rsx_hold = []() { const char* v = std::getenv("RB_RSX_HOLD"); return v && *v == '1'; }();
	static std::unordered_map<u32, u64> g_hold_recent; // page -> time (ms) the RSX last read it
	static std::atomic<bool> g_hold_active{false};
	static std::mutex g_hold_mutex;
	static std::vector<u32> g_hold_pages; // write-protected now
	static u64 g_hold_waits = 0, g_hold_wait_us = 0, g_hold_loads = 0, g_hold_busy_loads = 0, g_hold_skipped = 0;
	static std::map<std::string, std::pair<u32, u64>> g_hold_who; // writer -> waits, us

	// Game mode of the battle (mode state *0x00967cf4 + 0xc, recordsnalysis_game_modes.md); 4 = fight
	static u32 hold_game_mode()
	{
		const u32 base = vm::check_addr<4>(0x00967cf4, vm::page_readable) ? static_cast<u32>(vm::read32(0x00967cf4)) : 0u;
		return base && vm::check_addr<4>(base + 0xc, vm::page_readable) ? static_cast<u32>(vm::read32(base + 0xc)) : u32{umax};
	}

	static u64 hold_now_ms()
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	static bool hold_draw_idle()
	{
		static const u32 addr = []() -> u32
		{
			const char* v = std::getenv("RB_SNAP_IDLE_WAIT");
			return v && *v ? static_cast<u32>(std::strtoul(v, nullptr, 16)) : 0;
		}();

		return !addr || !vm::check_addr<8>(addr, vm::page_readable) || *vm::get_super_ptr<u64>(addr) == 0u;
	}

	// Main thread, every frame boundary: new RSX reads, old ones forgotten
	static void hold_update()
	{
		if (!g_rsx_hold)
		{
			return;
		}

		const u64 now = hold_now_ms();

		for (const auto& [page, n] : rsx::capture::rb_take_read_pages())
		{
			g_hold_recent[page] = now;
		}

		static u32 last_mode = umax;

		if (const u32 mode = hold_game_mode(); mode != last_mode)
		{
			last_mode = mode;
			g_hold_recent.clear();
		}

		if (static u32 n = 0; ++n % 64 == 0)
		{
			std::erase_if(g_hold_recent, [&](const auto& e) { return now - e.second > 2000; });
		}
	}

	static void hold_release_locked()
	{
		for (const u32 page : g_hold_pages)
		{
			if (vm::check_addr(page, vm::page_readable))
			{
				const bool clean = g_track_active && g_track && g_track[page >> 12].load() == track_clean;
				utils::memory_protect(vm::g_base_addr + page, 0x1000, clean ? utils::protection::ro : utils::protection::rw);
			}
		}

		g_hold_pages.clear();
		g_hold_active = false;
	}

	// Main thread, frame boundaries: the RSX is done -> writers are free again
	static void hold_check()
	{
		if (g_hold_active && hold_draw_idle())
		{
			std::lock_guard lock(g_hold_mutex);
			hold_release_locked();
		}
	}

	// After an online load that did not wait for the RSX
	static void hold_begin()
	{
		std::lock_guard lock(g_hold_mutex);
		hold_release_locked();
		g_hold_loads++;

		if (!hold_draw_idle())
		{
			g_hold_busy_loads++;

			for (const auto& [page, t] : g_hold_recent)
			{
				if (g_store.index.count(page) && vm::check_addr(page, vm::page_readable))
				{
					g_hold_pages.push_back(page);
				}
			}

			std::sort(g_hold_pages.begin(), g_hold_pages.end());

			for (usz i = 0; i < g_hold_pages.size();)
			{
				usz j = i + 1;

				while (j < g_hold_pages.size() && g_hold_pages[j] == g_hold_pages[j - 1] + 0x1000)
				{
					j++;
				}

				utils::memory_protect(vm::g_base_addr + g_hold_pages[i], (j - i) * 0x1000, utils::protection::ro);
				i = j;
			}

			g_hold_skipped += g_hold_pages.size();
			g_hold_active = !g_hold_pages.empty();
		}

		if (g_hold_loads % 300 == 0)
		{
			rb_log.notice("RSX hold (300 loads): %u with the RSX still busy, %u pages held per such load, %u writers waited (avg %u us), %u pages in the RSX set",
				g_hold_busy_loads, g_hold_skipped / std::max<u64>(g_hold_busy_loads, 1), g_hold_waits, g_hold_wait_us / std::max<u64>(g_hold_waits, 1), g_hold_recent.size());
			std::string text;

			for (const auto& [who, w] : g_hold_who)
			{
				fmt::append(text, " [%s: %u x, avg %u us]", who, w.first, w.second / std::max<u32>(w.first, 1));
			}

			rb_log.notice("RSX hold writers:%s", text);
			g_hold_who.clear();
			g_hold_busy_loads = g_hold_skipped = g_hold_waits = g_hold_wait_us = 0;
		}
	}

	// Any thread: write fault on a held page -> wait for the RSX (at most 100 ms), then all held pages are free
	static bool hold_fault(u32 addr, bool is_writing)
	{
		if (!g_hold_active || !is_writing)
		{
			return false;
		}

		{
			std::lock_guard lock(g_hold_mutex);

			if (!std::binary_search(g_hold_pages.begin(), g_hold_pages.end(), addr & ~0xfffu))
			{
				return false;
			}
		}

		// Who writes (logged with the statistics)
		std::string who;
		const auto cpu = cpu_thread::get_current();

		if (cpu && cpu->get_class() == thread_class::ppu)
		{
			const auto& ppu = *static_cast<ppu_thread*>(cpu);
			const auto name = ppu.ppu_tname.load();
			who = fmt::format("%s cia 0x%08x lr 0x%08x", cpu->id == 0x1000000 ? "main" : name ? name->c_str() : "ppu", ppu.cia, static_cast<u32>(ppu.lr));
		}
		else
		{
			who = std::string(thread_ctrl::get_name());
		}

		// Only the game's threads wait: the RSX thread (or any host thread) would wait for itself
		const bool game_thread = cpu && (cpu->get_class() == thread_class::ppu || cpu->get_class() == thread_class::spu);
		const auto start = std::chrono::steady_clock::now();

		// The waiting thread counts as waiting for the emulator (the RSX thread pauses the cpu threads for some of its
		// work: without the flag it waited for this thread and this thread for it - test hd2: 28 ms per wait)
		if (game_thread)
		{
			cpu->state += cpu_flag::wait;
		}

		while (game_thread && g_hold_active && !hold_draw_idle() && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100))
		{
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}

		if (game_thread)
		{
			cpu->state -= cpu_flag::wait;
		}

		std::lock_guard lock(g_hold_mutex);

		if (g_hold_active)
		{
			const u64 us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
			g_hold_waits++;
			g_hold_wait_us += us;
			auto& w = g_hold_who[who];
			w.first++;
			w.second += us;
			hold_release_locked();
		}

		return true;
	}

	static void rsx_reads_check(u32 slot)
	{
		if (!rsx::capture::rb_reads_enabled())
		{
			return;
		}

		static std::map<u32, u32>& read_pages = g_rsx_read_pages_all;
		static std::map<u32, u32> changed;
		static u64 loads = 0;

		for (const auto& [page, n] : rsx::capture::rb_take_read_pages())
		{
			read_pages[page] += n;
		}

		for (const auto& [page, n] : read_pages)
		{
			const auto found = g_store.index.find(page);

			if (found != g_store.index.end() && vm::check_addr(page, vm::page_readable) &&
				std::memcmp(vm::get_super_ptr<u8>(page), g_store.units[found->second].data + slot * 0x1000ull, 0x1000) != 0)
			{
				changed[page]++;
			}
		}

		if (++loads % 300 == 0)
		{
			u32 in_snap = 0, excluded = 0;

			for (const auto& [page, n] : read_pages)
			{
				const int st = state_addr_saved(page);
				in_snap += st == 1;
				excluded += st == 2;
			}

			std::string text;
			u32 begin = 0, end = 0, peak = 0;
			u64 draws = 0;

			const auto flush = [&]()
			{
				if (end)
				{
					fmt::append(text, " 0x%08x-0x%08x(%u/%u)", begin, end, peak, draws);
				}
			};

			for (const auto& [page, n] : changed)
			{
				if (page != end)
				{
					flush();
					begin = page;
					peak = 0;
					draws = 0;
				}

				end = page + 0x1000;
				peak = std::max(peak, n);
				draws += read_pages.count(page) ? read_pages[page] : 0;
			}

			flush();
			rb_log.notice("RSX reads (%u loads): %u pages read by the RSX, %u in the snapshot, %u excluded; changed by loads (loads/draws):%s",
				loads, read_pages.size(), in_snap, excluded, text);
			changed.clear();

			text.clear();
			begin = end = peak = 0;
			draws = 0;

			for (const auto& [page, n] : g_rsx_resim_changed)
			{
				if (page != end)
				{
					flush();
					begin = page;
					peak = 0;
					draws = 0;
				}

				end = page + 0x1000;
				peak = std::max(peak, n);
				draws += read_pages.count(page) ? read_pages[page] : 0;
			}

			flush();
			rb_log.notice("RSX reads (%u loads): %u pages written during the re-simulations (rollbacks/draws):%s", loads, g_rsx_resim_changed.size(), text);
			g_rsx_resim_changed.clear();
		}

		// After the load (the caller copies next; the excluded pages do not change by it): the content the re-simulation starts with
		g_rsx_before.clear();

		for (const auto& [page, n] : read_pages)
		{
			if (state_addr_saved(page) != 1 && vm::check_addr(page, vm::page_readable))
			{
				g_rsx_before[page] = rsx_page_hash(page);
			}
		}

		static const bool guard = []() { const char* v = std::getenv("RB_RSX_READS"); return v && *v == '2'; }();

		if (guard)
		{
			if (loads % 300 == 0)
			{
				std::lock_guard lock(g_guard_mutex);
				std::vector<std::pair<u32, std::string>> top;

				for (const auto& [who, n] : g_guard_hits)
				{
					top.emplace_back(n, who);
				}

				std::sort(top.rbegin(), top.rend());
				std::string text;

				for (usz i = 0; i < top.size() && i < 40; i++)
				{
					fmt::append(text, "\n  %6u %s", top[i].first, top[i].second);
				}

				u32 begin = 0, end = 0, peak = 0;

				const auto flush = [&]()
				{
					if (end)
					{
						fmt::append(text, " 0x%08x-0x%08x(%u)", begin, end, peak);
					}
				};

				text += "\n  pages:";

				for (const auto& [page, n] : g_guard_page_hits)
				{
					if (page != end)
					{
						flush();
						begin = page;
						peak = 0;
					}

					end = page + 0x1000;
					peak = std::max(peak, n);
				}

				flush();
				rb_log.notice("RSX reads guard (%u loads, %u pages guarded in total): first accesses during the re-simulations:%s", loads, g_guard_total, text);
				g_guard_hits.clear();
				g_guard_page_hits.clear();
				g_guard_total = 0;
			}

			g_rsx_guard_pending = true;
		}
	}

	// After the load's copy: guard the RSX-read snapshot pages (see on_guard_fault)
	static void rsx_reads_after_load()
	{
		if (!std::exchange(g_rsx_guard_pending, false))
		{
			return;
		}

		std::lock_guard lock(g_guard_mutex);

		for (const u32 page : g_guard_pages)
		{
			guard_release(page);
		}

		g_guard_pages.clear();

		for (const auto& [page, n] : g_rsx_read_pages_all)
		{
			if (g_store.index.count(page) && vm::check_addr(page, vm::page_readable))
			{
				g_guard_pages.insert(page);
				utils::memory_protect(vm::g_base_addr + page, 0x1000, utils::protection::no);
			}
		}

		g_guard_total += g_guard_pages.size();
		g_guard_active = true;
	}

	// Cooling (with write tracking): a calibrated page that is identical in every ring slot and in guest memory is
	// not changing any more (e.g. a buffer filled once during the round intro). It is dropped from the snapshot
	// and tracked again: shadow = its current content, read-only. A later write brings it back like any new page
	// (older snapshots get the shadow content, which they all had). The check covers 1/64 of the pages per frame;
	// the layout change is applied once per sweep, because remapping the ring copies all of it.
	static void cold_check()
	{
		static const bool enabled = []() { const char* v = std::getenv("RB_SNAP_COOL"); return !v || *v != '0'; }();

		if (!enabled || !g_track_active || !g_cal_done || g_cal_pages.empty())
		{
			return;
		}

		for (u32 i = 0; i < ring_size; i++)
		{
			if (!g_ring_valid[i])
			{
				return;
			}
		}

		static usz pos = 0;
		usz cooled = 0;
		const usz n = g_cal_pages.size();
		const usz step = n / 64 + 1;

		// RB_SNAP_COOL_HYST (default 0 = off): a page cooled down this many times (it became dirty again after cooling) is
		// not cooled any more. Off by default: most pages cool and re-heat in bursts, so with a limit of 2 they all stayed in
		// the store for good (o165: 2000 -> 4800 pages within 20 s, load/save 1.35 -> 2.8 ms); the churn is cheaper.
		static const u32 cool_max = []() { const char* v = std::getenv("RB_SNAP_COOL_HYST"); return v ? static_cast<u32>(std::atoi(v)) : 0u; }();

		if (g_cal_cooled.size() != n)
		{
			g_cal_cooled.assign(n, 0);
		}

		for (usz k = 0; k < step; k++)
		{
			const usz i = (pos + k) % n;

			if (!g_cal_dirty[i] || (cool_max && g_cal_cooled[i] >= cool_max))
			{
				continue;
			}

			const u32 addr = g_cal_pages[i];

			// RB_SNAP_COOL_RANGES="a-b,...": only pages inside these ranges may cool down (default: all)
			static const std::vector<range_t> cool_ranges = []()
			{
				std::vector<range_t> out;
				const char* v = std::getenv("RB_SNAP_COOL_RANGES");

				for (const std::string& item : fmt::split(v ? v : "", {","}))
				{
					const auto parts = fmt::split(item, {"-"});

					if (parts.size() == 2)
					{
						out.push_back({static_cast<u32>(std::stoul(parts[0], nullptr, 16)), static_cast<u32>(std::stoul(parts[1], nullptr, 16))});
					}
				}

				return out;
			}();

			if (!cool_ranges.empty() && std::none_of(cool_ranges.begin(), cool_ranges.end(), [&](const range_t& r) { return addr >= r.begin && addr < r.end; }))
			{
				continue;
			}

			const auto found = g_store.index.find(addr);

			if (found == g_store.index.end() || !vm::check_addr(addr, vm::page_readable))
			{
				continue;
			}

			const u8* cur = vm::get_super_ptr<u8>(addr);
			const u8* copies = g_store.units[found->second].data;
			bool same = true;

			for (u32 r = 0; r < ring_size && same; r++)
			{
				same = std::memcmp(cur, copies + r * 0x1000ull, 0x1000) == 0;
			}

			if (!same)
			{
				continue;
			}

			// Shadow first, then read-only, then verify: a write before the protection is seen by the verify,
			// a write after it by the access violation handler
			u8* shadow = g_cal_shadow.data() + i * 0x1000ull;
			std::memcpy(shadow, cur, 0x1000);
			g_track[addr >> 12] = track_clean;
			protect_pages(addr, 0x1000, utils::protection::ro);

			if (std::memcmp(shadow, cur, 0x1000) != 0)
			{
				if (u8 st = track_clean; g_track[addr >> 12].compare_exchange_strong(st, track_dirty))
				{
					protect_pages(addr, 0x1000, utils::protection::rw);
					track_mark_dirty(addr >> 12);
				}

				continue;
			}

			g_cal_dirty[i] = 0;
			g_cal_cooled[i] = static_cast<u8>(std::min<u32>(g_cal_cooled[i] + 1u, 255u));
			g_store.remove(addr);
			cooled++;
		}

		pos = (pos + step) % n;

		static usz total_cooled = 0;
		static u64 last_report = 0;
		total_cooled += cooled;

		if (game_frame() >= last_report + 600)
		{
			last_report = game_frame();
			rb_log.notice("Snapshot: %u pages cooled down so far, snapshot now %u KiB (%u pages)", total_cooled, g_store.bytes() >> 10, g_store.units.size());
		}
	}

	// Removes the pages marked by mark_foreign from the snapshot (main thread, between frames)
	static void remove_foreign_pages()
	{
		std::vector<u32> pages;

		{
			std::lock_guard lock(g_foreign_mutex);
			pages.swap(g_foreign_queue);
		}

		for (const u32 addr : pages)
		{
			g_store.remove(addr);
		}

		if (!pages.empty())
		{
			static u64 total = 0;
			total += pages.size();
			rb_log.notice("Snapshot: %u foreign pages removed (%u in total)", pages.size(), total);
		}
	}

	void rollback_csv(const std::string& line)
	{
		static const char* path = std::getenv("RB_ROLLBACK_CSV");

		if (!path || !*path)
		{
			return;
		}

		static fs::file f(fmt::format("%s_%u.csv", path, static_cast<u32>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xffff)), fs::rewrite);
		f.write(fmt::format("%s,%u\n", line, ::g_audio_dump_frames.load()));
	}

	u64 state_load_count()
	{
		return g_rollbacks;
	}

	int state_addr_saved(u32 addr)
	{
		for (const auto* list : {&g_excluded, &g_sync_excluded})
		{
			for (const range_t& r : *list)
			{
				if (addr >= r.begin && addr < r.end)
				{
					return 2;
				}
			}
		}

		return g_store.index.count(addr & ~0xfffu) ? 1 : 0;
	}

	static std::atomic<u32> g_kernel_changes{0};
	static u32 g_kernel_changes_seen = 0;

	void state_kernel_change(const char* what, u32 addr)
	{
		// RB_KERNEL_IGNORE_THREADS (name prefixes, default "bnus,CnuSound,HLE ,fios,SpursHdlr,_libpngdec"): kernel objects
		// created / destroyed by the sound, decoder and loader threads do not drop the snapshots. Their state is outside the
		// rollback anyway; dropping the ring when a music/voice decoder ended left a misprediction found a few frames later
		// with nothing to roll back to ("refused: no snapshot", a permanent desync, o196).
		if (const auto cpu = cpu_thread::get_current(); cpu && cpu->get_class() == thread_class::ppu && cpu->id != 0x1000000)
		{
			static const std::vector<std::string> prefixes = []()
			{
				const char* v = std::getenv("RB_KERNEL_IGNORE_THREADS");
				return fmt::split(v ? v : "bnus,CnuSound,HLE ,fios,SpursHdlr,_libpngdec", {","});
			}();

			const auto name_ptr = static_cast<ppu_thread*>(cpu)->ppu_tname.load();

			// lwmutex / lwcond created or destroyed by any thread other than the main thread: their control structures are excluded
			// from the snapshot (update_sync_exclusions) and the game only keeps pointers to them, so a load across the create /
			// destroy is harmless. The trophy unlock at every KO (any_workerThread) created and destroyed one and dropped the ring:
			// a misprediction in the last frames before the KO could not be rolled back (o197).
			if (std::string_view{what}.starts_with("_sys_lwmutex_") || std::string_view{what}.starts_with("_sys_lwcond_"))
			{
				static std::mutex lw_mutex;
				static std::set<std::string> lw_reported;
				std::lock_guard lock(lw_mutex);

				if (lw_reported.size() < 32 && lw_reported.insert(std::string(what) + (name_ptr ? *name_ptr : std::string())).second)
				{
					rb_log.notice("Snapshot: kernel change %s(0x%x) by %s ignored (lightweight sync object of a non-main thread)", what, addr,
						name_ptr ? *name_ptr : std::string("?"));
				}

				return;
			}

			if (name_ptr && std::any_of(prefixes.begin(), prefixes.end(), [&](const std::string& p) { return !p.empty() && name_ptr->starts_with(p); }))
			{
				static std::mutex mutex;
				static std::set<std::string> reported;
				std::lock_guard lock(mutex);

				if (reported.size() < 64 && reported.insert(what).second)
				{
					rb_log.notice("Snapshot: kernel change %s(0x%x) by %s ignored (thread outside the rollback)", what, addr, *name_ptr);
				}

				return;
			}
		}

		g_kernel_changes++;
		static std::atomic<u32> reported = 0;

		if (reported++ < 50)
		{
			rb_log.notice("Snapshot: %s(0x%x) - older snapshots can no longer be loaded", what, addr);
		}
	}

	// Main thread: drops every snapshot when the memory map changed since the last check
	namespace
	{
		struct async_exclusion
		{
			u32 begin, end;
			const char* what;
			bool add;
		};

		std::mutex g_async_mutex;
		std::vector<async_exclusion> g_async_exclusions;

		// Main thread, at a save or load boundary
		void apply_async_exclusions()
		{
			std::vector<async_exclusion> list;

			{
				std::lock_guard lock(g_async_mutex);

				if (g_async_exclusions.empty())
				{
					return;
				}

				list.swap(g_async_exclusions);
			}

			for (const auto& e : list)
			{
				if (e.add)
				{
					state_exclude(e.begin, e.end, e.what);
				}
				else if (std::erase_if(g_excluded, [&](const range_t& r) { return r.begin == e.begin && r.end == e.end; }))
				{
					g_layout_gen++;
					rb_log.success("Snapshot includes 0x%08x-0x%08x again (%s)", e.begin, e.end, e.what);
				}
			}
		}
	}

	void state_exclude_async(u32 begin, u32 end, const char* what)
	{
		std::lock_guard lock(g_async_mutex);
		g_async_exclusions.push_back({begin, end, what, true});
	}

	void state_unexclude_async(u32 begin, u32 end, const char* what)
	{
		std::lock_guard lock(g_async_mutex);
		g_async_exclusions.push_back({begin, end, what, false});
	}

	void state_foreign_window(u32 begin, u32 end, bool add)
	{
		std::lock_guard lock(g_foreign_window_mutex);
		std::erase(g_foreign_windows, std::pair{begin, end});

		if (add)
		{
			g_foreign_windows.emplace_back(begin, end);
		}
	}

	static void check_kernel_changes()
	{
		apply_async_exclusions();

		if (const u32 n = g_kernel_changes.load(); n != g_kernel_changes_seen)
		{
			g_kernel_changes_seen = n;

			for (u32 i = 0; i < ring_size; i++)
			{
				g_ring_valid[i] = false;
			}
		}
	}

	std::vector<u32> state_foreign_pages()
	{
		std::lock_guard lock(g_foreign_mutex);
		return g_foreign_tracked;
	}

	bool on_write_fault(u32 addr)
	{
		if (!g_track_active)
		{
			return false;
		}

		const u32 page = addr >> 12;
		u8 st = g_track[page].load();

		if (st == track_none)
		{
			return false;
		}

		// RB_DRAW_WRITES=1 (diagnostics): snapshot pages written by the game's draw thread (NU::Draw::RequestManager) -
		// what a save or load concurrent with drawing would tear; logged as ranges every 2000 new faults
		if (static const bool draw_diag = std::getenv("RB_DRAW_WRITES") && *std::getenv("RB_DRAW_WRITES") == '1'; draw_diag)
		{
			if (const auto cpu = cpu_thread::get_current(); cpu && cpu->get_class() == thread_class::ppu && cpu->id != 0x1000000)
			{
				const auto name = static_cast<ppu_thread*>(cpu)->ppu_tname.load();

				if (name && name->starts_with("NU::Draw"))
				{
					static std::mutex m;
					static std::map<u32, u64> pages;
					static u64 faults = 0;
					std::lock_guard lock(m);
					pages[page]++;

					if (faults < 40)
					{
						const auto& ppu = *static_cast<ppu_thread*>(cpu);
						rb_log.notice("Draw thread write: 0x%08x by cia 0x%08x lr 0x%08x r3 0x%x r4 0x%x", addr, ppu.cia, ppu.lr, ppu.gpr[3], ppu.gpr[4]);
					}

					if (++faults % 2000 == 0 || (faults & (faults - 1)) == 0)
					{
						std::string text;
						u32 begin = 0, end = 0;
						u64 hits = 0;

						for (const auto& [pg, n] : pages)
						{
							if (pg != end)
							{
								if (end)
								{
									fmt::append(text, " 0x%08x-0x%08x(%u)", begin << 12, end << 12, hits);
								}

								begin = pg;
								hits = 0;
							}

							end = pg + 1;
							hits += n;
						}

						fmt::append(text, " 0x%08x-0x%08x(%u)", begin << 12, end << 12, hits);
						rb_log.notice("Draw thread writes to snapshot pages (%u faults, %u pages):%s", faults, pages.size(), text);
					}
				}
			}
		}

		if (g_foreign)
		{
			const auto cpu = cpu_thread::get_current();

			static const std::vector<std::string> prefixes = []()
			{
				const char* v = std::getenv("RB_SNAP_FOREIGN_THREADS");
				return fmt::split(v && *v ? v : "net_", {","});
			}();

			if (!cpu && !g_foreign_track_only && !g_foreign_named_only)
			{
				mark_foreign(page, std::string(thread_ctrl::get_name()));
			}
			else if (cpu && cpu->get_class() == thread_class::ppu && cpu->id != 0x1000000)
			{
				const auto name = static_cast<ppu_thread*>(cpu)->ppu_tname.load();

				if (name && std::any_of(prefixes.begin(), prefixes.end(), [&](const std::string& p) { return name->starts_with(p); }))
				{
					mark_foreign(page, *name);
				}
			}
		}

		if (st == track_clean && g_track[page].compare_exchange_strong(st, track_dirty))
		{
			track_mark_dirty(page);
		}

		protect_pages(page << 12, 0x1000, utils::protection::rw);
		return true;
	}

	void note_write(u32 addr, u32 size)
	{
		if (!g_track_active || !size)
		{
			return;
		}

		for (u32 page = addr >> 12; page <= (addr + size - 1) >> 12; page++)
		{
			if (u8 st = track_clean; g_track[page].compare_exchange_strong(st, track_dirty))
			{
				protect_pages(page << 12, 0x1000, utils::protection::rw);
				track_mark_dirty(page);
			}
		}
	}

	// File reads write through the host view (no fault): the tracking misses them and a rollback restores the buffer.
	// RB_FILE_READ_LOG=1: reads by threads other than the main thread are logged (thread, address, size; first 400).
	// RB_SNAP_FILE_FOREIGN_THREADS=<thread name prefixes>: whole pages covered by reads of these threads (64-byte blocks
	// accumulated over small reads) are not rolled back. Only for the sound stream: data the game loads for the fight must
	// stay in the snapshot (all reads excluded desynced in round 2, o172).
	std::atomic<u32> g_file_foreign_scope = 0;

	void state_file_foreign_scope(bool enter)
	{
		static const bool on = !std::getenv("RB_SNAP_SOUND_READS") || *std::getenv("RB_SNAP_SOUND_READS") != '0';

		if (on)
		{
			enter ? g_file_foreign_scope++ : g_file_foreign_scope--;
		}
	}

	void note_file_read(u32 addr, u64 size)
	{
		if (!g_track_active || !size)
		{
			return;
		}

		const auto cpu = cpu_thread::get_current();

		// The main thread: only inside the sound update of a live frame (the music stream refill; o174/o175 analysis)
		const bool main_scope = cpu && cpu->id == 0x1000000;

		if (main_scope && !g_file_foreign_scope)
		{
			return;
		}

		// PPU threads by their guest name, host threads (asynchronous file I/O) by their host name
		std::string name;

		if (cpu && cpu->get_class() == thread_class::ppu)
		{
			const auto name_ptr = static_cast<ppu_thread*>(cpu)->ppu_tname.load();
			name = name_ptr ? *name_ptr : std::string();
		}
		else
		{
			name = std::string(thread_ctrl::get_name());
		}

		static const bool log_reads = std::getenv("RB_FILE_READ_LOG") && *std::getenv("RB_FILE_READ_LOG") == '1';

		if (static std::atomic<u32> logged = 0; log_reads && logged++ < 400)
		{
			rb_log.notice("File read by %s: 0x%08x size 0x%x", name, addr, size);
		}

		static const std::vector<std::string> prefixes = []()
		{
			const char* v = std::getenv("RB_SNAP_FILE_FOREIGN_THREADS");
			return v && *v ? fmt::split(v, {","}) : std::vector<std::string>{};
		}();

		if (!g_foreign || (!main_scope && (prefixes.empty() || !std::any_of(prefixes.begin(), prefixes.end(), [&](const std::string& p) { return name.starts_with(p); }))))
		{
			return;
		}

		static std::mutex mutex;
		static std::unordered_map<u32, u64> covered;
		std::lock_guard lock(mutex);
		const u64 begin = addr, stop = u64{addr} + size;

		for (u64 page = begin >> 12; page <= (stop - 1) >> 12; page++)
		{
			const u64 lo = std::max(begin, page << 12), hi = std::min(stop, (page + 1) << 12);
			const u32 b0 = static_cast<u32>(((lo & 0xfff) + 63) >> 6);
			const u32 b1 = static_cast<u32>(hi - (page << 12)) >> 6;

			if (b1 <= b0)
			{
				continue;
			}

			const u64 bits = (b1 - b0 >= 64 ? ~u64{0} : ((u64{1} << (b1 - b0)) - 1)) << b0;
			u64& mask = covered[static_cast<u32>(page)];
			mask |= bits;

			if (mask == ~u64{0})
			{
				mark_foreign(static_cast<u32>(page), "file reads of " + name);
			}
		}
	}

	void state_request_rollback(u64 to_frame, std::function<void()> before, std::function<void()> after, u64 max_depth, u64 max_tag, u64 min_tag)
	{
		g_request_to = to_frame;
		g_request_max_depth = max_depth;
		g_request_max_tag = max_tag;
		g_request_min_tag = min_tag;
		g_request_before = std::move(before);
		g_request_after = std::move(after);
	}

	void state_set_save_tag(std::function<u64()> tag)
	{
		g_save_tag = std::move(tag);
	}

	void state_set_save_check(std::function<bool()> check)
	{
		g_save_check = std::move(check);
	}

	bool state_stress_ready()
	{
		return g_stress_every && g_has_snapshot;
	}

	bool state_resimulating(u64 game_frame)
	{
		return g_net_resim || (g_rollbacks && game_frame < g_last_rollback_from);
	}

	std::vector<u32> state_saved_pages()
	{
		std::vector<u32> out;
		out.reserve(g_store.units.size());

		for (const page_unit& u : g_store.units)
		{
			out.push_back(u.page);
		}

		return out;
	}

	void state_set_spu_busy_check(std::function<bool()> check)
	{
		g_spu_busy = std::move(check);
	}

	void state_sample_fifo()
	{
		if (!g_has_snapshot && g_fifo_shift != 20)
		{
			sample_fifo();
		}
	}

	void state_exclude(u32 begin, u32 end, const char* what)
	{
		if (std::any_of(g_excluded.begin(), g_excluded.end(), [&](const range_t& r) { return r.begin == begin && r.end == end; }))
		{
			return;
		}

		g_excluded.push_back({begin, end});
		g_layout_gen++; // the saved parts of the pages are recomputed
		rb_log.success("Snapshot excludes 0x%08x-0x%08x (%s)", begin, end, what);
	}

	void state_untracked_range(u32 begin, u32 end, bool add, const char* what)
	{
		const auto it = std::find_if(g_excluded.begin(), g_excluded.end(), [&](const range_t& r) { return r.begin == begin && r.end == end; });

		if (add)
		{
			if (it != g_excluded.end())
			{
				return;
			}

			g_excluded.push_back({begin, end});
			g_layout_gen++;
			u32 pages = 0;

			// Whole pages leave the store and the write tracking: writable, "dirty" for the calibration (never added, swept or
			// cooled again), so the stream costs no fault / add / cooling any more
			for (u32 page = (begin + 0xfff) & ~0xfffu; page + 0x1000 <= end; page += 0x1000)
			{
				const auto found = g_cal_index.find(page);

				if (found == g_cal_index.end())
				{
					continue;
				}

				g_store.remove(page);
				g_cal_dirty[found->second] = 1;

				if (g_track && g_track[page >> 12].exchange(track_dirty) == track_clean)
				{
					protect_pages(page, 0x1000, utils::protection::rw);
				}

				pages++;
			}

			rb_log.success("Snapshot excludes 0x%08x-0x%08x (%s), %u pages no longer saved or tracked", begin, end, what, pages);
			return;
		}

		if (it == g_excluded.end())
		{
			return;
		}

		g_excluded.erase(it);
		g_layout_gen++;
		u32 pages = 0;

		// Back in the snapshot: every slot gets the current content (normally a new match follows, whose memory map change
		// invalidates the ring anyway)
		for (u32 page = (begin + 0xfff) & ~0xfffu; page + 0x1000 <= end; page += 0x1000)
		{
			if (g_cal_index.count(page) && vm::check_addr(page, vm::page_readable))
			{
				g_store.add(page, vm::get_super_ptr<u8>(page));
				pages++;
			}
		}

		rb_log.success("Snapshot includes 0x%08x-0x%08x again (%s), %u pages", begin, end, what, pages);
	}

	bool state_net_ready()
	{
		return g_net_mode && g_has_snapshot && !g_store.units.empty();
	}

	void state_net_set_resim(bool resim)
	{
		g_net_resim = resim;
	}

	void state_net_save(u64 game_frame, const u32 (&seq)[2])
	{
		check_kernel_changes();

		static const bool save_idle = []() { const char* v = std::getenv("RB_SNAP_SAVE_IDLE"); return v && *v == '1'; }();

		if (save_idle && !g_net_resim)
		{
			wait_for_draw_idle(true);
		}

		if (g_cal_continuous && g_cal_done && calibrate_step())
		{
			for (const u32 page : g_new_dirty_pages)
			{
				g_store.add(page, g_cal_shadow.data() + g_cal_index[page] * 0x1000ull);
			}
		}

		remove_foreign_pages();

		// The online rollback saves only frames where the simulation advanced, not every frame: snapshots of this frame and
		// later ones (a discarded timeline during re-simulation) are dropped, then a free slot or the oldest one is used
		for (u32 i = 0; i < ring_size; i++)
		{
			if (g_ring_valid[i] && g_ring_info[i].game_frame >= game_frame)
			{
				g_ring_valid[i] = false;
			}
		}

		u32 slot = 0;

		for (u32 i = 0; i < ring_size; i++)
		{
			if (!g_ring_valid[i])
			{
				slot = i;
				break;
			}

			if (g_ring_info[i].game_frame < g_ring_info[slot].game_frame)
			{
				slot = i;
			}
		}

		const auto start = std::chrono::steady_clock::now();
		task_check(game_frame, "after frame");
		g_store.copy<true>(slot);
		const u64 us = elapsed_us(start);

		g_save_us_total += us;
		g_save_us_max = std::max(g_save_us_max, us);
		g_saves++;
		g_ring_info[slot] = {game_frame, {seq[0], seq[1]}};
		g_ring_valid[slot] = true;
		cold_check();
	}

	bool state_net_load(u64 game_frame, snapshot_info& out)
	{
		check_kernel_changes();

		u32 slot = ring_size;

		for (u32 i = 0; i < ring_size; i++)
		{
			if (g_ring_valid[i] && g_ring_info[i].game_frame == game_frame)
			{
				slot = i;
			}
		}

		if (slot == ring_size)
		{
			rb_log.error("Netplay: no snapshot of game frame %u", game_frame);
			return false;
		}

		wait_for_draw_idle();
		const auto start = std::chrono::steady_clock::now();
		g_store.copy<false>(slot);
		task_check(game_frame, "after load");
		const u64 us = elapsed_us(start);

		g_load_us_total += us;
		g_load_us_max = std::max(g_load_us_max, us);
		g_rollbacks++;

		if (g_rollbacks % 100 == 1)
		{
			rb_log.success("Netplay: load #%u of game frame %u in %u us (saves avg %u us max %u us, loads avg %u us max %u us, %u KiB)",
				g_rollbacks, game_frame, us, g_save_us_total / std::max<u64>(g_saves, 1), g_save_us_max, g_load_us_total / g_rollbacks,
				g_load_us_max, g_store.bytes() >> 10);
		}

		out = g_ring_info[slot];
		return true;
	}

	bool state_on_frame_start(u64 game_frame, const u32 (&seq)[2], snapshot_info& out)
	{
		std::call_once(g_init, init);

		if (g_ranges.empty())
		{
			return false;
		}

		// Frames relative to the sync point are resolved once it is known
		if (const u64 sync = sync_game_frame(); sync != umax && (g_save_rel != umax || g_cal_rel != umax || g_restore_rel != umax))
		{
			if (g_restore_rel != umax)
			{
				g_restore_at = sync + g_restore_rel;
				g_restore_rel = umax;
				rb_log.success("Snapshot: restore at game frame %u", g_restore_at);
			}

			if (g_save_rel != umax)
			{
				g_save_at = sync + g_save_rel;
				g_save_rel = umax;

				// The stress test was scheduled from the (then unknown) save frame
				if (g_stress_every)
				{
					g_next_rollback = g_save_at + (g_pattern.empty() ? g_stress_every : g_pattern[0].first);
					rb_log.success("Snapshot stress test: first rollback at game frame %u", g_next_rollback);
				}
			}

			if (g_cal_rel != umax)
			{
				g_calibrate_from = sync + g_cal_rel;
				g_cal_rel = umax;
			}

			rb_log.success("Snapshot: sync point at game frame %u, calibration from %u, first save at %u", sync, g_calibrate_from, g_save_at);
		}

		if (game_frame >= g_calibrate_from && game_frame < g_save_at && !g_has_snapshot)
		{
			calibrate_step();
		}

		if (!g_has_snapshot && g_fifo_shift != 20)
		{
			sample_fifo();
		}

		if (game_frame == g_save_at && !g_has_snapshot)
		{
			if (g_cal_frames && !g_cal_done)
			{
				finish_calibration();
			}

			const auto start = std::chrono::steady_clock::now();
			copy_ranges<true>(g_buffer.data());
			const auto us = elapsed_us(start);

			g_saved = {game_frame, {seq[0], seq[1]}};
			g_has_snapshot = true;
			rb_log.success("State saved at game frame %u (port reads %u/%u) in %d us", game_frame, seq[0], seq[1], us);

			if (g_stress_every || g_net_mode)
			{
				// Every page of the snapshot, all ring copies = the current content
				for (const range_t& r : g_ranges)
				{
					for (u32 page = r.begin & ~0xfffu; page < r.end; page += 0x1000)
					{
						if (vm::check_addr(page, vm::page_readable))
						{
							g_store.add(page, vm::get_super_ptr<u8>(page));
						}
					}
				}

				g_store.pieces_gen = g_layout_gen;
				const u32 slot = game_frame % ring_size;
				g_ring_info[slot] = g_saved;
				g_ring_valid[slot] = true;
				rb_log.success("Snapshot ring: %u pages (%u KiB per slot)", g_store.units.size(), g_store.bytes() >> 10);
			}

			return false;
		}

		if (g_stress_every && g_has_snapshot && game_frame > g_save_at)
		{
			// Pages marked foreign while already in the snapshot (named writer threads, audio stream windows) leave it here
			// too: before, only state_net_save removed them and the stress ring kept restoring them
			remove_foreign_pages();

			hold_update();
			hold_check();

			// RB_SNAP_SAVE_IDLE=1: the draw thread must not be writing render data while the state is hashed
			// (calibration) and saved; waiting between the two would let it change pages after their hash
			static const bool save_idle = []() { const char* v = std::getenv("RB_SNAP_SAVE_IDLE"); return v && *v == '1'; }();

			// RB_SNAP_LOAD_IDLE=0: a boundary with a rollback request neither waits for the draw thread nor calibrates (its
			// state is discarded by the load): the re-simulation runs while the draw thread still draws the previous frame
			// (the wait was ~8 ms before every rollback, o123), instead of after it
			static const bool load_idle = []() { const char* v = std::getenv("RB_SNAP_LOAD_IDLE"); return !v || *v != '0'; }();
			const bool fast_rollback = !load_idle && g_request_to != umax;

			static const bool no_idle = []() { const char* v = std::getenv("RB_SNAP_NO_IDLE"); return v && *v == '1'; }();

			if (save_idle && no_idle && !g_idle_learned && !fast_rollback && !(g_spu_busy && g_spu_busy()))
			{
				static const u32 learn_target = []() { const char* v = std::getenv("RB_SNAP_NO_IDLE_LEARN"); return v ? static_cast<u32>(std::atoi(v)) : 600u; }();
				static std::vector<u8> scratch;
				static std::map<u32, std::array<bool, 0x1000>> foreign;
				static u32 learnt = 0;

				scratch.resize(g_store.units.size() * 0x1000ull);

				for (usz i = 0; i < g_store.units.size(); i++)
				{
					if (vm::check_addr(g_store.units[i].page, vm::page_readable))
					{
						std::memcpy(scratch.data() + i * 0x1000ull, vm::get_super_ptr<u8>(g_store.units[i].page), 0x1000);
					}
				}

				wait_for_draw_idle();

				for (usz i = 0; i < g_store.units.size(); i++)
				{
					const u32 page = g_store.units[i].page;

					if (!vm::check_addr(page, vm::page_readable))
					{
						continue;
					}

					const u8* now = vm::get_super_ptr<u8>(page);
					const u8* was = scratch.data() + i * 0x1000ull;

					if (std::memcmp(now, was, 0x1000) != 0)
					{
						auto& m = foreign[page];

						for (u32 b = 0; b < 0x1000; b++)
						{
							m[b] |= now[b] != was[b];
						}
					}
				}

				if (++learnt >= learn_target)
				{
					u32 ranges = 0, bytes = 0;

					for (const auto& [page, m] : foreign)
					{
						// Ranges of foreign bytes, gaps below 16 bytes merged (aligned to 4 bytes)
						for (u32 b = 0; b < 0x1000;)
						{
							if (!m[b])
							{
								b++;
								continue;
							}

							u32 e = b;

							for (u32 last = b; e < 0x1000 && e - last < 16; e++)
							{
								if (m[e])
								{
									last = e;
								}
							}

							while (e > b && !m[e - 1])
							{
								e--;
							}

							const u32 rb = (page + b) & ~3u, re = (page + e + 3) & ~3u;
							g_excluded.push_back({rb, re});
							ranges++;
							bytes += re - rb;
							b = e;
						}
					}

					g_layout_gen++;
					g_idle_learned = true;
					rb_log.success("Snapshot: %u bytes in %u ranges on %u pages written by other threads excluded; no more waits for the draw thread",
						bytes, ranges, foreign.size());
				}
			}
			else if (save_idle && !fast_rollback && !(no_idle && g_idle_learned))
			{
				// RB_SNAP_IDLE_DIFF=1 (diagnostics): pages of the snapshot that change while the main thread only waits for the
				// draw thread were written by another thread (draw thread, SPU, sound); logged every 600 waits with counts
				static const bool idle_diff = []() { const char* v = std::getenv("RB_SNAP_IDLE_DIFF"); return v && *v == '1'; }();
				static std::vector<u64> before;
				static std::map<u32, u32> changed;
				static u64 diffs = 0;

				const auto page_hash = [](u32 page)
				{
					const u64* p = reinterpret_cast<const u64*>(vm::get_super_ptr<u8>(page));
					u64 h = 0xcbf29ce484222325;

					for (u32 i = 0; i < 512; i++)
					{
						h = (h ^ p[i]) * 0x100000001b3;
					}

					return h;
				};

				if (idle_diff)
				{
					before.resize(g_store.units.size());

					for (usz i = 0; i < g_store.units.size(); i++)
					{
						before[i] = vm::check_addr(g_store.units[i].page, vm::page_readable) ? page_hash(g_store.units[i].page) : 0;
					}
				}

				wait_for_draw_idle(true);

				if (idle_diff)
				{
					for (usz i = 0; i < g_store.units.size() && i < before.size(); i++)
					{
						const u32 page = g_store.units[i].page;

						if (vm::check_addr(page, vm::page_readable) && page_hash(page) != before[i])
						{
							changed[page]++;
						}
					}

					if (++diffs % 600 == 0)
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

						rb_log.notice("Pages changed during the draw thread wait (600 waits, %u pages; ranges with max count):%s", changed.size(), text);
						changed.clear();
					}
				}
			}

			// Pages that start changing later in the fight are added to the snapshot; older snapshots are
			// completed with their calibration-start content (remap_snapshot)
			// (not at the boundaries of a re-simulation: the next live boundary finds the new pages; the write tracking keeps them)
			if (g_cal_continuous && g_cal_done && !fast_rollback && !state_resimulating(game_frame))
			{
				if (const usz added = calibrate_step())
				{
					// The new pages had their calibration-start (or cooling) content in every older snapshot
					for (const u32 page : g_new_dirty_pages)
					{
						g_store.add(page, g_cal_shadow.data() + g_cal_index[page] * 0x1000ull);
					}

					static u32 reported = 0;

					if (reported++ < 200)
					{
						rb_log.notice("Snapshot: %u new pages at game frame %u (%s)", added, game_frame, fmt::merge(g_new_pages, ","));
					}
				}
			}

			// Whole rollback cost: from the restore until the frame where the rollback happened starts again
			if (g_resim_start.time_since_epoch().count() && game_frame == g_last_rollback_from)
			{
				const u64 us = elapsed_us(g_resim_start);
				rsx_reads_resim_end();

				// RB_RSX_HOLD: the live frame writes its render data as in any frame (the game's own double buffering); test
				// hd2: held into the live frame, its render preparation (0x006093a8 from 0x005b894c) waited in 17 % of the rollbacks
				if (g_hold_active)
				{
					std::lock_guard lock(g_hold_mutex);
					hold_release_locked();
				}

				rollback_csv(fmt::format("E,%u", us));
				g_resim_start = {};
				native::sample_gate(false);
				g_resim_us_total += us;
				g_resim_us_max = std::max(g_resim_us_max, us);
				g_resims++;

				if (g_resims % 50 == 1)
				{
					rb_log.success("Stress: rollback + resimulation %u us (avg %u us, max %u us over %u rollbacks)",
						us, g_resim_us_total / g_resims, g_resim_us_max, g_resims);
				}

				// Slow rollbacks (over 32 ms): depth and the game frame, to find what makes the maximum
				if (static u32 slow_logged = 0; us > 32000 && slow_logged < 60)
				{
					slow_logged++;
					rb_log.warning("Slow rollback: %u us, %u frames re-simulated, ending at game frame %u (game mode %u)", us, g_resim_depth, game_frame, last_game_mode());
				}

				// Per rollback depth, after the warm-up (snapshot settled): average, max, and how many exceed 10/14 ms
				struct depth_stat { u64 n = 0, total = 0, max = 0, over10 = 0, over14 = 0; };
				static depth_stat by_depth[8]{};
				static u64 counted = 0;

				if (game_frame >= 5200 && g_resim_depth < 8)
				{
					depth_stat& d = by_depth[g_resim_depth];
					d.n++;
					d.total += us;
					d.max = std::max(d.max, us);
					d.over10 += us > 10000;
					d.over14 += us > 14000;

					if (++counted % 200 == 0)
					{
						std::string text;

						for (u32 i = 1; i < 8; i++)
						{
							if (by_depth[i].n)
							{
								fmt::append(text, " [depth %u: %u x, avg %u us, max %u us, >10ms %u, >14ms %u]", i, by_depth[i].n,
									by_depth[i].total / by_depth[i].n, by_depth[i].max, by_depth[i].over10, by_depth[i].over14);
							}
						}

						rb_log.success("Stress: rollback cost by depth (from frame 5200):%s", text);
					}
				}
			}

			check_kernel_changes();

			// Rollback requested by the online module
			if (g_request_to != umax)
			{
				u64 to = g_request_to;
				g_request_to = umax;

				// The newest snapshot not after the requested frame (online mode: the ring holds only frames where the
				// simulation advanced, any slot; the requested frame may not be saved: SPURS job running, kernel change)
				u32 to_slot = ring_size;

				for (u32 i = 0; i < ring_size; i++)
				{
					// Online: the slot's state must not have simulated the mispredicted remote frame yet (tag <= max tag; o95: a
					// slot saved after it was taken, the corrected frame was never re-simulated)
					if (g_ring_valid[i] && g_ring_info[i].game_frame <= to && (g_request_max_tag == umax || g_ring_tag[i] <= g_request_max_tag) &&
						(to_slot == ring_size || g_ring_info[i].game_frame > g_ring_info[to_slot].game_frame))
					{
						to_slot = i;
					}
				}

				if (to_slot != ring_size)
				{
					to = g_ring_info[to_slot].game_frame;
				}

				if (to_slot == ring_size && g_request_max_tag != umax)
				{
					if (static u32 logged = 0; logged++ < 50)
					{
						rb_log.error("Rollback to frame %u (remote frame %u) refused: no snapshot before that remote frame - the misprediction stays in the simulation (desync guaranteed, %u so far)",
							to, g_request_max_tag, g_refused_rollbacks + 1);
					}

					g_refused_rollbacks++;
				}
				else if (to_slot != ring_size && to < game_frame && (game_frame - to > g_request_max_depth || g_ring_tag[to_slot] < g_request_min_tag))
				{
					// Too deep (the ring holds only old snapshots): refused, the online module counts it as not rolled back
					if (static u32 logged = 0; logged++ < 50)
					{
						rb_log.error("Rollback to frame %u refused: re-simulation of %u frames (remote frames %u..%u), limit %u", to, game_frame - to,
							g_ring_tag[to_slot], g_request_max_tag, g_request_max_depth);
					}

					g_refused_rollbacks++;
				}
				else if (to_slot != ring_size && to < game_frame)
				{
					if (g_request_before)
					{
						g_request_before();
					}

					// RB_RSX_HOLD: fight (game mode 4) only - from the KO on the re-simulated frames draw in full
					const bool hold = g_rsx_hold && hold_game_mode() == 4;

					if (!fast_rollback && !g_idle_learned)
					{
						g_wait_rsx_ok = hold;
						wait_for_draw_idle();
						g_wait_rsx_ok = false;
						wait_for_flip();
					}

					const auto start = std::chrono::steady_clock::now();
					g_resim_start = start;
					native::sample_gate(true);
					g_resim_depth = static_cast<u32>(game_frame - to);
					rsx_reads_check(to_slot);
					online::sound_runtime_before_load();
					film_save();
					g_store.skip_pages = hold ? &g_hold_recent : nullptr;
					g_store.copy<false>(to_slot);
					g_store.skip_pages = nullptr;

					if (hold)
					{
						hold_begin();
					}

					film_restore();
					task_check(to, "after load");
					film_undo_after_load(to);
					online::sound_runtime_after_load();
					rsx_reads_after_load();

					// Snapshots after the restored frame belong to the discarded timeline (online mode saves no boundary
					// during the re-simulation, so they would otherwise stay valid for a second rollback)
					for (u32 i = 0; i < ring_size; i++)
					{
						if (g_ring_valid[i] && g_ring_info[i].game_frame > to)
						{
							g_ring_valid[i] = false;
						}
					}

					const u64 us = elapsed_us(start);
					g_load_us_total += us;
					g_load_us_max = std::max(g_load_us_max, us);
					g_last_rollback_from = game_frame;
					g_rollbacks++;

					if (g_request_after)
					{
						g_request_after();
					}

					rollback_csv(fmt::format("R,%u,%u,%u", g_rollbacks, game_frame, to));

					if (g_rollbacks % 100 == 1)
					{
						rb_log.success("Online rollback #%u from frame %u to %u (load %u us; saves avg %u us, loads avg %u us, %u impossible)",
							g_rollbacks, game_frame, to, us, g_save_us_total / std::max<u64>(g_saves, 1), g_load_us_total / g_rollbacks, g_request_failed);
					}

					out = g_ring_info[to_slot];
					return true;
				}

				if (g_request_failed++ < 50)
				{
					if (to_slot < ring_size)
					{
						rb_log.error("Online rollback to frame %u impossible at frame %u (slot %u holds %u, valid %d)", to, game_frame, to_slot,
							g_ring_info[to_slot].game_frame, g_ring_valid[to_slot]);
					}
					else
					{
						rb_log.error("Online rollback to frame %u impossible at frame %u (no usable slot)", to, game_frame);
					}
				}
			}

			const u64 from = game_frame - current_depth();
			const u32 from_slot = from % ring_size;

			// Scheduled rollback; not again when the same frame is reached after a rollback
			const bool due = game_frame >= g_next_rollback && game_frame > g_last_rollback_from;
			const bool possible = g_ring_valid[from_slot] && g_ring_info[from_slot].game_frame == from;

			const auto schedule_next = [&]()
			{
				if (!g_pattern.empty())
				{
					g_pattern_idx = (g_pattern_idx + 1) % g_pattern.size();
				}

				g_next_rollback = game_frame + (g_pattern.empty() ? g_stress_every : g_pattern[g_pattern_idx].first);
			};

			if (due && !possible)
			{
				// Ring was invalidated (new calibrated pages): skip this rollback
				g_last_rollback_from = game_frame;
				schedule_next();
			}
			else if (due)
			{
				const auto wait_start = std::chrono::steady_clock::now();
				wait_for_draw_idle();
				wait_for_flip();
				const u64 wait_us = elapsed_us(wait_start);
				g_wait_us_total += wait_us;
				g_wait_us_max = std::max(g_wait_us_max, wait_us);

				const auto start = std::chrono::steady_clock::now();
				g_resim_start = wait_start;
				native::sample_gate(true);
				g_resim_depth = static_cast<u32>(game_frame - from);
				online::sound_runtime_before_load();
				film_save();
				g_store.copy<false>(from_slot);
				film_restore();
				task_check(from, "after load");
				film_undo_after_load(from);
				online::sound_runtime_after_load();
				online::stress_sound_after_load(from);
				const u64 us = elapsed_us(start);

				g_load_us_total += us;
				g_load_us_max = std::max(g_load_us_max, us);
				g_last_rollback_from = game_frame;
				g_rollbacks++;
				schedule_next();

				if (g_rollbacks % 50 == 1)
				{
					rb_log.success("Stress: rollback #%u from frame %u to %u (load %u us; saves avg %u us max %u us, loads avg %u us max %u us, flip wait avg %u us max %u us)",
						g_rollbacks, game_frame, from, us, g_save_us_total / std::max<u64>(g_saves, 1), g_save_us_max,
						g_load_us_total / g_rollbacks, g_load_us_max, g_wait_us_total / g_rollbacks, g_wait_us_max);
				}

				out = g_ring_info[from_slot];
				return true;
			}

			u32 slot = game_frame % ring_size;

			// The online module can veto a snapshot (a SPURS job still running, the simulation did not advance)
			if (g_save_check && !g_save_check())
			{

				if (g_saves_skipped++ % 100 == 0)
				{
					rb_log.notice("Snapshot of frame %u skipped (%u so far)", game_frame, g_saves_skipped);
				}

				return false;
			}

			// Online mode (sparse ring): snapshots of this frame and later ones belong to a discarded timeline; then a free
			// slot or the oldest one
			if (g_save_check)
			{
				for (u32 i = 0; i < ring_size; i++)
				{
					if (g_ring_valid[i] && g_ring_info[i].game_frame >= game_frame)
					{
						g_ring_valid[i] = false;
					}
				}

				slot = 0;

				for (u32 i = 0; i < ring_size; i++)
				{
					if (!g_ring_valid[i])
					{
						slot = i;
						break;
					}

					if (g_ring_info[i].game_frame < g_ring_info[slot].game_frame)
					{
						slot = i;
					}
				}

				// The newest snapshot of the same simulated frame (a waiting frame) is replaced instead of the oldest one
				if (g_save_tag)
				{
					const u64 tag = g_save_tag();
					u32 newest = ring_size;

					for (u32 i = 0; i < ring_size; i++)
					{
						if (g_ring_valid[i] && (newest == ring_size || g_ring_info[i].game_frame > g_ring_info[newest].game_frame))
						{
							newest = i;
						}
					}

					if (newest != ring_size && g_ring_tag[newest] == tag)
					{
						slot = newest;
					}
				}
			}

			// Pages first written in a re-simulated frame (a wall hit only in the corrected timeline, a cooled page written
			// again) were otherwise added at the next live boundary with their old (shadow) content in every slot - also in
			// this one, whose state already has the write. A rollback to it restored a mix of both timelines (task lists
			// into the other timeline -> endless task walk, the online wall splat hang 2026-10-05).
			// RB_SNAP_RESIM_CAL=0: old behaviour (diagnostics)
			static const bool resim_cal = !std::getenv("RB_SNAP_RESIM_CAL") || *std::getenv("RB_SNAP_RESIM_CAL") != '0';

			if (resim_cal && g_cal_continuous && g_cal_done && state_resimulating(game_frame))
			{
				if (const usz added = calibrate_step())
				{
					for (const u32 page : g_new_dirty_pages)
					{
						g_store.add(page, g_cal_shadow.data() + g_cal_index[page] * 0x1000ull);
					}

					if (static u32 reported = 0; reported++ < 100)
					{
						rb_log.notice("Snapshot: %u new pages before the re-simulated save of game frame %u (%s)", added, game_frame, fmt::merge(g_new_pages, ","));
					}
				}
			}

			const auto start = std::chrono::steady_clock::now();
			task_check(game_frame, "after frame");
			g_store.copy<true>(slot);
			const u64 us = elapsed_us(start);

			g_save_us_total += us;
			g_save_us_max = std::max(g_save_us_max, us);
			g_saves++;
			g_ring_info[slot] = {game_frame, {seq[0], seq[1]}};
			g_ring_tag[slot] = g_save_tag ? g_save_tag() : umax;
			g_ring_valid[slot] = true;
			cold_check();
			return false;
		}

		if (game_frame == g_restore_at && g_has_snapshot)
		{
			wait_for_draw_idle();
			wait_for_flip();

			const auto start = std::chrono::steady_clock::now();
			copy_ranges<false>(g_buffer.data());
			const auto us = elapsed_us(start);

			// One restore per run
			g_restore_at = umax;
			out = g_saved;
			rb_log.success("State restored at game frame %u to game frame %u in %d us", game_frame, g_saved.game_frame, us);
			return true;
		}

		return false;
	}
}
