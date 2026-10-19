#pragma once

#include <atomic>
#include <functional>
#include <vector>

#include "util/types.hpp"

// Rollback experiment (Tekken 6): saving and restoring game state in guest memory.
// Test mode controlled by environment variables, read once on first use:
//   RB_SNAP_RANGES     = comma separated guest ranges (0xBEGIN-0xEND) that form the saved state
//   RB_SNAP_SAVE_AT    = game frame at which the state is saved
//   RB_SNAP_RESTORE_AT = game frame at which the saved state is restored and the input replay rewound
namespace rb
{
	struct snapshot_info
	{
		u64 game_frame;
		u32 seq[2];
	};

	// Called at the start of every port 0 pad read (frame boundary), before any input is consumed.
	// Returns true when the state was restored and input positions must be rewound to `out`.
	bool state_on_frame_start(u64 game_frame, const u32 (&seq)[2], snapshot_info& out);

	// Stress test with prediction (RB_SNAP_PREDICT=1): frames that will be rolled back later are first
	// simulated with a predicted input for port 1 (the last input before the rollback window, like GGPO).
	// Returns true for such a frame and the game frame whose input is repeated.
	bool state_is_speculative(u64 game_frame, u64& predicted_from);

	// True for frames that are re-simulated after a stress test rollback (the frames before the one where the
	// rollback happened); RB_FAST_RESIM runs them without rendering
	bool state_resimulating(u64 game_frame);

	// Snapshot loads so far (online, stress and netplay rollbacks)
	u64 state_load_count();

	// Diagnostics: host time (us) spent so far in snapshot saves, snapshot loads, waits for the draw thread and waits for
	// SPURS jobs (online); the re-simulated frame parts report splits the boundary time with them
	enum class diag_t { save, load, draw_wait, spurs_wait, count };
	std::atomic<u64>& state_diag_us(diag_t what);

	struct diag_scope
	{
		diag_t what;
		u64 t0;
		explicit diag_scope(diag_t w);
		~diag_scope();
	};

	// Write tracking of the calibrated pages (RB_SNAP_TRACK=1). on_write_fault is called by the access violation
	// handler: true when the page was a tracked clean page (now dirty and writable). note_write is called by the
	// RPCS3 paths that write guest memory through the unprotected view (128-byte atomic stores, reservations).
	// Netplay (RB_NET=1): the snapshot ring is driven by GGPO. Ready once the first snapshot exists (RB_SNAP_SAVE_AT).
	bool state_net_ready();
	void state_net_set_resim(bool resim);
	void state_net_save(u64 game_frame, const u32 (&seq)[2]);
	bool state_net_load(u64 game_frame, snapshot_info& out);
	// Excludes a range from save/restore (like RB_SNAP_EXCLUDE); must be called before the sync point
	void state_exclude(u32 begin, u32 end, const char* what);
	// T5DR: sample the RSX GET/PUT pointers (command blocks left out of the snapshot), from the sync point to the first save
	void state_sample_fifo();
	// From any thread (HLE modules): queued, applied by the main thread at the next save or load
	void state_exclude_async(u32 begin, u32 end, const char* what);
	void state_unexclude_async(u32 begin, u32 end, const char* what);
	// From any thread: pages in [begin, end) whose writes the tracking misses (SPU DMA, host writes) become foreign (not
	// rolled back) even with RB_SNAP_FOREIGN=3; PPU writes there stay game state. Used next to an open cellAdec instance
	// (the game's AT3P stream ring buffer). add = false removes the window.
	void state_foreign_window(u32 begin, u32 end, bool add);
	// Main thread, between frames: [begin, end) is excluded from the snapshot and its whole pages from the write tracking
	// (add = false: back in the snapshot with the current content). RB_FILM_UNDO: the film data buffer.
	void state_untracked_range(u32 begin, u32 end, bool add, const char* what);
	void state_set_spu_busy_check(std::function<bool()> check);
	std::vector<u32> state_saved_pages(); // pages of the snapshot (diagnostics) // RB_SNAP_NO_IDLE: learn only while no SPU job runs
	// RB_SNAP_FOREIGN=2: pages first written by the network threads (or host writes), still saved and restored
	std::vector<u32> state_foreign_pages();
	// A memory map change by the game (sys_memory_allocate/free, sys_mmapper): kernel state that a snapshot does not
	// restore, so no snapshot taken before it may be loaded any more (called from any thread)
	void state_kernel_change(const char* what, u32 addr);

	// Online rollback through the snapshot ring of the stress test (RB_SNAP_STRESS_EVERY: a snapshot of every frame boundary,
	// the restore at a frame boundary, the frames before the rollback frame are re-simulated): the online module requests a
	// rollback to a game frame; it happens at the next frame boundary, `before` runs right before the restore, `after`
	// right after it (both on the main thread)
	void state_request_rollback(u64 to_frame, std::function<void()> before, std::function<void()> after, u64 max_depth = umax,
		u64 max_tag = umax, u64 min_tag = 0);
	// Online mode: a value stored with every snapshot (the remote simulated frame); a rollback request takes only a slot with
	// min_tag <= tag <= max_tag
	void state_set_save_tag(std::function<u64()> tag);
	// A snapshot of every frame boundary exists (stress ring active)
	bool state_stress_ready();
	// Called before every snapshot of the stress ring (main thread); false = this frame boundary is not saved
	void state_set_save_check(std::function<bool()> check);
	// 1: the address is saved and restored by the snapshot, 0: not in the snapshot, 2: excluded
	int state_addr_saved(u32 addr);

	bool on_write_fault(u32 addr);
	// RB_RSX_READS=2 (diagnostics): access to an RSX-read snapshot page during a re-simulation
	bool on_guard_fault(u32 addr, bool is_writing);
	void note_write(u32 addr, u32 size);
	// A file read into guest memory (lv2_file::op_read, written through the host view without a fault)
	void note_file_read(u32 addr, u64 size);
	// Sound update of a live frame (main thread): its file reads (music stream refill) are not rolled back
	void state_file_foreign_scope(bool enter);
}
