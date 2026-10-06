#pragma once

#include "util/types.hpp"

#include <string>
#include <vector>

struct CellPadData;
class ppu_thread;

// Rollback experiment (Tekken 6): recording / replay of pad input and frame statistics.
// Controlled by environment variables, read once on first use:
//   RB_MODE  = off (default) | record | replay
//   RB_FILE  = path of the input recording (default: <emu dir>/rb_input.bin)
//   RB_STATS = path of a per-frame CSV with pad read counts (optional)
//   RB_SYNC_ADDR = guest address of a u32 that becomes non-zero at the sync point (fight start). The record
//              stores the pad read positions at that moment; replay realigns its read positions to them,
//              so timing differences in menus/loading do not shift the inputs of the fight.
namespace rb
{
	// Called at the end of cellPadGetData after the real pad data was written to `data`.
	// In record mode the result is appended to the recording, in replay mode `data` is overwritten.
	void on_pad_get_data(u32 port_no, CellPadData* data);

	// Called at the start of cellPadGetData before g_pad_mutex is taken (state save/restore).
	void before_pad_get_data(u32 port_no);

	// Called at the end of cellPadGetData after g_pad_mutex was released (frame boundary work).
	void after_pad_get_data(u32 port_no);

	// Called by RSX once per guest flip (rsx::thread::on_frame_end).
	void on_guest_flip();

	// Called from cellGcmResetFlipStatus (PPU side, typically once per game frame).
	void on_reset_flip_status();

	// Called while the main executable is loaded (after patches, before PPU analysis/compilation):
	// redirects Tekken 6's per-frame task walk to the rollback frame hook. Adds patched addresses to `applied`.
	void install_static_hooks(std::vector<u32>& applied);

	// The rollback state (snapshots, sync point, online prediction, many function statics) exists once per process.
	// A second boot in the same process would run on the stale state of the first one, so it starts a fresh RPCS3
	// process for `path` instead. True: the new process was started, this one has to quit.
	bool relaunch_for_boot(const std::string& path);
	bool relaunch_pending(); // a further boot in this process would need relaunch_for_boot

	// RB_RESIM_FILM=0: the match recording stays outside the rollback (copied before a snapshot load, written back after)
	u32 net_manager_object();
	bool in_cosmetic_work();
	u32 current_task_method(); // vtable of the task whose step runs now (0 = none) // inside work that re-simulated frames leave out (draw pass, RB_RESIM_SKIP tasks) // tnOnlineBattle_Manager object seen in the task walk (0 = none)
	u32 last_game_mode(); // game mode (FUN_0020c754) seen by the last re-simulated draw pass (diagnostics)
	void film_save();
	void film_restore();
	void film_undo_after_load(u64 target); // RB_FILM_UNDO: undo the film data written by the frames after `target`

	// Number of guest flips seen since boot.
	u64 frame();

	// Number of game frames (pad reads on port 0) since boot.
	u64 game_frame();

	// Deterministic wall clock: when RB_FIXED_TIME (unix seconds) is set, returns
	// RB_FIXED_TIME + game_frame / 60 s and true; otherwise false and the real clock is used.
	bool get_fixed_time(s64& sec, s64& nsec);

	// Frame based sys_time_get_system_time for selected callers: when `lr` (return address in game code)
	// is listed in RB_FRAME_TIME_LR (comma separated hex), returns game_frame * 1/60 s in microseconds.
	bool get_frame_system_time(u32 lr, u64& usec);

	// Guest timebase (mftb). With RB_VIRTUAL_TB=1 the main game thread gets a frame based timebase after the
	// sync point (game frame * 1/60 s + a small step per read), so the simulation does not depend on how long
	// a frame took on the host (snapshot cost, resimulated frames). Other threads see the real timebase.
	u64 guest_timebase();

	// Main loop frame pieces, used by the netplay module for re-simulated frames
	void run_task_walk(ppu_thread& ppu, u32 root); // one frame of the game (task walk of the main loop)
	// RB_TASK_CHECK=<game frame>: from that frame on, checks the global task list of the main loop root (diagnostics; called
	// by rb_state right after a snapshot load and right before a save). `when` = "after load" / "after frame".
	void task_check(u64 game_frame, const char* when);
	// Game frame of the current main loop iteration, set at its boundary (after a possible load). Unlike the pad read position
	// (get_seq), it does not advance when port 0 is read at the start of the frame.
	u64 boundary_frame();
	u64 sync_game_frame();
	void force_sync_point(const char* reason);      // sync point now (online fight start)                         // game frame of the sync point (umax before it)
	void begin_frame();                            // per-frame bookkeeping (virtual timebase, frame number)
	void get_seq(u32 (&seq)[2]);                   // pad read positions (port 0 = game frame)
	void set_seq(const u32 (&seq)[2]);
	bool recorded_input(u32 port, u32 seq, CellPadData& out); // recorded pad data (replay file)
	bool live_pad(CellPadData& out);              // last data of the local controller (port 0)

	namespace net
	{
		// Netplay (RB_NET=1, see rb_net.cpp). run_frame is called at the main loop frame boundary and returns
		// false when the frame is to be run normally. pad_input provides the pad data of a player.
		bool run_frame(ppu_thread& ppu, u32 root, u64 game_frame);
		bool pad_input(u32 port, CellPadData* data);
	}

	namespace online
	{
		// Online mode of the game (rb_online.cpp): called at every main loop frame boundary (rollback of
		// mispredicted remote inputs happens there); hooks of the input manager at load time
		void on_frame(ppu_thread& ppu, u32 root);
		void end_of_frame(ppu_thread& ppu); // live frame end: early receive of the remote inputs
		void sound_learn_begin(); // around sound library calls of live frames: learns the library state (excluded)
		void sound_learn_end();
		void install_hooks(std::vector<u32>& applied);
		bool is_resimulating();
		bool fight_active(); // the online fight with prediction runs (rollbacks possible)
		void stress_sound_after_load(u64 target); // RB_STRESS_SOUND: sound bookkeeping after an offline stress load
		void sound_runtime_before_load(); // RB_SOUND_KEEP_RUNTIME: playback runtime of the sound core voices around a load
		void sound_runtime_after_load();
	}
}
