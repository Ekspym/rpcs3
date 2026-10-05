#pragma once

#include <string>

#include "util/types.hpp"

// Rollback experiment (Tekken 6): per-frame checksums of guest memory for determinism testing.
// Controlled by environment variables, read once on first use:
//   RB_HASH       = path of the output file (checksums disabled when empty)
//   RB_HASH_EVERY = hash every N-th game frame (default 60)
//   RB_HASH_FROM  = first game frame to hash (default 0)
//   RB_WATCH      = comma separated 32-bit words logged every game frame: absolute hex address
//                   (0x00D142AC) or relative to r13 of the pad-reading thread (r13-0x7000)
//   RB_WATCH_OUT  = path of the CSV with watched values
//   RB_STATE_RANGES = comma separated guest ranges (0xBEGIN-0xEND) hashed every game frame into the
//                   "state_hash" column of the watch CSV. 32-bit words in 0x30000000-0x37FFFFFF are treated
//                   as heap pointers and skipped, because heap layout differs between sessions.
//   RB_DUMP_FRAMES = comma separated game frames at which all hashed memory is dumped
//   RB_DUMP_PREFIX = path prefix of the dumps (<prefix>_<frame>.bin)
namespace rb
{
	// Called on every game frame (first pad read of the frame) from the thread that reads the pad.
	void checksum_on_frame(u64 game_frame);

	// Full memory dump (RBDM, see compare_dumps.py) right now, to <prefix>_<frame>.bin
	void dump_memory_now(const std::string& prefix, u64 game_frame);
}
