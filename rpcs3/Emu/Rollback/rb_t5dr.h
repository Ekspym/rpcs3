#pragma once

#include "util/types.hpp"

#include "Emu/Cell/PPUThread.h"
#include "Emu/Memory/vm.h"

#include <string_view>

// Tekken 5: Dark Resurrection Online (PS3, PSN), European release NPEA00019
// (content ID EP9000-NPEA00019_00-TEKKENRETAIL0000, EBOOT entry 0x00457898).
//
// All addresses below come from static analysis of the decrypted EBOOT of exactly that build.
// Another region (or an update) moves them: install_t5dr_hooks() refuses to patch when the
// code signature does not match, so a wrong build only loses the rollback, it does not crash.
//
// Engine layout (same family as Tekken 6, offsets differ):
//
//   main loop        FUN_00011058, loop body 0x000110f0..0x00011190, one iteration per frame:
//                      FUN_0025eda4(root, 0)        task phase bookkeeping
//                      FUN_0025d5f0(...)
//                      FUN_00323d24()               frame end: GCM flip / flip status wait (vblank pacing)
//                      FUN_0025eda4(root, 0)
//                      FUN_0025cfe0(...)
//                      FUN_00143e44()
//                      FUN_0025fca4(root)           TASK WALK  <- frame hook (only caller: 0x00011134)
//                      FUN_0009bcbc(), FUN_000ab768()
//                    while (FUN_00164ea4(...))
//   task root        *(TOC - 0x7FAC) = *(0x00479054) = 0x10622A80 (BSS, constant pointer)
//   task walk        FUN_0025fca4(root): root+0x40 = root+0x68, then FUN_0025fba0(root, root->+0x10, root)
//   task list        node+0x10 next, node+0x40 flags, node+0x4C run counter, node+0x00 vtable,
//                    node+0x2C step override (OPD), otherwise vtable slot 2 (FUN_0025ebe4)
//   pad device       vtable 0x1008CC60, read method FUN_0037d5d8 (cellPadGetData, raw path cellPadGetRawData)
//   online VS mode   RTTI tcb_gm_netvs_t, vtable 0x10083928 (game's own delay-based netcode, FUN_00246xxx)
//   game UDP         sendto wrapper FUN_00234b20, recvfrom wrapper FUN_00234d48
//   TOC (r2)         0x00481000
namespace rb::t5dr
{
	constexpr std::string_view title_id = "NPEA00019";

	constexpr u64 toc = 0x00481000;
	constexpr u32 task_walk = 0x0025fca4;      // patched: branch to the rollback frame step (HLE)
	constexpr u32 task_walk_body = 0x0025fba0; // FUN_0025fba0(root, first, end): the list walk itself
	constexpr u32 task_root_ptr = 0x00479054;  // TOC slot holding the root pointer (diagnostics)

	// Instruction words that must be present before anything is patched (wrong build / region => no hooks)
	struct code_word
	{
		u32 addr;
		u32 word;
	};

	constexpr code_word code_signature[] =
	{
		{0x0025fca4, 0x3800ffb0}, // li r0, -0x50                (task walk entry)
		{0x0025fcd0, 0x80030068}, // lwz r0, 0x68(r3)
		{0x0025fcd4, 0x80830010}, // lwz r4, 0x10(r3)
		{0x0025fcd8, 0x90030040}, // stw r0, 0x40(r3)
		{0x0025fd04, 0x4bfffe9c}, // b FUN_0025fba0
		{0x00011134, 0x4824eb71}, // bl FUN_0025fca4               (main loop call site)
		{0x0025fba0, 0x38000070}, // li r0, 0x70                 (list walk entry)
	};

	inline bool is_title(std::string_view id)
	{
		return id == title_id;
	}

	// One frame of the game: the original body of FUN_0025fca4, run on the guest. Used for live frames and
	// for the frames that a rollback re-simulates.
	inline void run_task_walk(ppu_thread& ppu, u32 root)
	{
		// Own stack frame for the call (the walk saves LR into the caller frame)
		const u64 old_r1 = ppu.gpr[1];
		ppu.gpr[1] = (old_r1 - 0x100) & ~0xfull;
		vm::write32(static_cast<u32>(ppu.gpr[1]), static_cast<u32>(old_r1));

		vm::write32(root + 0x40, vm::read32(root + 0x68));
		ppu.gpr[3] = root;
		ppu.gpr[4] = vm::read32(root + 0x10);
		ppu.gpr[5] = root;
		ppu.fast_call(task_walk_body, toc);

		ppu.gpr[1] = old_r1;
	}
}
