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
//   pad input        NOT read by the main loop: thread "keyread" (FUN_00122450) waits on a semaphore that the
//                    GCM vblank handler FUN_0025cf30 posts (FUN_001220e0), reads both pads, converts them
//                    (FUN_0008fd30) and writes one u16 per player into a 16-entry ring (*(TOC-19040),
//                    index *(*(TOC-19052)), lwmutex *(TOC-19032)). The game takes the NEWEST ring entry once
//                    per frame per player: FUN_00122360(player), only caller 0x00013140 (input task FUN_00012f38).
//                    Which entry is the newest depends on vblank vs. frame timing, so the rollback works on the
//                    value FUN_00122360 returns (key_latest hook), not on cellPadGetData.
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
	constexpr u32 key_latest = 0x00122360;     // patched: FUN_00122360(player) -> u16 game buttons of the frame (HLE)
	constexpr u32 key_special = 0x0004174c;    // called first by FUN_00122360 in game mode 4 / sub mode 17
	constexpr s32 toc_key_mode = -19012;       // TOC slot -> game mode struct (+0 mode, +16 sub mode)
	constexpr s32 toc_key_index = -19052;      // TOC slot -> u32 ring write counter
	constexpr s32 toc_key_ring = -19040;       // TOC slot -> u16 ring[2][16]

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
		{0x00122360, 0x38000070}, // li r0, 0x70                 (newest key sample, FUN_00122360)
		{0x00122364, 0x8122b5bc}, // lwz r9, -19012(r2)
		{0x001223ac, 0x8122b594}, // lwz r9, -19052(r2)
		{0x001223cc, 0x8122b5a0}, // lwz r9, -19040(r2)
		{0x00013140, 0x4810f221}, // bl FUN_00122360               (its only call site)
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

// ---------- Button mapping (RB_T5_BUTTONS) ----------
// Applied to the local controller (port 0) before the game or the netplay sees it, so it works for pads and the
// keyboard alike, offline and online. Format: "L1=2+3,L2=1+4,R1=1+2,R2=3+4,L3=1,SQUARE=none" with the sources
// L1 L2 R1 R2 L3 R3 SQUARE TRIANGLE CROSS CIRCLE SELECT and the game buttons 1 (left punch, square),
// 2 (right punch, triangle), 3 (left kick, cross), 4 (right kick, circle), START, or "none".
// Buttons that are not listed keep their normal function.
#include "Emu/Io/pad_types.h"
#include <array>
#include <cstdlib>
#include <string>

namespace rb::t5dr
{
	struct button_map
	{
		struct entry { int word; u16 bit; int press; };            // a physical button
		struct action { u16 d1 = 0; u16 d2 = 0; bool mapped = false; };
		std::array<action, 11> actions{};
		bool any = false;

		static constexpr entry sources[11] = {
			{3, CELL_PAD_CTRL_L1, CELL_PAD_BTN_OFFSET_PRESS_L1}, {3, CELL_PAD_CTRL_L2, CELL_PAD_BTN_OFFSET_PRESS_L2},
			{3, CELL_PAD_CTRL_R1, CELL_PAD_BTN_OFFSET_PRESS_R1}, {3, CELL_PAD_CTRL_R2, CELL_PAD_BTN_OFFSET_PRESS_R2},
			{2, CELL_PAD_CTRL_L3, -1}, {2, CELL_PAD_CTRL_R3, -1},
			{3, CELL_PAD_CTRL_SQUARE, CELL_PAD_BTN_OFFSET_PRESS_SQUARE}, {3, CELL_PAD_CTRL_TRIANGLE, CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE},
			{3, CELL_PAD_CTRL_CROSS, CELL_PAD_BTN_OFFSET_PRESS_CROSS}, {3, CELL_PAD_CTRL_CIRCLE, CELL_PAD_BTN_OFFSET_PRESS_CIRCLE},
			{2, CELL_PAD_CTRL_SELECT, -1},
		};

		static int source_index(const std::string& s)
		{
			static const char* names[11] = {"L1", "L2", "R1", "R2", "L3", "R3", "SQUARE", "TRIANGLE", "CROSS", "CIRCLE", "SELECT"};
			for (int i = 0; i < 11; i++) if (s == names[i]) return i;
			return -1;
		}

		static button_map parse(const char* spec)
		{
			button_map m;
			std::string s = spec ? spec : "";
			usz pos = 0;

			while (pos < s.size())
			{
				usz end = s.find(',', pos);
				if (end == std::string::npos) end = s.size();
				const std::string item = s.substr(pos, end - pos);
				pos = end + 1;
				const usz eq = item.find('=');
				if (eq == std::string::npos) continue;
				const int src = source_index(item.substr(0, eq));
				if (src < 0) continue;

				action a;
				a.mapped = true;
				const std::string what = item.substr(eq + 1);

				if (what == "START")
				{
					a.d1 = CELL_PAD_CTRL_START;
				}
				else if (what != "none")
				{
					for (char c : what)
					{
						if (c == '1') a.d2 |= CELL_PAD_CTRL_SQUARE;
						if (c == '2') a.d2 |= CELL_PAD_CTRL_TRIANGLE;
						if (c == '3') a.d2 |= CELL_PAD_CTRL_CROSS;
						if (c == '4') a.d2 |= CELL_PAD_CTRL_CIRCLE;
					}
				}

				m.actions[src] = a;
				m.any = true;
			}

			return m;
		}

		static const button_map& get()
		{
			static const button_map m = parse(std::getenv("RB_T5_BUTTONS"));
			return m;
		}

		void apply(CellPadData& pad) const
		{
			if (!any) return;

			const u16 in1 = pad.button[CELL_PAD_BTN_OFFSET_DIGITAL1];
			const u16 in2 = pad.button[CELL_PAD_BTN_OFFSET_DIGITAL2];
			u16 out1 = in1, out2 = in2;
			u16 add1 = 0, add2 = 0;

			// Remove every remapped source, then add what the pressed ones do
			for (int i = 0; i < 11; i++)
			{
				if (!actions[i].mapped) continue;
				const entry& e = sources[i];
				const bool down = ((e.word == 2 ? in1 : in2) & e.bit) != 0;
				(e.word == 2 ? out1 : out2) &= ~e.bit;
				if (e.press >= 0) pad.button[e.press] = 0;
				if (down) { add1 |= actions[i].d1; add2 |= actions[i].d2; }
			}

			out1 |= add1;
			out2 |= add2;
			pad.button[CELL_PAD_BTN_OFFSET_DIGITAL1] = out1;
			pad.button[CELL_PAD_BTN_OFFSET_DIGITAL2] = out2;

			// Pressure values of face buttons follow the digital state
			const std::pair<u16, int> faces[4] = {
				{CELL_PAD_CTRL_SQUARE, CELL_PAD_BTN_OFFSET_PRESS_SQUARE}, {CELL_PAD_CTRL_TRIANGLE, CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE},
				{CELL_PAD_CTRL_CROSS, CELL_PAD_BTN_OFFSET_PRESS_CROSS}, {CELL_PAD_CTRL_CIRCLE, CELL_PAD_BTN_OFFSET_PRESS_CIRCLE}};

			for (const auto& [bit, press] : faces)
			{
				if (add2 & bit) pad.button[press] = 255;
			}
		}
	};
}
