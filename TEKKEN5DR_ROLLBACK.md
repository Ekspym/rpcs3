# Tekken 5 Dark Resurrection rollback (RPCS3 fork)

Port of Ekspym's Tekken 6 rollback ([Ekspym/rpcs3](https://github.com/Ekspym/rpcs3), branch `tekken6-rollback`) to
**Tekken 5: Dark Resurrection Online, PSN, Europe – NPEA00019** (`EP9000-NPEA00019_00-TEKKENRETAIL0000`).
Branch: `tekken5dr-rollback`. The rollback lives in `rpcs3/Emu/Rollback/`; the remaining changes are small hooks
in the emulator. RPCS3 and the rollback changes are licensed under GPLv2 (see `LICENSE`).

## Building (Windows, Visual Studio)

The repositories are expected side by side:

```
<dir>/rpcs3      this fork, branch tekken5dr-rollback (git submodule update --init --recursive)
<dir>/ggpo       https://github.com/Ekspym/ggpo, branch tekken6-rollback
```

1. GGPO as a static library with the static CRT, into `ggpo/build_rb`:
   `cmake -S ggpo -B ggpo/build_rb -A x64 -DBUILD_SHARED_LIBS=OFF` (the branch sets `/MT`),
   then `cmake --build ggpo/build_rb --config Release`.
   The linker finds `GGPO.lib` in `ggpo/build_rb/lib/x64/Release`.
2. RPCS3 as described in `BUILDING.md` (`rpcs3.sln`, Release x64).

The game and the PS3 firmware are not part of this repository.

> **Status: first port, not yet tested in game.** The hooks come from static analysis of the decrypted
> EBOOT. Everything below the "Calibration" heading has to be done with the game running before two
> players can expect a match that stays in sync.

## What the port does

| Piece | Tekken 6 (original) | Tekken 5 DR (this branch) |
|---|---|---|
| Frame hook | task walk `0x00100250` | task walk `0x0025fca4` (only caller: main loop `0x00011134`) |
| One game frame | `run_task_walk` (per node, `FUN_000ff344`) | `t5dr::run_task_walk`: original body of `0x0025fca4` → list walk `0x0025fba0` |
| Snapshots / rollback | `rb_state` (generic) | unchanged, reused |
| GGPO session | `rb_net` (`RB_NET=1`) | unchanged, reused (session name `tekken5dr`) |
| Game's own online mode | `rb_online` (Tekken 6 addresses) | **not used**, switched off for T5DR |
| Render skip in re-simulated frames | Tekken 6 hooks (`RB_FAST_RESIM`, …) | not ported (not needed: re-simulated frames only run the task walk, the flip `0x00323d24` stays outside) |

Safety: before patching, `install_t5dr_hooks` compares 7 instruction words (`rb_t5dr.h`). Any other build
or region (for example a North American release) logs an error and runs without rollback instead of crashing.

The match is an **offline Versus** match: the local controller is fed to the local player's port, the
remote player's input arrives through GGPO on the other port (ports 0 and 1 of `cellPadGetData`). The
game's own PSN/RPCN online mode is not involved.

## Running (once calibrated)

Both players: same RPCS3 build, same game build, same `config_NPEA00019.yml`. See `play_online_t5dr.bat`
for the environment variables. Your ladder program starts RPCS3 with them:

| Variable | Meaning |
|---|---|
| `RB_NET=1` | rollback netplay on |
| `RB_NET_PLAYER=1\|2` | 1 = port 0 (left side), 2 = port 1 |
| `RB_NET_LOCAL_PORT`, `RB_NET_REMOTE=ip:port` | UDP endpoints (default 7000 / 127.0.0.1:7001) |
| `RB_NET_DELAY` | input delay frames (GGPO fork: max 3 predicted frames) |
| `RB_NET_START=+N` | session starts N frames after the sync point |
| `RB_T5_SYNC_TASK=<class>` | sync point = first frame with a task of that class (new, see below) |
| `RB_REPLAY_FROM_SYNC=1` | replay: live controller until the sync point, the recording after it |
| `RB_T5_BUTTONS=L2=1+4,R2=3+4,...` | button mapping of the local controller (rb_t5dr.h), set by the launcher |
| `RB_SNAP_RANGES`, `RB_SNAP_SAVE_AT`, `RB_SNAP_RESTORE_AT`, `RB_SNAP_TRACK` | snapshot setup (rb_state.h) |

## Calibration (to do with the game running)

1. **Hook check.** Boot the game with the fork. The log must show
   `Tekken 5 DR frame hook: 0x0025fca4 ... -> HLE ...` and the game must play normally with `RB_NET` unset.
2. **Find the fight task.** *Done (first test run): `tcb_swg_game_t` appears when the round goes live
   (`tcb_selectmode_t` = character select, `tcb_gamemain_t` = match loading). Also found: after boot the game makes
   no kernel changes (no memory allocation, mutex or thread creation) in menus, loading or fights, so snapshots stay
   loadable the whole time.* Original step: `RB_T5_TASKLOG=1`, play one Versus match. The log prints the task list (RTTI class
   names) every time it changes. Note the class that appears when the round starts and leaves after the match.
   Put it in `RB_T5_SYNC_TASK` (substring match).
3. **Determinism.** Same match twice with `RB_MODE=record` then `RB_MODE=replay` and `RB_HASH=<file>`.
   The game reads the pads on its own `keyread` thread at vblank and takes the newest sample once per frame, so the
   rollback hooks that per-frame read (`0x00122360`) instead of `cellPadGetData`: one input per player per game frame.
   The menus do not take the same number of frames each run, so use
   `RB_T5_SYNC_TASK=tcb_swg_game_t` in both runs and `RB_REPLAY_FROM_SYNC=1` in the replay: drive the menus by hand to
   the same characters and stage, the recording takes over when the round starts. Bare file names (`RB_HASH=hash.txt`)
   are written to the RPCS3 config folder (next to `rpcs3.exe` on Windows).
   (`rb_input.h`, `rb_checksum.h`). The per-frame hashes from the sync point on must be identical across runs
   *and across both PCs*. If not, find the source with `RB_WATCH` / `RB_DUMP_FRAMES`. Usual causes:
   `sys_time_get_system_time` (4 callers: `0x0001047c`, `0x0034bad8`, `0x0034bb88`, `0x003c4f7c`; use
   `RB_FRAME_TIME_LR`), `cellRtc`, the timebase (`RB_VIRTUAL_TB=1`), SPURS jobs.
   *Done 2026-10-19: one round replayed identically (input hook `0x00122360` + random generators reseeded at the
   sync point, `RB_T5_RESEED`).*
4. **Snapshot ranges.** First test (replay of a recording): `RB_SNAP_RANGES=~0x00450000-0x00490000,~0x10000000-0x10b00000,~0x30000000-0x30300000,~0x50000000-0x55d00000,~0x59400000-0x59900000`,
   `RB_SNAP_CALIBRATE_FROM=+0`, `RB_SNAP_SAVE_AT=+300`, `RB_SNAP_RESTORE_AT=+900` (`+N` = frames after the sync point),
   `RB_SNAP_TRACK=1`. The hash file then has two samples of frames 300-900 after the sync point, which must be equal.
   *Done 2026-10-19 (build be7cab4): full save of these ranges (no `~`) at +600, restore at +1200: the fight continued
   identically (random generator fingerprints equal after the restore; remaining byte differences only in sound
   buffers, timing counters and the GCM context, which are left alone on purpose). Calibrated (`~`) ranges miss pages
   the fight first writes after the calibration (object pool at 0x10648000): for the per-frame ring, every page has to
   be write-tracked. Full save costs ~19 ms.*
   Older notes: Start with calibrated ranges over the writable memory and write tracking:
   `RB_SNAP_RANGES=~0x00450000-0x004A0000,~0x10050000-0x10770000,~0x20000000-0x40000000`, `RB_SNAP_TRACK=1`,
   `RB_SNAP_SAVE_AT=+60`, `RB_SNAP_RESTORE_AT=+600`. The restore test must continue the fight exactly as
   the replay (same hashes). Then the stress test (`RB_SNAP_STRESS_EVERY`) with prediction (`RB_SNAP_PREDICT=1`).
5. **Two instances on one PC.** `RB_NET=1`, player 1 on 7000→7001, player 2 on 7001→7000, both with
   `RB_NET_INPUT=record` and the same recording, then `ggpo.network.delay` to simulate latency.
6. **Menus.** Both instances must reach the fight in the same state (same characters, stage, settings).
   Easiest first version: the ladder sends both clients the match settings and the session starts at the
   sync point; the input recording / replay of the menus (`RB_SYNC_ADDR` logic) can be used for testing.
   Long term, the Tekken 6 way is to hook the game's own online versus (`tcb_gm_netvs_t`, vtable
   `0x10083928`, code around `0x00246000-0x00249000`, UDP wrappers `0x00234b20` / `0x00234d48`), so menus
   and character select run over RPCN and only the fight uses rollback.

## Tekken 5 DR address map (NPEA00019)

| What | Address |
|---|---|
| TOC (r2) | `0x00481000` |
| main loop / loop body | `0x00011058` / `0x000110f0`–`0x00011190` |
| frame end (GCM flip, flip status) | `0x00323d24` → `0x00355b70` → `0x0038ba58` → `0x003537c0` |
| pad input (per frame) | thread `keyread` `0x00122450` (woken at vblank by `0x0025cf30`) → ring `*(TOC-19040)`; game reads newest entry via `0x00122360(player)` (hooked, `rb_t5_keys`), caller `0x00013140` |
| task root | `*(0x00479054)` = `0x10622a80` |
| task walk / list walk / node step / method call | `0x0025fca4` / `0x0025fba0` / `0x0025fa30` / `0x0025ebe4` |
| task node | `+0x00` vtable, `+0x10` next, `+0x2c` step override, `+0x40` flags, `+0x4c` run count |
| pad device vtable / read method | `0x1008cc60` / `0x0037d5d8` (`cellPadGetData` at `0x0037d7a0`) |
| pad output | `+0x10` held buttons (game mapping), `+0x14` pressed, `+0x18` raw, `+0x24` 8 analog halfwords |
| online versus mode | `tcb_gm_netvs_t`, vtable `0x10083928` |
| game UDP send / receive | `0x00234b20` (sendto) / `0x00234d48` (recvfrom) |
| imports | sys_io (pad), cellGcmSys, sys_net, sceNp (matching, signaling), cellSpurs, cellAudio, cellAtrac |
