# Tekken 6 rollback (RPCS3 fork)

Source code of the Tekken 6 rollback netcode (BLUS30359 v1.03, online over RPCN).
Releases and the player guide: https://github.com/Ekspym/Tekken6-rollback

This branch (`tekken6-rollback`) is RPCS3 plus the rollback changes. The rollback itself lives in
`rpcs3/Emu/Rollback/` (`rb_state` snapshots and restores, `rb_input` / `rb_online` the GGPO session and
the input handling, `rb_net` the GGPO glue, `rb_native` the natively recompiled game code); the remaining
changes are small hooks in the emulator.

## Building (Windows, Visual Studio)

The repositories are expected side by side:

```
<dir>/rpcs3      this fork, branch tekken6-rollback
<dir>/ggpo       https://github.com/Ekspym/ggpo, branch tekken6-rollback
<dir>/ps3recomp  https://github.com/Ekspym/ps3recomp, branch tekken6-rollback (only for the native code)
```

1. GGPO as a static library with the static CRT, into `ggpo/build_rb`:
   `cmake -S ggpo -B ggpo/build_rb -A x64 -DBUILD_SHARED_LIBS=OFF` (the branch sets `/MT`)
   then `cmake --build ggpo/build_rb --config Release`.
   The linker finds `GGPO.lib` in `ggpo/build_rb/lib/x64/Release`.
2. RPCS3 as described in `BUILDING.md` (`rpcs3.sln`, Release x64).

The game and the PS3 firmware are not part of this repository.

RPCS3 is licensed under GPLv2 (see `LICENSE`); the rollback changes are under the same license.
