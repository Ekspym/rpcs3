@echo off
rem Determinism test, step 2: pick the SAME characters and stage by hand. When the round starts the recording plays.
set RB_MODE=replay
set RB_FILE=t5_test.bin
set RB_HASH=hash_replay.txt
set RB_T5_SYNC_TASK=tcb_swg_game_t
set RB_REPLAY_FROM_SYNC=1
start "" "%~dp0rpcs3.exe"
