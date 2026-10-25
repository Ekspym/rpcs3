@echo off
rem Determinism test, step 1: record one fight. Pick characters/stage by hand, play the round, close RPCS3.
set RB_MODE=record
set RB_FILE=t5_test.bin
set RB_HASH=hash_record.txt
set RB_T5_SYNC_TASK=tcb_swg_game_t
start "" "%~dp0rpcs3.exe"
