@echo off
rem Finding the fight state (health, positions, timer) for the desync check: replays the recorded fight and saves the
rem game's memory at 6 moments of the round (dump_<frame>.bin, about 12 MB each). Put this file in bin next to
rem rpcs3.exe and t5_test.bin, pick characters and stage with the controller, let the round play out, close RPCS3.
rem Write down the characters, and roughly how much health each player had left at the end.
set RB_MODE=replay
set RB_FILE=t5_test.bin
set RB_T5_SYNC_TASK=tcb_swg_game_t
set RB_REPLAY_FROM_SYNC=1
set RB_DUMP_PREFIX=dump
set RB_DUMP_RANGES=0x00450000-0x00490000,0x10000000-0x10b00000
set RB_DUMP_FRAMES=1890,2430,3030,3630,4230,4830
start "" "%~dp0rpcs3.exe"
