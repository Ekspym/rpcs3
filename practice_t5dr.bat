@echo off
rem Tekken 5 DR practice tools (the launcher's Practice button sets the same). Put this file in bin next to rpcs3.exe.
rem In the game: Practice mode, set the dummy to be controlled by player 2 / controller.
rem Hold SELECT and press: Square save position, Cross reset to it (the recording plays from there), Triangle record
rem the dummy (your controller plays player 2), Circle play the recording, R1 loop, L2 frame data on/off,
rem L1 capture 240 frames of fight data for the developer.
set RB_T5_PRACTICE=1
set RB_T5_SYNC_TASK=tcb_swg_game_t
set RB_SNAP_RING=1
set RB_SNAP_RANGES=~0x00450000-0x00490000,~0x10000000-0x10b00000,~0x30000000-0x30300000,~0x50000000-0x55d00000,~0x59400000-0x59900000
set RB_SNAP_TRACK=1
set RB_SNAP_CONT_CAL=1
set RB_SNAP_CALIBRATE_FROM=+0
set RB_SNAP_SAVE_AT=+120
set RB_SNAP_RESTORE_AT=999999999
set RB_DUMP_RANGES=0x10090000-0x100a0000
start "" "%~dp0rpcs3.exe"
