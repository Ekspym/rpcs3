@echo off
rem Netplay test on one PC, player 2. Put this file in the RPCS3 folder bin2 (a copy of bin),
rem next to rpcs3.exe and t5_test.bin (the recording from test_record_t5dr.bat). Start test_netplay_p1.bat in bin and
rem test_netplay_p2.bat in bin2, drive both menus to the SAME characters and stage. When the round starts, both play
rem the recording against each other over GGPO (ports 7000 / 7001). Compare hash_net_p1.txt with hash_net_p2.txt.

rem The recording drives the fight, the menus are played by hand
set RB_MODE=replay
set RB_FILE=t5_test.bin
set RB_HASH=hash_net_p2.txt
set RB_T5_SYNC_TASK=tcb_swg_game_t
set RB_REPLAY_FROM_SYNC=1

rem Snapshots: the setup that passed the stress test
set RB_SNAP_RANGES=~0x00450000-0x00490000,~0x10000000-0x10b00000,~0x30000000-0x30300000,~0x50000000-0x55d00000,~0x59400000-0x59900000
set RB_SNAP_TRACK=1
set RB_SNAP_CONT_CAL=1
set RB_SNAP_CALIBRATE_FROM=+0
set RB_SNAP_SAVE_AT=+600
set RB_SNAP_RESTORE_AT=999999999

rem Netplay: player 2, this window on port 7001, the other on 7000
set RB_NET=1
set RB_NET_INPUT=record
set RB_NET_PLAYER=2
set RB_NET_LOCAL_PORT=7001
set RB_NET_REMOTE=127.0.0.1:7000
set RB_NET_DELAY=2
set RB_NET_START=+620

start "" "%~dp0rpcs3.exe"
