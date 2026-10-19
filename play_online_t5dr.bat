@echo off
rem Tekken 5 Dark Resurrection (NPEA00019) - rollback netplay launcher (template, see TEKKEN5DR_ROLLBACK.md)
rem Usage: play_online_t5dr.bat <player 1|2> <remote ip:port> [local port] [delay]
rem Your ladder client can start rpcs3.exe with the same variables instead of this file.

set RB_NET=1
set RB_NET_PLAYER=%1
set RB_NET_REMOTE=%2
set RB_NET_LOCAL_PORT=%3
if "%RB_NET_LOCAL_PORT%"=="" set RB_NET_LOCAL_PORT=7000
set RB_NET_DELAY=%4
if "%RB_NET_DELAY%"=="" set RB_NET_DELAY=2

rem Fight start: fill in the task class found with RB_T5_TASKLOG=1 (calibration step 2)
set RB_T5_SYNC_TASK=tcb_swg_game_t
set RB_NET_START=+60

rem Snapshots (calibration step 4)
set RB_SNAP_RANGES=~0x00450000-0x004A0000,~0x10050000-0x10770000,~0x20000000-0x40000000
set RB_SNAP_TRACK=1
set RB_SNAP_CALIBRATE_FROM=+1
set RB_SNAP_SAVE_AT=+60
set RB_SNAP_RESTORE_AT=999999999

rem Determinism (calibration step 3)
set RB_VIRTUAL_TB=1

start "" "%~dp0rpcs3.exe"
