@echo off
rem Double-click to start the flight controller PID Tuner (opens MATLAB, then the tuner window).
start "" "D:\matlab\bin\matlab.exe" -sd "%~dp0" -r "fc_tuner_app"
