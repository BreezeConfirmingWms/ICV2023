@echo off
set CurrentPath=%cd%
cd %CurrentPath%
path %ProgramFiles%\CMake\bin;%PATH%

rmdir /s /q build
if not exist build md build
cd build
cmake .. -G "Visual Studio 16 2019"
cd ..
PAUSE
