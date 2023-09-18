@echo off
set CurrentPath=%cd%
cd %CurrentPath%

if exist build rmdir /s /q build
if not exist build md build
cd build
cmake .. -G "Visual Studio 15 2017 Win64" -DCLOUD_PLATFORM=on -DCMAKE_BUILD_TYPE=Release

cd ..
if not exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" ( echo "ERROR: VS2017 not found" )
for /f "tokens=*" %%a in ('"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -property productPath') do set devenv=%%a
"%devenv:~0,-4%.com" .\build\ADASSample.sln /build Release

cd ..\\bin\\Release

if exist AEB.zip del AEB.zip
if exist LKA.zip del LKA.zip

"C:\\Program Files\\7-Zip\\7z.exe" a -tzip "AEB.zip" "HDMapModule.dll" "SimOneAPI.dll" "SSD.dll" "AEB.exe"
"C:\\Program Files\\7-Zip\\7z.exe" a -tzip "LKA.zip" "HDMapModule.dll" "SimOneAPI.dll" "SSD.dll" "LKA.exe"

xcopy /s /y AEB.zip ..\\..\\..\\..\\..\\WebApp\\WebServers\\AssetServer\\assets\\controller\\win32
xcopy /s /y LKA.zip ..\\..\\..\\..\\..\\WebApp\\WebServers\\AssetServer\\assets\\controller\\win32

if %ERRORLEVEL% GEQ 1 EXIT /B 1
