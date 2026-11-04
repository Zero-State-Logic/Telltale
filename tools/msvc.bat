@echo off
rem Runs the MSVC C compiler with the x64 build environment loaded.
rem Usage (Git Bash):   cmd //c "tools\\msvc.bat /O2 file.c /Fe:build\\file.exe"
rem Usage (PowerShell): cmd /c "tools\msvc.bat /O2 file.c /Fe:build\file.exe"
set PATH=C:\Program Files (x86)\Microsoft Visual Studio\Installer;%PATH%
call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cl /nologo /W4 /std:c11 %*
