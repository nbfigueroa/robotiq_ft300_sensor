cls
@echo off

setlocal
set PATH=%PATH%;C:\mingw\bin;C:\mingw\msys\1.0\bin;C:\MinGW\libexec\gcc\mingw32\MinGW-Version

echo *************************************************
echo This script compile you Windows sensor project
echo *************************************************
echo PREREQUISITE:
echo Have MinGW installed on your computer

pause

if not exist "c:\mingw\msys\1.0\bin\make.exe" goto no_msys_error

cd ..
call make

if errorlevel 1 goto make_error

echo Now your driver of sensor is ready
goto end

:no_msys_error
echo MinGW isn't correctly installed.
goto end

:make_error
echo There is some error in the program.

:end
pause