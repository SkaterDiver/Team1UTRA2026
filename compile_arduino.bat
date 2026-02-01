@echo off
echo Compiling servo_test for Arduino Uno R4...
arduino-cli compile --fqbn arduino:renesas_uno:minima main.ino
if %ERRORLEVEL% EQU 0 (
    echo.
    echo Compilation successful!
) else (
    echo.
    echo Compilation failed!
)
pause