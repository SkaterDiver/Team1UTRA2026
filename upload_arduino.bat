@echo off
echo ============================================
echo Arduino Uno R4 Upload Script
echo ============================================
echo.

echo Detecting connected boards...
arduino-cli board list
echo.

set /p PORT="Enter the COM port (e.g., COM3): "

echo.
echo Compiling and uploading to %PORT%...
arduino-cli compile --fqbn arduino:renesas_uno:minima main.ino
arduino-cli upload -p %PORT% --fqbn arduino:renesas_uno:minima main.ino

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Upload successful!
    echo.
    echo To monitor serial output, run:
    echo arduino-cli monitor -p %PORT% -c baudrate=9600
) else (
    echo.
    echo Upload failed! Check:
    echo   - Is the correct COM port selected?
    echo   - Is the Arduino connected?
    echo   - Is another program using the port?
)
pause