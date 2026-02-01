@echo off
echo ============================================
echo Arduino Uno R4 Setup Script
echo ============================================
echo.

echo Step 1: Initializing Arduino CLI config...
arduino-cli config init 2>nul
echo Done.
echo.

echo Step 2: Updating core index...
arduino-cli core update-index
echo Done.
echo.

echo Step 3: Installing Arduino Uno R4 core (Renesas)...
arduino-cli core install arduino:renesas_uno
echo Done.
echo.

echo Step 4: Compiling servo_test for Arduino Uno R4...
arduino-cli compile --fqbn arduino:renesas_uno:minima main.ino
echo.

echo Step 5: Listing connected boards...
arduino-cli board list
echo.

echo ============================================
echo Setup complete!
echo.
echo If your board shows as "Unknown", that's OK.
echo Just note the COM port number for uploading.
echo.
echo Next steps:
echo   1. Connect your Arduino Uno R4 via USB
echo   2. Run: .\upload_arduino.bat
echo ============================================
pause