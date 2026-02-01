# Quick Start Guide - Arduino Uno R4

## Getting Started

### 1. Run First-Time Setup
```powershell
cd main
.\setup_arduino.bat
```

This will:
- Initialize Arduino CLI
- Install the Renesas core for Uno R4
- Compile the servo test sketch

### 2. Connect Your Arduino Uno R4
- Connect via USB-C cable
- Note the COM port (shown by the setup script)

### 3. Upload to Board
```powershell
.\upload_arduino.bat
```

### 4. Monitor Serial Output
```powershell
arduino-cli monitor -p COM3 -c baudrate=9600
```
Replace COM3 with your port.

## Hardware Setup

Connect your servo motor:
- **Signal (yellow/orange)** → Pin 9
- **Power (red)** → 5V
- **Ground (black/brown)** → GND

## After Upload

Watch for:
1. **Built-in LED blinks 3 times** → Code is running
2. **LED blinks during sweep** → Loop is executing
3. **Servo moves** → Everything working!

## Workflow

1. Edit `main.ino` in VSCode/Cursor
2. Compile: `.\compile_arduino.bat`
3. Upload: `.\upload_arduino.bat`

## Arduino Uno R4 Variants

If you have the **WiFi version** instead of Minima, edit the .bat files and change:
```
arduino:renesas_uno:minima
```
to:
```
arduino:renesas_uno:unor4wifi
```

## Troubleshooting

**Board not detected**
- Try a different USB-C cable (some are charge-only)
- Try a different USB port

**Upload fails**
- Close any serial monitor before uploading
- Press the reset button on the board and try again

**LED doesn't blink**
- Check if upload actually completed
- Try pressing the reset button