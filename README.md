# ROBA BattleBot ESP32

ESP32-based autonomous combat robot for MEAM 5100, featuring wall following, servo attack, TopHat health I2C, and manual/web control.

## Overview

This repository contains the final code and project documentation for a MEAM 5100 Robot Battle Arena (RoBA) robot. The robot uses a differential-drive chassis, two ToF sensors for wall following, a servo-based attack arm, and TopHat I2C communication for wireless packet reporting and health-based shutdown.

The system supports both manual and autonomous behaviors, including browser-based control, scripted tower routines, wall following, and experimental Vive-based point-to-point navigation.

## Features

- Differential-drive mobile base
- Browser-based manual control over WiFi
- Servo-driven whisker attack mechanism
- Right-wall following with ToF sensors
- Scripted low/high tower attack routines
- Experimental Vive-assisted GoTo navigation
- TopHat I2C integration at 2 Hz for wireless packet reporting
- Health-based motion disable with 15 s respawn timer

## Repository structure

```text
roba-battlebot-esp32-clean/
├── README.md
├── .gitignore
├── bom/
│   └── bom.md
├── docs/
│   ├── CONTROL_MODES.md
│   ├── HARDWARE_OVERVIEW.md
│   └── PROJECT_NOTES.md
├── library/
│   ├── html510.h
│   ├── vive510.h
└── src/
    ├── code.ino
```

## Source files
- `src/code.ino`: public version with WiFi credentials removed

## Main subsystems

- ESP32-S3 main controller
- L298N motor driver with PWM speed control
- MG90S servo attack arm
- Two VL53L0X ToF range sensors on I2C
- TopHat I2C master communication
- Optional Vive localization support through `vive510`
- Web control interface through `html510`

## Libraries

This project uses:

- `Wire.h`
- `Adafruit_VL53L0X.h`
- `html510.h`
- `vive510.h`

Make sure the  support libraries for `html510` and `vive510` are installed before compiling.

