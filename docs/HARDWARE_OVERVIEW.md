# Hardware Overview

## Chassis

The robot uses a two-deck acrylic chassis with a differential-drive layout.

- The lower deck carries most of the mass and electronics.
- The upper deck carries the servo arm, TopHat, and Vive-related electronics.
- A front caster supports the chassis.
- The motors are mounted low to keep the center of gravity down.

## Sensors

### ToF sensors
Two VL53L0X sensors are mounted near the front-right corner:
- one looking forward
- one looking right

These sensors support right-wall following and obstacle response.

### Vive photodiode
A single Vive photodiode circuit is mounted on the upper structure for localization experiments.

### TopHat
The TopHat provides:
- whisker switch interaction
- LED ring health display
- I2C health/status communication

## Actuators

### Drive motors
Two geared DC motors power the left and right wheels through an L298N driver.

### Attack servo
An MG90S servo swings an acrylic attack arm to engage enemy whisker switches.

## Power architecture

The design separates power domains:

- **3S LiPo** for motors and high-power hardware
- **USB power bank** for ESP32-S3, ToF sensors, and sensitive electronics
- **12 V to 5 V regulator** for the servo

This helps reduce noise and protects the controller from motor-related power issues.
