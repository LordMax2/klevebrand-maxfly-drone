# Klevebrand Drone Gyro – BNO085 on SPI

BNO085 implementation of the [Klevebrand All-Drone Controller](https://github.com/LordMax2/klevebrand-all-drone-controller) `DroneGyroConcept`, over SPI. It has the same modes and readings as the I2C [klevebrand-drone-gyro-bno085](https://github.com/LordMax2/klevebrand-drone-gyro-bno085).

- Euler angles from the stabilized rotation vector, angle rates in acro mode
- Acceleration rotated into the world frame, with gravity removed, for position estimation
- The reset and PS0/WAKE lines go through any type that meets `DigitalOutputConcept`, so they can be on pads that have no Arduino pin number

```cpp
// SPI1, chip select 38, interrupt 41, reset and wake lines of your own output type
Bno08xSpiDroneGyro<SomeDigitalOutputType> gyro(SPI1, 38, 41, reset_line, wake_line);

gyro.setup();
gyro.setModeEulerAndAcceleration();
```

The board must strap PS1 high. Pick the bus pins (for example `SPI1.setMISO(39)` on Teensy 4) before `setup()`, which starts the bus.

MIT License
