# Klevebrand Drone Position – BMP581 and NEO-M9N

Position and velocity estimation for the [Klevebrand All-Drone Controller](https://github.com/LordMax2/klevebrand-all-drone-controller) `DronePositionConcept`. It is [klevebrand-drone-position-bmp280-neom9n](https://github.com/LordMax2/klevebrand-drone-position-bmp280-neom9n) with a BMP581 on SPI in place of the BMP280 on I2C.

- `u-blox NEO-M9N` GNSS for position, ground speed and altitude, on any `HardwareSerial`
- One Kalman filter per axis (east, north, altitude), predicted with the gyro's world-frame acceleration and corrected by the GNSS
- `BMP581` on the default SPI bus for pressure and temperature, which keep updating even without a GPS

```cpp
// Any DroneGyroConcept gyro whose acceleration is in the world frame, without gravity
DroneBmp581Neom9nPosition position(&gyro, Serial1, barometer_chip_select_pin);

position.setup(); // waits for a 3D GPS fix

position.run(gyro.reload());
```

MIT License
