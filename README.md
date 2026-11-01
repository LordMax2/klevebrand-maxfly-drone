# Klevebrand MaxFly Drone

Open-source C++ flight controller **specifically for quadcopters**, built around one clear idea: **it doesn't need to be harder than it needs to be**.

This project takes the core from [Klevebrand All-Drone Controller](https://github.com/LordMax2/klevebrand-all-drone-controller) and implements everything you need for a solid, flyable quadcopter.

Runs on the **All Drone Controller** flight controller board: an NXP **i.MX RT1062** (600 MHz Cortex-M7) on the Teensy 4.1 core.

## Features

- **High-performance C++ code** — clean, simple, easy to understand, modern C++
- **Fast hardware** — i.MX RT1062 at 600 MHz with 1 MB of on-chip RAM, on the Teensy 4.1 Arduino core
- **Precise motion tracking** — uses the excellent **BNO085** 9-DOF IMU (gyro + accel + mag)
- **Self-calibrating PID stabilization** — the killer feature from the core library:  
  **Black Box optimization** + **Simulated Annealing** automatically tunes PID values  
  → no endless manual tweaking for different motors, props, batteries or frames  
  → gets noticeably better the more you fly it  
  → quietly keeps adapting in real-time to wind, temperature, motor wear, etc.
- **Radio support** — works with standard PWM receivers + can use 4G/LTE modems for serious long-range control
- **GNSS/GPS** — Available GPS interface to be able to implement autopilot or real time flying metrics of GNSS/GPS data.

## Artificial Intelligence for PID Calibration

Tuning PID for a quadcopter so it flies stable and feels good is usually time-consuming, fiddly, and frustrating.

This flight controller fixes that.

It uses a **Simulated Annealing** algorithm to run small test movements on its own, then smartly adjusts the PID parameters. This black-box method finds a near-optimal tune with almost no work from you — giving crisp, stable flight much faster.

The optimization **keeps running quietly during normal flights** and adapts on the fly to things like gusty wind, rain, aging motors, different battery levels, or prop changes.

Over time it tunes itself to match **your flying style** — making the quad feel more and more dialed-in the way **you** like to fly.

## How it relates to Klevebrand All-Drone Controller

This is the **quadcopter-specific implementation** built on top of the general-purpose library:  
https://github.com/LordMax2/klevebrand-all-drone-controller

- The core lib handles the universal PID logic, sensor abstraction, self-tuning magic, etc.
- MaxFly Drone adds the quad-specific mixer, motor outputs, receiver parsing, and the flight controller board support so you can actually build and fly a quad right away.

## Components

The sensor drivers are PlatformIO libraries of their own in `components/`, so other projects can reuse them:

- [`klevebrand-drone-gyro-bno085-spi`](components/klevebrand-drone-gyro-bno085-spi): the BNO085 on SPI, as a core library gyro.
- [`klevebrand-drone-position-bmp581-neom9n`](components/klevebrand-drone-position-bmp581-neom9n): position and velocity from the NEO-M9N, fused with the gyro's acceleration, plus pressure and temperature from the BMP581.

`platformio.ini` pulls them in through `symlink://`, and each `library.json` lists its own dependencies. A component never includes anything from `include/` or `src/`: the board's pins, buses and serial ports go in through its constructor, in `src/main.cpp`.

To give a component a package of its own, move its folder into a repository, add the `repository` to its `library.json`, publish it, and replace its `symlink://` line in `lib_deps` with the package.

## Hardware

- **Sensors:** BNO085 on SPI1, BMP581 on SPI, NEO-M9N GPS on `Serial1`. The ICM-45686 is kept deselected; nothing reads it yet.
- **Skywire modem:** on `Serial2`. The firmware switches on its supply, pulses ON_OFF and asserts RTS, then runs the startup commands.
- **Motors:** OUT1-OUT4 are left front, right front, left back and right back, driven as 400 Hz PWM (1000-2000 µs). The output buffers stay disabled until every motor sends an idle pulse.
- **Receiver:** RC IN channels 1-8.
- **LEDs:** the error LED is lit while the drone starts up (sensors, GPS fix, modem). When setup finishes it goes off and the status LED comes on.

### Flashing

The board has no Teensy bootloader chip, so Teensy Loader can't program it. With a J-Link on the SWD header, run `pio run -t upload`. Without one, hold BOOT and press RESET to enter NXP's ROM serial downloader, then flash `.pio/build/MIMXRT1062/firmware.hex` with MCUXpresso Secure Provisioning or `blhost`.

### Before the first flight

With the props off, check:
- the motor order and spin directions
- that roll, pitch and yaw react in the right direction
- that your ESCs accept 400 Hz PWM

## License

MIT License — see the `LICENSE` file.
