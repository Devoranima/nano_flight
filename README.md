# Quadcopter flight controller on Arduino Nano

A flight controller for a 5" X-frame quadcopter: Arduino Nano (ATmega328P),
Adafruit 9DOF board (LSM303 + L3GD20), BMP085 barometer, and an nRF24L01 link
to a hand-built remote.

Sensors driver was written against the I2C registers
directly (Adafruit unified-sensor stack did not fit in the Nano's memory
once the control loop was in place). Attitude comes from a complementary
filter, stabilisation from a PID per axis, and the loop runs at 100 Hz.

This is the software half of a diploma project. The write-up (Russian) is in
[`docs/thesis.pdf`](docs/thesis.pdf)

The project was lost during OS migration (yes, I know, I should have used git), 
so I had to restore this repo based on my memory (which is not very trustworthy),
my thesis and with some help from claude.

ALSO, the drone caugth a short during pid calibration on a stand (my soldering skills imrpoved a bit since then), 
so I could not fully finish it :(

## Hardware

| | |
|---|---|
| Frame | TBS Source One V5, carbon, X layout |
| Motors | 4 × XING 2207, 2750 KV, 5×4.5×3 props |
| ESCs | BLHeli_S |
| Flight controller | Arduino Nano v3 (ATmega328P, 16 MHz) |
| IMU | Adafruit 9DOF — LSM303 (accel + mag), L3GD20 (gyro) |
| Barometer | BMP085 |
| Radio | nRF24L01 2.4 GHz, external antenna, 10 µF cap across the supply |
| Battery | 3S LiPo, 11.1 V, 5200 mAh |
| Remote | nRF24L01, joystick, rotary encoder, 2 buttons |

All-up weight ≈ 700 g. Wiring diagrams are in [`docs/figures/`](docs/figures/).

## Layout

```
drone/          flight controller sketch (Arduino Nano)
  drone.ino       complementary filter, flight states, telemetry
  drone.h         shared types, loop timing, filter constants
  my_sensors.*    hand-written I2C driver: LSM303, L3GD20, BMP085
  pid.*           discrete PID
  motors.*        X mixer, ESC output
  radio.*         nRF24L01, drone side
  pins.h          pin map
remote/         remote sketch
  remote.ino      input reading, packet TX, telemetry RX
  remote.h        radio protocol
  pins.h          pin map
recovered/      the salvaged files, untouched
docs/           thesis PDF and figures pulled out of it
```

## How it works

**Sensors** (`my_sensors.*`). Register access over `Wire`, with
`writeRegister` / `readMultiple` helpers. Gyro at 250 dps full scale,
accelerometer at 100 Hz. The magnetometer is read but never initialised or
used, cause it is too close to the motors to be worth anything, so yaw is gyro-only.

The BMP085 is the awkward part. A pressure conversion at OSS=2 takes 14 ms,
which on its own blows the 10 ms, so I had to make `readAltitude()` as a
three-state machine (`IDLE_STATE → WAIT_TEMP → WAIT_PRESS`) that advances one
step per call and returns the previous altitude immediately. Altitude moves
slowly enough that this costs nothing.

**Attitude** (`readSensors()` in `drone.ino`). Roll and pitch from the gravity
vector via `atan2`, rates integrated from the gyro, the two blended:

```
angle = α · (angle + ω·dt) + (1 − α) · angle_accel,   α = 0.98
```

Gyro carries the fast changes, accelerometer pulls out its drift. Madgwick and
Mahony were both tried first, but the quaternion maths was too slow on a 16 MHz
AVR to hold 100 Hz.

**Control** (`pid.*`, `motors.*`). One PID per axis, integral clamped against
wind-up, output mixed onto the four motors:

```
FL = base + pitch − roll + yaw      FR = base + pitch + roll − yaw
BL = base − pitch − roll − yaw      BR = base − pitch + roll + yaw
```

Only roll and pitch are kinda live. Yaw and altitude hold are
wired through but have zero gains.

**Flight states.** `IDLE → DEPARTING → FLYING → LANDING → IDLE`, plus
`SHOTDOWN` as a kill switch. Take-off ramps to hover over 2s, landing bleeds
off over 4s, both holding attitude the whole time. No packet for a second means 
that the drone should land by itself.

**Radio** (`radio.*` / `remote.ino`). `Control_Data_t` out at 10 Hz,
`State_t` telemetry back on the same tick, both as raw structs — so the
protocol block in `drone/drone.h` and `remote/remote.h` must stay identical.

## Building

Arduino IDE, or:

```sh
arduino-cli compile --fqbn arduino:avr:nano drone
arduino-cli compile --fqbn arduino:avr:micro remote
```

Needs `Wire`, `Servo`, `SPI` and [RF24](https://github.com/nRF24/RF24). The
`stopListening(address)` overload the code uses is only in newer RF24
releases; on an older one, replace it with `openWritingPipe(address)`.

Props off until the motor order and the direction of the attitude estimate are
both confirmed.


### Status

Not flight tested since restoration. The code parses and type-checks cleanly
on my gcc against Arduino headers; it has not been built with
avr-gcc, flashed, or flown in this form, and the flash/RAM footprint on the
Nano is unmeasured.
