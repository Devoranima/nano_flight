# INTRODUCTION

Drone development is one of the most active fields of engineering today. Drones perform aerial photography, monitor wildlife, deliver packages, and assist rescue services and law enforcement. Society needs control systems for quadcopters that are simple, reliable, and inexpensive, so that they can be used both in commercial and in research projects. The main challenge today is to create algorithms that provide stable and accurate flight even when the drone has limited computational capability. This is what makes the project relevant at the present time.

The theoretical significance lies in creating and improving control algorithms that ensure stable drone flight. From a practical standpoint, the result of this work is a quadcopter prototype with an implemented flight controller that can be used for further development or educational purposes. The control algorithm that was written can be adapted for other types of drones or autonomous systems, increasing its versatility.

For some time now, the control of unmanned aerial vehicles has been one of the most relevant topics in scientific and engineering practice. Anderson and Moore (2017) [1] emphasize the importance of fast and accurate processing of inertial sensor data for achieving stable drone flight. Furthermore, a number of studies on open drone control platforms (e.g., Smith et al., 2020 [2]) point out that affordable microcontrollers are nonetheless capable of delivering sufficient performance given proper algorithm optimization. On the other hand, existing sensor libraries, such as the Adafruit Unified Sensor library, often require modification to fit specific tasks, which highlights the need to develop specialized solutions. This work is therefore built around creating its own implementation of sensor interfacing and control algorithms, which distinguishes it from existing solutions.

The goal of the work is to design and implement a control algorithm for an unmanned quadcopter that ensures stable and accurate flight using an affordable hardware platform based on the Arduino Nano microcontroller with an ATmega328P processor, which has sufficient computational power for control and stabilization tasks. To accomplish this, a quadcopter with an X-shaped motor layout was assembled. The Adafruit 9DOF board (with the LSM303 and L3GD20 sensors) and a BMP085 barometer were used to determine the drone's spatial orientation. To ensure reliable operation, a custom software library for I2C sensor communication was developed, which made it possible to optimize data processing. Within the scope of this work, the following control components were implemented: complementary-filter data fusion, automatic sensor calibration, barometer-based altitude control, and flight stabilization algorithms.

# CHAPTER 1. Assembly and mechanics of the drone

This section describes the process of designing and assembling the mechanical part of the quadcopter, including the choice of materials and components, the approach to their integration, and solutions to problems that arose during implementation.

## 1. Structural base

A TBS Source One V5 carbon frame, a very popular platform in this class of drones, was chosen for the build. 4 mm carbon-fiber components are used for the main load-bearing parts; 2 mm for secondary parts. Carbon fiber offers high strength at low weight, but — most importantly — it helps reduce the load on the motors, thereby increasing flight time. Installing components is made easier by the modular design and simple replacement when needed. The motors were mounted in an X configuration to achieve a balance between stability and maneuverability, since this layout distributes thrust evenly across all four axes. The layout is shown in Figure 1.

**Figure 1** — Motor layout diagram (FRONT / LEFT / RIGHT / REAR)

## 2. Motors

Four brushless XING 2207 motors rated at 2750 KV were chosen, offering high efficiency, which makes them an excellent choice for mid-size drones. Each motor was fitted with a 5×4.5×3 propeller (5 inches in diameter, 4.5 pitch, three blades).

The motors are driven by BLHeli_S electronic speed controllers (ESCs). These controllers communicate via a PWM signal, which greatly simplifies motor control. Each ESC connects on one side to its motor, and on the other to the battery and the flight controller. T-Plug and XT60 connectors were used for simple and safe power connections, along with Dupont connectors for connecting to the microcontroller board. The ESCs were placed inside the drone's body to minimize wire length and improve signal quality. The ESC and motor wiring diagram is shown in Figure 2.

**Figure 2** — Motor and ESC wiring diagram

## 3. Power

The quadcopter is powered by a 3S lithium-polymer battery (three cells, 11.1 V nominal voltage) with a capacity of 5200 mAh and a maximum discharge current rating of 90C. Such a battery provides enough power to run the motors and electronics for 10–15 minutes of flight, depending on maneuver intensity and external conditions such as air temperature and wind. The battery is secured to the frame with adjustable hook-and-loop straps, which allows for quick swaps and ensures a secure hold during flight. Stock vibration-damping pads were used to protect the battery from mechanical shock and vibration.

To power the Arduino Nano board, a step-down voltage converter (buck converter) with a 7 V output and up to 3 A of current was used. The converter provides stable power to the controller and the sensors connected to it, preventing malfunctions during operation.

## 4. Controller and sensors

The central element of the control system is the Arduino Nano microcontroller board, built around an ATmega328P running at 16 MHz. This microcontroller was chosen for its compactness, availability, and sufficient computational power to run the control algorithms and process sensor data.

The Adafruit 9DOF board, which includes the LSM303 (accelerometer and magnetometer) and L3GD20 (gyroscope) sensors, was used to determine the quadcopter's spatial orientation. The accelerometer measures acceleration along three axes, the magnetometer determines direction relative to Earth's magnetic field, and the gyroscope measures angular rates. This data is used to compute the copter's orientation angles. A BMP085 barometric sensor was used to compute pressure and altitude; it measures atmospheric pressure and temperature and derives altitude from them with an accuracy of 0.25 m. All sensors are connected to the Arduino Nano over the I2C protocol, which allows several devices to be read at once using only four wires for the connection (VIN, GND, SCL, SDA).

## 5. Control link

An NRF24L01 2.4 GHz radio module was used for wireless control of the quadcopter. It provides a reliable single-hop, two-way link between the remote device and the quadcopter over a range of up to 1000 m in line-of-sight conditions. Transmission latency is very low, which is ideal for real-time control requirements. This module connects to the Arduino Nano over the SPI interface, and an external antenna is used to improve link stability. On the remote side, an identical NRF24L01 module is connected along with joysticks and potentiometers, which are used to send commands.

The wiring diagram of the Arduino Nano board, sensors, radio module, and speed controllers is shown in Figure 3.

**Figure 3** — Main component wiring diagram

## 6. Assembly process

Motors were mounted to the frame first, after which their orientation was checked to ensure the correct thrust direction. Next, the speed controllers were installed, connecting to the motors and battery through a custom-built power distribution board. The step-down voltage converter was also mounted on this power distribution board. This whole assembly was in turn secured inside the body. A perfboard with the Arduino Nano soldered onto it, along with all the sensors and the NRF24L01 radio module, was mounted on top of the battery on flexible mounts to keep the Arduino's serial port easily accessible. Nylon cable ties were used to secure the wiring and prevent it from shifting during flight.

Several technical difficulties came up during assembly. The main problem was the placement of the speed controllers, which tended to pick up interference when mounted incorrectly. To fix this, the controllers were moved from the arms to the center of the body, and the signal wires running to the Arduino Nano were twisted together. Another problem was that the speed controllers occasionally behaved incorrectly. This was resolved by reflashing the controllers' firmware using an Arduino Uno and the BLHeliSuite software. In addition, interference resulting in corrupted data was observed when using the radio module. The issue turned out to be the module's high sensitivity to supply voltage. To fix this, a 10 µF capacitor was installed directly across the radio module's supply.

The result of the build was a quadcopter with a total mass of about 700 g (including the battery), possessing high strength and stability. The final construction turned out compact, with an optimized component layout, which simplified further software configuration and testing of the control algorithms. The resulting mechanical base provides a reliable platform for the control and stabilization tasks described in the following sections of this work.

# CHAPTER 2. Working with the sensors

The Adafruit driver library package was initially used as the basis for working with the sensors. During the development of the flight controller, it became clear that the Nano microcontroller's memory was severely limited. Under these constraints, using third-party libraries carrying a lot of unnecessary functionality became questionable. After extensive testing, the decision was made to drop third-party sensor libraries in favor of writing custom functionality that makes more sensible use of the microcontroller's resources.

As mentioned, the sensor chips are accessed over the I2C protocol, which offers addressable communication between devices using only 4 wires [3]. The I2C protocol is built around:

- SDA (Serial Data Line) — the line data is transmitted over;
- SCL (Serial Clock Line) — the clock line, which synchronizes data transmission.

A bus can have only one master, while all other devices are slaves. The master initiates data transfer by generating clock pulses and control signals, while the slaves respond to the master's requests. The wiring is shown in Figure 4.

**Figure 4** — Device wiring over the I2C interface

Owing to how widespread the protocol is, modern Arduino boards (including the Nano v3) ship with the Wire.h library preinstalled, providing a ready-made interface for quickly and easily building I2C device functionality.

The library was developed using the documentation for the Adafruit Unified L3GD20 Driver [4] and Adafruit LSM303 Accelerometer/Magnetometer Library [5] projects. In addition, the publicly available documentation released by STMicroelectronics [6] was used to understand the sensors' operating principles and register layout.

While writing the code for reading the accelerometer, gyroscope, magnetometer, and barometer data, helper functions for simplified register access were also written. For example, sensor activation and configuration used the `writeRegister` function (Listing 1), which implements register writes for the sensor chips.

```c
void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
```
**Listing 1** — writeRegister function code

And the `readMultiple` function (Listing 2) was written to read data from the registers.

```c
void readMultiple(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *dest) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
      dest[i] = Wire.read();
    }
}
```
**Listing 2** — readMultiple function code

The corresponding sensor registers are used for sensor configuration and enablement. Key configuration registers include the gyroscope's CTRL_REG4 (Table 1), as well as part of the magnetometer's CRB_REG_M register.

**Table 1** — CTRL_REG4 register layout

| BDU | BLE | FS1 | FS2 | - | 0 | 0 | SIM |
|---|---|---|---|---|---|---|---|
| Block data update. | Big/little endian data selection. | Full scale selection | | | | | SPI serial interface mode selection |

For example, writing 0x00 to the CTRL_REG4 register sets the gyroscope's full-scale sensitivity to 250 dps (degrees per second).

While writing the code that computes the drone's altitude from the BMP085 sensor data, a problem with long delays arose, stemming from how the sensor works. In the worst case, the time taken for one read cycle went up to 22 ms, more than double the theoretical cycle time. To solve this, a so-called "state machine" was used, which avoids stalling the controller for a long time by exploiting the fact that altitude does not change as sharply as the orientation angles do. This way, each time the altitude-reading function is called, the controller advances through one of its steps and moves the "machine" into the next state. There are three such states in total: IDLE, WAIT_TEMP, and WAIT_PRESS. In the IDLE state, a request for the current temperature is issued, after which the state is set to WAIT_TEMP. During the WAIT_TEMP step, the controller requests the current pressure reading from the sensor and sets the state to WAIT_PRESS. In the final step, the current altitude is computed, at the end of which the state is reset back to IDLE.

The result of this work was a library implementing a system of non-blocking operations, whose execution time (~6 ms) fits within the theoretical read-cycle budget (< 10 ms).

# CHAPTER 3. Complementary filter

In aerodynamics and robotics, a system of three angles is used to describe an object's spatial tilt:

- Roll — rotation of the object around its longitudinal axis, passing through its center of mass;
- Pitch — rotation of the object around its lateral axis, also passing through its center of mass;
- Yaw — rotation of the object around its vertical axis, also passing through its center of mass.

This representation is based on Euler angles, which describe the rotation of an absolutely rigid body in three-dimensional Euclidean space.

Accurate values of the drone's spatial position are critical for the control algorithms to work correctly. Computing the tilt angles from the accelerometer, gyroscope, and magnetometer data requires what is known as a complementary filter, whose job is to combine and transform data from several sensors and output the copter's tilt angles [7]. The filter plays a key role in the drone's operation, since it compensates for the shortcomings of each sensor individually and produces an accurate real-time physical representation of orientation [8]. This section describes the filter's operating principle, its implementation, and a comparison with alternative solutions.

## 1. Theoretical basis of the orientation angles

The drone's orientation angles are determined from inertial sensor data: the accelerometer, the gyroscope, and the magnetometer. The accelerometer measures acceleration along three axes (x, y, z), including gravitational acceleration, which makes it possible to compute the roll and pitch angles relative to the gravity vector. The formulas for computing the roll (φ) and pitch (θ) angles from the accelerometer data (ax, ay, az) are as follows:

```
φ = atan2(ay, az)
θ = atan2(−ax, √(ay² + az²))
```

where atan2 is the two-argument arctangent function, which takes the signs of its arguments into account. Values computed this way are very accurate while the object is stationary or moving at constant velocity. However, in the presence of acceleration, the accelerometer data becomes heavily contaminated by noise, which means it cannot be relied upon alone.

The gyroscope measures angular rates along three axes (ωx, ωy, ωz), which can be integrated over time to obtain the orientation angles:

```
φ(t) = φ(t₀) + ∫ωx(τ)dτ   [t₀ to t]
θ(t) = θ(t₀) + ∫ωy(τ)dτ   [t₀ to t]
ψ(t) = ψ(t₀) + ∫ωz(τ)dτ   [t₀ to t]
```

The downside of using the gyroscope is the integral nature of the computation, which leads to accumulated error (drift) caused by noise in the data. This makes the readings unreliable over extended use.

The magnetometer works by measuring the direction of Earth's magnetic field. It lets the yaw angle be corrected, but it is highly sensitive to external magnetic interference (e.g., from powerful motors).

Obtaining accurate tilt angles for the drone requires combining data from several sensors. This is achieved through sensor-fusion algorithms, the most popular of which is the complementary filter.

## 2. Operating principle of the complementary filter

The complementary filter is based on the idea of fusing accelerometer and gyroscope data. In various forms, the magnetometer can also be incorporated, but it is not used in this implementation. As already noted, the accelerometer provides fairly accurate roll and pitch data under static or slowly changing conditions, but is subject to high-frequency noise from vibration and acceleration. The gyroscope, on the other hand, is accurate at high frequencies, where it tracks fast changes in orientation well, but suffers from drift at low frequencies due to accumulated integration error.

In this implementation, the complementary filter combines the data using a weighted sum:

```
φ = α · φ_gyro + (1 − α) · φ_accel
θ = α · θ_gyro + (1 − α) · θ_accel
```

where α is the filter's "trust" coefficient toward the gyroscope (typically in the 0.9–0.99 range), φ_gyro and θ_gyro are the angles obtained by integrating the gyroscope data, and φ_accel and θ_accel are the angles computed from the accelerometer data. A high value of α (e.g., 0.98) is preferable for quadcopters, since the gyroscope provides accuracy during fast maneuvers, while the accelerometer corrects drift over long intervals.

## 3. Implementation of the complementary filter

The algorithm works as follows:

1) **Sensor data acquisition.** Acceleration along the x, y, z axes is requested from the accelerometer, and angular rates ωx, ωy, ωz in degrees per second are requested from the gyroscope. The angular rates are then converted to radians per second by multiplying by a factor of 0.0174533 (π/180).

2) **Time-delta calculation.** To integrate the angular rates obtained from the gyroscope, the time interval dt between the current and previous measurement is determined using the `micros()` function to get the time in microseconds.

3) **Accelerometer angle computation.** The roll and pitch angles are computed using the formulas given above, using the `atan2` function. The yaw angle is not computed from the accelerometer data, since that would require a magnetometer, which is not used in this implementation due to interference.

4) **Gyroscope data integration.** The orientation angles (φ, θ, ψ) are updated by integrating the angular rates:

```
φ += ωx · dτ
θ += ωy · dτ
ψ  = ωz · dτ
```

5) **Applying the complementary filter.** The roll and pitch angles are corrected using the complementary filter:

```
φ = α · φ + (1 − α) · φ_accel
θ = α · θ + (1 − α) · θ_accel
```

where α = 0.98, giving priority to the gyroscope data while still letting the accelerometer correct drift.

6) **Conversion to degrees.** For output and further use, the angles are converted from radians to degrees by multiplying by 180/π.

7) **Update rate.** The algorithm runs at a rate of about 100 Hz, achieved by adding a 10 ms delay (`delay(10)`), providing a sufficiently high rate for real-time quadcopter control.

The code is optimized for minimal computation time, avoiding the use of heavyweight libraries.

## 4. Comparison with alternative methods

Ready-made solutions for sensor data fusion were initially considered for the project, but each of the solutions examined had a number of drawbacks compared to a custom complementary-filter implementation. Among the methods evaluated, the Madgwick and Mahony filters stand out, as they are commonly used in unmanned control systems. Let's look at their characteristics and compare them to the chosen solution.

The Madgwick filter is an algorithm proposed by Sebastian Madgwick that uses gradient descent as its orientation-error minimization function, combining accelerometer, gyroscope, and magnetometer data. It works by computing orientation as quaternions, which avoids the gimbal-lock singularity problem characteristic of Euler angles. This filter is effective for highly dynamic systems with accurate sensors, but requires significant computational resources due to its iterative computations and quaternion math. For the chosen Arduino Nano, with its limited performance (16 MHz) and memory, this leads to large delays, which is critical for the drone's operation.

The Mahony algorithm also works with quaternions and uses a proportional-integral controller to correct errors based on accelerometer and magnetometer data. It is less demanding on computational resources, but still requires computing rotation matrices and normalizing quaternions. This significantly increases data-processing time, which is likewise unacceptable for the drone to operate correctly.

Despite its significant advantages, the complementary filter used here has a number of limitations. High sensitivity to sensor calibration quality leads to errors in the initial accelerometer and gyroscope data, which can result in inaccurate computed orientation. In addition, the filter does not use magnetometer data to correct yaw, which is a problem during long flights, where gyroscope drift becomes noticeable. In this work, that problem is mitigated by regular sensor calibration before flight.

To improve accuracy in the future, a hybrid approach could be considered, where the complementary filter is combined with elements of the Mahony filter to increase accuracy with minimal added computational load. However, for the current implementation the complementary filter turned out to be the optimal solution, providing a balance between performance, accuracy, and simplicity.

# CHAPTER 4. Drone orientation control

A proportional-integral-derivative (PID) controller is used in this work to control the drone's orientation. It processes the data obtained from the complementary filter and distributes thrust between the motors to achieve the required orientation angles. This section describes the theoretical basis of orientation control, the operating principles of the PID controller, and its tuning.

## 1. Fundamentals of orientation control

As already mentioned, the copter's spatial orientation is described using three angles corresponding to the drone's orthogonal axes. Controlling these angles requires varying the rotation speed of each of the four motors to produce the necessary torques.

The X-frame quadcopter has four motors, labeled M1 (front-left), M2 (front-right), M3 (rear-right), and M4 (rear-left). Orientation control is achieved through differential adjustment of motor thrust:

- **Roll:** to bank right, the thrust of motors M1 and M4 (left side) is increased and that of M2 and M3 (right side) is decreased. To bank left, the reverse is done.
- **Pitch:** to tilt forward, the thrust of motors M3 and M4 (rear) is increased and that of M1 and M2 (front) is decreased. To tilt backward, the reverse is done.
- **Yaw:** to rotate clockwise, the thrust of motors M1 and M3 (the diagonal pair spinning counter-clockwise) is increased and that of M2 and M4 is decreased. To rotate counter-clockwise, the reverse is done.

The total thrust produced by the motors in turn affects the drone's altitude. This section focuses mainly on orientation control, since it is the more complex of the two.

## 2. Thrust distribution principles

Thrust is distributed among the motors using control signals computed by the PID controller. As already mentioned, each motor is driven through its corresponding electronic speed controller (ESC) using pulse-width modulation (PWM). The PWM signal sets the thrust level in the 0–100% range (in this case using a pulse-width configuration from 1200 to 1900 microseconds). Correction signals for each motor are formed based on the difference between the current and target orientation angles (the control error).

The thrust-distribution algorithm can be represented as follows. Let T_base be the base thrust that maintains altitude, and ΔT_roll, ΔT_pitch, ΔT_yaw the correction signals for controlling roll, pitch, and yaw respectively [9]. The thrust for each motor is then computed as:

```
T_M1 = T_base + ΔT_roll − ΔT_pitch + ΔT_yaw
T_M2 = T_base − ΔT_roll − ΔT_pitch − ΔT_yaw
T_M3 = T_base − ΔT_roll + ΔT_pitch + ΔT_yaw
T_M4 = T_base + ΔT_roll + ΔT_pitch − ΔT_yaw
```

The values ΔT_roll, ΔT_pitch, ΔT_yaw are computed by the PID controller based on the error for each of the angles. To prevent thrust values from exceeding allowed limits (e.g., negative thrust or exceeding the ESC's maximum), the signals are clamped to a known range for the input signals (1200:1900 µs).

## 3. PID controller implementation

The PID controller is needed to minimize the error between the current orientation angles (φ, θ, ψ), obtained via the complementary filter, and the target angles (φ_target, θ_target, ψ_target). The PID controller computes the control signal u(t) using the formula:

```
u(t) = Kp · e(t) + Ki · ∫e(τ)dτ [0 to t] + Kd · de(t)/dt
```

where:
- e(t) is the error (φ − φ_target, θ − θ_target, ψ − ψ_target),
- Kp is the proportional coefficient, responsible for the reaction to the current error,
- Ki is the integral coefficient, which eliminates accumulated error,
- Kd is the derivative coefficient, which accounts for the rate of change of the error.

Each orientation angle (roll, pitch, yaw) uses its own PID controller. The algorithm consists of the following steps:

1) **Orientation data acquisition.** The current angles (φ, θ, ψ) are obtained from the complementary filter described in the previous section.

2) **Error computation.** For each angle, the error is computed:

```
e_roll(t)  = φ − φ_target
e_pitch(t) = θ − θ_target
e_yaw(t)   = ψ − ψ_target
```

3) **PID update.** For each angle, the control signal is computed as:

```
u(t) = Kp · e(t) + Ki · Σe(τ)dτ + Kd · (e(t) − e(t−1))/dt
```

The integral term is represented using a discrete sum, and the derivative term through the difference of the errors over two consecutive steps.

4) **Thrust distribution.** The control signals u_roll, u_pitch, u_yaw are converted into thrust-correction values ΔT_roll, ΔT_pitch, ΔT_yaw with appropriate scaling.

5) **Signal clamping.** To prevent invalid values (e.g., negative thrust), the signals are clamped:

```
T_Mi = max(1100, min(1900, T_Mi))
```

6) **Signal output.** The computed thrust values are sent to the ESCs through the Arduino Nano's PWM outputs.

## 4. PID controller tuning

The Kp, Ki, and Kd coefficients were tuned experimentally using purpose-built test rigs, since accurately modeling the drone's dynamics is essentially infeasible given the limited computational resources available. The tuning process included the following stages:

1) **Initial Kp tuning:** The proportional coefficient was tuned first, to achieve a fast response to error without strong oscillation. For roll and pitch, an initial value of Kp = 9 showed stable behavior, while for yaw, Kp = 8.5 was used due to the lower inertia along that axis.

2) **Adding Kd:** The derivative coefficient (Kd = 0.3 for roll and pitch, Kd = 0.25 for yaw) was introduced to damp the oscillations that occur during fast orientation changes.

3) **Integral term Ki:** The integral coefficient (Ki = 0.03 for all axes) was added to eliminate steady-state error, but its value was kept minimal to avoid accumulating error over prolonged use.

The values above provided stable stabilization during maneuvers and while holding a set orientation.

# CHAPTER 5. Radio control and communication

The standard way of setting up real-time communication between a quadcopter and its remote control is to use a radio module. Radio control provides wireless transmission of control commands from the operator to the drone, including orientation control, thrust, and execution of special maneuvers such as takeoff and landing, as well as receiving real-time drone status data. This project uses an NRF24L01 radio module, operating at 2.4 GHz, to implement two-way communication. The remote control is built around an Arduino Micro board. This section describes the principles behind the radio-control setup, the remote's hardware and software, and the data transmission and reception algorithms.

## 1. Radio control principles

Drone radio control involves transmitting control signals from the remote to the drone and receiving telemetry from the drone back to the remote [10]. The control signals include:

- Thrust level (gas), which determines the overall motor power;
- Joystick position values (X and Y), which set the target roll and pitch angles;
- Special commands, such as takeoff (DEPART), landing (LAND), or emergency shutdown (SHUTDOWN).

The criteria for a reliable link are minimal transmission latency, strong interference immunity, and sufficient range. The NRF24L01 radio module was chosen as the basis for the radio link, as it supports two-way communication with dynamic packet length, allowing it to carry not only control commands but also telemetry from the drone. The module's operating frequency is 2.4 GHz, with a range of up to 1000 m in line-of-sight conditions when using an external antenna.

## 2. Remote control hardware

The remote control is built around an Arduino Micro board with an ATmega32U4 processor running at 16 MHz. This platform was chosen for its compactness, low cost, and availability. The remote's hardware includes the following components:

1) **NRF24L01 radio module.** The module connects to the Arduino board over the SPI (Serial Peripheral Interface) using the CE (Chip Enable) and CSN (Chip Select Not) pins. The NRF24L01 provides two-way communication with the drone, supporting data transmission with less than 1 ms of latency. A version with an external antenna was used to improve range and link stability. Since the drone is used for research purposes, a low transmit power level (RF24_PA_LOW) was set to minimize power consumption.

2) **Joystick.** A two-axis analog joystick connects to the Arduino board's analog pins (VRX_PIN for the X axis, VRY_PIN for the Y axis). The joystick outputs analog signals in the 0–1023 range, which are converted to values from -100 to 100 to set the roll and pitch angles.

3) **Rotary encoder.** The encoder is used to adjust the thrust level (gas). It connects to digital pins (CLK_PIN and DT_PIN) with internal pull-up resistors (INPUT_PULLUP). The encoder generates signals as it is turned, which are interpreted as increasing or decreasing thrust in the -100 to 100 range. This allows precise control of motor power.

4) **Buttons.** Two buttons (UP_PIN and DOWN_PIN) with pull-up resistors are used to send special commands: takeoff (DEPART) when the top button is pressed, and landing (LAND) when the bottom button is pressed.

5) **Power source.** The remote is powered from an external battery. This makes the remote portable and self-contained.

The hardware components are mounted on a compact board, and the wiring is secured to minimize interference and improve reliability.

## 3. Remote control software

The remote's software was developed using the RF24 library. The program performs the following functions:

1) Initialization of the radio module, joystick, encoder, and buttons.
2) Reading data from the input devices (joystick, encoder, buttons).
3) Building and sending control data to the drone.
4) Receiving and processing telemetry from the drone.
5) Outputting diagnostic information over the serial port.

The main data structures used in the program are:

- **Control_Data_t:** Contains the data for controlling the drone, including joystick position (X, Y), thrust level (gas), and a special command (NONE_COMMAND, DEPART, LAND, SHUTDOWN).
- **State_t:** Describes the drone's state, including current thrust (gas) and current mode (IDLE, DEPARTING, FLYING, LANDING, SHOTDOWN).

The remote's operating algorithm includes the following stages:

1) **Initialization (setup):** At this stage, the NRF24L01 radio module is configured: setting the transmit ("Transmitter") and receive ("Receiver") addresses, enabling dynamic packet length, setting the low power level, and starting receive mode. In addition, the input controls are initialized and configured here. The encoder pins are set to INPUT_PULLUP mode and their initial state is stored to track changes. The joystick's analog pins are configured as INPUT. The button pins are also set to INPUT_PULLUP mode to detect presses.

2) **Reading data (loop):** When the encoder's CLK pin changes state, the direction of rotation is determined (from the state of the DT pin) and the thrust value (remote_controls.gas) is updated. The value is clamped to the [-100, 100] range using the `constrain` function. The joystick's analog signals from the VRX and VRY pins are read, converted to the [-100, 100] range using the `map` function, and inverted to match the drone's direction of movement. The state of the UP_PIN and DOWN_PIN button pins is checked. Pressing the top button generates a DEPART command, the bottom one a LAND command; otherwise, NONE_COMMAND is set.

3) **Sending data to the drone (sendControlsToDrone):** The radio module switches to transmit mode (stopListening), and the remote_controls structure is sent to the drone via the radio.write function. If the transmission succeeds, the receiver_online flag is set to true, otherwise to false, signaling loss of the link. After sending, the radio module returns to receive mode (startListening).

4) **Receiving telemetry (readDroneState):** When data is available in the radio module's buffer (radio.available), the drone_state structure, containing the current thrust and drone state, is read. The data is printed over the serial port for diagnostics (printDroneState).

# CONCLUSION

The development of an unmanned quadcopter with an X-shaped motor layout, described in this work, represents a comprehensive undertaking spanning mechanical design, sensor data processing, orientation control, and radio control setup. The main goal of the work was to create a quadcopter flight-controller prototype using an affordable hardware platform based on the Arduino Nano microcontroller, capable of stable and controllable flight. In the course of this work, significant results were achieved that confirm the viability of the proposed approach and its applicability in both educational and research settings.

During the design and assembly of the quadcopter's mechanical structure, a strong and lightweight construction was built on a carbon frame, fitted with motors rated at 2750 KV, speed controllers, and a three-cell battery with a nominal voltage of 11.1 V. The use of dampers and careful component placement inside the body helped minimize vibration, providing a reliable base for the sensors and control algorithms to operate on. The sensor data processing system, built around the complementary filter, achieved accurate orientation determination at a 100 Hz update rate, which is sufficient for real-time control on the Arduino Nano's limited resources.

Orientation control is implemented using a PID controller, which effectively compensates for deviations from the target angles by differentially distributing thrust between the motors. Tuning the PID controller's coefficients made it possible to achieve stable flight despite the constraints imposed by sensor noise and measurement drift. The radio control system, based on the NRF24L01 module, provided a reliable two-way link between the remote and the drone, allowing the operator to set thrust, orientation, and special commands (takeoff, landing) at a 10 Hz update rate. The remote's software, including processing of the joystick, encoder, and button signals, was optimized to run on the Arduino Micro, demonstrating resilience to temporary link loss.

The results achieved confirm that it is possible to build a functional quadcopter using inexpensive, readily available components. The prototype has sufficient stability and controllability to perform basic flight tasks, and the algorithms and software that were developed can be adapted for other types of drones or autonomous systems. The theoretical significance of the work lies in the study and implementation of the complementary filter and the PID controller under constrained computational resources, which contributes to the field of robotic-systems control. The practical significance lies in producing a working prototype that can serve as a foundation for further development or be used for educational purposes in teaching the principles of unmanned-vehicle control.

Despite the results achieved, the work still has potential for further improvement. The main recommendations and directions for future work include:

1) **Improving sensor data processing.** To improve the accuracy of orientation determination, particularly the yaw angle, it is recommended to incorporate magnetometer data with additional filtering to compensate for magnetic interference. Hybrid algorithms combining the complementary filter with elements of the Madgwick or Mahony filters could also be used to improve accuracy with minimal added computational load.

2) **Optimizing the PID controller.** Automating the tuning of the PID controller's coefficients using methods such as Ziegler-Nichols or genetic algorithms would reduce tuning time and improve control stability. Adding an adaptive PID controller that adjusts its coefficients depending on the flight mode could improve the drone's behavior during sharp maneuvers.

3) **Increasing link range and reliability.** To extend the radio control range, higher-power radio modules or alternative protocols such as LoRa, which offer greater range at low power consumption, could be used. It is also recommended to implement automatic channel switching to minimize interference in the 2.4 GHz band.

4) **Expanding functionality.** Adding a GPS module for navigation and autonomous flight, as well as a camera for video streaming, would expand the drone's range of applications, including aerial photography and monitoring. Implementing autonomous control algorithms, such as altitude hold or route following, would raise the system's level of automation.

5) **Moving to a more capable platform.** Using microcontrollers with greater computational power, such as the STM32 or ESP32, would make it possible to implement more sophisticated control and data-processing algorithms while keeping the system compact. This would also simplify integrating additional sensors and communication modules, though at the cost of increased system complexity.

6) **Real-world testing.** Conducting flight tests under various conditions (wind, temperature swings, electromagnetic interference) would make it possible to assess the system's reliability and identify directions for further optimization. Comparison with commercial solutions such as Betaflight or Ardupilot would help identify the competitive advantages and shortcomings of the developed prototype.

Prospects for further development include creating a modular platform that could be adapted for various tasks, such as cargo delivery, agricultural monitoring, or search-and-rescue operations. The algorithms developed here could also be used in educational courses on robotics and unmanned systems, contributing to the popularization of engineering disciplines. Overall, this work demonstrates a successful implementation of a basic quadcopter control system and opens up broad opportunities for further research and practical application.
