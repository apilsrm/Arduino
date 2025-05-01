# Motor Control Code Documentation

This document explains an Arduino program that controls a DC motor to move to specific positions using an IR slot sensor for position tracking and limit switches to set boundaries. The code is designed for beginners, with clear explanations of the hardware, code flow, and each function's purpose. By the end, you'll understand how the motor moves, how positions are tracked, and how to use the program.

---

## Table of Contents

1. [Purpose of the Code](#purpose-of-the-code)
2. [Hardware Requirements](#hardware-requirements)
3. [How the System Works](#how-the-system-works)
4. [Code Overview](#code-overview)
5. [Detailed Code Explanation](#detailed-code-explanation)
   - [Global Definitions](#global-definitions)
   - [Setup Function](#setup-function)
   - [Loop Function](#loop-function)
   - [Move to Position](#move-to-position)
   - [Handle Limit Switches](#handle-limit-switches)
   - [Set Motor Direction and Speed](#set-motor-direction-and-speed)
   - [Soft Start Mechanism](#soft-start-mechanism)
   - [Read Sensor for Position Tracking](#read-sensor-for-position-tracking)
   - [Calibrate Position](#calibrate-position)
6. [How to Use the Code](#how-to-use-the-code)
7. [Troubleshooting Tips](#troubleshooting-tips)
8. [Conclusion](#conclusion)

---

## Purpose of the Code

This Arduino program controls a DC motor to move a mechanism (e.g., a lift or slider) to specific positions between two physical limits (upper and lower). The motor's position is tracked using an IR slot sensor, and limit switches prevent the motor from moving beyond safe boundaries. The code:

- Calibrates the motor's range by finding the upper and lower limits.
- Allows users to input a target position via the Serial Monitor.
- Moves the motor to the desired position with controlled speeds.
- Uses a "soft start" to ensure the motor starts reliably, especially under load.
- Saves the current position to EEPROM (Arduino's memory) to remember it after power-off.
- Slows down downward motion using micro-delays to prevent it from being too fast.

The code is designed for applications like automated sliders, lifts, or positioning systems where precise control is needed.

---

## Hardware Requirements

To run this code, you need the following components:

1. **Arduino Board** (e.g., Arduino Uno, Mega, or compatible).
2. **DC Motor** with a motor driver (e.g., L298N or TB6612FNG) to control direction and speed.
3. **IR Slot Sensor** (optical encoder) to track motor movement by counting slots on a rotating disc.
4. **Two Limit Switches** (one for upper limit, one for lower limit) to define the motor's range.
5. **Power Supply** suitable for the motor and driver (e.g., 12V for the motor, 5V for Arduino).
6. **Wires and Breadboard** for connections.

### Pin Connections

| Component           | Arduino Pin | Description                                      |
| ------------------- | ----------- | ------------------------------------------------ |
| IR Slot Sensor (D0) | 18          | Detects slots to track position (interrupt pin). |
| Upper Limit Switch  | 7           | Stops motor at the top (LOW when pressed).       |
| Lower Limit Switch  | 6           | Stops motor at the bottom (LOW when pressed).    |
| PWM (Motor Speed)   | 10          | Controls motor speed via PWM signal.             |
| IN1 (Motor Driver)  | 35          | Motor driver input for direction control.        |
| IN2 (Motor Driver)  | 33          | Motor driver input for direction control.        |

### Notes

- The IR sensor must be connected to an interrupt-capable pin (e.g., pin 18 on Arduino Mega).
- Limit switches use internal pull-up resistors (INPUT_PULLUP), so they read HIGH normally and LOW when pressed.
- Ensure the motor driver is powered correctly and can handle the motor's current.

---

## How the System Works

Imagine a lift that moves up and down a track. The motor turns a wheel with slots, and the IR sensor counts these slots to know the lift's position. Two limit switches stop the lift at the top and bottom to prevent damage. The Arduino:

1. **Calibrates** by moving the motor down to the lower limit (position 0) and up to the upper limit (maximum position).
2. **Tracks Position** using the IR sensor, which increments or decrements a position counter based on motor direction.
3. **Moves to Target** positions entered by the user (e.g., position 5 or 15) by comparing the current position to the target.
4. **Controls Speed** differently for upward (250 PWM) and downward (160 PWM) motions, with micro-delays (15ms) to slow downward movement.
5. **Soft Starts** the motor by applying an initial pulse and ramping up speed to ensure reliable startup under load.
6. **Stops** if a limit switch is pressed and moves slightly in the opposite direction to clear it.
7. **Saves Position** to EEPROM so the system remembers its state after power-off.

---

## Code Overview

The code is structured into several functions, each with a specific job:

- **setup()**: Initializes pins, sets up the IR sensor interrupt, calibrates the motor, and loads saved positions.
- **loop()**: Checks for user input (target position) and monitors limit switches.
- **moveToPosition()**: Moves the motor to a user-specified position using direction-specific speeds.
- **handleLimitHit()**: Stops the motor and moves away from a limit switch if pressed.
- **setMotor()**: Sets the motor's direction (up/down/stop) and speed.
- **softStart()**: Gradually ramps up motor speed to ensure reliable startup.
- **readSensor()**: Updates the position counter when the IR sensor detects a slot.
- **calibratePosition()**: Finds the motor's range (min and max positions) and sets an initial position.

The code uses the **EEPROM library** to save and load positions and the **Arduino interrupt system** for accurate position tracking.

---

## Detailed Code Explanation

### Global Definitions

At the start, the code defines constants and variables:

- **Pin Definitions**:
  - `SENSOR_PIN (18)`: IR sensor for position tracking.
  - `UPPER_LIMIT_PIN (7)`, `LOWER_LIMIT_PIN (6)`: Limit switches.
  - `PWM (10)`: PWM pin for motor speed.
  - `IN1 (35)`, `IN2 (33)`: Motor driver direction pins.{Incase you need IN3-as-IN1 and IN4-as-IN2}
- **Position Variables**:
  - `pos`: Current position (volatile, updated in an interrupt).
  - `maxPos`, `minPos`: Maximum and minimum positions (set during calibration).
  - `INITIAL_POS (10)`: Default position after calibration.
- **Speed and Timing**:
  - `UPWARD_SPEED (250)`: Speed for upward motion (0-255 PWM).
  - `DOWNWARD_SPEED (160)`: Speed for downward motion.
  - `DOWNWARD_DELAY (15)`: Micro-delay (ms) to slow downward motion.
  - `START_PULSE_SPEED (200)`, `START_PULSE_DURATION (100)`: Initial pulse for soft start.

These constants make it easy to adjust settings without digging into the code.

### Setup Function

The `setup()` function runs once when the Arduino starts:

1. Initializes serial communication (`Serial.begin(9600)`) for debugging.
2. Sets pin modes:
   - IR sensor as input.
   - Limit switches as inputs with pull-up resistors.
   - PWM, IN1, and IN2 as outputs.
3. Initializes motor pins to OFF to prevent unwanted movement.
4. Attaches an interrupt to the IR sensor pin to call `readSensor()` on signal changes.
5. Runs `calibratePosition()` to find the motor's range.
6. Loads saved positions from EEPROM (`pos`, `maxPos`, `minPos`).

### Loop Function

The `loop()` function runs continuously:

1. Checks for user input via Serial Monitor (e.g., typing "15" to move to position 15).
2. Validates the input to ensure it’s within `minPos` and `maxPos`.
3. Calls `moveToPosition()` to move to the target position and saves the new position to EEPROM.
4. Monitors limit switches and calls `handleLimitHit()` if either is pressed.

### Move to Position

The `moveToPosition(targetPos)` function moves the motor to a specified position:

1. Compares the current `pos` to `targetPos`.
2. If `pos < targetPos`, moves upward using `softStart` (first move) or `setMotor` with `UPWARD_SPEED`.
3. If `pos > targetPos`, moves downward using `softStart` (first move) or `setMotor` with `DOWNWARD_SPEED`, adding `DOWNWARD_DELAY` to slow it down.
4. Stops if a limit switch is pressed or the target is reached.
5. Updates `pos` via the IR sensor interrupt and stops the motor when done.

### Handle Limit Switches

The `handleLimitHit()` function runs when a limit switch is pressed:

1. Stops the motor immediately.
2. If the upper limit is hit, moves downward briefly using `softStart` to clear the switch.
3. If the lower limit is hit, moves upward briefly using `softStart`.
4. Stops the motor after clearing the switch.

### Set Motor Direction and Speed

The `setMotor(dir, pwmVal, pwm, in1, in2)` function controls the motor:

- `dir = 1`: Upward (IN1 HIGH, IN2 LOW).
- `dir = -1`: Downward (IN1 LOW, IN2 HIGH).
- `dir = 0`: Stop (IN1 LOW, IN2 LOW, PWM = 0).
- Sets the speed using `analogWrite(pwm, pwmVal)` (0-255).

### Soft Start Mechanism

The `softStart(dir, targetSpeed)` function ensures the motor starts reliably:

1. Applies an initial pulse at `START_PULSE_SPEED` (200) for `START_PULSE_DURATION` (100ms) to overcome static friction.
2. Ramps up the speed from half the pulse speed (100) to `targetSpeed` in steps of 10, with 20ms delays for smooth acceleration.
3. Sets the final speed to `targetSpeed`.

This prevents stalling, especially for upward motion with a heavy load.

### Read Sensor for Position Tracking

The `readSensor()` function runs whenever the IR sensor detects a slot (interrupt):

1. Debounces the signal to avoid false triggers (20ms delay).
2. Checks the motor direction (via IN1 and IN2).
3. Increments `pos` for upward motion or decrements for downward motion, unless a limit switch is pressed.
4. Prints the updated position for debugging.

### Calibrate Position

The `calibratePosition()` function sets the motor’s range:

1. Moves downward using `softStart` and `DOWNWARD_SPEED` until the lower limit switch is pressed, setting `pos = 0` and `minPos = 0`.
2. Moves upward using `softStart` and `UPWARD_SPEED` until the upper limit switch is pressed, setting `maxPos` to the current `pos`.
3. Moves to `INITIAL_POS` (10) and saves all positions to EEPROM.
4. Uses `DOWNWARD_DELAY` during downward calibration to control speed.

---

## How to Use the Code

1. **Connect the Hardware**:

   - Wire the motor driver, IR sensor, and limit switches to the Arduino as per the pin table.
   - Ensure the power supply matches the motor and driver requirements.

2. **Upload the Code**:

   - Open the Arduino IDE, copy the code, and upload it to your Arduino board.
   - Ensure the EEPROM library is included (it’s built into the Arduino IDE).

3. **Open Serial Monitor**:

   - Set the Serial Monitor to 9600 baud.
   - Watch the calibration process (motor moves down, then up, then to position 10).
   - Calibration messages will appear, showing `minPos`, `maxPos`, and the initial position.

4. **Control the Motor**:

   - Type a number (e.g., 5, 15) in the Serial Monitor and press Enter to move the motor to that position.
   - The number must be between `minPos` (0) and `maxPos` (set during calibration, e.g., 26).
   - The motor will move to the target, and the Serial Monitor will show the current position.

5. **Monitor Limits**:
   - If the motor hits a limit switch, it stops and moves slightly in the opposite direction.
   - Check the Serial Monitor for limit hit messages.

---

## Troubleshooting Tips

- **Motor Doesn’t Start**:

  - Check power supply voltage and current capacity.
  - Verify motor driver connections (IN1, IN2, PWM),{Incase you need IN3-as-IN1 and IN4-as-IN2}.
  - Increase `START_PULSE_SPEED` (e.g., to 220) or `START_PULSE_DURATION` (e.g., to 150ms) in the code.
  - Ensure the load isn’t too heavy for the motor.

- **Downward Motion Too Fast/Slow**:

  - Adjust `DOWNWARD_DELAY` (e.g., increase to 20ms to slow down, decrease to 10ms to speed up).
  - Increase `DOWNWARD_SPEED` slightly (e.g., to 170) if the motor stalls.

- **Position Tracking Inaccurate**:

  - Ensure the IR sensor is aligned with the slotted disc and not obstructed.
  - Check for loose connections to the sensor pin (18).
  - Increase the debounce delay in `readSensor()` (e.g., to 30ms) if false triggers occur.

- **Limit Switches Not Working**:

  - Verify switch wiring (normally HIGH, LOW when pressed).
  - Test switches with a multimeter or by printing their state in the Serial Monitor.

- **Serial Monitor Issues**:
  - Ensure the baud rate is 9600.
  - Type only numbers in the Serial Monitor, followed by Enter.

---

## Conclusion

This Arduino program provides a robust way to control a DC motor for precise positioning, with features like soft start, position tracking, and limit switch protection. It’s beginner-friendly, with clear Serial Monitor feedback and adjustable parameters (`UPWARD_SPEED`, `DOWNWARD_SPEED`, `DOWNWARD_DELAY`). By understanding the code’s flow and experimenting with the hardware, you can adapt it for various projects, such as automated lifts, sliders, or robotic arms. If you encounter issues, use the troubleshooting tips or adjust the code’s constants to match your setup.

Happy building!
