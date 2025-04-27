# PID Motor Control Code Documentation

This document explains an Arduino program that controls a DC motor to move to specific positions with high precision using a PID (Proportional-Integral-Derivative) controller. The code tracks the motor's position with an IR slot sensor and uses limit switches to set safe boundaries. It’s designed for beginners, with clear explanations of the hardware, code flow, and how to use the program. By the end, you’ll understand how the motor moves, how PID control works, and how to set up and run the code.

---

## Table of Contents

1.  Purpose of the Code
2.  Hardware Requirements
3.  How the System Works
4.  What is PID Control?
5.  Code Overview
6.  Detailed Code Explanation
    - Global Definitions
    - Setup Function
    - Loop Function
    - Move to Position (PID Control)
    - Handle Limit Switches
    - Set Motor Direction and Speed
    - Read Sensor for Position Tracking
    - Calibrate Position
7.  How to Use the Code
8.  Tuning PID Parameters
9.  Troubleshooting Tips
10. Conclusion

---

## Purpose of the Code

This Arduino program controls a DC motor to move a mechanism (e.g., a lift, slider, or robotic arm) to exact positions between two physical limits (upper and lower). The key features are:

- Uses a **PID controller** to move the motor smoothly and accurately to a target position.
- Tracks the motor’s position with an **IR slot sensor** that counts slots on a rotating disc.
- Prevents over-travel with **limit switches** at the top and bottom.
- **Calibrates** the motor’s range automatically on startup.
- Allows users to input target positions via the **Serial Monitor**.
- Saves the current position to **EEPROM** (Arduino’s memory) to remember it after power-off.
- Logs position, velocity, and motor speed for debugging.
- Applies smooth speed control to avoid jerky movements, especially near the target.

This code is ideal for projects requiring precise positioning, such as automated doors, conveyor belts, or camera sliders.

---

## Hardware Requirements

To use this code, you need:

1. **Arduino Board** (e.g., Arduino Mega, as it uses specific pins like 21 and 53).
2. **DC Motor** with a **motor driver** (e.g., L298N or TB6612FNG) to control speed and direction.
3. **IR Slot Sensor** (optical encoder) to count slots on a disc attached to the motor.
4. **Two Limit Switches** for upper and lower boundaries.
5. **Power Supply** for the motor (e.g., 12V) and Arduino (e.g., 5V via USB or 7-12V).
6. **Wires and Breadboard** for connections.

### Pin Connections

| Component           | Arduino Pin | Description                                      |
| ------------------- | ----------- | ------------------------------------------------ |
| IR Slot Sensor (D0) | 21          | Detects slots for position tracking (interrupt). |
| Upper Limit Switch  | 4           | Stops motor at the top (LOW when pressed).       |
| Lower Limit Switch  | 5           | Stops motor at the bottom (LOW when pressed).    |
| PWM (Motor Speed)   | 53          | Controls motor speed via PWM signal.             |
| IN1 (Motor Driver)  | 49          | Motor driver input for direction control.        |
| IN2 (Motor Driver)  | 51          | Motor driver input for direction control.        |

### Notes

- The IR sensor must connect to an interrupt-capable pin (e.g., 21 on Arduino Mega).
- Limit switches use internal pull-up resistors, so they read HIGH normally and LOW when pressed.
- Ensure the motor driver’s power supply matches the motor’s requirements.

---

## How the System Works

Think of a lift moving up and down a track. The motor turns a wheel with slots, and the IR sensor counts these slots to track the lift’s position. Limit switches stop the lift at the top and bottom to avoid damage. The Arduino:

1. **Calibrates** by moving down to the lower limit (position 0) and up to the upper limit (maximum position).
2. **Tracks Position** using the IR sensor, incrementing or decrementing a counter based on motor direction.
3. **Uses PID Control** to calculate the motor speed needed to reach a target position smoothly.
4. **Adjusts Speed** dynamically, slowing down near the target to avoid overshooting.
5. **Logs Data** (position, velocity, speed) to the Serial Monitor for debugging.
6. **Stops** if a limit switch is pressed and moves to the initial position (10).
7. **Saves Position** to EEPROM to retain it after power-off.

---

## What is PID Control?

PID control is a method to make the motor move precisely to a target position. It calculates the motor speed based on three factors:

- **Proportional (P)**: How far the motor is from the target (bigger distance = faster speed).
- **Integral (I)**: Accounts for small errors over time to ensure the motor reaches the exact position.
- **Derivative (D)**: Predicts future errors based on how fast the position is changing to avoid overshooting.

The code adjusts these factors dynamically to slow the motor as it nears the target, making movements smooth and accurate.

---

## Code Overview

The code is organized into functions, each with a specific role:

- **setup ()**: Initializes pins, sets up the IR sensor interrupt, calibrates the motor, and loads saved positions.
- **loop ()**: Checks for user input (target position) and monitors limit switches.
- **moveToPosition ()**: Uses PID control to move the motor to a target position.
- **handleLimitHit ()**: Stops the motor and moves to the initial position if a limit switch is pressed.
- **setMotor ()**: Sets the motor’s direction and speed.
- **readSensor ()**: Updates the position counter via an interrupt.
- **CalibratePosition ()**: Finds the motor’s range and sets the initial position.

The code uses the **EEPROM library** to store positions and an **interrupt** for accurate position tracking.

---

## Detailed Code Explanation

### Global Definitions

The code starts with constants and variables:

- **Pin Definitions**:
  - `SENSOR_PIN (21)`: IR sensor for position tracking.
  - `UPPER_LIMIT_PIN (4)`, `LOWER_LIMIT_PIN (5)`: Limit switches.
  - `PWM (53)`, `IN1 (49)`, `IN2 (51)`: Motor driver pins.
- **Position Variables**:
  - `pos`: Current position (updated in an interrupt).
  - `maxPos`, `minPos`: Maximum and minimum positions (set during calibration).
  - `INITIAL_POS (10)`: Default position after calibration or limit hit.
- **PID Parameters**:
  - `Kp (2.0)`, `Ki (0.1)`, `Kd (0.2)`: PID gains for tuning control.
  - `integral`, `lastError`: Track PID calculations.
- **Motion Control**:
  - `DECELERATION_ZONE (100)`: Distance near the target where speed reduces.
  - `MAX_PWM (200)`, `MIN_PWM (150)`: Speed limits (0-255).
  - `PWM_STEP_LIMIT (15)`: Limits speed change for smooth deceleration.
- **Prediction and Logging**:
  - `PREDICTION_GAIN (0.5)`: Adjusts position prediction for smoother control.
  - `LOG_INTERVAL (50)`: Logs data every 50ms.

These settings make the code customizable and easy to tune.

### Setup Function

The `setup()` function runs once at startup:

1. Starts serial communication (`Serial.begin(9600)`) for debugging.
2. Sets pin modes:
   - IR sensor as input.
   - Limit switches as inputs with pull-up resistors.
   - PWM, IN1, and IN2 as outputs.
3. Attaches an interrupt to the IR sensor pin to call `readSensor()` on signal changes.
4. Runs `calibratePosition()` to find the motor’s range.
5. Loads saved positions (`pos`, `maxPos`, `minPos`) from EEPROM.

### Loop Function

The `loop()` function runs continuously:

1. Checks for user input via Serial Monitor (e.g., typing “15” to move to position 15).
2. If a limit switch is pressed, calls `handleLimitHit()`.
3. Validates the input to ensure it’s between `minPos` and `maxPos`.
4. Calls `moveToPosition()` to move to the target and saves the position to EEPROM.
5. Clears the serial buffer to avoid duplicate inputs.

### Move to Position (PID Control)

The `moveToPosition(targetPos)` function uses PID to move the motor:

1.  Initializes timing, velocity tracking, and PID variables.
2.  Runs a loop until the motor reaches the target or a timeout (7 seconds) occurs.
3.  Checks for limit switches and stops if pressed.
4.  Calculates:
    - **Error**: Difference between target and current position.
    - **Velocity**: Rate of position change, filtered to reduce noise.
    - **PID Output**: Combines proportional, integral, and derivative terms, with a prediction factor.
5.  Adjusts PID gains (`Kp`, `Kd`) near the target for smoother stopping.
6.  Sets motor speed (`desiredPWM`) between `MIN_PWM` and `MAX_PWM`, slowing down in the `DECELERATION_ZONE`.
7.  Limits speed changes (`PWM_STEP_LIMIT`) for smooth deceleration.
8.  Sets motor direction and speed using `setMotor()`.
9.  Logs position, velocity, and PWM every 50ms.
10. Stops the motor when the target is reached.

### Handle Limit Switches

The `_handleLimitHit()` function runs when a limit switch is pressed:

1. Stops the motor immediately.
2. If the upper limit is hit, moves downward briefly (150 PWM, 500ms) and then to `INITIAL_POS`.
3. If the lower limit is hit, moves upward briefly and then to `INITIAL_POS`.
4. Prints messages to the Serial Monitor for debugging.

### Set Motor Direction and Speed

The `setMotor(dir, pwmVal, pwm, in1, in2)` function controls the motor:

- `dir = 1`: Forward (IN1 LOW, IN2 HIGH).
- `dir = -1`: Reverse (IN1 HIGH, IN2 LOW).
- `dir = 0`: Stop (IN1 LOW, IN2 LOW).
- Sets speed using `analogWrite(pwm, pwmVal)` (0-255).

### Read Sensor for Position Tracking

The `readSensor()` function runs on IR sensor interrupts:

1. Debounces the signal (40ms delay) to avoid false triggers.
2. Checks motor direction (via IN1 and IN2) and also(IN3 as IN1 And IN4 as IN2).
3. Increments `pos` for forward motion or decrements for reverse.
4. Updates the last state and time for debouncing.

### Calibrate Position

The `calibratePosition()` function sets the motor’s range:

1. Moves downward (150 PWM) until the lower limit switch is pressed, setting `pos = 0` and `minPos = 0`.
2. Moves upward (150 PWM) until the upper limit switch is pressed, setting `maxPos` to the current `pos`.
3. Moves downward briefly (500ms) to clear the upper limit.
4. Saves `minPos`, `maxPos`, and `pos` to EEPROM.
5. Prints calibration details to the Serial Monitor.

---

## How to Use the Code

1. **Connect the Hardware**:

   - Wire the motor driver, IR sensor, and limit switches as per the pin table.
   - Ensure the power supply matches the motor and driver.

2. **Upload the Code**:

   - Open the Arduino IDE, paste the code, and upload it to your Arduino (e.g., Mega).
   - The EEPROM library is included by default in the IDE.

3. **Open Serial Monitor**:

   - Set the Serial Monitor to 9600 baud.
   - Watch the calibration process (motor moves down, up, then stops).
   - Calibration messages show `minPos`, `maxPos`, and the final position.

4. **Control the Motor**:

   - Type a number (e.g., 5, 20) in the Serial Monitor and press Enter to move to that position.
   - The number must be between `minPos` (0) and `maxPos` (set during calibration).
   - The motor moves smoothly, and the Serial Monitor logs position, velocity, and PWM.

5. **Monitor Limits**:

   - If a limit switch is pressed, the motor stops, moves briefly in the opposite direction, and goes to `INITIAL_POS` (10).
   - Check the Serial Monitor for limit hit messages.

---

## Tuning PID Parameters

The PID gains (`Kp`, `Ki`, `Kd`) control how the motor moves:

- **Kp (2.0)**: Increase for faster response, but too high causes overshooting.
- **Ki (0.1)**: Increase to eliminate steady-state error, but too high causes oscillations.
- **Kd (0.2)**: Increase to reduce overshooting, but too high causes jitter.

### Tuning Tips

1. Start with `Ki = 0` and `Kd = 0`, adjust `Kp` until the motor moves quickly but slightly overshoots.
2. Increase `Kd` to reduce overshooting.
3. Add a small `Ki` to eliminate any remaining error.
4. Test with different target positions and adjust gains if the motor oscillates or stops too early.
5. Modify `PREDICTION_GAIN` (0.5) to adjust how much velocity affects control (lower for less prediction).

---

## Troubleshooting Tips

- **Motor Doesn’t Move**:

  - Check power supply voltage and current.
  - Verify motor driver connections (IN1, IN2, PWM).
  - Increase `MIN_PWM` (e.g., to 160) if the motor needs more power to start.
  - Ensure the load isn’t too heavy.

- **Motor Overshoots or Oscillates**:

  - Reduce `Kp` or increase `Kd` in the PID parameters.
  - Lower `PREDICTION_GAIN` (e.g., to 0.3).
  - Increase `DECELERATION_ZONE` (e.g., to 150) for earlier slowdown.

- **Position Tracking Inaccurate**:

  - Ensure the IR sensor is aligned with the slotted disc.
  - Check the sensor’s connection to pin 21.
  - Increase `debounceDelay` (e.g., to 50ms) in `readSensor()` if false triggers occur.

- **Limit Switches Not Working**:

  - Verify switch wiring (HIGH normally, LOW when pressed).
  - Test switches by printing their state in the Serial Monitor.

- **Serial Monitor Issues**:

  - Ensure the baud rate is 9600.
  - Type only numbers, followed by Enter.

---

## Conclusion

This Arduino program uses PID control to move a DC motor to precise positions, with features like position tracking, limit switch protection, and data logging. It’s beginner-friendly, with detailed Serial Monitor output and tunable parameters (`Kp`, `Ki`, `Kd`, `MAX_PWM`, etc.). By understanding the code and experimenting with the hardware, you can adapt it for projects like automated sliders, lifts, or robotic systems. Use the tuning and troubleshooting tips to optimize performance for your setup.

Happy building!
