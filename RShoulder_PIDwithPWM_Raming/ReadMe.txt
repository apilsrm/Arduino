 PID + PWM Ramping 
 
✅ TRYING TO DO :
1)PID control with Kp, Ki, Kd

2)Smooth stop (deceleration + ramping): Deceleration zone to reduce speed as you approach the target

3)Position prediction with derivative.

4)Motor speed clamping (MIN_PWM to MAX_PWM)

5)EEPROM save/load

6)Limit switch handling


| Feature              |  Code                                               |
|----------------------|-----------------------------------------------------|
| Sensor               | IR slot sensor (single-channel counting)            |
| Interrupt            | IR sensor pin (direction inferred from motor state) |
| Control Loop         | PID logic only runs during moveToPosition()         |
| Target Signal        | Discrete position target from serial input          |
| PID Tuning           | Full PID with prediction and deceleration zone      |
| Motor Control        | Dynamic PWM with ramping and deceleration handling  |
| EEPROM Use           | Stores/loads current, min, and max positions        |
| Limit Switches       | Actively used for calibration and safety            |
| Calibration          | Full auto-calibration with EEPROM write             |
| Logging              | Detailed logging with timestamps and event counts   |