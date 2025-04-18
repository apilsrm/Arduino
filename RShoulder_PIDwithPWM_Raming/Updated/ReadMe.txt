Updated Features 
- Real-Time Velocity Estimation
- Low-Pass Filtering of velocity (alpha smoothing)
- Adaptive PID Tuning (based on distance to target)
- Dynamic PWM Adjustment (based on error and velocity)
- Timeout Handling 



✅ Features & Algorithms Used
- Full PID Control (Proportional, Integral, Derivative)

- Velocity-Based Prediction (future position estimation)

- Dynamic PWM Adjustment (based on error and velocity)

- Smooth Ramping & Deceleration in motion

- Deceleration Zone Handling (reduces speed near target)

- Real-Time Velocity Estimation (dPos / dt)

- Low-Pass Filtering of velocity (alpha smoothing)

- Adaptive PID Tuning (based on distance to target)

- EEPROM Storage for:

     Current position ,Min position,Max position

- IR Sensor Interrupt for encoder/position feedback

- Debounce Logic for sensor reading

- Limit Switch Safety for upper/lower bounds

- Auto Calibration Routine on startup

- Serial Command Interface to input target position

- Timeout Handling (stop if target not reached)

- PWM Smoothing using PWM_STEP_LIMIT (limits sudden drops)
