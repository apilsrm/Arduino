


#include <EEPROM.h>  // Library for EEPROM to store position

#define SENSOR_PIN 20   // IR Slot Sensor D0 connected to pin 18
#define UPPER_LIMIT_PIN 8  // Upper limit switch on pin 2
#define LOWER_LIMIT_PIN 9  // Lower limit switch on pin 3
#define PWM 11         // PWM pin for motor speed control
#define IN2 37         // Motor driver input 2
#define IN1 39         // Motor driver input 1

volatile int pos = 0;  // Current position, volatile because it's modified in an interrupt
int maxPos = 26;        // Maximum position (determined by calibration)
int minPos = 0;        // Minimum position (determined by calibration)
const int INITIAL_POS = 10; // Initial position
const int UPWARD_SPEED = 255;  // Target speed for upward motion
const int DOWNWARD_SPEED = 160;  // Target speed for downward motion (slightly increased)
const int DOWNWARD_DELAY = 15;  // Micro-delay in ms for downward motion (adjusted)
const int START_PULSE_SPEED = 200;  // Initial pulse speed to kickstart motor
const int START_PULSE_DURATION = 100;  // Duration of initial pulse in ms

void setup() {
  Serial.begin(9600);  
  pinMode(SENSOR_PIN, INPUT);       // Set IR sensor pin as input
  pinMode(UPPER_LIMIT_PIN, INPUT_PULLUP);  // Upper limit switch (HIGH by default, LOW when pressed)
  pinMode(LOWER_LIMIT_PIN, INPUT_PULLUP);  // Lower limit switch (HIGH by default, LOW when pressed)
  pinMode(PWM, OUTPUT);        // Set PWM pin as output
  pinMode(IN1, OUTPUT);        // Set IN1 as output for motor direction
  pinMode(IN2, OUTPUT);        // Set IN2 as output for motor direction

  // Initialize motor driver pins to OFF
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, 0);

  // Attach interrupt to IR sensor pin to track position changes
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), readSensor, CHANGE);

  // Calibration on every startup
  calibratePosition();

  // Load saved positions from EEPROM after calibration
  pos = EEPROM.read(0);     // Current position
  maxPos = EEPROM.read(1);  // Load maxPos from EEPROM (address 1)
  minPos = EEPROM.read(2);  // Load minPos from EEPROM (address 2)

  Serial.print("Loaded position from EEPROM: ");
  Serial.println(pos);
  Serial.print("Loaded minPos: ");
  Serial.println(minPos);
  Serial.print("Loaded maxPos: ");
  Serial.println(maxPos);
}

void loop() {
  // Check for user input via Serial Monitor
  if (Serial.available() > 0) {
    int target = Serial.parseInt();  // Read target position from Serial input
    if (target >= minPos && target <= maxPos) {  // Validate target within calibrated range
      moveToPosition(target);  // Move to user-defined position
      Serial.print("Moved to position: ");
      Serial.println(pos);
      EEPROM.write(0, pos);
    } else {
      Serial.print("Invalid position! Must be between ");
      Serial.print(minPos);
      Serial.print(" and ");
      Serial.println(maxPos);
    }
    // Clear serial buffer
    while (Serial.available()) Serial.read();
  }

  // Check limit switches during operation
  if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
    handleLimitHit();
  }
}

// Function to move motor to a specific position
void moveToPosition(int targetPos) {
  Serial.print("Moving to position: ");
  Serial.println(targetPos);
  int lastPos = -1;
  bool firstMove = true;  // Flag for initial movement
  while (abs(pos - targetPos) > 0) {
    if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
      handleLimitHit();
      break;
    }
    if (pos < targetPos) {
      // Upward motion
      if (firstMove) {
        softStart(1, UPWARD_SPEED);  // Apply soft start for upward motion
        firstMove = false;
      } else {
        setMotor(1, UPWARD_SPEED, PWM, IN1, IN2);  // Continue at target speed
      }
      Serial.print("Moving upward, pos: ");
      Serial.println(pos);
    } else if (pos > targetPos) {
      // Downward motion
      if (firstMove) {
        softStart(-1, DOWNWARD_SPEED);  // Apply soft start for downward motion
        firstMove = false;
      } else {
        setMotor(-1, DOWNWARD_SPEED, PWM, IN1, IN2);  // Continue at target speed
      }
      Serial.print("Moving downward, pos: ");
      Serial.println(pos);
      delay(DOWNWARD_DELAY);  // Add micro-delay to slow downward motion
    }
    if (pos != lastPos) {
      lastPos = pos;
    }
    delay(50);  // Small delay for smooth movement
  }
  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor when target is reached
  Serial.print("Target reached, pos: ");
  Serial.println(pos);
}

// Function to handle limit switch activation
void handleLimitHit() {
  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor immediately
  Serial.print("Limit hit at position: ");
  Serial.println(pos);

  // Move slightly in opposite direction to release the limit switch
  if (digitalRead(UPPER_LIMIT_PIN) == LOW) {
    Serial.println("Upper limit hit, moving downward briefly...");
    softStart(-1, DOWNWARD_SPEED);  // Soft start for downward motion
    delay(500);  // Move for 500ms to clear the switch
    setMotor(0, 0, PWM, IN1, IN2);
  } else if (digitalRead(LOWER_LIMIT_PIN) == LOW) {
    Serial.println("Lower limit hit, moving upward briefly...");
    softStart(1, UPWARD_SPEED);  // Soft start for upward motion
    delay(500);  // Move for 500ms to clear the switch
    setMotor(0, 0, PWM, IN1, IN2);
  }
}

// Function to set motor direction and speed
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  // Set motor speed (0-255)
  if (dir == -1) {  // Downward (IN1 LOW, IN2 HIGH)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (dir == 1) {  // Upward (IN1 HIGH, IN2 LOW)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {  // Stop motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);  // Ensure PWM is off when stopped
  }
}

// Function to implement soft start
void softStart(int dir, int targetSpeed) {
  // Apply initial pulse to kickstart motor
  setMotor(dir, START_PULSE_SPEED, PWM, IN1, IN2);
  delay(START_PULSE_DURATION);

  // Ramp up to target speed
  int startSpeed = START_PULSE_SPEED / 2;  // Start at half the pulse speed
  for (int speed = startSpeed; speed <= targetSpeed; speed += 10) {
    setMotor(dir, speed, PWM, IN1, IN2);
    delay(20);  // Small delay for smooth ramp-up
  }
  // Ensure final speed is set
  setMotor(dir, targetSpeed, PWM, IN1, IN2);
}

void readSensor() {
  static int lastState = HIGH;  // Track previous sensor state
  static unsigned long lastInterruptTime = 0;
  const unsigned long debounceDelay = 20;

  int currentState = digitalRead(SENSOR_PIN);
  unsigned long currentTime = millis();

  if ((currentState != lastState) && (currentTime - lastInterruptTime >= debounceDelay)) {  // Detect edge change
    if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) {  // Moving downward
      if (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
        pos--;
        Serial.print("Decrease: ");
        Serial.println(pos);
      }  
    } else if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) {  // Moving upward
      if (digitalRead(UPPER_LIMIT_PIN) == HIGH) {
        pos++;
        Serial.print("Increase: ");
        Serial.println(pos);
      }  
    }
    lastInterruptTime = currentTime;  // Update current time
  }
  lastState = currentState; // Update last state
}

// Function to calibrate min and max positions using physical limit switches
void calibratePosition() {
  // Step 1: Move downward to find lower limit
  Serial.println("Calibrating: Finding lower limit...");
  softStart(-1, DOWNWARD_SPEED);  // Soft start for downward motion
  while (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
    setMotor(-1, DOWNWARD_SPEED, PWM, IN1, IN2);
    delay(DOWNWARD_DELAY);  // Add micro-delay to slow downward motion
  }
  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor
  pos = 0;  // Set this as the zero point
  minPos = 0;
  Serial.print("Lower limit hit, set as minPos: ");
  Serial.println(minPos);
  EEPROM.write(2, minPos);  // Save minPos to EEPROM (address 2)
  delay(500);  // Short pause

  // Step 2: Move upward to find upper limit
  Serial.println("Calibrating: Finding upper limit...");
  softStart(1, UPWARD_SPEED);  // Soft start for upward motion
  while (digitalRead(UPPER_LIMIT_PIN) == HIGH) {
    setMotor(1, UPWARD_SPEED, PWM, IN1, IN2);
    delay(10);  // Wait until upper limit switch is triggered
  }
  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor
  maxPos = pos;  // Set this as the max position
  Serial.print("Upper limit hit, set as maxPos: ");
  Serial.println(maxPos);
  EEPROM.write(1, maxPos);  // Save maxPos to EEPROM (address 1)

  // Step 3: Move to initial position
  Serial.println("Moving to initial position...");
  moveToPosition(INITIAL_POS);  // Move to initial position
  pos = INITIAL_POS;
  EEPROM.write(0, pos);  // Save initial position to EEPROM (address 0)
  Serial.print("Calibration complete, stopped at initial position: ");
  Serial.println(INITIAL_POS);
}