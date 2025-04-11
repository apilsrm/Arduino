#include <EEPROM.h>  // Library for EEPROM to store position

#define SENSOR_PIN 18   // IR Slot Sensor D0 connected to pin 18
#define UPPER_LIMIT_PIN 2  // Upper limit switch on pin 2
#define LOWER_LIMIT_PIN 3  // Lower limit switch on pin 3
#define PWM 43         // PWM pin for motor speed control
#define IN2 47         // Motor driver input 2
#define IN1 45         // Motor driver input 1

volatile int pos = 0;  // Current position, volatile because it's modified in an interrupt
int maxPos = 0;        // Maximum position (determined by calibration)
int minPos = 0;        // Minimum position (determined by calibration)
const int INITIAL_POS = 6; // Initial position

void setup() {
  Serial.begin(9600);  
  pinMode(SENSOR_PIN, INPUT);       // Set IR sensor pin as input
  pinMode(UPPER_LIMIT_PIN, INPUT_PULLUP);  // Upper limit switch (HIGH by default, LOW when pressed)
  pinMode(LOWER_LIMIT_PIN, INPUT_PULLUP);  // Lower limit switch (HIGH by default, LOW when pressed)
  pinMode(PWM, OUTPUT);        // Set PWM pin as output
  pinMode(IN1, OUTPUT);        // Set IN1 as output for motor direction
  pinMode(IN2, OUTPUT);        // Set IN2 as output for motor direction

  // Attach interrupt to IR sensor pin to track position changes ()
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), readSensor, CHANGE);

  // calibration on every startup
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
      moveToPosition(target, 130);  // Move to user-defined position
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
void moveToPosition(int targetPos, int speed) {
  Serial.print("Moving to position: ");
  Serial.println(targetPos);
  int lastPos = -1;
    while (abs(pos - targetPos) > 0) {
    if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
      handleLimitHit();
      break;
    }
    if (pos < targetPos) {
      setMotor(1, speed, PWM, IN1, IN2);  // Move upward
      // Serial.print("Moving upward, pos: ");
      // Serial.println(pos);
    } else if (pos > targetPos) {
      setMotor(-1, speed, PWM, IN1, IN2);  // Move downward
      // Serial.print("Moving downward, pos: ");
      // Serial.println(pos);
    }
     if (pos != lastPos) {
      Serial.print((pos < targetPos) ? "Moving upward, pos: " : "Moving downward, pos: ");
      Serial.println(pos);
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

  // Move slightly in opposite direction to release the limit switch and then to initial position
  if (digitalRead(UPPER_LIMIT_PIN) == LOW) {
    Serial.println("Upper limit hit, moving downward briefly...");
    setMotor(-1, 130, PWM, IN1, IN2);  // Move downward
    delay(500);  // Move for 500ms to clear the switch
    setMotor(0, 0, PWM, IN1, IN2);
    Serial.println("Returning to initial position...");
    moveToPosition(INITIAL_POS, 130);  // Move to initial position
  } else if (digitalRead(LOWER_LIMIT_PIN) == LOW) {
    Serial.println("Lower limit hit, moving upward briefly...");
    setMotor(1, 130, PWM, IN1, IN2);  // Move upward
    delay(500);  // Move for 500ms to clear the switch
    setMotor(0, 0, PWM, IN1, IN2);
    Serial.println("Returning to initial position...");
    moveToPosition(INITIAL_POS, 130);  // Move to initial position
  }
}

// Function to set motor direction and speed
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  // Set motor speed (0-255)
  if (dir == -1) {  //   clock Down (IN1 LOW, IN2 HIGH)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (dir == 1) {  //  upward (IN1 HIGH, IN2 LOW)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {  // Stop motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readSensor() {
  static int lastState = HIGH;  // Track previous sensor state
  static unsigned long lastInterruptTime = 0;
  const unsigned long debounceDelay = 20;

  int currentState = digitalRead(SENSOR_PIN);
  unsigned long currentTime = millis();

  if ((currentState != lastState) && (currentTime - lastInterruptTime >= debounceDelay)) {  // Detect edge change
    if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) {  // Moving  downward
      if (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
        pos--;
      //  Serial.print("Decrease: ");
      //  Serial.println(pos);
        }  
    } else if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) {  // Moving upward
      if (digitalRead(UPPER_LIMIT_PIN) == HIGH)
      {
        pos++;
      //  Serial.print("Increase: ");
      //  Serial.println(pos);
      }  
    }
    

    lastInterruptTime = currentTime;  // Update current time
  }
  lastState = currentState; //update last state after
}

// Function to calibrate min and max positions using physical limit switches
void calibratePosition() {
  // Step 1: Move downward to find lower limit
  Serial.println("Calibrating: Finding lower limit...");
  setMotor(-1, 130, PWM, IN1, IN2);  // Move downward (IN1 LOW, IN2 HIGH)
  while (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
    delay(10);  // Wait until lower limit switch is triggered
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
  pos = 0;  // Reset position counter
  setMotor(1, 130, PWM, IN1, IN2);  // Move upward (IN1 HIGH, IN2 HIGH)
  while (digitalRead(UPPER_LIMIT_PIN) == HIGH) {
    delay(10);  // Wait until upper limit switch is triggered
  }
  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor
  maxPos = pos;  // Set this as the max position
  Serial.print("Upper limit hit, set as maxPos: ");
  Serial.println(maxPos);
  EEPROM.write(1, maxPos);  // Save maxPos to EEPROM (address 1)

  // // Step 3: Move to initial position
  Serial.println("Moving to initial position...");
  moveToPosition(INITIAL_POS, 130);  // Move to initial position (15)
  pos = INITIAL_POS;
  EEPROM.write(0, pos);  // Save initial position to EEPROM (address 0)
  Serial.print("Calibration complete, stopped at initial position: ");
  Serial.println(INITIAL_POS);
}