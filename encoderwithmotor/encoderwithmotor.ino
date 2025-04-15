#define encoderOutA 6 // CLK
#define encoderOutB 7 // DT
int counter = 0;
int State;
int old_State;

void setup() {
  pinMode(encoderOutA, INPUT);
  pinMode(encoderOutB, INPUT);
  Serial.begin(9600);
  old_State = digitalRead(encoderOutA); // Read initial state of Channel A
}

void loop() {
  State = digitalRead(encoderOutA);
  if (State != old_State) {
    if (digitalRead(encoderOutB) != State) {
      counter++;
    } else {
      counter--;
    }
    Serial.print("Position: ");
    Serial.println(counter);
  }
  old_State = State;
}


------------------
#include <EEPROM.h>  // Library for EEPROM to store position

#define SENSOR_PIN 2  // IR Slot Sensor D0 connected to pin 2
#define PWM 10        // PWM pin for motor speed control
#define IN2 8         // Motor driver input 2
#define IN1 9         // Motor driver input 1

volatile int pos = 0;  // Current position, volatile because it's modified in an interrupt
const int DUMMY_MAX_POS = 100;  // Dummy maximum position (adjust as needed)
int maxPos = DUMMY_MAX_POS;  // Maximum position, initialized to dummy value

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud for debugging
  pinMode(SENSOR_PIN, INPUT);  // Set sensor pin as input
  pinMode(PWM, OUTPUT);        // Set PWM pin as output
  pinMode(IN1, OUTPUT);        // Set IN1 as output for motor direction
  pinMode(IN2, OUTPUT);        // Set IN2 as output for motor direction

  // Attach interrupt to sensor pin to track position changes
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), readSensor, CHANGE);

  // Load last saved position from EEPROM (address 0)
  pos = EEPROM.read(0);
  if (pos < 0 || pos > 255) pos = 0;  // Ensure valid range (EEPROM stores 0-255)
  Serial.print("Loaded position from EEPROM: ");
  Serial.println(pos);

  // Perform position calibration (without limit switch)
  calibratePosition();
}

void loop() {
  // Check for user input via Serial Monitor
  if (Serial.available() > 0) {
    int target = Serial.parseInt();  // Read target position from Serial input
    if (target >= 0 && target <= maxPos) {  // Validate target within calibrated range
      moveToPosition(target, 50);  // Move to user-defined position
      Serial.print("Moved to position: ");
      Serial.println(pos);
      EEPROM.write(0, pos);  // Save new position to EEPROM
    } else {
      Serial.println("Invalid position! Must be between 0 and maxPos.");
    }
    // Clear serial buffer
    while (Serial.available()) Serial.read();
  }
}

// Function to move motor to a specific position
void moveToPosition(int targetPos, int speed) {
  while (pos != targetPos) {  // Continue until target position is reached
    if (pos < targetPos) {
      setMotor(1, speed, PWM, IN1, IN2);  // Move forward
    } else if (pos > targetPos) {
      setMotor(-1, speed, PWM, IN1, IN2);  // Move backward
    }
    delay(10);  // Small delay for smooth movement
  }
  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor when target is reached
  Serial.print("Target reached, pos: ");
  Serial.println(pos);
}

// Function to set motor direction and speed
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  // Set motor speed (0-255)

  if (dir == 1) {  // Forward direction
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {  // Backward direction
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {  // Stop motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Interrupt handler to update position based on sensor
void readSensor() {
  static int lastState = HIGH;  // Track previous sensor state
  int currentState = digitalRead(SENSOR_PIN);
  if (currentState != lastState) {  // Detect edge change
    if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) {  // Moving forward
      pos++;
    } else if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) {  // Moving backward
      pos--;
    }
    lastState = currentState;  // Update last state
  }
}

// Function to calibrate zero and max positions (without limit switch)
void calibratePosition() {
  // Step 1: Move backward to find zero position
  Serial.println("Calibrating zero position...");
  while (pos > 0) {  // Move back until pos = 0
    setMotor(-1, 50, PWM, IN1, IN2);
    delay(10);
  }
  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor
  pos = 0;  // Reset position to zero
  EEPROM.write(0, pos);  // Save zero position to EEPROM
  Serial.println("Zero position calibrated");

  // Step 2: Set max position to dummy value (no limit switch)
  maxPos = DUMMY_MAX_POS;  // Use predefined dummy value
  Serial.print("Max position set to dummy value: ");
  Serial.println(maxPos);

  // Optional: Move to maxPos and back to zero to test range
  Serial.println("Testing range with dummy maxPos...");
  moveToPosition(maxPos, 50);  // Move to dummy max position
  delay(1000);  // Wait to observe
  moveToPosition(0, 50);  // Return to zero
  Serial.println("Calibration complete, returned to zero");
}