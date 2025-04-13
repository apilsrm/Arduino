#include <EEPROM.h>

#define SENSOR_PIN 18 // IR Slot Sensor D0 connected to pin 18
#define UPPER_LIMIT_PIN 2
#define LOWER_LIMIT_PIN 3
#define PWM 43
#define IN2 47
#define IN1 45

volatile int pos = 0;
int maxPos = 0;
int minPos = 0;
const int INITIAL_POS = 10;

// Basic PID Variables
float Kp = 5.0;
float Ki = 0.1;
float Kd = 1.0;
float integral = 0;
float lastError = 0; // Previous error for derivative calculation

const int MAX_PWM = 200;
const int MIN_PWM = 130;

void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(UPPER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LOWER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), readSensor, CHANGE);
  calibratePosition();

  pos = EEPROM.read(0);
  maxPos = EEPROM.read(1);
  minPos = EEPROM.read(2);

  Serial.print("Loaded position: ");
  Serial.println(pos);
}

void loop() {
  if (Serial.available() > 0) {
    if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
      handleLimitHit();
    }

    int target = Serial.parseInt();
    if (target >= minPos && target <= maxPos) {

      moveToPosition(target); // Move with PID control
      Serial.print("Moved to position: ");
      Serial.println(pos);
      EEPROM.write(0, pos);
    } else {
      Serial.print("Invalid position! Must be between ");
      Serial.print(minPos);
      Serial.print(" and ");
      Serial.println(maxPos);
    }

    while (Serial.available()) Serial.read();
  }
}

// PID-controlled move function
void moveToPosition(int targetPos) {
  unsigned long lastTime = millis();

  // Reset integral term at start
  integral = 0;
  lastError = 0;
  
     // Allow small tolerance
  while (abs(pos - targetPos) > 1) { 
    if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
      handleLimitHit();
      break;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert ms to seconds
    if (dt == 0) dt = 0.05;   // Safety fallback
    lastTime = currentTime;

     // Calculate PID terms
    float error = targetPos - pos;
    integral += error * dt;
    float derivative = (error - lastError) / dt;

    float output = Kp * error + Ki * integral + Kd * derivative;
    int pwm = constrain(abs(output), MIN_PWM, MAX_PWM);
    int direction = (output >= 0) ? 1 : -1;

    setMotor(direction, pwm, PWM, IN1, IN2);
    lastError = error;

    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(", PWM: ");
    Serial.println(pwm);
 
    delay(50); // Small delay for smooth control
  }

  setMotor(0, 0, PWM, IN1, IN2);
  Serial.print("Target reached at pos: ");
  Serial.println(pos);
}

// Function to handle limit switch activation
void handleLimitHit() {
  setMotor(0, 0, PWM, IN1, IN2);
  Serial.print("Limit hit at position: ");
  Serial.println(pos);

  if (digitalRead(UPPER_LIMIT_PIN) == LOW) {
    Serial.println("Upper limit hit. Moving down briefly...");
    setMotor(-1, 130, PWM, IN1, IN2);
    delay(500);
    setMotor(0, 0, PWM, IN1, IN2);

    Serial.println("Returning to initial position...");
    moveToPosition(INITIAL_POS);

  } else if (digitalRead(LOWER_LIMIT_PIN) == LOW) {
    Serial.println("Lower limit hit. Moving up briefly...");
    setMotor(1, 130, PWM, IN1, IN2);
    delay(500);
    setMotor(0, 0, PWM, IN1, IN2);

    Serial.println("Returning to initial position...");
    moveToPosition(INITIAL_POS);
  }
}

// Motor control function()
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Sensor interrupt function()
void readSensor() {
  static int lastState = HIGH;
  static unsigned long lastInterruptTime = 0;
  const unsigned long debounceDelay = 20;

  int currentState = digitalRead(SENSOR_PIN);
  unsigned long currentTime = millis();

  if ((currentState != lastState) && (currentTime - lastInterruptTime >= debounceDelay)) {
    if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) pos--;
    else if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) pos++;

    lastInterruptTime = currentTime;
  }
  lastState = currentState;
}


// Calibration function
void calibratePosition() {
  Serial.println("Calibrating: Finding lower limit...");
  setMotor(-1, 130, PWM, IN1, IN2);
  while (digitalRead(LOWER_LIMIT_PIN) == HIGH){
  delay(10);
  }
  setMotor(0, 0, PWM, IN1, IN2);
  pos = 0;  
  minPos = 0;
  Serial.print("Lower limit hit, set as minPos: ");
  Serial.println(minPos);
  EEPROM.write(2, minPos);
  delay(500);

 Serial.println("Calibrating: Finding upper limit...");
  pos = 0;
  setMotor(1, 130, PWM, IN1, IN2);
  while (digitalRead(UPPER_LIMIT_PIN) == HIGH){
    delay(10);
  }
  setMotor(0, 0, PWM, IN1, IN2);
  maxPos = pos;
  Serial.print("Upper limit hit, set as maxPos: ");
  Serial.println(maxPos);
  EEPROM.write(1, maxPos);

  // // Move slightly downward after hitting upper limit
  // setMotor(-1, 130, PWM, IN1, IN2);
  // delay(500);
  // setMotor(0, 0, PWM, IN1, IN2);

 Serial.println("Moving to initial position...");
moveToPosition(INITIAL_POS);  
  EEPROM.write(0, pos);  
  Serial.print("Calibration complete, stopped at position: ");
  Serial.println(pos);
}
