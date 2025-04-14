//Real-time velocity estimation and low-pass filtering
//Velocity-based adaptive PID tuning
//Full PID with prediction and deceleration zone
//Dynamic PWM with ramping and deceleration handling
//Stores/loads current, min, and max positions


#include <EEPROM.h>  // Library for EEPROM to store position

#define SENSOR_PIN 18   // IR Slot Sensor D0 connected to pin 18
#define UPPER_LIMIT_PIN 2  // Upper limit switch on pin 2
#define LOWER_LIMIT_PIN 3  // Lower limit switch on pin 3
#define PWM 43         // PWM pin for motor speed control
#define IN2 47         // Motor driver input 2
#define IN1 45         // Motor driver input 1

volatile int pos = 0;  // Current position
int maxPos = 0;        // Maximum position
int minPos = 0;        // Minimum position
const int INITIAL_POS = 10; // Initial position

// PID Variables
float Kp = 2.0;    // Proportional gain (tune this)
float Ki = 0.1;    // Integral gain (tune this)
float Kd = 0.2;    // Derivative gain (tune this)
float integral = 0; // Integral term
float lastError = 0; // Previous error for derivative calculation

// Motion control parameters
const int DECELERATION_ZONE = 100;
const int MAX_PWM = 200;
const int MIN_PWM = 130;
const int PWM_STEP_LIMIT = 15; // Used only for deceleration steps

// Position prediction parameters
const float PREDICTION_GAIN = 0.5; // Adjusts how much prediction affects speed

// Logging variables
const int LOG_INTERVAL = 50;       // Log every 50ms
unsigned long lastLogTime = 0;
int logCount = 0;

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

  Serial.print("Loaded position from EEPROM: ");
  Serial.println(pos);
  Serial.print("Loaded minPos: ");
  Serial.println(minPos);
  Serial.print("Loaded maxPos: ");
  Serial.println(maxPos);
}

void loop() {
  if (Serial.available() > 0) {
    if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
      Serial.print("From loooppppp");
      handleLimitHit();
    }
    int target = Serial.parseInt();  
    if (target >= minPos && target <= maxPos) {  
      moveToPosition(target);  // Move with PID control
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
  unsigned long startTime = millis();
  unsigned long lastTime = startTime;
  int prevPos = pos;
  float velocity = 0;
  float velocityFilt = 0;
  float alpha = 0.2; // Low-pass filter factor

  integral = 0;
  lastError = targetPos - pos;
  static int lastPWM = MIN_PWM;
  
  const unsigned long TIMEOUT = 8000; // Stop trying after 8 seconds


  while (abs(pos - targetPos) > 0) {
    if (millis() - startTime > TIMEOUT) {
      Serial.println("Timeout! Target not reached.");
      break;
    }

    if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
      handleLimitHit();
      break;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    if (dt == 0) dt = 0.01;
    lastTime = currentTime;

    // Velocity estimate with low-pass filter
    int dPos = pos - prevPos;
    velocity = dPos / dt;
    velocityFilt = (1 - alpha) * velocityFilt + alpha * velocity;
    prevPos = pos;

    float error = targetPos - pos;
    float remaining = abs(error);
    float derivative = (error - lastError) / dt;

    // ⬇ Adaptive PID (reduce Kp/Kd as we near the target)
    float adaptiveKp = map(remaining, 0, DECELERATION_ZONE, Kp / 2, Kp);
    float adaptiveKd = map(remaining, 0, DECELERATION_ZONE, Kd / 2, Kd);

    integral += error * dt;

    float predictedError = error + velocityFilt * PREDICTION_GAIN;
    float output = adaptiveKp * predictedError + Ki * integral + adaptiveKd * derivative;

    int currentMaxPWM = MAX_PWM;
    if (remaining < DECELERATION_ZONE) {
      currentMaxPWM = map(remaining, 0, DECELERATION_ZONE, MIN_PWM, MAX_PWM);
    }

    int desiredPWM = constrain(abs(output), MIN_PWM, currentMaxPWM);

    // Smooth deceleration
    if (desiredPWM < lastPWM) {
      desiredPWM = constrain(desiredPWM, lastPWM - PWM_STEP_LIMIT, lastPWM);
    }
    lastPWM = desiredPWM;

    int direction = (output >= 0) ? 1 : -1;
    setMotor(direction, desiredPWM, PWM, IN1, IN2);
    lastError = error;

    // Logging
    if (currentTime - lastLogTime >= LOG_INTERVAL) {
      Serial.print("Time: ");
      Serial.print(currentTime);
      Serial.print(" ms, Pos: ");
      Serial.print(pos);
      Serial.print(", Vel: ");
      Serial.print(velocityFilt);
      Serial.print(", PWM: ");
      Serial.println(desiredPWM);
      lastLogTime = currentTime;
    }

    delay(30); // Faster update = smoother motion
  }

  // Final braking
  setMotor(0, 0, PWM, IN1, IN2);
  Serial.print("Target reached at position: ");
  Serial.println(pos);
}


// Function to handle limit switch activation
void handleLimitHit() {
  setMotor(0, 0, PWM, IN1, IN2);  
  Serial.print("Limit hit at position: ");
  Serial.println(pos);

  if (digitalRead(UPPER_LIMIT_PIN) == LOW) {
    Serial.println("Upper limit hit, moving downward briefly...");
    setMotor(-1, 130, PWM, IN1, IN2);  
    delay(500);  
    setMotor(0, 0, PWM, IN1, IN2);
    Serial.println("Returning to initial position...");
    moveToPosition(INITIAL_POS);  
  } else if (digitalRead(LOWER_LIMIT_PIN) == LOW) {
    Serial.println("Lower limit hit, moving upward briefly...");
    setMotor(1, 130, PWM, IN1, IN2);  
    delay(500);  
    setMotor(0, 0, PWM, IN1, IN2);
    Serial.println("Returning to initial position...");
    moveToPosition(INITIAL_POS);  
  }
}

// Motor control function ()
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
  while (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
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
  while (digitalRead(UPPER_LIMIT_PIN) == HIGH) {
    delay(10);  
  }
  setMotor(0, 0, PWM, IN1, IN2);  
  maxPos = pos;  
  Serial.print("Upper limit hit, set as maxPos: ");
  Serial.println(maxPos);
  EEPROM.write(1, maxPos);  

  // Move slightly downward after hitting upper limit
  Serial.println("Moving away from upper limit...");
  setMotor(-1, 130, PWM, IN1, IN2);  
  delay(500);  
  setMotor(0, 0, PWM, IN1, IN2);

  Serial.println("Moving to initial position...");
  moveToPosition(INITIAL_POS);  
  EEPROM.write(0, pos);  
  Serial.print("Calibration complete, stopped at position: ");
  Serial.println(pos);
}