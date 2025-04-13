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
float Kp = 5.0;    // Proportional gain (tune this)
float Ki = 0.1;     // Integral gain (tune this)
float Kd = 1.0;     // Derivative gain (tune this)
float integral = 0; // Integral term
float lastError = 0;// Previous error for derivative calculation


//motion control parameters
const int DECELERATION_ZONE = 100;
// const int ACCELERATION_ZONE = 50;
const int MAX_PWM = 200;
const int MIN_PWM = 130;
const int PWM_STEP_LIMIT = 15; //changes to 15 units per cycle

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
  unsigned long lastTime = millis();
  Serial.print("Moving to position: ");
  Serial.println(targetPos);
  
  integral = 0; // Reset integral term at start
  lastError = 0; // Reset last error
  // lastError = targetPos - pos;

  static int lastPWM = 0; //for acceleration ramping
  lastPWM = 130;
  
  while (abs(pos - targetPos) > 1) { // Allow small tolerance
    if (digitalRead(UPPER_LIMIT_PIN) == LOW || digitalRead(LOWER_LIMIT_PIN) == LOW) {
    Serial.print("From movetoposition");

      handleLimitHit();
      break;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert ms to seconds
    if (dt == 0) dt= 0.05; //for safety 
    lastTime = currentTime;

    // Calculate PID terms
    float error = targetPos - pos;
    float remaining = abs(error);

    //dynamic speed limiting (reduces max speed as it nears the target)
    int currentMaxPWM = MAX_PWM;
    if(remaining < DECELERATION_ZONE) {
    currentMaxPWM = map(remaining, 0, DECELERATION_ZONE, MIN_PWM, MAX_PWM);
    }

    // acceleration ramping 
    integral += error * dt; // Multiply by time step
    float derivative = (error - lastError) / dt;

     //  Position Prediction
    float predictedError = error + derivative * PREDICTION_GAIN;
    float output = Kp * predictedError + Ki * integral + Kd * derivative;
    // float output = Kp * error + Ki * integral + Kd * derivative;

    //smooth speed transitions(soft start/anit-jerk)
    // int desiredPWM = constrain(abs(output), MIN_PWM, currentMaxPWM);
    int desiredPWM = constrain(abs(output), 130, currentMaxPWM);

    desiredPWM = constrain(desiredPWM, lastPWM - PWM_STEP_LIMIT, lastPWM + PWM_STEP_LIMIT);
     
    lastPWM = desiredPWM;


    // direction control
    int direction = (output >= 0) ? 1 : -1;

    // Set motor direction and speed
    setMotor(direction, desiredPWM, PWM, IN1, IN2);

    lastError = error; // Update last error
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(", Derivative: ");
    Serial.println(derivative);

    // Log position changes

    // Serial.print((direction == 1) ? "Moving upward, pos: " : "Moving downward, pos: ");
    // Serial.println(pos);

    // unsigned long currentTime = millis();

    if (currentTime - lastLogTime >= LOG_INTERVAL) {
      Serial.print("Time: ");
      Serial.print(currentTime);
      Serial.print(", Position: ");
      Serial.print(pos);
      Serial.print(", Speed: ");
      Serial.println(desiredPWM);
      lastLogTime = currentTime;
      logCount++;
    }

   
    delay(50);  // Small delay for smooth control
  }

  // Final Braking Sequence (Active deceleration instead of abrupt power cutoff)
  // for (int pwm = lastPWM; pwm > MIN_PWM; pwm -= 20) {
  //   setMotor((targetPos > pos) ? 1 : -1, pwm, PWM, IN1, IN2);
  //   delay(30);
  // }
int direction = (lastError >= 0) ? 1 : -1;
for (int pwm = lastPWM; pwm > MIN_PWM; pwm -= 20) {
  setMotor(direction, pwm, PWM, IN1, IN2);
  delay(30);
}


  setMotor(0, 0, PWM, IN1, IN2);  // Stop motor
  Serial.print("Target reached, pos: ");
  Serial.println(pos);

   //  Summary Log
  Serial.print("Total logs: ");
  Serial.println(logCount);

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

// Motor control function (unchanged)
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

// Sensor interrupt function (unchanged)
void readSensor() {
  static int lastState = HIGH;  
  static unsigned long lastInterruptTime = 0;
  const unsigned long debounceDelay = 20;

  int currentState = digitalRead(SENSOR_PIN);
  unsigned long currentTime = millis();

  if ((currentState != lastState) && (currentTime - lastInterruptTime >= debounceDelay)) {  
    // if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) {  
    //   if (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
    //     pos--;
    //   }  
    // } else if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) {  
    //   if (digitalRead(UPPER_LIMIT_PIN) == HIGH) {
    //     pos++;
    //   }  
    // }
    if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) pos--;
    else if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) pos++;

    lastInterruptTime = currentTime;  
  }
  lastState = currentState; 
}

// Calibration function (unchanged)
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

  // Serial.println("Moving to initial position...");
  // moveToPosition(INITIAL_POS);  
  // pos = INITIAL_POS;
  setMotor(-1, 130, PWM, IN1, IN2);  
  delay(500);  
  setMotor(0, 0, PWM, IN1, IN2);

  EEPROM.write(0, pos);  
  Serial.print("Calibration complete, stopped at  position: ");
  Serial.println(pos);
}