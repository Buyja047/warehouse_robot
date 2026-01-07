#include <Arduino.h>

/* =========================================================
   1. PIN DEFINITIONS
   ========================================================= */
static const int L_INA = 38;
static const int L_INB = 37;
static const int L_PWM = 36;
static const int ENC_L = 4;

static const int R_INA = 35;
static const int R_INB = 0; // Note: GPIO 0 is a strapping pin
static const int R_PWM = 45;
static const int ENC_R = 5;

static const int IR_L = 15;
static const int IR_M = 7;
static const int IR_R = 6;

/* =========================================================
   2. CONFIGURATION & TUNING
   ========================================================= */
const int PWM_FREQ = 20000;
const int PWM_RES  = 10; // 0-1023

// ADJUST THESE TO TUNE PERFORMANCE
float Kp = 0.1;  // Proportional: How hard it turns when error occurs
float Ki = 0.005;   // Integral: Fixes long-term drift
float Kd = 0.05;   // Derivative: Dampens oscillation/shaking

int BASE_SPEED = 350;   // Speed on straight line (0-1023)
int MAX_SPEED  = 650;   // Maximum speed limit

// Sensor Logic
const bool BLACK_LINE = false; // Set false if sensors output LOW on black

/* =========================================================
   3. GLOBAL VARIABLES
   ========================================================= */
int lastError = 0;
float integral = 0;

/* =========================================================
   4. MOTOR FUNCTIONS
   ========================================================= */
void setMotorSpeed(int left, int right) {
  // Constrain to prevent PWM overflow
  left = constrain(left, -MAX_SPEED, MAX_SPEED);
  right = constrain(right, -MAX_SPEED, MAX_SPEED);

  // Left Motor
  if (left >= 0) {
    digitalWrite(L_INA, HIGH);
    digitalWrite(L_INB, LOW);
    ledcWrite(L_PWM, left);
  } else {
    digitalWrite(L_INA, LOW);
    digitalWrite(L_INB, HIGH);
    ledcWrite(L_PWM, abs(left));
  }

  // Right Motor
  if (right >= 0) {
    digitalWrite(R_INA, HIGH);
    digitalWrite(R_INB, LOW);
    ledcWrite(R_PWM, right);
  } else {
    digitalWrite(R_INA, LOW);
    digitalWrite(R_INB, HIGH);
    ledcWrite(R_PWM, abs(right));
  }
}

/* =========================================================
   5. SETUP & LOOP
   ========================================================= */
void setup() {
  Serial.begin(115200);
  
  // Motor Output Setup
  pinMode(L_INA, OUTPUT); pinMode(L_INB, OUTPUT);
  pinMode(R_INA, OUTPUT); pinMode(R_INB, OUTPUT);
  
  ledcAttach(L_PWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_PWM, PWM_FREQ, PWM_RES);

  // Sensor Input Setup
  pinMode(IR_L, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_R, INPUT);

  Serial.println("Robot Initialized. Open Serial Plotter (115200) to see debug data.");
}

void loop() {
  // 1. Read Sensors
  bool sL = (digitalRead(IR_L) == (BLACK_LINE ? HIGH : LOW));
  bool sM = (digitalRead(IR_M) == (BLACK_LINE ? HIGH : LOW));
  bool sR = (digitalRead(IR_R) == (BLACK_LINE ? HIGH : LOW));

  // 2. Calculate Weighted Error
  // (L M R) -> Error Value
  int error = 0;
  if (!sL && sM && !sR)  error = 0;   // Centered
  else if (sL && sM)     error = -1;  // Slightly Left
  else if (sL && !sM)    error = -2;  // Hard Left
  else if (sM && sR)     error = 1;   // Slightly Right
  else if (!sM && sR)    error = 2;   // Hard Right
  else if (!sL && !sM && !sR) {       // Lost line
    error = (lastError > 0) ? 3 : -3; 
  }

  // 3. PID Math
  integral += error;
  integral = constrain(integral, -100, 100); // Prevent integral windup
  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // 4. Determine Driving Direction
  // Since sensors are 250mm behind wheels, we drive in the direction of the sensors.
  // We use negative BASE_SPEED to go "backward" toward the sensors.
  int leftMotorSpeed  = -BASE_SPEED + correction;
  int rightMotorSpeed = -BASE_SPEED - correction;

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

  // 5. PLOTTER DEBUGGING
  // Format: Name:Value,Name:Value
  Serial.print("Error:");      Serial.print(error);      Serial.print(",");
  Serial.print("Correction:"); Serial.print(correction); Serial.print(",");
  Serial.print("L_PWM:");      Serial.print(leftMotorSpeed); Serial.print(",");
  Serial.print("R_PWM:");      Serial.println(rightMotorSpeed);

  lastError = error;
  delay(10); // 100Hz Refresh rate
}