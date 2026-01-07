// Line Following Robot with ESP32 S3
// Robot specifications:
// - Wheel distance: 450mm
// - Sensor offset: 250mm behind wheels
// - IR sensor spacing: 50mm
// - Line width: 90mm
// - Wheel diameter: 0.137m
// - Encoder: 460 PPM

#include <Arduino.h>

// Pin Definitions
static const int LIFT_PWM = 39;

// Left Wheel
static const int L_INA = 38;
static const int L_INB = 37;
static const int L_PWM = 36;
static const int ENC_L = 4;

// Right Wheel
static const int R_INA = 35;
static const int R_INB = 0;
static const int R_PWM = 45;
static const int ENC_R = 5;

// IR Sensors
static const int IR_L = 15;
static const int IR_M = 7;
static const int IR_R = 6;

// Constants based on your robot
const float WHEEL_DIAMETER = 0.137;      // meters
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float TRACK_WIDTH = 0.450;         // meters (450mm)
const float SENSOR_OFFSET = 0.250;       // meters (250mm behind wheels)
const float SENSOR_SPACING = 0.050;      // meters (50mm)
const int ENCODER_PPR = 460;             // Pulses Per Revolution

// PID Controller Parameters
float Kp = 30.0;    // Proportional gain (start with this value)
float Ki = 0.05;    // Integral gain
float Kd = 20.0;    // Derivative gain

// Speed parameters
int baseSpeed = 150;     // Base PWM speed (0-255)
int maxSpeed = 200;      // Maximum PWM speed
int minSpeed = 100;      // Minimum PWM speed

// Variables for PID control
float error = 0;
float lastError = 0;
float P = 0;
float I = 0;
float D = 0;
float PIDvalue = 0;

// Variables for encoders
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// IR sensor states
int leftSensor = 0;
int middleSensor = 0;
int rightSensor = 0;

// Timer for control loop
unsigned long lastTime = 0;
const int SAMPLE_TIME = 10;  // 10ms sample time

// PWM channels
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int LEFT_PWM_CHANNEL = 0;
const int RIGHT_PWM_CHANNEL = 1;

// Motor control functions
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Left motor control
  if (leftSpeed >= 0) {
    digitalWrite(L_INA, HIGH);
    digitalWrite(L_INB, LOW);
    analogWrite(L_PWM, leftSpeed);
  } else {
    digitalWrite(L_INA, LOW);
    digitalWrite(L_INB, HIGH);
    analogWrite(L_PWM, -leftSpeed);
  }
  
  // Right motor control
  if (rightSpeed >= 0) {
    digitalWrite(R_INA, HIGH);
    digitalWrite(R_INB, LOW);
    analogWrite(R_PWM, rightSpeed);
  } else {
    digitalWrite(R_INA, LOW);
    digitalWrite(R_INB, HIGH);
    analogWrite(R_PWM, -rightSpeed);
  }
}

// Encoder interrupt handlers
void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount++;
}

// Read IR sensors
void readIRSensors() {
  leftSensor = digitalRead(IR_L);
  middleSensor = digitalRead(IR_M);
  rightSensor = digitalRead(IR_R);
}

// Calculate error based on sensor readings
// With 9cm line and 5cm sensor spacing, multiple sensors can see the line
int calculateError() {
  // For common IR sensors: 1 = white/not on line, 0 = black/on line
  // If your sensors work opposite, invert the readings by changing == 0 to == 1
  
  // Case 1: All on white (line lost)
  if (leftSensor == 1 && middleSensor == 1 && rightSensor == 1) {
    return 10; // Special code for line lost
  }
  
  // Case 2: Perfectly centered (all on line)
  if (leftSensor == 0 && middleSensor == 0 && rightSensor == 0) {
    return 0;
  }
  
  // Case 3: Only middle on line
  if (leftSensor == 1 && middleSensor == 0 && rightSensor == 1) {
    return 0;
  }
  
  // Case 4: Left sensor on line
  if (leftSensor == 0 && middleSensor == 1 && rightSensor == 1) {
    return -2; // Turn left
  }
  
  // Case 5: Right sensor on line
  if (leftSensor == 1 && middleSensor == 1 && rightSensor == 0) {
    return 2; // Turn right
  }
  
  // Case 6: Left and middle on line
  if (leftSensor == 0 && middleSensor == 0 && rightSensor == 1) {
    return -1; // Slight left
  }
  
  // Case 7: Middle and right on line
  if (leftSensor == 1 && middleSensor == 0 && rightSensor == 0) {
    return 1; // Slight right
  }
  
  // Case 8: Left and right on line (45 degree)
  if (leftSensor == 0 && middleSensor == 1 && rightSensor == 0) {
    return 0; // Continue straight
  }
  
  return 0;
}

// Calculate PID value
float calculatePID(int error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  
  if (deltaTime <= 0) deltaTime = 0.01; // Avoid division by zero
  
  P = error;
  I = I + (error * deltaTime);
  D = (error - lastError) / deltaTime;
  
  // Store for next iteration
  lastError = error;
  lastTime = currentTime;
  
  // Anti-windup for integral term
  if (I > 300) I = 300;
  if (I < -300) I = -300;
  
  return (Kp * P) + (Ki * I) + (Kd * D);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Robot...");
  
  // Configure motor control pins
  pinMode(L_INA, OUTPUT);
  pinMode(L_INB, OUTPUT);
  pinMode(R_INA, OUTPUT);
  pinMode(R_INB, OUTPUT);
  
  // Configure PWM pins
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  
  // Initialize PWM (using analogWrite which should work on ESP32-S3)
  // Note: analogWrite resolution is 8-bit (0-255) by default
  
  // Configure IR sensor pins
  pinMode(IR_L, INPUT_PULLUP);  // Using pullup for better noise immunity
  pinMode(IR_M, INPUT_PULLUP);
  pinMode(IR_R, INPUT_PULLUP);
  
  // Configure encoder pins with interrupts
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightEncoderISR, RISING);
  
  // Initialize motor drivers (all off)
  digitalWrite(L_INA, LOW);
  digitalWrite(L_INB, LOW);
  digitalWrite(R_INA, LOW);
  digitalWrite(R_INB, LOW);
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
  
  // Test sequence
  Serial.println("Testing Motors...");
  
  // Quick motor test
  setMotorSpeed(100, 100);
  delay(500);
  setMotorSpeed(0, 0);
  delay(500);
  
  setMotorSpeed(-100, -100);
  delay(500);
  setMotorSpeed(0, 0);
  delay(500);
  
  Serial.println("Robot Ready!");
  Serial.println("Sensor Readings: L M R | Error | PID | L Speed | R Speed");
  Serial.println("--------------------------------------------------------");
  
  lastTime = millis(); // Initialize timer
}

void loop() {
  // Read sensors
  readIRSensors();
  
  // Calculate error
  error = calculateError();
  
  // Handle line lost condition
  if (error == 10) {
    // Line lost - slow down and search
    setMotorSpeed(50, -50); // Rotate in place to find line
    delay(20);
    
    // Check sensors again
    readIRSensors();
    error = calculateError();
    
    if (error == 10) {
      // Still lost, continue searching
      return;
    }
  }
  
  // Calculate PID correction
  PIDvalue = calculatePID(error);
  
  // Calculate motor speeds
  int leftMotorSpeed = baseSpeed - PIDvalue;
  int rightMotorSpeed = baseSpeed + PIDvalue;
  
  // Constrain speeds to safe range
  leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);
  
  // Apply motor speeds
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
  
  // Debug information
  Serial.print("Sensors: ");
  Serial.print(leftSensor);
  Serial.print(" ");
  Serial.print(middleSensor);
  Serial.print(" ");
  Serial.print(rightSensor);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(PIDvalue);
  Serial.print(" | L: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" | R: ");
  Serial.println(rightMotorSpeed);
  
  // Control loop delay
  delay(SAMPLE_TIME);
}

// Utility functions for precise movement (optional)
void moveForward(int distanceCM) {
  // Reset encoders
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  
  // Calculate target encoder counts
  float distanceM = distanceCM / 100.0;
  float wheelRevolutions = distanceM / WHEEL_CIRCUMFERENCE;
  long targetCounts = wheelRevolutions * ENCODER_PPR;
  
  // Move forward until target reached
  while (leftEncoderCount < targetCounts && rightEncoderCount < targetCounts) {
    setMotorSpeed(baseSpeed, baseSpeed);
    delay(10);
  }
  
  // Stop
  setMotorSpeed(0, 0);
}

void rotate(int degrees) {
  // Reset encoders
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  
  // Calculate wheel travel for rotation
  float rotationRadians = degrees * PI / 180.0;
  float wheelDistance = (rotationRadians * TRACK_WIDTH) / 2.0;
  float wheelRevolutions = wheelDistance / WHEEL_CIRCUMFERENCE;
  long targetCounts = wheelRevolutions * ENCODER_PPR;
  
  // Rotate
  if (degrees > 0) {
    // Rotate clockwise (right)
    setMotorSpeed(baseSpeed, -baseSpeed);
    while (leftEncoderCount < targetCounts) {
      delay(10);
    }
  } else {
    // Rotate counter-clockwise (left)
    setMotorSpeed(-baseSpeed, baseSpeed);
    while (rightEncoderCount < targetCounts) {
      delay(10);
    }
  }
  
  // Stop
  setMotorSpeed(0, 0);
}

// Test function to check sensor readings
void testSensors() {
  while (true) {
    readIRSensors();
    Serial.print("IR Sensors - Left: ");
    Serial.print(leftSensor);
    Serial.print(" | Middle: ");
    Serial.print(middleSensor);
    Serial.print(" | Right: ");
    Serial.println(rightSensor);
    delay(500);
  }
}

// Test function to check motors
void testMotors() {
  Serial.println("Testing Left Motor Forward");
  digitalWrite(L_INA, HIGH);
  digitalWrite(L_INB, LOW);
  analogWrite(L_PWM, 150);
  delay(2000);
  analogWrite(L_PWM, 0);
  
  Serial.println("Testing Right Motor Forward");
  digitalWrite(R_INA, HIGH);
  digitalWrite(R_INB, LOW);
  analogWrite(R_PWM, 150);
  delay(2000);
  analogWrite(R_PWM, 0);
  
  Serial.println("Motor Test Complete");
}