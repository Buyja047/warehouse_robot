// ESP32 S3 Line Following Robot
// Wheel distance: 450mm, IR sensors 250mm behind wheels
// Sensor spacing: 50mm from center, Line width: 90mm
// Wheel diameter: 137mm, Encoder PPM: 460

// ========== PIN DEFINITIONS ==========
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

// IR Sensors (0 = on line, 1 = off line for typical IR sensors)
static const int IR_L = 15;
static const int IR_M = 7;
static const int IR_R = 6;

// ========== ROBOT PARAMETERS ==========
const float WHEEL_DIAMETER = 0.137;  // meters
const int ENCODER_PPM = 460;         // pulses per meter
const float WHEEL_BASE = 0.450;      // meters (distance between wheels)
const float SENSOR_OFFSET = 0.250;   // meters (sensors behind wheels)

// ========== PID PARAMETERS ==========
float Kp = 0.08;   // Proportional gain - adjust for responsiveness
float Ki = 0.005;    // Integral gain - usually keep low or 0
float Kd = 0.05;   // Derivative gain - adjust for stability

// ========== SPEED SETTINGS ==========
int baseSpeed = 60;      // Base PWM speed (0-255)
int maxSpeed = 255;       // Maximum PWM speed
int minSpeed = 50;        // Minimum PWM to overcome friction

// ========== GLOBAL VARIABLES ==========
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// ========== ENCODER INTERRUPTS (must be declared before setup) ==========
void IRAM_ATTR encoderLeftISR() {
  encoderCountL++;
}

void IRAM_ATTR encoderRightISR() {
  encoderCountR++;
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(L_INA, OUTPUT);
  pinMode(L_INB, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_INA, OUTPUT);
  pinMode(R_INB, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  
  // Encoder pins
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encoderRightISR, RISING);
  
  // IR sensor pins
  pinMode(IR_L, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_R, INPUT);
  
  // Stop motors initially
  stopMotors();
  
  delay(1000);
  Serial.println("Line Following Robot Ready!");
}

// ========== MAIN LOOP ==========
void loop() {
  // Read IR sensors (assuming 0=black/line, 1=white/floor)
  int irL = digitalRead(IR_L);
  int irM = digitalRead(IR_M);
  int irR = digitalRead(IR_R);
  
  // Calculate position error based on sensor readings
  float error = calculateError(irL, irM, irR);
  
  // Calculate PID correction
  float correction = calculatePID(error);
  
  // Calculate individual wheel speeds
  int leftSpeed = constrain(baseSpeed - correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);
  
  // Apply speeds to motors
  setMotorSpeed(leftSpeed, rightSpeed);
  
  // Debug output
  if (millis() % 200 < 10) {  // Print every 200ms
    Serial.printf("IR[L:%d M:%d R:%d] Error:%.2f Corr:%.1f Speed[L:%d R:%d]\n", 
                  irL, irM, irR, error, correction, leftSpeed, rightSpeed);
  }
  
  delay(10);  // Small delay for stability
}

// ========== ERROR CALCULATION ==========
float calculateError(int irL, int irM, int irR) {
  // Error values based on sensor positions
  // Left sensor at -50mm, Middle at 0mm, Right at +50mm
  
  // Create 8 possible states (LMR combinations)
  if (irL == 0 && irM == 0 && irR == 0) {
    return 0;  // All on line - centered
  }
  else if (irL == 0 && irM == 0 && irR == 1) {
    return -25;  // Slight left
  }
  else if (irL == 0 && irM == 1 && irR == 1) {
    return -50;  // More left
  }
  else if (irL == 1 && irM == 0 && irR == 0) {
    return 25;   // Slight right
  }
  else if (irL == 1 && irM == 1 && irR == 0) {
    return 50;   // More right
  }
  else if (irL == 1 && irM == 0 && irR == 1) {
    return 0;    // Only center - perfectly aligned
  }
  else if (irL == 0 && irM == 1 && irR == 0) {
    return lastError > 0 ? 75 : -75;  // Sharp turn based on last direction
  }
  else {  // All sensors off line (1,1,1)
    // Lost the line - maintain last correction direction
    return lastError > 0 ? 100 : -100;
  }
}

// ========== PID CONTROLLER ==========
float calculatePID(float error) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  
  if (deltaTime == 0) deltaTime = 0.01;  // Prevent division by zero
  
  // Proportional term
  float P = Kp * error;
  
  // Integral term (with anti-windup)
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100);
  float I = Ki * integral;
  
  // Derivative term
  float derivative = (error - lastError) / deltaTime;
  float D = Kd * derivative;
  
  // Calculate total correction
  float correction = P + I + D;
  
  // Update for next iteration
  lastError = error;
  lastTime = currentTime;
  
  return correction;
}

// ========== MOTOR CONTROL ==========
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(L_INA, HIGH);
    digitalWrite(L_INB, LOW);
    analogWrite(L_PWM, abs(leftSpeed));
  } else {
    digitalWrite(L_INA, LOW);
    digitalWrite(L_INB, HIGH);
    analogWrite(L_PWM, abs(leftSpeed));
  }
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(R_INA, HIGH);
    digitalWrite(R_INB, LOW);
    analogWrite(R_PWM, abs(rightSpeed));
  } else {
    digitalWrite(R_INA, LOW);
    digitalWrite(R_INB, HIGH);
    analogWrite(R_PWM, abs(rightSpeed));
  }
}

void stopMotors() {
  digitalWrite(L_INA, LOW);
  digitalWrite(L_INB, LOW);
  analogWrite(L_PWM, 0);
  digitalWrite(R_INA, LOW);
  digitalWrite(R_INB, LOW);
  analogWrite(R_PWM, 0);
}

// ========== UTILITY FUNCTIONS ==========
float getDistanceTraveled() {
  long avgCount = (encoderCountL + encoderCountR) / 2;
  return (float)avgCount / ENCODER_PPM;
}

void resetEncoders() {
  encoderCountL = 0;
  encoderCountR = 0;
}