#include <Arduino.h>
#include <esp_arduino_version.h>

// ======================================================
// 1) PIN DEFINITIONS
// ======================================================
// --- PUSH ---
static const int PUSH_INA = 1;
static const int PUSH_INB = 2;
static const int PUSH_PWM = 42;
static const int FRONT_LIMIT = 17;
static const int BACK_LIMIT  = 16;

// --- LIFT ---
static const int LIFT_INA = 41;
static const int LIFT_INB = 40;
static const int LIFT_PWM = 39;

// --- LEFT WHEEL ---
static const int L_INA = 38;
static const int L_INB = 37;
static const int L_PWM = 36;
static const int ENC_L = 4;

// --- RIGHT WHEEL ---
static const int R_INA = 35;
static const int R_INB = 0;   // WARNING: GPIO0 is boot strap pin
static const int R_PWM = 45;
static const int ENC_R = 5;

// --- IR sensors ---
static const int IR_R = 6;
static const int IR_M = 7;
static const int IR_L = 15;

// ======================================================
// 2) TYPES (MUST BE ABOVE ANY FUNCTION IN .INO)
// ======================================================
enum State {
  FOLLOW_TO_MARKER1,
  WAIT_AT_MARKER1,
  FOLLOW_TO_MARKER2,
  TURN_LEFT_90,
  LIFT_UP,
  PUSH_FORWARD,
  PUSH_BACKWARD,
  LIFT_DOWN,

  // NEW: after all actions
  TURN_LEFT_90_2,      // second 90 deg
  FOLLOW_COUNT2,       // count 2 markers then stop

  DONE,
  ERROR_STATE
};

enum FinishReason { FIN_NONE=0, FIN_MARKER=1, FIN_TIMEOUT=2 };

// ======================================================
// 3) CONFIGURATION
// ======================================================
const int IR_BLACK_VALUE = LOW;
const uint32_t MARKER_STABLE_MS = 20;
const uint32_t WAIT_4S_MS = 4000;

// PID (encoder heading keep)
float Kp = 1.4;
float Ki = 0.00005;
float Kd = 0.5;

const int BASE_PWM_PID = 500;
const int MIN_PWM_PID  = 250;
const uint32_t RAMP_UP_MS = 1200;

const uint32_t TO_MARKER1_TIMEOUT_MS = 30000;
const uint32_t TO_MARKER2_TIMEOUT_MS = 30000;

// NEW: after task, count 2 markers then stop
const uint32_t COUNT2_TIMEOUT_MS = 45000; // хэрэгтэй бол өөрчил

// TURN
const long TARGET_TURN_COUNTS = 300;
const long S_TARGET_TURN_COUNTS = 280;
const int TURN_SPEED = 350;

// LIFT & PUSH
const int LIFT_SPEED = 1023;
const int PUSH_SPEED = 800;
const uint32_t LIFT_UP_TIME   = 13500;
const uint32_t LIFT_DOWN_TIME = 9500;
const uint32_t PUSH_TIMEOUT   = 50000;

// PWM
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;
const int PWM_MAX  = 1023;

// ======================================================
// 4) LEDC CHANNELS
// ======================================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static const int CH_L_WHEEL = L_PWM;
  static const int CH_R_WHEEL = R_PWM;
  static const int CH_PUSH    = PUSH_PWM;
  static const int CH_LIFT    = LIFT_PWM;
#else
  static const int CH_L_WHEEL = 0;
  static const int CH_R_WHEEL = 1;
  static const int CH_PUSH    = 2;
  static const int CH_LIFT    = 3;
#endif

// ======================================================
// 5) GLOBALS
// ======================================================
State state = FOLLOW_TO_MARKER1;
uint32_t stateT0 = 0;

volatile long encL_cnt = 0;
volatile long encR_cnt = 0;

float pid_integral = 0;
float pid_prevError = 0;

uint32_t markerTimer = 0;
bool marker2_armed = false;

// NEW: for counting markers (2 lines)
int  markerCount = 0;
bool countArmed  = false;   // must leave current marker first before counting
// (markerTimer ашиглана)

// ======================================================
// 6) ISR & UTILS
// ======================================================
void IRAM_ATTR isrEncL() { encL_cnt++; }
void IRAM_ATTR isrEncR() { encR_cnt++; }

void resetEncoders() {
  noInterrupts();
  encL_cnt = 0;
  encR_cnt = 0;
  interrupts();
}

long getAvgEnc() {
  noInterrupts();
  long l = encL_cnt;
  long r = encR_cnt;
  interrupts();
  return (l + r) / 2;
}

static inline float clamp01(float x) {
  if (x < 0) return 0;
  if (x > 1) return 1;
  return x;
}

static inline bool isBlackIR(int pin) {
  return (digitalRead(pin) == IR_BLACK_VALUE);
}
static inline bool isMarker000() {
  return isBlackIR(IR_L) && isBlackIR(IR_M) && isBlackIR(IR_R);
}

// ======================================================
// 7) MOTOR DRIVER
// ======================================================
void setupPwm(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ, PWM_RES);
  ledcWrite(pin, 0);
#else
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
#endif
}

void pwmWrite(int ch, int val) {
  val = constrain(val, 0, PWM_MAX);
  ledcWrite(ch, val);
}

void drive(int ina, int inb, int ch, int speedSigned) {
  speedSigned = constrain(speedSigned, -PWM_MAX, PWM_MAX);

  if (speedSigned > 0) {
    digitalWrite(ina, HIGH); digitalWrite(inb, LOW);
    pwmWrite(ch, speedSigned);
  } else if (speedSigned < 0) {
    digitalWrite(ina, LOW); digitalWrite(inb, HIGH);
    pwmWrite(ch, -speedSigned);
  } else {
    digitalWrite(ina, LOW); digitalWrite(inb, LOW);
    pwmWrite(ch, 0);
  }
}

void brakeWheels() {
  digitalWrite(L_INA, HIGH); digitalWrite(L_INB, HIGH); pwmWrite(CH_L_WHEEL, PWM_MAX);
  digitalWrite(R_INA, HIGH); digitalWrite(R_INB, HIGH); pwmWrite(CH_R_WHEEL, PWM_MAX);
}

void stopAll() {
  drive(L_INA, L_INB, CH_L_WHEEL, 0);
  drive(R_INA, R_INB, CH_R_WHEEL, 0);
  drive(PUSH_INA, PUSH_INB, CH_PUSH, 0);
  drive(LIFT_INA, LIFT_INB, CH_LIFT, 0);
}

// ======================================================
// 8) STATE HELPERS
// ======================================================
void enterState(State s) {
  state = s;
  stateT0 = millis();
  markerTimer = 0;

  if (s == FOLLOW_TO_MARKER1 || s == FOLLOW_TO_MARKER2 || s == TURN_LEFT_90 ||
      s == TURN_LEFT_90_2 || s == FOLLOW_COUNT2) {
    resetEncoders();
    pid_integral = 0;
    pid_prevError = 0;
  }

  if (s == FOLLOW_TO_MARKER2) {
    marker2_armed = false; // must leave marker1 first
  }

  if (s == FOLLOW_COUNT2) {
    markerCount = 0;
    countArmed  = false; // must leave current marker first
  }

  if (s == DONE || s == ERROR_STATE) stopAll();

  Serial.printf(">>> Entering State: %d\n", (int)s);
}

FinishReason pidDriveUntilMarker(bool useArmingLeaveFirst, uint32_t timeoutMs) {
  uint32_t now = millis();

  if (now - stateT0 >= timeoutMs) {
    brakeWheels();
    return FIN_TIMEOUT;
  }

  bool marker = isMarker000();

  if (!useArmingLeaveFirst) {
    if (marker) {
      if (markerTimer == 0) markerTimer = now;
      if (now - markerTimer >= MARKER_STABLE_MS) {
        brakeWheels();
        return FIN_MARKER;
      }
    } else {
      markerTimer = 0;
    }
  } else {
    if (!marker2_armed) {
      if (!marker) marker2_armed = true; // left marker1
      markerTimer = 0;
    } else {
      if (marker) {
        if (markerTimer == 0) markerTimer = now;
        if (now - markerTimer >= MARKER_STABLE_MS) {
          brakeWheels();
          return FIN_MARKER;
        }
      } else {
        markerTimer = 0;
      }
    }
  }

  long cL, cR;
  noInterrupts(); cL = encL_cnt; cR = encR_cnt; interrupts();

  float startFactor = clamp01((now - stateT0) / (float)RAMP_UP_MS);
  int base = (int)(MIN_PWM_PID + (BASE_PWM_PID - MIN_PWM_PID) * startFactor);

  float error = (float)(cL - cR);
  pid_integral += error;

  if (pid_integral > 200000)  pid_integral = 200000;
  if (pid_integral < -200000) pid_integral = -200000;

  float derivative = error - pid_prevError;
  pid_prevError = error;

  float corr = (Kp * error) + (Ki * pid_integral) + (Kd * derivative);

  int spdL = (int)(base - corr);
  int spdR = (int)(base + corr);

  drive(L_INA, L_INB, CH_L_WHEEL, spdL);
  drive(R_INA, R_INB, CH_R_WHEEL, spdR);

  return FIN_NONE;
}

// NEW: drive and count N markers, then stop
FinishReason pidDriveCountMarkers(int targetCount, uint32_t timeoutMs) {
  uint32_t now = millis();

  if (now - stateT0 >= timeoutMs) {
    brakeWheels();
    return FIN_TIMEOUT;
  }

  bool marker = isMarker000();

  // must leave current marker first (start condition)
  if (!countArmed) {
    if (!marker) countArmed = true; // left the starting marker area
    markerTimer = 0;
  } else {
    // count markers with debounce, then require leaving again
    if (marker) {
      if (markerTimer == 0) markerTimer = now;
      if (now - markerTimer >= MARKER_STABLE_MS) {
        markerCount++;
        Serial.printf("MARKER COUNT = %d / %d\n", markerCount, targetCount);

        // after counting one marker, force leave-before-next
        countArmed = false;
        markerTimer = 0;

        if (markerCount >= targetCount) {
          brakeWheels();
          return FIN_MARKER;
        }
      }
    } else {
      markerTimer = 0;
    }
  }

  // PID heading keep (same as other straight)
  long cL, cR;
  noInterrupts(); cL = encL_cnt; cR = encR_cnt; interrupts();

  float startFactor = clamp01((now - stateT0) / (float)RAMP_UP_MS);
  int base = (int)(MIN_PWM_PID + (BASE_PWM_PID - MIN_PWM_PID) * startFactor);

  float error = (float)(cL - cR);
  pid_integral += error;

  if (pid_integral > 200000)  pid_integral = 200000;
  if (pid_integral < -200000) pid_integral = -200000;

  float derivative = error - pid_prevError;
  pid_prevError = error;

  float corr = (Kp * error) + (Ki * pid_integral) + (Kd * derivative);

  int spdL = (int)(base - corr);
  int spdR = (int)(base + corr);

  drive(L_INA, L_INB, CH_L_WHEEL, spdL);
  drive(R_INA, R_INB, CH_R_WHEEL, spdR);

  return FIN_NONE;
}

// ======================================================
// 9) SETUP & LOOP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PUSH_INA, OUTPUT); pinMode(PUSH_INB, OUTPUT);
  pinMode(LIFT_INA, OUTPUT); pinMode(LIFT_INB, OUTPUT);
  pinMode(L_INA, OUTPUT);    pinMode(L_INB, OUTPUT);
  pinMode(R_INA, OUTPUT);    pinMode(R_INB, OUTPUT);

  setupPwm(L_PWM, CH_L_WHEEL);
  setupPwm(R_PWM, CH_R_WHEEL);
  setupPwm(PUSH_PWM, CH_PUSH);
  setupPwm(LIFT_PWM, CH_LIFT);

  pinMode(FRONT_LIMIT, INPUT_PULLUP);
  pinMode(BACK_LIMIT,  INPUT_PULLUP);

  pinMode(IR_L, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_R, INPUT);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrEncR, RISING);

  stopAll();

  Serial.println("=== FLOW: M1 STOP -> WAIT 4s -> M2 STOP -> TURN -> LIFT/PUSH/DOWN -> TURN -> COUNT2 STOP ===");
  enterState(FOLLOW_TO_MARKER1);
}

void loop() {
  switch (state) {

    case FOLLOW_TO_MARKER1: {
      FinishReason fr = pidDriveUntilMarker(false, TO_MARKER1_TIMEOUT_MS);
      if (fr == FIN_MARKER) {
        stopAll();
        enterState(WAIT_AT_MARKER1);
      } else if (fr == FIN_TIMEOUT) {
        Serial.println("ERROR: marker1 timeout");
        enterState(ERROR_STATE);
      }
    } break;

    case WAIT_AT_MARKER1: {
      stopAll();
      if (millis() - stateT0 >= WAIT_4S_MS) {
        enterState(FOLLOW_TO_MARKER2);
      }
    } break;

    case FOLLOW_TO_MARKER2: {
      FinishReason fr = pidDriveUntilMarker(true, TO_MARKER2_TIMEOUT_MS);
      if (fr == FIN_MARKER) {
        stopAll();
        delay(300);
        enterState(TURN_LEFT_90);
      } else if (fr == FIN_TIMEOUT) {
        Serial.println("ERROR: marker2 timeout");
        enterState(ERROR_STATE);
      }
    } break;

    case TURN_LEFT_90: {
      long cnt = getAvgEnc();
      if (cnt < TARGET_TURN_COUNTS) {
        drive(L_INA, L_INB, CH_L_WHEEL, -TURN_SPEED);
        drive(R_INA, R_INB, CH_R_WHEEL,  TURN_SPEED);
      } else {
        brakeWheels();
        delay(400);
        enterState(LIFT_UP);
      }
    } break;

    case LIFT_UP:
      if (millis() - stateT0 < LIFT_UP_TIME) {
        drive(LIFT_INA, LIFT_INB, CH_LIFT, LIFT_SPEED);
      } else {
        drive(LIFT_INA, LIFT_INB, CH_LIFT, 0);
        enterState(PUSH_FORWARD);
      }
      break;

    case PUSH_FORWARD: {
      bool hit = (digitalRead(FRONT_LIMIT) == LOW);
      if (!hit && (millis() - stateT0 < PUSH_TIMEOUT)) {
        drive(PUSH_INA, PUSH_INB, CH_PUSH, PUSH_SPEED);
      } else {
        drive(PUSH_INA, PUSH_INB, CH_PUSH, 0);
        enterState(PUSH_BACKWARD);
      }
    } break;

    case PUSH_BACKWARD: {
      bool hit = (digitalRead(BACK_LIMIT) == LOW);
      if (!hit && (millis() - stateT0 < PUSH_TIMEOUT)) {
        drive(PUSH_INA, PUSH_INB, CH_PUSH, -PUSH_SPEED);
      } else {
        drive(PUSH_INA, PUSH_INB, CH_PUSH, 0);
        enterState(LIFT_DOWN);
      }
    } break;

    case LIFT_DOWN:
      if (millis() - stateT0 < LIFT_DOWN_TIME) {
        drive(LIFT_INA, LIFT_INB, CH_LIFT, -LIFT_SPEED);
      } else {
        drive(LIFT_INA, LIFT_INB, CH_LIFT, 0);

        // NEW: second 90 deg turn after finishing all actions
        enterState(TURN_LEFT_90_2);
      }
      break;

    // NEW: 2 дахь 90° эргэлт
    case TURN_LEFT_90_2: {
      long cnt = getAvgEnc();
      if (cnt < S_TARGET_TURN_COUNTS) {
        drive(L_INA, L_INB, CH_L_WHEEL, -TURN_SPEED);
        drive(R_INA, R_INB, CH_R_WHEEL,  TURN_SPEED);
      } else {
        brakeWheels();
        delay(300);
        enterState(FOLLOW_COUNT2);
      }
    } break;

    // NEW: 2 marker тоолоод зогсоно
    case FOLLOW_COUNT2: {
      FinishReason fr = pidDriveCountMarkers(2, COUNT2_TIMEOUT_MS);
      if (fr == FIN_MARKER) {
        stopAll();
        enterState(DONE);
      } else if (fr == FIN_TIMEOUT) {
        Serial.println("ERROR: count2 timeout");
        enterState(ERROR_STATE);
      }
    } break;

    case DONE:
    case ERROR_STATE:
    default:
      stopAll();
      break;
  }
}
