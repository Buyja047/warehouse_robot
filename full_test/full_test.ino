#include <Arduino.h>
#include <esp_arduino_version.h>

// ======================================================
// 1. PIN DEFINITIONS
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
static const int R_INB = 0;
static const int R_PWM = 45;
static const int ENC_R = 5;

// --- IR SENSORS ---
static const int IR_L = 15;
static const int IR_M = 7;
static const int IR_R = 6;

// ======================================================
// 2. CONFIGURATION
// ======================================================

// --- IR SENSOR LOGIC ---
// Set to 0 if sensor returns LOW on Black Line.
// Set to 1 if sensor returns HIGH on Black Line.
const int IR_BLACK_VALUE = 0;

// --- TIMING ---
const uint32_t START_WAIT_MS = 4000;     // 4 seconds delay after start marker
const uint32_t MARKER_STABLE_MS = 100;   // How long 000 must be seen to count as a marker

// --- LINE FOLLOWING SPEED ---
const int BASE_SPEED = 400;             // 0-1023 (10-bit PWM)
const int TURN_CORRECTION = 250;        // How much to slow down/speed up when turning

// --- 90 DEGREE TURN ---
const int TURN_SPEED = 400;
const long TURN_TARGET_COUNTS = 350;    // ADJUST THIS for exactly 90 degrees

// --- LIFT & PUSH ---
const int LIFT_SPEED = 1023;
const int PUSH_SPEED = 800;
const uint32_t LIFT_UP_TIME   = 14000;
const uint32_t LIFT_DOWN_TIME = 7000;
const uint32_t PUSH_TIMEOUT   = 5000;

// --- PWM SETTINGS ---
const int PWM_FREQ = 20000;
const int PWM_RES  = 10; // 10-bit resolution (0-1023)
const int PWM_MAX  = 1023;

// ======================================================
// 3. LEDC SETUP
// ======================================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino-ESP32 v3: use PIN-based ledcWrite(pin, duty)
  const int CH_L    = L_PWM;
  const int CH_R    = R_PWM;
  const int CH_PUSH = PUSH_PWM;
  const int CH_LIFT = LIFT_PWM;
#else
  // Arduino-ESP32 v2: use channel-based ledcWrite(ch, duty)
  const int CH_L    = 0;
  const int CH_R    = 1;
  const int CH_PUSH = 2;
  const int CH_LIFT = 3;
#endif

// ======================================================
// 4. STATE MACHINE
// ======================================================
enum State {
  WAIT_FOR_START_MARKER, // 1. Wait for 000
  START_DELAY,           // 2. Wait 4 seconds
  FOLLOW_LINE,           // 3. Follow line until next 000
  TURN_LEFT,             // 4. Rotate 90 deg
  LIFT_UP,               // 5. Lift
  PUSH_OUT,              // 6. Push
  PUSH_IN,               // 7. Pull back
  LIFT_DOWN,             // 8. Lower
  DONE,
  ERROR_STATE
};

State state = WAIT_FOR_START_MARKER;
uint32_t stateTimer = 0;
uint32_t markerTimer = 0;

// Encoders
volatile long encL_cnt = 0;
volatile long encR_cnt = 0;
void IRAM_ATTR isrEncL() { encL_cnt++; }
void IRAM_ATTR isrEncR() { encR_cnt++; }

// ======================================================
// 4.1 LINE FOLLOW HOLD-TURN (NEW)
// ======================================================
// turnHold: -1 = hold left turn, +1 = hold right turn, 0 = normal
static int turnHold = 0;
// lastTurn: remembers last direction (-1 / +1), used when line lost
static int lastTurn = 0;

// ======================================================
// 5. HELPER FUNCTIONS
// ======================================================

// Setup PWM
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

// Drive Motor
void motorDrive(int ina, int inb, int ch, int speed) {
  int s = constrain(speed, -PWM_MAX, PWM_MAX);

  if (s > 0) {
    digitalWrite(ina, HIGH); digitalWrite(inb, LOW);
    ledcWrite(ch, s);
  } else if (s < 0) {
    digitalWrite(ina, LOW); digitalWrite(inb, HIGH);
    ledcWrite(ch, -s);
  } else {
    digitalWrite(ina, LOW); digitalWrite(inb, LOW);
    ledcWrite(ch, 0);
  }
}

void stopMotors() {
  motorDrive(L_INA, L_INB, CH_L, 0);
  motorDrive(R_INA, R_INB, CH_R, 0);
}

// Hard Brake (locks wheels) - verify your driver supports this safely
void brakeMotors() {
  digitalWrite(L_INA, HIGH); digitalWrite(L_INB, HIGH);
  ledcWrite(CH_L, PWM_MAX);

  digitalWrite(R_INA, HIGH); digitalWrite(R_INB, HIGH);
  ledcWrite(CH_R, PWM_MAX);
}

void stopAll() {
  stopMotors();
  motorDrive(PUSH_INA, PUSH_INB, CH_PUSH, 0);
  motorDrive(LIFT_INA, LIFT_INB, CH_LIFT, 0);
}

// Read IR (Returns true if BLACK detected)
bool isBlack(int pin) {
  return (digitalRead(pin) == IR_BLACK_VALUE);
}

// ======================================================
// 6. SETUP & LOOP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Pins
  pinMode(PUSH_INA, OUTPUT); pinMode(PUSH_INB, OUTPUT);
  pinMode(LIFT_INA, OUTPUT); pinMode(LIFT_INB, OUTPUT);
  pinMode(L_INA, OUTPUT);    pinMode(L_INB, OUTPUT);
  pinMode(R_INA, OUTPUT);    pinMode(R_INB, OUTPUT);

  pinMode(IR_L, INPUT); pinMode(IR_M, INPUT); pinMode(IR_R, INPUT);
  pinMode(FRONT_LIMIT, INPUT_PULLUP);
  pinMode(BACK_LIMIT, INPUT_PULLUP);

  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrEncR, RISING);

  setupPwm(L_PWM, CH_L);
  setupPwm(R_PWM, CH_R);
  setupPwm(PUSH_PWM, CH_PUSH);
  setupPwm(LIFT_PWM, CH_LIFT);

  stopAll();
  Serial.println("=== ROBOT READY: LINE FOLLOWER + LIFT ===");
}

void loop() {
  // Read Sensors
  bool sl = isBlack(IR_L);
  bool sm = isBlack(IR_M);
  bool sr = isBlack(IR_R);

  // Check for "000" (All Black)
  bool isMarker = (sl && sm && sr);

  switch (state) {

    // --- 1. WAIT FOR START MARKER (000) ---
    case WAIT_FOR_START_MARKER:
      if (isMarker) {
        if (markerTimer == 0) markerTimer = millis();
        if (millis() - markerTimer > MARKER_STABLE_MS) {
          stopMotors();
          state = START_DELAY;
          stateTimer = millis();
          Serial.println("START DETECTED -> WAITING 4s");
        }
      } else {
        markerTimer = 0;
      }
      break;

    // --- 2. 4 SECOND DELAY ---
    case START_DELAY:
      if (millis() - stateTimer > START_WAIT_MS) {
        // reset hold-turn logic when starting to follow
        turnHold = 0;
        lastTurn = 0;

        state = FOLLOW_LINE;

        // Move slightly to get OFF the start marker so we don't detect it as End immediately
        motorDrive(L_INA, L_INB, CH_L, BASE_SPEED);
        motorDrive(R_INA, R_INB, CH_R, BASE_SPEED);
        delay(500);

        Serial.println("GO -> LINE FOLLOW");
      }
      break;

    // --- 3. FOLLOW LINE ---
    case FOLLOW_LINE: {
      // A. Check for End Marker (000)
      if (isMarker) {
        if (markerTimer == 0) markerTimer = millis();
        if (millis() - markerTimer > MARKER_STABLE_MS) {
          // stop and prepare to turn
          brakeMotors();
          delay(500);

          // clear hold-turn so it doesn't interfere later
          turnHold = 0;

          noInterrupts(); encL_cnt = 0; encR_cnt = 0; interrupts();
          state = TURN_LEFT;
          stateTimer = millis();
          Serial.println("END MARKER -> TURN LEFT");
          return;
        }
      } else {
        markerTimer = 0;
      }

      // B. HOLD-TURN LINE FOLLOW (NEW BEHAVIOR)
      int leftSpeed  = BASE_SPEED;
      int rightSpeed = BASE_SPEED;

      // 1) If we are holding a turn, keep turning until center sensor sees black
      if (turnHold == +1) { // holding RIGHT
        if (sm) {
          turnHold = 0; // release when center found
        } else {
          leftSpeed  = BASE_SPEED + TURN_CORRECTION;
          rightSpeed = BASE_SPEED - TURN_CORRECTION;
        }
      } else if (turnHold == -1) { // holding LEFT
        if (sm) {
          turnHold = 0;
        } else {
          leftSpeed  = BASE_SPEED - TURN_CORRECTION;
          rightSpeed = BASE_SPEED + TURN_CORRECTION;
        }
      }

      // 2) If not holding, decide what to do now
      if (turnHold == 0) {
        if (sm) {
          // centered: go straight
          leftSpeed  = BASE_SPEED;
          rightSpeed = BASE_SPEED;
        } else if (sr) {
          // right sensor sees line: start holding right until center sees it
          turnHold = +1;
          lastTurn = +1;
          leftSpeed  = BASE_SPEED + TURN_CORRECTION;
          rightSpeed = BASE_SPEED - TURN_CORRECTION;
        } else if (sl) {
          // left sensor sees line: start holding left until center sees it
          turnHold = -1;
          lastTurn = -1;
          leftSpeed  = BASE_SPEED - TURN_CORRECTION;
          rightSpeed = BASE_SPEED + TURN_CORRECTION;
        } else {
          // line lost (white/white/white): continue searching in last known direction
          if (lastTurn == +1) {
            leftSpeed  = BASE_SPEED + TURN_CORRECTION;
            rightSpeed = BASE_SPEED - TURN_CORRECTION;
          } else if (lastTurn == -1) {
            leftSpeed  = BASE_SPEED - TURN_CORRECTION;
            rightSpeed = BASE_SPEED + TURN_CORRECTION;
          } else {
            // no history: just go straight
            leftSpeed  = BASE_SPEED;
            rightSpeed = BASE_SPEED;
          }
        }
      }

      motorDrive(L_INA, L_INB, CH_L, leftSpeed);
      motorDrive(R_INA, R_INB, CH_R, rightSpeed);
    } break;

    // --- 4. ROTATE 90 DEGREES ---
    case TURN_LEFT: {
      long currentCount;
      noInterrupts(); currentCount = (encL_cnt + encR_cnt) / 2; interrupts();

      if (currentCount < TURN_TARGET_COUNTS) {
        motorDrive(L_INA, L_INB, CH_L, -TURN_SPEED);
        motorDrive(R_INA, R_INB, CH_R,  TURN_SPEED);
      } else {
        brakeMotors();
        delay(500);
        state = LIFT_UP;
        stateTimer = millis();
        Serial.println("TURN DONE -> LIFT UP");
      }
    } break;

    // --- 5. LIFT UP ---
    case LIFT_UP:
      if (millis() - stateTimer < LIFT_UP_TIME) {
        motorDrive(LIFT_INA, LIFT_INB, CH_LIFT, LIFT_SPEED);
      } else {
        motorDrive(LIFT_INA, LIFT_INB, CH_LIFT, 0);
        state = PUSH_OUT;
        stateTimer = millis();
        Serial.println("LIFT UP DONE -> PUSH OUT");
      }
      break;

    // --- 6. PUSH OUT ---
    case PUSH_OUT: {
      bool hit = (digitalRead(FRONT_LIMIT) == LOW); // active LOW
      bool timeout = (millis() - stateTimer > PUSH_TIMEOUT);

      if (!hit && !timeout) {
        motorDrive(PUSH_INA, PUSH_INB, CH_PUSH, PUSH_SPEED);
      } else {
        motorDrive(PUSH_INA, PUSH_INB, CH_PUSH, 0);
        state = PUSH_IN;
        stateTimer = millis();
        Serial.println("PUSH OUT DONE -> PUSH IN");
        delay(500);
      }
    } break;

    // --- 7. PUSH IN ---
    case PUSH_IN: {
      bool hit = (digitalRead(BACK_LIMIT) == LOW); // active LOW
      bool timeout = (millis() - stateTimer > PUSH_TIMEOUT);

      if (!hit && !timeout) {
        motorDrive(PUSH_INA, PUSH_INB, CH_PUSH, -PUSH_SPEED);
      } else {
        motorDrive(PUSH_INA, PUSH_INB, CH_PUSH, 0);
        state = LIFT_DOWN;
        stateTimer = millis();
        Serial.println("PUSH IN DONE -> LIFT DOWN");
      }
    } break;

    // --- 8. LIFT DOWN ---
    case LIFT_DOWN:
      if (millis() - stateTimer < LIFT_DOWN_TIME) {
        motorDrive(LIFT_INA, LIFT_INB, CH_LIFT, -LIFT_SPEED);
      } else {
        motorDrive(LIFT_INA, LIFT_INB, CH_LIFT, 0);
        state = DONE;
        Serial.println("MISSION COMPLETE");
      }
      break;

    // --- 9. FINISH ---
    case DONE:
    case ERROR_STATE:
      stopAll();
      break;
  }
}
