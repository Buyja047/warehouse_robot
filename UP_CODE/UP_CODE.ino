#include <Arduino.h>
#include <esp_arduino_version.h>

// ======================================================
// ESP32-S3 Line Follower + Heading Hold + Cross-Line STOP
// - 3x Digital IR sensors: LOW on BLACK (your case)
// - Encoder-based heading hold on straight segments
// - Cross-line (111 for N ms) -> STOP state
// - Serial Plotter output + Live tuning commands
//
// NOTE (HARDWARE): You are using strap pins GPIO0 and GPIO45.
//  - R_INB = GPIO0  (strap pin)
//  - R_PWM = GPIO45 (strap pin)
// If boot/upload problems occur, move these wires to safe GPIOs.
// ======================================================

// ======================================================
// 1) TYPES (must be at top)
// ======================================================
struct LineReading {
  uint8_t bL, bM, bR;   // 1=black, 0=white
  float   err_m;        // lateral error (m): -left, +right
  bool    seen;         // any sensor sees line
};

struct WheelSpeed {
  int32_t dL, dR;       // pulses in this loop period
  float   vL_mps, vR_mps;
};

// ======================================================
// 2) PIN DEFINITIONS (yours)
// ======================================================
// LEFT WHEEL
static const int L_INA = 38;
static const int L_INB = 37;
static const int L_PWM = 36;
static const int ENC_L = 4;

// RIGHT WHEEL
static const int R_INA = 35;
static const int R_INB = 0;     // strap pin
static const int R_PWM = 45;    // strap pin
static const int ENC_R = 5;

// IR SENSORS (digital)
static const int IR_L  = 15;
static const int IR_M  = 7;
static const int IR_R  = 6;

// ======================================================
// 3) ROBOT PARAMETERS (yours)
// ======================================================
static const float WHEEL_DIAM_M = 0.137f;
static const float ENC_PPR      = 460.0f;  // pulses per wheel revolution (your stated)
static const float WHEEL_CIRC_M = 3.1415926f * WHEEL_DIAM_M;
static const float M_PER_PULSE  = WHEEL_CIRC_M / ENC_PPR;

static const float WHEEL_BASE_M = 0.450f;  // 450mm (wheel distance)
static const float SENSOR_HALF_SPACING_M = 0.05f; // center -> left/right = 5cm

// ======================================================
// 4) LOOP TIMING
// ======================================================
static const uint32_t LOOP_HZ    = 200;
static const uint32_t LOOP_DT_US = 1000000UL / LOOP_HZ;

// ======================================================
// 5) IR SENSOR LOGIC (YOU CONFIRMED: LOW on BLACK)
// ======================================================
static const bool IR_BLACK_IS_LOW = false;  // your sensors

static inline uint8_t readBlackDigital(int pin) {
  int v = digitalRead(pin);
  return (IR_BLACK_IS_LOW) ? (v == LOW) : (v == HIGH);
}

// ======================================================
// 6) PWM (LEDC) - ESP32 core v2.x and v3.x compatible
// ======================================================
static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t  PWM_RES_BITS = 10;             // 0..1023
static const int      PWM_MAX = (1 << PWM_RES_BITS) - 1;

static const int CH_L = 0;
static const int CH_R = 1;

static inline bool pwmAttach(int pin, int ch, uint32_t freq, uint8_t res) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  return ledcAttachChannel((uint8_t)pin, freq, res, (int8_t)ch);
#else
  ledcSetup(ch, freq, res);
  ledcAttachPin(pin, ch);
  return true;
#endif
}

static inline void pwmWriteCh(int ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteChannel((uint8_t)ch, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// ======================================================
// 7) CONTROL CONFIG (LIVE TUNING)
// ======================================================
// Line PID (start conservative; sensors are far behind axle)
static float Kp = 1.5f;
static float Ki = 0.0f;     // strongly recommended 0 initially
static float Kd = 0.25f;

// Base speed and limits (10-bit)
static int BASE_PWM   = 450;
static int TURN_LIMIT = 500;
static int SLEW_STEP  = 40;      // bigger = faster reaction
static int RECOVER_PWM = 340;

// Steering sign (flip if turns wrong)
static int STEER_SIGN = +1;

// Heading-hold (encoder) gain (tune)
// yawErr_m = (dR - dL)*M_PER_PULSE  (meters per loop)
// head_u = Kh * yawErr_m  -> PWM-like
static float Kh = 60000.0f;      // heading hold gain
static float Khd = 0.0f;         // optional heading derivative gain
static float eStraight_m = 0.010f; // only apply heading hold when |e| < 10mm

// Cross-line detection: require 111 held for N ms to stop
static uint32_t CROSS_HOLD_MS = 120;  // 80..200ms typical
static uint32_t CROSS_ARM_MS  = 800;  // ignore cross detection for first 0.8s after start/go

// ======================================================
// 8) ENCODERS
// ======================================================
volatile int32_t encL = 0;
volatile int32_t encR = 0;
void IRAM_ATTR isrEncL() { encL++; }
void IRAM_ATTR isrEncR() { encR++; }

// ======================================================
// 9) UTIL
// ======================================================
static inline int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline int slew(int cur, int tgt, int step) {
  if (tgt > cur) return (tgt - cur > step) ? (cur + step) : tgt;
  if (tgt < cur) return (cur - tgt > step) ? (cur - step) : tgt;
  return cur;
}

// ======================================================
// 10) MOTOR CONTROL
// ======================================================
static inline void setMotor(int ina, int inb, int pwmCh, int speedCmd) {
  speedCmd = clampi(speedCmd, -PWM_MAX, PWM_MAX);

  if (speedCmd >= 0) {
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);
    pwmWriteCh(pwmCh, (uint32_t)speedCmd);
  } else {
    digitalWrite(ina, LOW);
    digitalWrite(inb, HIGH);
    pwmWriteCh(pwmCh, (uint32_t)(-speedCmd));
  }
}

static inline void stopMotors() {
  digitalWrite(L_INA, LOW); digitalWrite(L_INB, LOW); pwmWriteCh(CH_L, 0);
  digitalWrite(R_INA, LOW); digitalWrite(R_INB, LOW); pwmWriteCh(CH_R, 0);
}

// ======================================================
// 11) LINE READING (weighted average error in meters)
// ======================================================
static int lastTurnDir = 0; // -1 left, +1 right

static LineReading readLine() {
  LineReading lr;
  lr.bL = readBlackDigital(IR_L);
  lr.bM = readBlackDigital(IR_M);
  lr.bR = readBlackDigital(IR_R);

  int sum = lr.bL + lr.bM + lr.bR;
  lr.seen = (sum != 0);

  if (!lr.seen) {
    lr.err_m = 0.0f;
    return lr;
  }

  // 111 => treat as centered (also used for cross detection separately)
  if (sum == 3) {
    lr.err_m = 0.0f;
    return lr;
  }

  // Weighted average: L=-0.05, M=0, R=+0.05
  float num = 0.0f;
  if (lr.bL) num += -SENSOR_HALF_SPACING_M;
  if (lr.bR) num +=  SENSOR_HALF_SPACING_M;

  lr.err_m = num / (float)sum;

  if (lr.err_m < -0.001f) lastTurnDir = -1;
  else if (lr.err_m >  0.001f) lastTurnDir = +1;

  return lr;
}

// ======================================================
// 12) SPEED ESTIMATION (debug + heading hold)
// ======================================================
static WheelSpeed estimateSpeed(int32_t &prevL, int32_t &prevR) {
  WheelSpeed ws;
  int32_t lNow, rNow;
  noInterrupts();
  lNow = encL; rNow = encR;
  interrupts();

  ws.dL = lNow - prevL;
  ws.dR = rNow - prevR;
  prevL = lNow;
  prevR = rNow;

  float dt = 1.0f / (float)LOOP_HZ;
  ws.vL_mps = (float)ws.dL * M_PER_PULSE / dt;
  ws.vR_mps = (float)ws.dR * M_PER_PULSE / dt;
  return ws;
}

// ======================================================
// 13) RUN STATE (FOLLOW / STOPPED)
// ======================================================
enum RunState { FOLLOW, STOPPED };
static RunState state = FOLLOW;

static uint32_t crossStartMs = 0;
static uint32_t runArmedAtMs = 0; // for CROSS_ARM_MS

// ======================================================
// 14) LIVE TUNING via Serial Monitor
// Commands (send with newline):
//   kp 2.0
//   ki 0
//   kd 0.30
//   base 450
//   turn 500
//   slew 60
//   sign -1
//   rec 340
//   kh 60000
//   khd 0
//   estr 0.010
//   cross 120
//   arm 800
//   go      (resume after STOP)
//   stop    (force stop)
//   zeroI   (reset integrator)
//   show
// ======================================================
static float lastYawErr_m = 0.0f;

static void printParams() {
  Serial.println("=== PARAMS ===");
  Serial.print("Kp="); Serial.println(Kp, 6);
  Serial.print("Ki="); Serial.println(Ki, 6);
  Serial.print("Kd="); Serial.println(Kd, 6);
  Serial.print("BASE_PWM="); Serial.println(BASE_PWM);
  Serial.print("TURN_LIMIT="); Serial.println(TURN_LIMIT);
  Serial.print("SLEW_STEP="); Serial.println(SLEW_STEP);
  Serial.print("STEER_SIGN="); Serial.println(STEER_SIGN);
  Serial.print("RECOVER_PWM="); Serial.println(RECOVER_PWM);
  Serial.print("Kh="); Serial.println(Kh, 3);
  Serial.print("Khd="); Serial.println(Khd, 3);
  Serial.print("eStraight_m="); Serial.println(eStraight_m, 6);
  Serial.print("CROSS_HOLD_MS="); Serial.println(CROSS_HOLD_MS);
  Serial.print("CROSS_ARM_MS="); Serial.println(CROSS_ARM_MS);
  Serial.print("STATE="); Serial.println(state == FOLLOW ? "FOLLOW" : "STOPPED");
  Serial.println("=============");
}

static void handleSerialTuning() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  String val = (sp < 0) ? ""   : line.substring(sp + 1);
  cmd.toLowerCase(); val.trim();

  if (cmd == "kp") { Kp = val.toFloat(); printParams(); }
  else if (cmd == "ki") { Ki = val.toFloat(); printParams(); }
  else if (cmd == "kd") { Kd = val.toFloat(); printParams(); }
  else if (cmd == "base") { BASE_PWM = clampi(val.toInt(), 0, PWM_MAX); printParams(); }
  else if (cmd == "turn") { TURN_LIMIT = clampi(val.toInt(), 0, PWM_MAX); printParams(); }
  else if (cmd == "slew") { SLEW_STEP = clampi(val.toInt(), 1, 400); printParams(); }
  else if (cmd == "sign") { STEER_SIGN = (val.toInt() >= 0) ? +1 : -1; printParams(); }
  else if (cmd == "rec") { RECOVER_PWM = clampi(val.toInt(), 0, PWM_MAX); printParams(); }
  else if (cmd == "kh") { Kh = val.toFloat(); printParams(); }
  else if (cmd == "khd") { Khd = val.toFloat(); printParams(); }
  else if (cmd == "go") {
    state = FOLLOW;
    crossStartMs = 0;
    runArmedAtMs = millis();
    Serial.println("GO -> FOLLOW");
    printParams();
  }
  else if (cmd == "stop") {
    state = STOPPED;
    stopMotors();
    Serial.println("STOP -> STOPPED");
    printParams();
  }
  else if (cmd == "zeroi") {
    // reset integrators/history
    lastYawErr_m = 0.0f;
    Serial.println("Integrator reset (I and heading history).");
  }
  else if (cmd == "show") { printParams(); }
  else {
    Serial.println("Unknown cmd. Use: kp ki kd base turn slew sign rec kh khd estr cross arm go stop zeroI show");
  }
}

// ======================================================
// 15) CONTROL STATE (PID)
// ======================================================
static float iErr = 0.0f;
static float ePrev = 0.0f;
static int cmdL = 0, cmdR = 0;

// ======================================================
// 16) SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Motor pins
  pinMode(L_INA, OUTPUT);
  pinMode(L_INB, OUTPUT);
  pinMode(R_INA, OUTPUT);

  // Strap pin handling for GPIO0
  pinMode(R_INB, INPUT_PULLUP);
  delay(10);
  pinMode(R_INB, OUTPUT);
  digitalWrite(R_INB, HIGH);

  // IR sensors (digital, LOW on black)
  pinMode(IR_L, INPUT_PULLUP);
  pinMode(IR_M, INPUT_PULLUP);
  pinMode(IR_R, INPUT_PULLUP);

  // PWM attach
  pwmAttach(L_PWM, CH_L, PWM_FREQ_HZ, PWM_RES_BITS);
  pwmAttach(R_PWM, CH_R, PWM_FREQ_HZ, PWM_RES_BITS);
  pwmWriteCh(CH_L, 0);
  pwmWriteCh(CH_R, 0);

  // Encoders
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrEncR, RISING);

  stopMotors();

  state = FOLLOW;
  runArmedAtMs = millis();

  printParams();

  Serial.println("READY. Use Serial Plotter: e_mm u L R base head sL sM sR yaw vL vR state");
  Serial.println("Tuning cmds: kp/ki/kd/base/turn/slew/sign/rec/kh/khd/estr/cross/arm/go/stop/zeroI/show");
}

// ======================================================
// 17) LOOP
// ======================================================
void loop() {
  handleSerialTuning();

  // STOPPED state: keep motors off, still stream plotter lines
  if (state == STOPPED) {
    stopMotors();
    // Plotter output
    Serial.print("e_mm:0 u:0 L:0 R:0 base:0 head:0 sL:0 sM:0 sR:0 yaw:0 vL:0 vR:0 state:0\n");
    delay(20);
    return;
  }

  // fixed-rate loop
  static uint32_t tPrev = micros();
  uint32_t now = micros();
  if ((uint32_t)(now - tPrev) < LOOP_DT_US) return;
  tPrev += LOOP_DT_US;

  // Read line
  LineReading lr = readLine();

  // Speed (debug + heading)
  static int32_t pL = 0, pR = 0;
  WheelSpeed ws = estimateSpeed(pL, pR);

  // ======================================================
  // CROSS-LINE DETECTION: require 111 for CROSS_HOLD_MS
  // - Only after CROSS_ARM_MS from "go/start"
  // ======================================================
  bool allBlack = (lr.bL && lr.bM && lr.bR);
  bool armed = (millis() - runArmedAtMs >= CROSS_ARM_MS);

  if (armed && allBlack) {
    if (crossStartMs == 0) crossStartMs = millis();
    if (millis() - crossStartMs >= CROSS_HOLD_MS) {
      stopMotors();
      state = STOPPED;
      Serial.println("CROSS DETECTED -> STOPPED (send 'go' to continue)");
      return;
    }
  } else {
    crossStartMs = 0;
  }

  // ======================================================
  // Lost line -> recovery spin + reset integrator (anti-windup)
  // ======================================================
  if (!lr.seen) {
    // reset I to avoid windup when blind
    iErr = 0.0f;
    ePrev = 0.0f;

    if (lastTurnDir == 0) lastTurnDir = +1;
    cmdL = -RECOVER_PWM * lastTurnDir;
    cmdR = +RECOVER_PWM * lastTurnDir;

    setMotor(L_INA, L_INB, CH_L, cmdL);
    setMotor(R_INA, R_INB, CH_R, cmdR);

    // Plotter output
    Serial.print("e_mm:0 u:0 L:");   Serial.print(cmdL);
    Serial.print(" R:");            Serial.print(cmdR);
    Serial.print(" base:0 head:0 ");
    Serial.print("sL:");           Serial.print(lr.bL);
    Serial.print(" sM:");          Serial.print(lr.bM);
    Serial.print(" sR:");          Serial.print(lr.bR);
    Serial.print(" yaw:0");
    Serial.print(" vL:");          Serial.print(ws.vL_mps, 3);
    Serial.print(" vR:");          Serial.print(ws.vR_mps, 3);
    Serial.print(" state:1\n");
    return;
  }

  // ======================================================
  // LINE PID (fixed dt)
  // ======================================================
  float dt = 1.0f / (float)LOOP_HZ;
  float e  = lr.err_m;

  iErr += e * dt;
  iErr = constrain(iErr, -0.20f, 0.20f);

  float de = (e - ePrev) / dt;
  ePrev = e;

  float uLine = (Kp * e) + (Ki * iErr) + (Kd * de);

  // meters -> PWM scaling
  int turnLine = (int)(uLine * 5000.0f);
  turnLine = clampi(turnLine, -TURN_LIMIT, TURN_LIMIT);

  // ======================================================
  // HEADING HOLD (ENCODER) on straight segments only
  // yawErr_m = (dR - dL) * M_PER_PULSE
  // ======================================================
  float yawErr_m = (float)(ws.dR - ws.dL) * M_PER_PULSE;
  float dyaw = (yawErr_m - lastYawErr_m) / dt;
  lastYawErr_m = yawErr_m;

  int headTurn = 0;
  if (fabs(e) < eStraight_m) {
    float uHead = (Kh * yawErr_m) + (Khd * dyaw);
    headTurn = (int)uHead;
    headTurn = clampi(headTurn, -TURN_LIMIT, TURN_LIMIT);
  }

  // Total turn
  int turn = (turnLine + headTurn) * STEER_SIGN;
  turn = clampi(turn, -TURN_LIMIT, TURN_LIMIT);

  // Speed reduce on large turns
  int base = BASE_PWM - (abs(turn) / 2);
  base = clampi(base, 200, BASE_PWM);

  int tgtL = clampi(base - turn, -PWM_MAX, PWM_MAX);
  int tgtR = clampi(base + turn, -PWM_MAX, PWM_MAX);

  // Slew limit (reaction speed)
  cmdL = slew(cmdL, tgtL, SLEW_STEP);
  cmdR = slew(cmdR, tgtR, SLEW_STEP);

  // Apply motors
  setMotor(L_INA, L_INB, CH_L, cmdL);
  setMotor(R_INA, R_INB, CH_R, cmdR);

  // ======================================================
  // SERIAL PLOTTER OUTPUT
  // Tools -> Serial Plotter
  // ======================================================
  Serial.print("e_mm:");  Serial.print((int)(e * 1000.0f));
  Serial.print(" u:");    Serial.print(turn);
  Serial.print(" L:");    Serial.print(cmdL);
  Serial.print(" R:");    Serial.print(cmdR);
  Serial.print(" base:"); Serial.print(base);
  Serial.print(" head:"); Serial.print(headTurn);
  Serial.print(" sL:");   Serial.print(lr.bL);
  Serial.print(" sM:");   Serial.print(lr.bM);
  Serial.print(" sR:");   Serial.print(lr.bR);
  Serial.print(" yaw:");  Serial.print(yawErr_m, 6);
  Serial.print(" vL:");   Serial.print(ws.vL_mps, 3);
  Serial.print(" vR:");   Serial.print(ws.vR_mps, 3);
  Serial.print(" state:1\n"); // 1=FOLLOW, 0=STOPPED (plotter-д амар)
}
