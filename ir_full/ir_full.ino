#include <Arduino.h>
#include <math.h>

// ===================== MECHANICS =====================
const float WHEEL_DIAMETER_M = 0.1395f;
const int   ENCODER_PPR      = 460;
const float TARGET_DIST_M    = 2.0f;

const float METERS_PER_PULSE = (PI * WHEEL_DIAMETER_M) / (float)ENCODER_PPR;

// ===================== SPEED PROFILE =====================
const int      BASE_PWM      = 450;
const int      MIN_PWM       = 260;
const uint32_t RAMP_UP_MS    = 1500;
const float    DECEL_DIST_M  = 0.6f;

// ===================== MOTOR TRIM =====================
int M1_TRIM = 0;   // Left motor offset
int M2_TRIM = 0;   // Right motor offset

// ===================== TURN CONTROLLER (PWM-domain) =====================
float Kp_turn_pwm = 320.0f;
float Kd_turn_pwm = 110.0f;

float ERR_ALPHA    = 0.50f;
int   MIN_TURN_PWM = 70;
float CORR_RATIO   = 0.90f;

const int   TURN_SLOW_THRESH = 120;
const float TURN_SLOW_SCALE  = 0.75f;

// ===================== SENSOR PINS (HW-871) =====================
// S1..S5 = таны логик ашиглах дараалал (S1 S2 S3 S4 S5)
static const int S1 = 6;   // OUT1
static const int S2 = 7;   // OUT2
static const int S3 = 16;  // OUT3 center
static const int S4 = 15;  // OUT4
static const int S5 = 17;  // OUT5

// HW-871 ихэнхдээ хар дээр LOW
static const int LINE_ACTIVE_LEVEL = LOW;

// ===================== SENSOR STABILIZER =====================
const int SENSOR_SAMPLES   = 5;     // 5 эсвэл 7 болгож болно
const int SENSOR_SAMPLE_US = 250;   // 200~600us турш

const int MASK_CONFIRM_CYCLES = 2;  // mask фликкер дарна

// ===================== PATTERN CONTROL CONFIG =====================
// 1) Хэрвээ зүүн/баруун урвуу эргээд байвал:
//    STEER_INVERT = -1 болгочих
int STEER_INVERT = +1;

// 2) Хэрвээ S1..S5 байрлал чинь физикээрээ эсрэг (S1 баруун тал, S5 зүүн тал гэх мэт) байвал:
//    MIRROR_MASK = true болгочих
bool MIRROR_MASK = false;

// Pattern correction хэмжээ (basePWM-ээс)
const float PATTERN_CORR_SCALE = 0.65f;
const int   PATTERN_CORR_MIN   = 110;

// Таны тусгай дүрэм:
// - 11011 -> чигээрээ
// - 10111 -> эргэнэ
// - 11101 -> эсрэг тийш эргэнэ
// - 01110 -> “эсрэгээрээ байх ёстой” => энд бид "сүүлд эргэсэн чиглэлийн эсрэг" гэж хийв
//   (Хэрвээ өмнө нь эргээгүй бол DIR_01110_DEFAULT ашиглана)
const int DIR_LEFT  = +1;   // +1 => corr positive (turn “left” логик)
const int DIR_RIGHT = -1;

int RULE_10111_DIR = DIR_LEFT;     // хүсвэл DIR_RIGHT болгож болно
int RULE_11101_DIR = DIR_RIGHT;    // эсрэг чиглэл
int DIR_01110_DEFAULT = DIR_RIGHT; // 01110-д default (хэрвээ өмнөх чиглэл байхгүй үед)

// ===================== MOTOR PINS =====================
#define M1INA   38
#define M1INB   37
#define M1PWM   36

#define M2INA   35
#define M2INB   0     // ⚠ boot strap pin (та OK гэсэн тул өөрчлөхгүй)
#define M2PWM   45

#define ENC1_PIN  4
#define ENC2_PIN  5

// ===================== PWM (LEDC core 3.x) =====================
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;
const uint32_t PWM_MAX = (1u << PWM_RES) - 1;

// ===================== GLOBALS =====================
volatile long enc1Count = 0;
volatile long enc2Count = 0;

bool isFinished = false;
uint32_t startMs = 0;

float lastRawErr   = 0.0f;
float err_f        = 0.0f;
float err_f_prev   = 0.0f;

uint32_t lastLineSeenMs = 0;
const uint32_t NO_LINE_TIMEOUT_MS = 200;

// mask stability
uint8_t lastMask = 0;
int     maskStreak = 0;
uint8_t stableMask = 0;

// last forced turn dir (+1 / -1 / 0)
int lastForcedDir = 0;

// ===================== INTERRUPTS =====================
void IRAM_ATTR enc1ISR() { enc1Count++; }
void IRAM_ATTR enc2ISR() { enc2Count++; }

static inline uint32_t clampDuty(int v) {
  if (v < 0) v = 0;
  if ((uint32_t)v > PWM_MAX) v = PWM_MAX;
  return (uint32_t)v;
}

// forward only
void setMotor1(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, (int)PWM_MAX);
  digitalWrite(M1INA, HIGH); digitalWrite(M1INB, LOW);
  ledcWrite(M1PWM, clampDuty(pwmValue));
}
void setMotor2(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, (int)PWM_MAX);
  digitalWrite(M2INA, HIGH); digitalWrite(M2INB, LOW);
  ledcWrite(M2PWM, clampDuty(pwmValue));
}

static inline float clamp01(float x) { return (x < 0) ? 0 : (x > 1) ? 1 : x; }
static inline bool isActiveLevel(int pin) { return (digitalRead(pin) == LINE_ACTIVE_LEVEL); }

uint8_t mirror5(uint8_t m) {
  // bit0<->bit4, bit1<->bit3
  uint8_t r = 0;
  r |= ((m >> 0) & 1) << 4;
  r |= ((m >> 1) & 1) << 3;
  r |= ((m >> 2) & 1) << 2;
  r |= ((m >> 3) & 1) << 1;
  r |= ((m >> 4) & 1) << 0;
  return r;
}

// Read stable sensors: s[0]=S1 ... s[4]=S5, mask bit0=S1 ... bit4=S5
void readSensorsStable(bool s[5], uint8_t &mask) {
  int cnt[5] = {0,0,0,0,0};

  for (int k = 0; k < SENSOR_SAMPLES; k++) {
    cnt[0] += isActiveLevel(S1);
    cnt[1] += isActiveLevel(S2);
    cnt[2] += isActiveLevel(S3);
    cnt[3] += isActiveLevel(S4);
    cnt[4] += isActiveLevel(S5);
    delayMicroseconds(SENSOR_SAMPLE_US);
  }

  mask = 0;
  for (int i = 0; i < 5; i++) {
    s[i] = (cnt[i] > SENSOR_SAMPLES / 2);
    if (s[i]) mask |= (1u << i);
  }
}

float computeLineErrorFromStates(const bool s[5], bool &lineSeen) {
  // S1..S5 weights
  const int w[5] = {-2, -1, 0, 1, 2};

  int sumW = 0, cnt = 0;
  for (int i = 0; i < 5; i++) {
    if (s[i]) { sumW += w[i]; cnt++; }
  }

  if (cnt == 0) {
    lineSeen = false;
    return (lastRawErr >= 0) ? 2.5f : -2.5f;
  }

  lineSeen = true;
  return (float)sumW / (float)cnt;
}

void printMaskS1toS5(uint8_t m) {
  // S1..S5 буюу bit0..bit4
  for (int i = 0; i < 5; i++) Serial.print((m >> i) & 1);
}

int patternCorrFromBase(int basePWM) {
  int c = (int)(basePWM * PATTERN_CORR_SCALE);
  if (c < PATTERN_CORR_MIN) c = PATTERN_CORR_MIN;
  return c;
}

// Pattern lookup command codes
// -4..+4 : forced turn intensity (negative/right, positive/left) (STEER_INVERT applied later)
//  0     : straight
//  99    : use PID (rawErr-based)
static const int8_t PATTERN_CMD[32] = {
  /*00000*/ 99,  // no line -> handled separately (search), keep 99 as placeholder
  /*00001*/ +4,  // S1 only -> hard to S1 side
  /*00010*/ +2,  // S2 only -> medium
  /*00011*/ +3,  // S1+S2 -> hard
  /*00100*/  0,  // S3 only -> straight
  /*00101*/ 99,  // S1+S3 weird -> PID
  /*00110*/ +1,  // S2+S3 -> slight
  /*00111*/ +2,  // S1+S2+S3 -> medium
  /*01000*/ -2,  // S4 only -> medium opposite
  /*01001*/ 99,  // S1+S4 -> PID
  /*01010*/ 99,  // S2+S4 -> PID
  /*01011*/ 99,  // S1+S2+S4 -> PID
  /*01100*/ -1,  // S3+S4 -> slight
  /*01101*/ 99,  // S1+S3+S4 -> PID
  /*01110*/ 99,  // SPECIAL: handled separately (01110)
  /*01111*/ -2,  // S1..S4 (wide) -> medium (junction-ish)
  /*10000*/ -4,  // S5 only -> hard
  /*10001*/ 99,
  /*10010*/ 99,
  /*10011*/ 99,
  /*10100*/ 99,
  /*10101*/ 99,
  /*10110*/ 99,
  /*10111*/ 99,  // SPECIAL: handled separately (10111)
  /*11000*/ -3,  // S4+S5 -> hard
  /*11001*/ 99,
  /*11010*/ 99,
  /*11011*/  0,  // SPECIAL but also straight
  /*11100*/ -2,  // S3+S4+S5 -> medium
  /*11101*/ 99,  // SPECIAL: handled separately (11101)
  /*11110*/ -2,  // S2..S5 (wide) -> medium
  /*11111*/  0   // all on (intersection) -> straight (та хүсвэл stop болгож болно)
};

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  pinMode(S5, INPUT_PULLUP);

  pinMode(M1INA, OUTPUT); pinMode(M1INB, OUTPUT);
  pinMode(M2INA, OUTPUT); pinMode(M2INB, OUTPUT);

  if (!ledcAttach(M1PWM, PWM_FREQ, PWM_RES) || !ledcAttach(M2PWM, PWM_FREQ, PWM_RES)) {
    Serial.println("LEDC attach failed.");
    while (1) delay(100);
  }
  ledcWrite(M1PWM, 0);
  ledcWrite(M2PWM, 0);

  pinMode(ENC1_PIN, INPUT_PULLUP);
  pinMode(ENC2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_PIN), enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_PIN), enc2ISR, RISING);

  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  interrupts();

  setMotor1(0);
  setMotor2(0);
  delay(800);

  startMs = millis();
  lastLineSeenMs = startMs;

  Serial.println("LF started (FULL PATTERN TABLE).");
}

void loop() {
  static uint32_t lastLoop = 0;
  const uint32_t LOOP_DT_MS = 20;
  uint32_t now = millis();
  if (now - lastLoop < LOOP_DT_MS) return;
  lastLoop = now;

  if (isFinished) {
    setMotor1(0); setMotor2(0);
    return;
  }

  // ====== Distance ======
  long c1, c2;
  noInterrupts();
  c1 = enc1Count;
  c2 = enc2Count;
  interrupts();

  float dist1 = (float)c1 * METERS_PER_PULSE;
  float dist2 = (float)c2 * METERS_PER_PULSE;
  float avgDist = (dist1 + dist2) * 0.5f;

  if (avgDist >= TARGET_DIST_M) {
    setMotor1(0); setMotor2(0);
    isFinished = true;
    Serial.println("STOP: reached target distance.");
    return;
  }

  // ====== Speed profile ======
  float startFactor = clamp01((now - startMs) / (float)RAMP_UP_MS);
  float remaining   = TARGET_DIST_M - avgDist;
  float endFactor   = clamp01(remaining / DECEL_DIST_M);
  float profile     = (endFactor < startFactor) ? endFactor : startFactor;
  int basePWM = (int)(MIN_PWM + (BASE_PWM - MIN_PWM) * profile);

  // ====== Sensors (stable) ======
  bool s[5];
  uint8_t mask;
  readSensorsStable(s, mask);

  // optional mirror (if sensor physical order reversed)
  if (MIRROR_MASK) {
    mask = mirror5(mask);
    // mirror s[] too (for rawErr)
    bool t0=s[0], t1=s[1];
    s[0]=s[4]; s[1]=s[3]; s[3]=t1; s[4]=t0;
    // s[2] unchanged
  }

  // mask confirm debounce
  if (mask == lastMask) maskStreak++;
  else { lastMask = mask; maskStreak = 0; }

  if (maskStreak >= MASK_CONFIRM_CYCLES) stableMask = mask;

  bool lineSeen;
  float rawErr = computeLineErrorFromStates(s, lineSeen);
  lastRawErr = rawErr;

  if (lineSeen) lastLineSeenMs = now;

  bool longNoLine = (!lineSeen && (now - lastLineSeenMs > NO_LINE_TIMEOUT_MS));
  if (longNoLine) basePWM = MIN_PWM;

  // ====== Control ======
  int pwmL = basePWM, pwmR = basePWM;
  float corr = 0.0f;

  int corrMax = (int)(basePWM * CORR_RATIO);
  if (corrMax < 90) corrMax = 90;

  // ---- Special rules FIRST (your requested logic) ----
  // stableMask is in S1..S5 bit order: bit0=S1 ... bit4=S5
  if (stableMask == 0b11011) {
    // 11011 -> straight
    corr = 0.0f;
    err_f = 0.0f;
    err_f_prev = 0.0f;
  }
  else if (stableMask == 0b10111) {
    // 10111 -> turn (forced)
    int pc = min(patternCorrFromBase(basePWM), corrMax);
    int dir = RULE_10111_DIR;
    lastForcedDir = dir;
    corr = (float)(STEER_INVERT * dir * pc);
  }
  else if (stableMask == 0b11101) {
    // 11101 -> opposite turn (forced)
    int pc = min(patternCorrFromBase(basePWM), corrMax);
    int dir = RULE_11101_DIR;
    lastForcedDir = dir;
    corr = (float)(STEER_INVERT * dir * pc);
  }
  else if (stableMask == 0b01110) {
    // 01110 -> "эсрэгээрээ" => lastForcedDir-ийн эсрэг, байхгүй бол default
    int pc = min(patternCorrFromBase(basePWM), corrMax);
    int dir = (lastForcedDir != 0) ? (-lastForcedDir) : DIR_01110_DEFAULT;
    corr = (float)(STEER_INVERT * dir * pc);
  }
  else if (stableMask == 0b00000) {
    // no line -> search turn (last error sign дагана)
    int pc = min(patternCorrFromBase(basePWM), corrMax);
    int dir = (lastRawErr >= 0) ? DIR_LEFT : DIR_RIGHT;
    corr = (float)(STEER_INVERT * dir * pc);
  }
  else {
    // ---- General pattern table ----
    int8_t cmd = PATTERN_CMD[stableMask & 0x1F];

    if (cmd == 0) {
      corr = 0.0f;
      err_f = 0.0f;
      err_f_prev = 0.0f;
    }
    else if (cmd == 99) {
      // PID fallback
      err_f = ERR_ALPHA * rawErr + (1.0f - ERR_ALPHA) * err_f;
      float derr = (err_f - err_f_prev);
      err_f_prev = err_f;

      corr = (Kp_turn_pwm * err_f) + (Kd_turn_pwm * derr);
      corr = constrain(corr, -(float)corrMax, (float)corrMax);

      if (!longNoLine) {
        if (fabs(err_f) > 0.35f && fabs(corr) < MIN_TURN_PWM) {
          corr = (corr >= 0) ? (float)MIN_TURN_PWM : -(float)MIN_TURN_PWM;
        }
      } else {
        corr = (rawErr >= 0) ? (float)corrMax : -(float)corrMax;
      }
    }
    else {
      // forced discrete turn: cmd (-4..+4)
      int pc = min(patternCorrFromBase(basePWM), corrMax);
      float scale = fabs((float)cmd) / 4.0f;  // 0.25..1.0
      int dir = (cmd > 0) ? DIR_LEFT : DIR_RIGHT;
      corr = (float)(STEER_INVERT * dir) * (pc * scale);
    }
  }

  // turn үед хурд бууруулах
  if (abs((int)corr) > TURN_SLOW_THRESH) {
    basePWM = max(MIN_PWM, (int)(basePWM * TURN_SLOW_SCALE));
  }

  pwmL = basePWM - (int)corr + M1_TRIM;
  pwmR = basePWM + (int)corr + M2_TRIM;

  pwmL = constrain(pwmL, 0, (int)PWM_MAX);
  pwmR = constrain(pwmR, 0, (int)PWM_MAX);

  setMotor1(pwmL);
  setMotor2(pwmR);

  // ====== Debug ======
  static uint32_t lastPlot = 0;
  if (now - lastPlot >= 60) {
    lastPlot = now;

    Serial.print("Mask(S1..S5):");
    printMaskS1toS5(stableMask);
    Serial.print(", RawErr:");
    Serial.print(rawErr, 2);
    Serial.print(", Corr:");
    Serial.print((int)corr);
    Serial.print(", L:");
    Serial.print(pwmL);
    Serial.print(", R:");
    Serial.print(pwmR);
    Serial.print(", Dist:");
    Serial.println(avgDist, 3);
  }
}
