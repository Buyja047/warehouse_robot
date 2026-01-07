#include <Arduino.h>

// ===================== MECHANICS =====================
const float WHEEL_DIAMETER_M = 0.1395f;
const int   ENCODER_PPR      = 460;
const float TARGET_DIST_M    = 50.0f;

const float METERS_PER_PULSE = (PI * WHEEL_DIAMETER_M) / (float)ENCODER_PPR;

// ===================== SPEED PROFILE =====================
const int      BASE_PWM      = 520;
const int      MIN_PWM       = 260;
const uint32_t RAMP_UP_MS    = 1500;
const float    DECEL_DIST_M  = 0.6f;

// ===================== TURN CONTROLLER (PWM-domain) =====================
// Error ~ -2..+2  (5 sensor weighted average)
float Kp_turn_pwm = 220.0f;    // PWM per 1.0 error  (эхэнд 180~280 хооронд турш)
float Kd_turn_pwm = 140.0f;    // PWM per delta-error (хэт савлавал бууруул)

float ERR_ALPHA   = 0.45f;     // error EMA filter (0.3~0.6)
int   MIN_TURN_PWM = 35;       // corr бага үед үрэлт давуулах доод засвар
float CORR_RATIO  = 0.75f;     // corrMax = basePWM*ratio (reverse үүсгэхгүй)

// ===================== HW-871 SENSOR PINS =====================
static const int S1 = 6;
static const int S2 = 7;
static const int S3 = 15;
static const int S4 = 16;
static const int S5 = 17;

// Хар зураас дээр LOW байвал LOW (ихэнхдээ тийм)
static const int LINE_ACTIVE_LEVEL = LOW;

// ===================== MOTOR PINS =====================
#define M1INA   38
#define M1INB   37
#define M1PWM   36

#define M2INA   35
#define M2INB   0     // ⚠ boot strap pin
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

// line tracking
float lastRawErr = 0.0f;
float err_f = 0.0f;
float err_f_prev = 0.0f;

uint32_t lastLineSeenMs = 0;
const uint32_t NO_LINE_TIMEOUT_MS = 200; // 0.2s

// Interrupts
void IRAM_ATTR enc1ISR() { enc1Count++; }
void IRAM_ATTR enc2ISR() { enc2Count++; }

static inline uint32_t clampDuty(int v) {
  if (v < 0) v = 0;
  if ((uint32_t)v > PWM_MAX) v = PWM_MAX;
  return (uint32_t)v;
}

// forward only (тогтвортой эргэлт)
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
static inline bool isOnLine(int pin) { return (digitalRead(pin) == LINE_ACTIVE_LEVEL); }

// returns raw error (-2..+2), sets lineSeen, outputs mask bit0..bit4
float computeLineError(bool &lineSeen, uint8_t &mask) {
  const int w[5] = {-2, -1, 0, 1, 2};
  bool s[5] = {
    isOnLine(S1), isOnLine(S2), isOnLine(S3), isOnLine(S4), isOnLine(S5)
  };

  mask = 0;
  for (int i = 0; i < 5; i++) if (s[i]) mask |= (1u << i);

  int sumW = 0, cnt = 0;
  for (int i = 0; i < 5; i++) {
    if (s[i]) { sumW += w[i]; cnt++; }
  }

  if (cnt == 0) {
    lineSeen = false;
    return (lastRawErr >= 0) ? 2.5f : -2.5f; // хайлт чиглэл
  }

  lineSeen = true;
  return (float)sumW / (float)cnt;
}

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

  // ====== IMPORTANT: reset distance counts ======
  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  interrupts();

  setMotor1(0);
  setMotor2(0);
  delay(800);

  startMs = millis();
  lastLineSeenMs = startMs;

  Serial.println("LF+STOP@5m (PWM-turn) started.");
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
    Serial.println("STOP: reached 5.0m");
    return;
  }

  // ====== Speed profile ======
  float startFactor = clamp01((now - startMs) / (float)RAMP_UP_MS);
  float remaining   = TARGET_DIST_M - avgDist;
  float endFactor   = clamp01(remaining / DECEL_DIST_M);
  float profile     = (endFactor < startFactor) ? endFactor : startFactor;

  int basePWM = (int)(MIN_PWM + (BASE_PWM - MIN_PWM) * profile);

  // ====== Line sense ======
  bool lineSeen;
  uint8_t mask;
  float rawErr = computeLineError(lineSeen, mask);
  lastRawErr = rawErr;

  if (lineSeen) lastLineSeenMs = now;

  bool longNoLine = (!lineSeen && (now - lastLineSeenMs > NO_LINE_TIMEOUT_MS));
  if (longNoLine) {
    basePWM = MIN_PWM; // алдагдвал удаашруулж хайна
  }

  // ====== Filtered error + D term (PWM domain) ======
  err_f = ERR_ALPHA * rawErr + (1.0f - ERR_ALPHA) * err_f;
  float derr = (err_f - err_f_prev);     // dt-гүй (scale тогтвортой)
  err_f_prev = err_f;

  float corr = (Kp_turn_pwm * err_f) + (Kd_turn_pwm * derr);

  // corr clamp relative to basePWM
  int corrMax = (int)(basePWM * CORR_RATIO);
  if (corrMax < 80) corrMax = 80;
  corr = constrain(corr, -(float)corrMax, (float)corrMax);

  // minimum correction when off-center (Mask!=center)
  if (!longNoLine) {
    if (fabs(err_f) > 0.35f && fabs(corr) < MIN_TURN_PWM) {
      corr = (corr >= 0) ? MIN_TURN_PWM : -MIN_TURN_PWM;
    }
  } else {
    // strong search turn when line missing
    corr = (rawErr >= 0) ? (float)corrMax : -(float)corrMax;
  }

  // ====== Motor commands ======
  int pwmL = basePWM - (int)corr;   // left
  int pwmR = basePWM + (int)corr;   // right

  pwmL = constrain(pwmL, 0, (int)PWM_MAX);
  pwmR = constrain(pwmR, 0, (int)PWM_MAX);

  setMotor1(pwmL);
  setMotor2(pwmR);

  // ====== Debug ======
  static uint32_t lastPlot = 0;
  if (now - lastPlot >= 50) {
    lastPlot = now;

    Serial.print("Dist:");     Serial.print(avgDist, 3); Serial.print(",");
    Serial.print("Base:");     Serial.print(basePWM);    Serial.print(",");
    Serial.print("Errx100:");  Serial.print((int)(rawErr * 100)); Serial.print(",");
    Serial.print("Corr:");     Serial.print((int)corr);  Serial.print(",");
    Serial.print("Mask:");     Serial.print(mask);       Serial.print(",");
    Serial.print("L:");        Serial.print(pwmL);       Serial.print(",");
    Serial.print("R:");        Serial.println(pwmR);
  }
}
