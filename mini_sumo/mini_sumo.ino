/* =============================================================================
   MINI SUMO - STATE MACHINE + ХУРДНЫ УДИРДЛАГА
   Чиглэл: 1 (Урагш), 0 (Тоормос), -1 (Ухрах)
   Хурд:   PWMA/PWMB пин дээр analogWrite (0..255)

   ЧУХАЛ (Timer1): Nano дээр Servo сан Timer1-ийг эзэлж D9/D10-ийн analogWrite-ийг
   унтраадаг. PWMB = D9 тул Servo.h ашиглаж БОЛОХГҮЙ — серво импульсийг гараар
   үүсгэсэн (setup-д зөвхөн нэг удаа), Timer1 чөлөөтэй үлдэнэ.

   ЧУХАЛ (A6/A7): lLine = A7 нь зөвхөн аналог орц. analogRead-аар л уншина.
   ============================================================================= */

#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 8
#define BIN1 6
#define BIN2 7
#define PWMB 9

#define left90  A0
#define rLine   A1
#define right90 A2
#define right45 A3
#define mid     A4
#define left45  A5
#define lLine   A7

#define SERVO_PIN 10
#define START_BUTTON 12

// =============================================================================
// ХУРДНЫ ТОХИРГОО (0..255).  255 = өмнөх бүтэн хурд.
// Бүгдийг нэг дор өөрчлөх бол SPEED_SCALE-ийг л мушги.
// =============================================================================
#define SPEED_SCALE     1.00  // 1.00 = доорх утгууд яг хэвээр, 0.80 = 20% удаан
                              // ЭНЭ БОЛ ГОЛ ТОХИРУУЛГА. Илүү удаан бол 0.70 гэх мэт.

#define SPD_CHARGE       110  // эхний сохор дайралт      (~43%)
#define SPD_ATTACK       120  // mid: яг урдаас түлхэх     (~47%)
#define SPD_CURVE        100  // 45°: хурц нум             (~39%)
#define SPD_SPIN          95  // 90°: байрандаа эргэх      (~37%)
#define SPD_SEARCH        90  // алдсан үеийн эргэлт       (~35%)
#define SPD_BACK         105  // хүрээнээс ухрах           (~41%)
#define SPD_ESC_TURN     110  // хүрээнээс зайлж эргэх     (~43%)

#define PWM_MIN           55  // үүнээс доош PWM-д мотор огт эргэхгүй (үхмэл бүс)
                              // Мотор чичрээд эргэхгүй бол ЭНИЙГ өсгө (65, 75...)

// --- PWM ШАЛГАХ ГОРИМ --------------------------------------------------------
// 1 болговол товч дархад тулаан эхлэхгүй, оронд нь хоёр мотор 5 шатаар
// (60 -> 100 -> 150 -> 200 -> 255) тус бүр 2.5 сек эргэнэ.
// РОБОТЫГ ӨРГӨЖ БАРЬ эсвэл тавцан дээр тавь — дугуй чөлөөтэй эргэх ёстой.
//   Шат бүрт хурд өөр байвал  -> PWM зөв ажиллаж байна, SPEED_SCALE-ээр тааруул.
//   Бүгд ижил хурдтай байвал  -> PWMA/PWMB утас драйверт очихгүй байна эсвэл
//                                драйвер дээрх PWM пин VCC рүү холбоостой (техник асуудал).
#define TEST_SPEED_MODE    0

// --- Мотор утасны чиглэл -----------------------------------------------------
// Энэ роботын мотор урвуу холбоостой: AIN1=LOW/AIN2=HIGH үед УРАГШ явдаг.
// Тиймээс хоёулаа 1. Ингэснээр кодын бүх газарт 1 = урагш гэсэн НЭГ конвенц
// үйлчилж, setMotors() дотроо тэмдгийг эргүүлж өгнө.
// Хэрэв нэг дугуй нь эсрэг эргэвэл ЗӨВХӨН тэр талынхыг нь 0 болго.
#define LEFT_INVERT        1
#define RIGHT_INVERT       1

// =============================================================================
// ХУГАЦААНЫ ТОХИРГОО
// АНХААР: хурдыг өөрчилвөл эргэлтийн хугацааг ЗААВАЛ дахин тааруул.
//         Хурд буурвал ижил өнцгийг эргэхэд ИЛҮҮ УДААН хугацаа хэрэгтэй.
// =============================================================================
#define START_DELAY_MS  5000
#define FIRST_CHARGE_MS 1500  // эхний чигээрээ дайрах хугацаа
#define BRAKE_MS          60
#define BACK_MS          190  // хурд дахин буурсан тул 130 -> 190
#define TURN_90_MS       520  // хурд дахин буурсан тул 360 -> 520
#define TURN_180_MS      900  // хурд дахин буурсан тул 620 -> 900
#define ESC_SETTLE_MS     40

// =============================================================================
// МЭДРЭГЧИЙН ТОХИРГОО
// =============================================================================
#define ENEMY_ACTIVE    HIGH
#define AUTO_CALIBRATE     1
#define WHITE_FACTOR    0.30
#define MIN_BLACK_MARGIN 120
#define LINE_CONFIRM       7
#define LINE_SAMPLE_US   700

// --- Серво -------------------------------------------------------------------
#define SERVO_START_ANGLE 90
#define SERVO_US_MIN     600
#define SERVO_US_MAX    2400
#define SERVO_SET_PULSES  35

// --- Дотоод төлвүүд ----------------------------------------------------------
bool running = false;
bool firstMove = false;
uint32_t firstMoveStart = 0;

uint16_t thrL = 450, thrR = 450;
uint8_t  hitL = 0, hitR = 0;
bool     lineL = false, lineR = false;
uint32_t lastSampleUs = 0;

bool eL90, eL45, eMid, eR45, eR90, eAny;
int8_t lastSeen = 1;

// ==========================================
// МОТОР УДИРДЛАГА — чиглэл (1/0/-1) + хурд (PWM)
// ==========================================
static uint8_t scaleSpeed(int pwm) {
  long v = (long)(pwm * SPEED_SCALE);
  if (v > 255) v = 255;
  if (v > 0 && v < PWM_MIN) v = PWM_MIN;      // үхмэл бүсийг давуулна
  if (v < 0) v = 0;
  return (uint8_t)v;
}

// l, r: 1 = урагш, -1 = ухрах, 0 = тоормос.  pwm: 0..255
void setMotors(int l, int r, int pwm) {
  uint8_t p = scaleSpeed(pwm);

#if LEFT_INVERT
  l = -l;
#endif
#if RIGHT_INVERT
  r = -r;
#endif

  // Зүүн мотор (A)
  if (l == 0) {                                       // богино тоормос
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255);
  } else {
    digitalWrite(AIN1, (l > 0) ? HIGH : LOW);
    digitalWrite(AIN2, (l > 0) ? LOW  : HIGH);
    analogWrite(PWMA, p);
  }

  // Баруун мотор (B)
  if (r == 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255);
  } else {
    digitalWrite(BIN1, (r > 0) ? HIGH : LOW);
    digitalWrite(BIN2, (r > 0) ? LOW  : HIGH);
    analogWrite(PWMB, p);
  }
}

void brakeMotors() { setMotors(0, 0, 0); }

void coastMotors() {                                   // чөлөөтэй зогсолт
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

// ==========================================
// PWM ШАЛГАХ ГОРИМ
// ==========================================
#if TEST_SPEED_MODE
void speedTest() {
  const int steps[] = { 60, 100, 150, 200, 255 };
  for (uint8_t i = 0; i < 5; i++) {
    setMotors(1, 1, steps[i]);                 // хоёулаа урагш
    delay(2500);
    coastMotors();
    delay(800);                                // шат хооронд завсар
  }
}
#endif

// ==========================================
// СЕРВО (Timer1 хэрэглэхгүй, гараар импульс)
// ==========================================
void servoPulse(int angle) {
  int us = map(constrain(angle, 0, 180), 0, 180, SERVO_US_MIN, SERVO_US_MAX);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

void servoGoTo(int angle) {
  for (uint8_t i = 0; i < SERVO_SET_PULSES; i++) { servoPulse(angle); delay(20); }
}

// ==========================================
// ЛАЙН МЭДРЭГЧ
// ==========================================
static inline bool isWhite(uint16_t v, uint16_t thr) { return v < thr; }

void lineTask() {
  if ((uint32_t)(micros() - lastSampleUs) < LINE_SAMPLE_US) return;
  lastSampleUs = micros();

  uint16_t l = analogRead(lLine);
  uint16_t r = analogRead(rLine);

  if (isWhite(l, thrL)) { if (hitL < 250) hitL++; } else hitL = 0;
  if (isWhite(r, thrR)) { if (hitR < 250) hitR++; } else hitR = 0;

  lineL = (hitL >= LINE_CONFIRM);
  lineR = (hitR >= LINE_CONFIRM);
}

void lineReset() { hitL = hitR = 0; lineL = lineR = false; lastSampleUs = micros(); }

void calibrateLine() {
#if AUTO_CALIBRATE
  uint32_t sumL = 0, sumR = 0;
  const uint16_t N = 200;
  for (uint16_t i = 0; i < N; i++) {
    sumL += analogRead(lLine);
    sumR += analogRead(rLine);
    delay(2);
  }
  uint16_t blackL = sumL / N;
  uint16_t blackR = sumR / N;

  long tL = (long)(blackL * WHITE_FACTOR);
  long tR = (long)(blackR * WHITE_FACTOR);
  if (blackL - tL < MIN_BLACK_MARGIN) tL = (long)blackL - MIN_BLACK_MARGIN;
  if (blackR - tR < MIN_BLACK_MARGIN) tR = (long)blackR - MIN_BLACK_MARGIN;
  thrL = (tL > 40) ? (uint16_t)tL : 450;
  thrR = (tR > 40) ? (uint16_t)tR : 450;
#endif
  lineReset();
}

// ==========================================
// МЭДРЭГЧ УНШИХ
// ==========================================
static inline bool ir(uint8_t pin) { return digitalRead(pin) == ENEMY_ACTIVE; }

void readEnemy() {
  eL90 = ir(left90); eL45 = ir(left45); eMid = ir(mid); eR45 = ir(right45); eR90 = ir(right90);
  eAny = eL90 || eL45 || eMid || eR45 || eR90;
  if      (eL45 || eL90) lastSeen = -1;
  else if (eR45 || eR90) lastSeen = +1;
}

// ==========================================
// ТУСЛАХ ФУНКЦҮҮД
// ==========================================
bool buttonDown() { return digitalRead(START_BUTTON) == LOW; }

bool buttonPressed() {
  if (!buttonDown()) return false;
  delay(25);
  if (!buttonDown()) return false;
  while (buttonDown()) { }
  delay(25);
  return true;
}

void driveFor(int left, int right, int pwm, uint16_t ms) {
  uint32_t t0 = millis();
  setMotors(left, right, pwm);
  while (millis() - t0 < ms) {
    lineTask();
    if (buttonDown()) { coastMotors(); running = false; return; }
  }
}

// ==========================================
// ХҮРЭЭНЭЭС ЗУГТАХ
// ==========================================
void escapeEdge(bool hitLeft, bool hitRight) {
  brakeMotors(); delay(BRAKE_MS);              // тас зуурах

  driveFor(-1, -1, SPD_BACK, BACK_MS);         // хоёулаа ухрах
  if (!running) return;

  if (hitLeft && hitRight) {
    driveFor(lastSeen > 0 ?  1 : -1,
             lastSeen > 0 ? -1 :  1, SPD_ESC_TURN, TURN_180_MS);
  } else if (hitLeft) {
    driveFor(1, -1, SPD_ESC_TURN, TURN_90_MS); // баруун эргэх
    lastSeen = 1;
  } else {
    driveFor(-1, 1, SPD_ESC_TURN, TURN_90_MS); // зүүн эргэх
    lastSeen = -1;
  }
  if (!running) return;

  brakeMotors(); delay(ESC_SETTLE_MS);
  lineReset();

  firstMove = false;                           // сохор дайралтыг цуцлана
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  coastMotors();

  pinMode(left90, INPUT); pinMode(left45, INPUT); pinMode(mid, INPUT);
  pinMode(right45, INPUT); pinMode(right90, INPUT);
  // rLine (A1), lLine (A7) — зөвхөн analogRead

  pinMode(SERVO_PIN, OUTPUT); digitalWrite(SERVO_PIN, LOW);
  pinMode(START_BUTTON, INPUT_PULLUP);
}

// ==========================================
// LOOP - STATE MACHINE
// ==========================================
void loop() {
  if (!running) {
    coastMotors();
    if (buttonPressed()) {
#if TEST_SPEED_MODE
      speedTest();                             // тулаан эхлэхгүй, зөвхөн PWM шалгана
      return;
#endif
      calibrateLine();                         // робот ХАР талбай дээр байна
      servoGoTo(SERVO_START_ANGLE);            // серво 90°

      uint32_t t0 = millis();
      while (millis() - t0 < START_DELAY_MS) {
        if (buttonDown()) { while (buttonDown()) {} return; }
      }

      lineReset();
      readEnemy();
      firstMove = true;
      firstMoveStart = millis();
      running = true;
    }
    return;
  }

  // 1. Хүрээг шалгах (үргэлж хамгийн түрүүнд)
  lineTask();
  if (lineL || lineR) { escapeEdge(lineL, lineR); return; }

  // 2. Аюулгүйн зогсоолт
  if (buttonPressed()) { coastMotors(); running = false; return; }

  // 3. Эхний сохор дайралт
  if (firstMove) {
    if (millis() - firstMoveStart < FIRST_CHARGE_MS) {
      setMotors(1, 1, SPD_CHARGE);             // зөвхөн чигээрээ
      return;
    } else {
      firstMove = false;
    }
  }

  // 4. Өрсөлдөгчийн төлвөөр дайрах
  //    ТЭМДГИЙН КОНВЕНЦ: 1 = урагш, -1 = ухрах, 0 = тоормос
  readEnemy();

  if (eMid) {
    setMotors(1, 1, SPD_ATTACK);               // яг урд: хоёулаа урагшаа
  }
  else if (eL45) {
    setMotors(0, 1, SPD_CURVE);                // зүүн урд: зүүнээ түгжээд зүүн тийш нумална
  }
  else if (eR45) {
    setMotors(1, 0, SPD_CURVE);                // баруун урд: баруунаа түгжээд баруун тийш
  }
  else if (eL90) {
    setMotors(-1, 1, SPD_SPIN);                // зүүн зах: байрандаа ЗҮҮН эргэх
  }
  else if (eR90) {
    setMotors(1, -1, SPD_SPIN);                // баруун зах: байрандаа БАРУУН эргэх
  }
  else {
    // 5. Алдсан бол сүүлд харсан тал руугаа эргэж хайх
    if (lastSeen > 0) setMotors(1, -1, SPD_SEARCH);   // баруун тийш
    else              setMotors(-1, 1, SPD_SEARCH);   // зүүн тийш
  }
}
