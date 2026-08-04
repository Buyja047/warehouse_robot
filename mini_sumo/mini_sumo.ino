/* =============================================================================
   MINI SUMO - STATE MACHINE + ХУРДНЫ УДИРДЛАГА
   Чиглэл: 1 (Урагш), 0 (Тоормос), -1 (Ухрах)

   ХУРДНЫ ХОЁР ГОРИМ (SPEED_MODE):
     0 = PWMA/PWMB пин дээр analogWrite. Зөвхөн тэр утас драйверт ХОЛБООТОЙ үед.
     1 = чиглэлийн пинийг өөрөө таслах (software PWM). AIN1/AIN2, BIN1/BIN2-ыг
         "явах <-> тоормос" хооронд 250 Гц-ээр сэлгэнэ. TB6612-т IN1=H,IN2=H нь
         тоормос тул үр дүн нь PWM-тэй яг ижил.
         >>> ЭНЭ ГОРИМ PWM УТАС ХОЛБООТОЙ Ч, ХОЛБООГҮЙ Ч АЖИЛЛАНА. <<<
         Тиймээс үндсэн утга нь 1.

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
// ХУРДНЫ ГОРИМ
// =============================================================================
#define SPEED_MODE          1   // 1 = чиглэлийн пин таслах (ЗӨВЛӨЖ БАЙНА)
                                // 0 = PWMA/PWMB дээр analogWrite
#define SOFT_PWM_PERIOD_US 4000 // software PWM-ийн үе (4000us = 250 Гц)

// =============================================================================
// ХУРДНЫ ТОХИРГОО (0..255).  255 = бүтэн хурд.
// =============================================================================
#define SPEED_SCALE     1.00  // ГОЛ ТОХИРУУЛГА: бүх хурдыг нэг дор мушгина
                              // 0.70 = 30% удаан, 1.20 = 20% хурдан

#define SPD_CHARGE       110  // эхний сохор дайралт      (~43%)
#define SPD_ATTACK       120  // mid: яг урдаас түлхэх     (~47%)
#define SPD_CURVE        100  // 45°: хурц нум             (~39%)
#define SPD_SPIN          95  // 90°: байрандаа эргэх      (~37%)
#define SPD_SEARCH        90  // алдсан үеийн эргэлт       (~35%)
#define SPD_BACK         105  // хүрээнээс ухрах           (~41%)
#define SPD_ESC_TURN     110  // хүрээнээс зайлж эргэх     (~43%)
#define SPD_ESC_FWD      110  // эргэсний дараа төв рүү чигээрээ явах (~43%)

#define PWM_MIN           55  // үүнээс доош мотор огт эргэхгүй (үхмэл бүс)
                              // Мотор чичрээд эргэхгүй бол ЭНИЙГ өсгө (65, 75...)

// --- ХУРД ШАЛГАХ ГОРИМ -------------------------------------------------------
// 1 болговол товч дархад тулаан эхлэхгүй, оронд нь хоёр мотор 5 шатаар
// (60 -> 100 -> 150 -> 200 -> 255) тус бүр 2.5 сек эргэнэ.
// РОБОТЫГ ӨРГӨЖ БАРЬ — дугуй чөлөөтэй эргэх ёстой.
//   Шат ахих тусам хурдсвал -> хурдны удирдлага ажиллаж байна.
//   Бүгд ижил хурдтай бол    -> SPEED_MODE-ыг эсрэгээр нь солиод дахин үз.
#define TEST_SPEED_MODE    0

// --- Мотор утасны чиглэл -----------------------------------------------------
// Энэ роботын мотор урвуу холбоостой: AIN1=LOW/AIN2=HIGH үед УРАГШ явдаг.
// Тиймээс хоёулаа 1. Кодын бүх газарт 1 = урагш гэсэн НЭГ конвенц үйлчилнэ.
// Нэг дугуй нь эсрэг эргэвэл ЗӨВХӨН тэр талынхыг нь 0 болго.
#define LEFT_INVERT        1
#define RIGHT_INVERT       1

// =============================================================================
// ХУГАЦААНЫ ТОХИРГОО
// АНХААР: хурдыг өөрчилвөл эргэлтийн хугацааг ЗААВАЛ дахин тааруул.
// =============================================================================
#define START_DELAY_MS  5000
#define FIRST_CHARGE_MS 1500  // эхний чигээрээ дайрах хугацаа
#define BRAKE_MS          60
#define BACK_MS          190
#define TURN_90_MS       520
#define TURN_180_MS      900
#define ESC_SETTLE_MS     40
#define ESC_FWD_MS       450  // эргэсний дараа төв рүү чигээрээ явах хугацаа
                              // ХЭТ УРТ БОЛГОХГҮЙ — нөгөө талын хүрээ рүү дүрэлзэнэ

// Төв рүү явж байхад өрсөлдөгч мэдрэгдвэл шууд дайралт руу шилжих үү?
// 1 = тийм (санал болгоно), 0 = ESC_FWD_MS дуустал сохроор явна
#define ESC_FWD_ABORT_ON_ENEMY 1

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

// Моторын хүссэн төлөв:  1 = урагш, -1 = ухрах, 0 = тоормос, 2 = чөлөөт
int8_t  cmdA = 2, cmdB = 2;
uint8_t dutyA = 0, dutyB = 0;

// ==========================================
// МОТОРЫН ПИН БИЧИХ (илүүц бичилтийг алгасана)
// ==========================================
static int8_t lastA = 99, lastB = 99;

static void applyA(int8_t st) {
  if (st == lastA) return;
  lastA = st;
  if      (st ==  1) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else if (st == -1) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  else if (st ==  0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); } // тоормос
  else               { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  } // чөлөөт
}

static void applyB(int8_t st) {
  if (st == lastB) return;
  lastB = st;
  if      (st ==  1) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else if (st == -1) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else if (st ==  0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); }
  else               { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  }
}

// ==========================================
// SOFTWARE PWM — байнга дуудагдах ёстой
// ==========================================
void motorTick() {
#if SPEED_MODE == 1
  uint16_t phase = (uint16_t)(micros() % SOFT_PWM_PERIOD_US);

  if (cmdA == 1 || cmdA == -1) {
    uint16_t onA = (uint16_t)((uint32_t)SOFT_PWM_PERIOD_US * dutyA / 255);
    applyA((phase < onA) ? cmdA : 0);          // ассан үе = явна, унтарсан үе = тоормос
  } else {
    applyA(cmdA);
  }

  if (cmdB == 1 || cmdB == -1) {
    uint16_t onB = (uint16_t)((uint32_t)SOFT_PWM_PERIOD_US * dutyB / 255);
    applyB((phase < onB) ? cmdB : 0);
  } else {
    applyB(cmdB);
  }
#endif
}

// ==========================================
// МОТОР УДИРДЛАГА
// ==========================================
static uint8_t scaleSpeed(int pwm) {
  long v = (long)(pwm * SPEED_SCALE);
  if (v > 255) v = 255;
  if (v > 0 && v < PWM_MIN) v = PWM_MIN;       // үхмэл бүсийг давуулна
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

  int8_t a = (l == 0) ? 0 : (l > 0 ? 1 : -1);
  int8_t b = (r == 0) ? 0 : (r > 0 ? 1 : -1);

#if SPEED_MODE == 0
  applyA(a);  analogWrite(PWMA, (a == 0) ? 255 : p);
  applyB(b);  analogWrite(PWMB, (b == 0) ? 255 : p);
#else
  cmdA = a; dutyA = p;
  cmdB = b; dutyB = p;
  motorTick();
#endif
}

void brakeMotors() { setMotors(0, 0, 0); }

void coastMotors() {
#if SPEED_MODE == 0
  applyA(2); analogWrite(PWMA, 0);
  applyB(2); analogWrite(PWMB, 0);
#else
  cmdA = 2; cmdB = 2;
  motorTick();
#endif
}

// Мотор эргэсээр байх ёстой үеийн хүлээлт
void tickDelay(uint16_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) motorTick();
}

// ==========================================
// ХУРД ШАЛГАХ ГОРИМ
// ==========================================
#if TEST_SPEED_MODE
void speedTest() {
  const int steps[] = { 60, 100, 150, 200, 255 };
  for (uint8_t i = 0; i < 5; i++) {
    setMotors(1, 1, steps[i]);                 // хоёулаа урагш
    tickDelay(2500);
    coastMotors();
    tickDelay(800);                            // шат хооронд завсар
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
  while (buttonDown()) { motorTick(); }
  delay(25);
  return true;
}

// Зугтах маневрын үе шат: хугацаа дуустал заавал гүйцэтгэнэ
void driveFor(int left, int right, int pwm, uint16_t ms) {
  uint32_t t0 = millis();
  setMotors(left, right, pwm);
  while (millis() - t0 < ms) {
    motorTick();
    lineTask();
    if (buttonDown()) { coastMotors(); running = false; return; }
  }
}

// Төв рүү явах: хүрээ (эсвэл өрсөлдөгч) илэрвэл хугацаанаас нь өмнө тасална
void driveForward(int pwm, uint16_t ms) {
  uint32_t t0 = millis();
  setMotors(1, 1, pwm);
  while (millis() - t0 < ms) {
    motorTick();
    lineTask();
    if (lineL || lineR) return;                // дахин хүрээ — loop зугтаана
    if (buttonDown()) { coastMotors(); running = false; return; }
#if ESC_FWD_ABORT_ON_ENEMY
    readEnemy();
    if (eAny) return;                          // өрсөлдөгч олдлоо — дайралт руу
#endif
  }
}

// ==========================================
// ХҮРЭЭНЭЭС ЗУГТАХ
// ==========================================
void escapeEdge(bool hitLeft, bool hitRight) {
  brakeMotors(); tickDelay(BRAKE_MS);          // тас зуурах

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

  brakeMotors(); tickDelay(ESC_SETTLE_MS);
  lineReset();                                 // хүрээнээс саллаа, тоолуур цэвэрлэнэ

  // Эргэж дууссан -> төв рүү чигээрээ явна.
  // Дахин хүрээ таарвал эсвэл өрсөлдөгч илэрвэл дундаас нь тасарна.
  driveForward(SPD_ESC_FWD, ESC_FWD_MS);

  firstMove = false;                           // сохор дайралтыг цуцлана
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

#if SPEED_MODE == 1
  digitalWrite(PWMA, HIGH);                    // PWM пин байнга нээлттэй,
  digitalWrite(PWMB, HIGH);                    // хурдыг чиглэлийн пинээр таслана
#endif

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
  motorTick();                                 // software PWM-ийг тэжээнэ

  if (!running) {
    coastMotors();
    if (buttonPressed()) {
#if TEST_SPEED_MODE
      speedTest();                             // тулаан эхлэхгүй, зөвхөн хурд шалгана
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
