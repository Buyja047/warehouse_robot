/* =============================================================================
   MINI SUMO ROBOT  —  Arduino Nano + TB6612FNG + 5x digital IR + 2x QTR analog

   Ажиллах зарчим:
     D12 товч дарна -> QTR калибровк (хар талбай дээр) -> серво 90 -> 5 сек
     тоолол -> тулаан. Тулааны үед: цагаан хүрээ мэдэрвэл ЭХНИЙ ЭЭЛЖИНД зугтана,
     үгүй бол өрсөлдөгчөө хайж түлхнэ. Тулааны үед D12-г дахин дарвал зогсоно.

   ЧУХАЛ (Timer1):
     Nano-гийн Servo сан Timer1-ийг эзэлдэг ба D9/D10-ийн analogWrite-ийг
     унтраадаг. PWMB = D9 тул Servo.h ашиглаж БОЛОХГҮЙ. Тиймээс сервоны импульсийг
     энд гараар (bit-bang) үүсгэж байгаа — Timer1 чөлөөтэй, PWMB бүрэн ажиллана.

   ЧУХАЛ (A6/A7):
     A6, A7 нь зөвхөн ANALOG орц. lLine = A7 тул analogRead-аар л уншина
     (digitalRead / pinMode ажиллахгүй). Тиймээс лайн мэдрэгчид analogRead-тэй.
   ============================================================================= */

// ==========================================
// ПИН
// ==========================================
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

// ==========================================
// ТОХИРГОО — энд тохируулна
// ==========================================

// --- Мотор утасны чиглэл -----------------------------------------------------
#define MOTOR_A_IS_LEFT   1   // A суваг зүүн мотор бол 1, баруун бол 0
#define LEFT_INVERT       0   // зүүн дугуй буруу эргэвэл 1 болго
#define RIGHT_INVERT      0   // баруун дугуй буруу эргэвэл 1 болго

// --- Хурд (0..255) -----------------------------------------------------------
#define SPD_ATTACK      255   // шууд түлхэх
#define SPD_PUSH        235   // 45° мэдэрсэн үеийн түлхэлт
#define SPD_TURN        215   // өрсөлдөгч рүү эргэх
#define SPD_SEARCH      150   // хайлтын урагш
#define SPD_SEARCH_SPIN 175   // хайлтын эргэлт
#define SPD_ESC_BACK    225   // хүрээнээс ухрах
#define SPD_ESC_TURN    230   // хүрээнээс эргэх
#define PWM_MIN          45   // үүнээс бага PWM-д мотор эргэхгүй (үхмэл бүс)

// --- Хугацаа (ms) ------------------------------------------------------------
#define START_DELAY_MS  5000  // дүрмийн 5 сек. Хэрэггүй бол 0 болго
#define BRAKE_MS          40  // хүрээ мэдрэхэд эхлээд тоормослох
#define BACK_MS          230  // ухрах хугацаа (ХЭТ УРТ БОЛГОХГҮЙ! ард нь ирмэг байна)
#define TURN_90_MS       270  // нэг талын хүрээнээс эргэх
#define TURN_180_MS      460  // хоёр талаараа хүрээ дээр гарсан үед эргэх
#define ESC_SETTLE_MS     60  // зугтсаны дараа мэдрэгчийг тайвшруулах
#define SEARCH_SPIN_MS   550  // хайлт: эргэх үе шат
#define SEARCH_FWD_MS    450  // хайлт: урагш очих үе шат

// --- Мэдрэгчийн туйлшрал -----------------------------------------------------
#define ENEMY_ACTIVE    HIGH  // 5 IR мэдрэгч мэдэрсэн үед 1 (HIGH) өгнө
#define QTR_WHITE_IS_LOW   1  // QTR: цагаан дээр утга БАГА бол 1, ИХ бол 0

// --- Цагаан хүрээ таних (хар талбай дээрх цагаан толбыг үл тоох) --------------
#define AUTO_CALIBRATE     1  // эхлэхэд хар талбайг хэмжиж босго өөрөө тавина
#define WHITE_FACTOR    0.55  // босго = хар_дундаж * энэ. Бага = мэдрэмж бага
#define MIN_BLACK_MARGIN 120  // босго нь хараас дор хаяж ийм зайд байх ёстой
#define LINE_THR_L_DEF   450  // авто калибровк унтарсан/бүтэлгүйтсэн үеийн босго
#define LINE_THR_R_DEF   450
#define LINE_CONFIRM       4  // ийм олон УДААГИЙН ДАРААЛСАН уншилтад цагаан бол
                              // л жинхэнэ хүрээ гэж үзнэ (толбыг шүүнэ)
#define LINE_SAMPLE_US   700  // хоёр QTR-ийг ийм үетэйгээр уншина
// => батлах хугацаа ≈ LINE_CONFIRM * LINE_SAMPLE_US = ~2.8 ms.
//    Толбо байсаар байвал LINE_CONFIRM-ыг 5..7 болго. Хүрээгээ алдвал 2..3 болго.

// --- Серво -------------------------------------------------------------------
#define SERVO_START_ANGLE  90 // эхлэхэд явах өнцөг
#define SERVO_US_MIN      600 // 0°  импульс (сервогоо дагаж тохируул)
#define SERVO_US_MAX     2400 // 180° импульс
#define SERVO_SET_PULSES   35 // байрлалд хүргэх импульсийн тоо (~0.7 сек)
#define SERVO_REFRESH_MS    0 // 0 = тулааны үед импульс явуулахгүй (санал болгоно).
                              // Серво ачаа барих ёстой бол 20 болго — гэхдээ
                              // хайлтын үед бүр 1.5 ms саатал үүсгэнэ.

// --- Бусад -------------------------------------------------------------------
#define BUTTON_ACTIVE     LOW // INPUT_PULLUP тул дарахад LOW
#define ENABLE_STALL_BREAK  1 // удаан түлхээд хөдөлгөөнгүй бол өнцгөөр дахин дайрах
#define STALL_MS         2600 // ийм удаан "mid" дээр түлхвэл заль хийнэ
#define DEBUG_SERIAL        1 // 1 = Serial мэдээлэл (тулааны үед бараг хэвлэхгүй)

// ==========================================
// ДОТООД ТӨЛӨВ
// ==========================================
bool running = false;                 // тулаан явж байна уу

uint16_t thrL = LINE_THR_L_DEF;       // цагаан хүрээний босго (зүүн QTR)
uint16_t thrR = LINE_THR_R_DEF;       // цагаан хүрээний босго (баруун QTR)

uint8_t  hitL = 0, hitR = 0;          // дараалсан цагаан уншилтын тоолуур
bool     lineL = false, lineR = false;
uint32_t lastSampleUs = 0;

bool eL90, eL45, eMid, eR45, eR90, eAny;

int8_t   lastSeen   = 1;              // -1 = зүүн, +1 = баруун (сүүлд харсан тал)
int8_t   searchDir  = 1;
uint32_t searchStart = 0;
uint32_t pushStart   = 0;
uint32_t servoLastUs = 0;

// ==========================================
// МОТОР (TB6612FNG)
// ==========================================
static int clampPwm(int p) {
  if (p > 255) p = 255;
  if (p > 0 && p < PWM_MIN) p = PWM_MIN;   // үхмэл бүсийг давуулна
  return p;
}

void motorA(int s) {                        // s: -255..255
  bool fwd = (s >= 0);
  int p = clampPwm(abs(s));
  if (p == 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0); return; }
  digitalWrite(AIN1, fwd ? HIGH : LOW);
  digitalWrite(AIN2, fwd ? LOW  : HIGH);
  analogWrite(PWMA, p);
}

void motorB(int s) {
  bool fwd = (s >= 0);
  int p = clampPwm(abs(s));
  if (p == 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0); return; }
  digitalWrite(BIN1, fwd ? HIGH : LOW);
  digitalWrite(BIN2, fwd ? LOW  : HIGH);
  analogWrite(PWMB, p);
}

// left/right: -255..255 (эерэг = урагш)
void drive(int left, int right) {
#if LEFT_INVERT
  left = -left;
#endif
#if RIGHT_INVERT
  right = -right;
#endif
#if MOTOR_A_IS_LEFT
  motorA(left);  motorB(right);
#else
  motorA(right); motorB(left);
#endif
}

// dir: +1 = баруун тийш эргэх, -1 = зүүн тийш эргэх (байрандаа)
void spin(int8_t dir, int speed) {
  drive(dir > 0 ? speed : -speed, dir > 0 ? -speed : speed);
}

void brakeMotors() {                         // TB6612 богино тоормос
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255);
}

void coast() { drive(0, 0); }

// ==========================================
// СЕРВО (Timer1 хэрэглэхгүй, гараар импульс)
// ==========================================
void servoPulse(int angle) {
  int us = map(constrain(angle, 0, 180), 0, 180, SERVO_US_MIN, SERVO_US_MAX);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(us);
  digitalWrite(SERVO_PIN, LOW);
}

void servoGoTo(int angle) {                  // байрлалд хүргэнэ (setup-д ашиглана)
  for (uint8_t i = 0; i < SERVO_SET_PULSES; i++) { servoPulse(angle); delay(20); }
}

void servoRefresh() {                        // сонголтоор: барих импульс
#if SERVO_REFRESH_MS > 0
  if (millis() - servoLastUs >= SERVO_REFRESH_MS) {
    servoLastUs = millis();
    servoPulse(SERVO_START_ANGLE);
  }
#endif
}

// ==========================================
// ЛАЙН МЭДРЭГЧ (цагаан хүрээ)
// ==========================================
static inline bool isWhite(uint16_t v, uint16_t thr) {
#if QTR_WHITE_IS_LOW
  return v < thr;
#else
  return v > thr;
#endif
}

// Дараалсан LINE_CONFIRM уншилтад цагаан гарсан үед л ХҮРЭЭ гэж баталгаажна.
// Ингэснээр хар талбай дээрх сарнисан цагаан толбо (богино хугацааны цохилт)
// шүүгдэж, зөвхөн 2 см өргөн тасралтгүй хүрээ таарна.
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

// Робот ХАР талбай дээр байх ёстой үед дуудна.
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

  #if QTR_WHITE_IS_LOW
    long tL = (long)(blackL * WHITE_FACTOR);
    long tR = (long)(blackR * WHITE_FACTOR);
    if (blackL - tL < MIN_BLACK_MARGIN) tL = (long)blackL - MIN_BLACK_MARGIN;
    if (blackR - tR < MIN_BLACK_MARGIN) tR = (long)blackR - MIN_BLACK_MARGIN;
    // Хар талбай хэт тод байвал (буруу газар калибровк хийсэн) анхны утгыг барина
    thrL = (tL > 40) ? (uint16_t)tL : LINE_THR_L_DEF;
    thrR = (tR > 40) ? (uint16_t)tR : LINE_THR_R_DEF;
  #else
    long tL = (long)blackL + (long)((1023 - blackL) * (1.0 - WHITE_FACTOR));
    long tR = (long)blackR + (long)((1023 - blackR) * (1.0 - WHITE_FACTOR));
    if (tL - blackL < MIN_BLACK_MARGIN) tL = (long)blackL + MIN_BLACK_MARGIN;
    if (tR - blackR < MIN_BLACK_MARGIN) tR = (long)blackR + MIN_BLACK_MARGIN;
    thrL = (tL < 1000) ? (uint16_t)tL : LINE_THR_L_DEF;
    thrR = (tR < 1000) ? (uint16_t)tR : LINE_THR_R_DEF;
  #endif

  #if DEBUG_SERIAL
    Serial.print(F("CAL black L/R = ")); Serial.print(blackL); Serial.print('/'); Serial.print(blackR);
    Serial.print(F("   thr L/R = "));    Serial.print(thrL);   Serial.print('/'); Serial.println(thrR);
  #endif
#endif
  lineReset();
}

// ==========================================
// ӨРСӨЛДӨГЧИЙН МЭДРЭГЧ
// ==========================================
static inline bool ir(uint8_t pin) { return digitalRead(pin) == ENEMY_ACTIVE; }

void readEnemy() {
  eL90 = ir(left90);
  eL45 = ir(left45);
  eMid = ir(mid);
  eR45 = ir(right45);
  eR90 = ir(right90);
  eAny = eL90 || eL45 || eMid || eR45 || eR90;

  if      (eL45 || eL90) lastSeen = -1;
  else if (eR45 || eR90) lastSeen = +1;
}

// ==========================================
// ТОВЧ
// ==========================================
bool buttonDown() { return digitalRead(START_BUTTON) == BUTTON_ACTIVE; }

bool buttonPressed() {                       // товшилтыг нэг удаа буцаана
  if (!buttonDown()) return false;
  delay(25);
  if (!buttonDown()) return false;
  while (buttonDown()) { }                   // тавихыг хүлээнэ
  delay(25);
  return true;
}

// ==========================================
// ХӨДӨЛГӨӨНИЙ ТУСЛАХ
// ==========================================
// Тодорхой хугацаанд явна. Энэ хооронд ч QTR-ийг уншсаар байна (тоолуур шинэ
// байлгахын тулд), гэхдээ зугтах маневрыг таслахгүй.
void driveFor(int left, int right, uint16_t ms) {
  uint32_t t0 = millis();
  drive(left, right);
  while (millis() - t0 < ms) {
    lineTask();
    if (buttonDown()) { coast(); running = false; return; }
  }
}

// ==========================================
// ХҮРЭЭНЭЭС ЗУГТАХ — ХАМГИЙН ӨНДӨР ТЭРГҮҮЛЭХ ЭРХ
// ==========================================
void escapeEdge(bool hitLeft, bool hitRight) {
#if DEBUG_SERIAL
  Serial.print(F("EDGE ")); Serial.print(hitLeft ? 'L' : '-'); Serial.println(hitRight ? 'R' : '-');
#endif

  brakeMotors();
  delay(BRAKE_MS);

  driveFor(-SPD_ESC_BACK, -SPD_ESC_BACK, BACK_MS);
  if (!running) return;

  if (hitLeft && hitRight) {
    // Урд талаараа бүтэн хүрээн дээр гарсан -> эргэж төв рүү хар
    driveFor(lastSeen > 0 ?  SPD_ESC_TURN : -SPD_ESC_TURN,
             lastSeen > 0 ? -SPD_ESC_TURN :  SPD_ESC_TURN, TURN_180_MS);
    searchDir = lastSeen;
  } else if (hitLeft) {
    // Хүрээ зүүн урд байна -> баруун тийш эргэнэ
    driveFor(SPD_ESC_TURN, -SPD_ESC_TURN, TURN_90_MS);
    searchDir = +1;
  } else {
    driveFor(-SPD_ESC_TURN, SPD_ESC_TURN, TURN_90_MS);
    searchDir = -1;
  }
  if (!running) return;

  brakeMotors();
  delay(ESC_SETTLE_MS);
  lineReset();
  searchStart = millis();
  pushStart = 0;
}

// ==========================================
// ДАЙРАЛТ
// ==========================================
void attack() {
  if (eMid) {
    if (eL45 && !eR45)      drive(SPD_PUSH * 3 / 5, SPD_PUSH);   // бага зэрэг зүүн
    else if (eR45 && !eL45) drive(SPD_PUSH, SPD_PUSH * 3 / 5);   // бага зэрэг баруун
    else                    drive(SPD_ATTACK, SPD_ATTACK);       // яг урд -> бүх хүчээр
    if (pushStart == 0) pushStart = millis();
    return;
  }
  pushStart = 0;

  if (eL45 && eR45)   { drive(SPD_ATTACK, SPD_ATTACK); return; } // бараг урд, ойрхон
  if (eL45)           { drive(SPD_TURN / 4, SPD_TURN); return; } // зүүн тийш нум
  if (eR45)           { drive(SPD_TURN, SPD_TURN / 4); return; }
  if (eL90)           { spin(-1, SPD_TURN); return; }            // байрандаа зүүн
  if (eR90)           { spin(+1, SPD_TURN); return; }
}

// Удаан түлхээд хөдлөхгүй бол бага зэрэг ухраад өнцгөөр дахин дайрна
void stallBreak() {
#if ENABLE_STALL_BREAK
  #if DEBUG_SERIAL
    Serial.println(F("STALL"));
  #endif
  driveFor(-SPD_ESC_BACK, -SPD_ESC_BACK, 150);
  if (!running) return;
  driveFor(lastSeen > 0 ?  SPD_TURN : -SPD_TURN,
           lastSeen > 0 ? -SPD_TURN :  SPD_TURN, 120);
  pushStart = 0;
  lineReset();
#endif
}

// ==========================================
// ХАЙЛТ
// ==========================================
void search() {
  uint32_t t = millis() - searchStart;

  if (t < SEARCH_SPIN_MS) {
    spin(searchDir, SPD_SEARCH_SPIN);          // сүүлд харсан тал руугаа эргэнэ
  } else if (t < (uint32_t)SEARCH_SPIN_MS + SEARCH_FWD_MS) {
    drive(SPD_SEARCH, SPD_SEARCH);             // талбайг тэмтчинэ
  } else {
    searchStart = millis();
    searchDir = -searchDir;                    // дараагийн эргэлт эсрэг тийш
  }
  servoRefresh();
}

// ==========================================
// SETUP / LOOP
// ==========================================
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
#endif

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);                    // драйверыг сэрээнэ
  coast();

  // Өрсөлдөгчийн 5 мэдрэгч (A0, A2..A5 нь дижитал орц болж чадна)
  pinMode(left90,  INPUT);
  pinMode(left45,  INPUT);
  pinMode(mid,     INPUT);
  pinMode(right45, INPUT);
  pinMode(right90, INPUT);
  // rLine (A1), lLine (A7) — зөвхөн analogRead, pinMode шаардлагагүй

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  pinMode(START_BUTTON, INPUT_PULLUP);

#if DEBUG_SERIAL
  Serial.println(F("MINI SUMO ready. D12 dar."));
#endif
}

void loop() {
  // ---------- ЗОГСОЛТ / ЭХЛҮҮЛЭХ ----------
  if (!running) {
    coast();
    if (buttonPressed()) {
      calibrateLine();                         // робот ХАР талбай дээр байна
      servoGoTo(SERVO_START_ANGLE);            // серво 90°
      servoLastUs = millis();

#if DEBUG_SERIAL
      Serial.println(F("START..."));
#endif
      uint32_t t0 = millis();                  // дүрмийн хүлээлт
      while (millis() - t0 < START_DELAY_MS) {
        if (buttonDown()) { while (buttonDown()) {} return; }   // цуцлах
      }

      lineReset();
      readEnemy();
      searchDir   = lastSeen;
      searchStart = millis();
      pushStart   = 0;
      running     = true;
    }
    return;
  }

  // ---------- ТУЛААН ----------
  lineTask();

  // 1) Хүрээ — юунаас ч түрүүнд
  if (lineL || lineR) { escapeEdge(lineL, lineR); return; }

  // 2) Аюулгүйн зогсоолт
  if (buttonPressed()) {
    coast();
    running = false;
#if DEBUG_SERIAL
    Serial.println(F("STOP"));
#endif
    return;
  }

  // 3) Өрсөлдөгч
  readEnemy();

  if (eAny) {
    attack();
#if ENABLE_STALL_BREAK
    if (pushStart && millis() - pushStart > STALL_MS) stallBreak();
#endif
  } else {
    if (pushStart) { searchStart = millis(); searchDir = lastSeen; pushStart = 0; }
    search();
  }
}
