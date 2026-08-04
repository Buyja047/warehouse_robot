/* =============================================================================
   MINI SUMO - 3 НЭЭЛТИЙН ТАКТИК + BLUETOOTH + STARTER МОДУЛЬ
   Чиглэл: 1 (Урагш), 0 (Тоормос), -1 (Ухрах)
   Хурд:   PWMA/PWMB пин дээр analogWrite (0..255), PWM_MAX-аар хязгаарлана

   ЧУХАЛ (Timer1): Nano дээр Servo сан Timer1-ийг эзэлж D9/D10-ийн analogWrite-ийг
   унтраадаг. PWMB = D9 тул Servo.h ашиглаж БОЛОХГҮЙ — серво импульсийг гараар
   үүсгэсэн (setup-д зөвхөн нэг удаа), Timer1 чөлөөтэй үлдэнэ.

   ЧУХАЛ (A6/A7): lLine = A7 нь зөвхөн аналог орц. analogRead-аар л уншина.

   ЧУХАЛ (Bluetooth): HC-05/06 нь D0(RX)/D1(TX) буюу техник хангамжийн Serial
   дээр сууна. Модуль залгаастай үед Arduino IDE-ээс код АЧААЛЛАХГҮЙ —
   ачаалахын өмнө BT модулийн RX/TX-ийг салга.
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

// --- ШИНЭ ПИН ----------------------------------------------------------------
#define STARTER_PIN   2       // starter модулийн дохио (сул байсан пин)
// Bluetooth: D0 = RX, D1 = TX (техник хангамжийн Serial)

// =============================================================================
// STARTER МОДУЛЬ
// =============================================================================
#define STARTER_ENABLED    1  // модуль ЗАЛГААГҮЙ бол 0 болго (эс бөгөөс пин
                              // хөвж робот өөрөө зогсоно)
#define STARTER_GO   HIGH     // модуль "ЯВ" гэхэд өгөх түвшин (LOW бол сольж өг)
#define STARTER_PULLUP     1  // ил задгай коллектор гаралттай модульд 1

// =============================================================================
// BLUETOOTH
// =============================================================================
#define BT_ENABLED         1
#define BT_BAUD         9600  // HC-06 ихэвчлэн 9600, HC-05 нь 38400 байж болно

/*  BLUETOOTH КОМАНДУУД (терминалаас нэг үсэг илгээнэ):
      1 эсвэл L : ЗҮҮН талаар дайрах тактик
      2 эсвэл S : ЧИГЭЭРЭЭ дайрах тактик
      3 эсвэл R : БАРУУН талаар дайрах тактик
      G         : тулаан эхлүүлэх (starter модулийн оронд)
      X         : яаралтай зогсоох
      D         : QTR утга дамжуулахыг асаах/унтраах (босго тааруулахад)
      ?         : одоогийн төлөв харах
*/

// =============================================================================
// НЭЭЛТИЙН ТАКТИК
// =============================================================================
#define STRAT_STRAIGHT 0
#define STRAT_LEFT     1
#define STRAT_RIGHT    2
#define STRAT_DEFAULT  STRAT_STRAIGHT   // BT-ээр сонгоогүй үеийн анхны тактик

#define OPEN_TURN_MS     180  // хажуу тийш эргэх хугацаа (нээлтийн 1-р шат)
#define OPEN_DASH_MS     420  // хажуугаар давших хугацаа (2-р шат)
#define OPEN_BACK_MS     200  // төв рүү эргэж харах хугацаа (3-р шат)
#define OPEN_STRAIGHT_MS 500  // чигээрээ дайрах хугацаа

// =============================================================================
// ХУРДНЫ ТОХИРГОО (0..255)
// =============================================================================
#define SPEED_SCALE     0.60  // бүх хурдыг нэг дор мушгина
#define PWM_MAX          100  // ХАТУУ ДЭЭД ХЯЗГААР. Үүнээс дээш хурд гарахгүй.
                              // (Хурдыг бүхэлд нь өсгөх бол ЭНИЙГ өсгө, 100 -> 150)

#define SPD_CHARGE       200  // нээлтийн давшилт
#define SPD_ATTACK       200  // mid: яг урдаас түлхэх
#define SPD_CURVE        150  // 45°: хурц нум
#define SPD_SPIN         140  // 90°: байрандаа эргэх
#define SPD_SEARCH       135  // өрсөлдөгчийг алдсан үеийн эргэлт
#define SPD_OPEN_TURN    180  // нээлтийн эргэлт

// ЗУГТАХ ХУРД нь ДАЙРАХ ХУРДНААС БАГА БАЙЖ БОЛОХГҮЙ — эс бөгөөс өөрийн
// импульсээрээ хүрээнээс гарна. Тиймээс SPD_ATTACK-тай тэнцүү байлгав.
#define SPD_BACK         200  // хүрээнээс ухрах
#define SPD_ESC_TURN     200  // хүрээнээс зайлж эргэх
#define SPD_ESC_FWD      180  // эргэсний дараа төв рүү явах

#define PWM_MIN           55  // үүнээс доош мотор огт эргэхгүй (үхмэл бүс)

// --- Мотор утасны чиглэл -----------------------------------------------------
// Энэ роботын мотор урвуу холбоостой: AIN1=LOW/AIN2=HIGH үед УРАГШ явдаг.
// Нэг дугуй нь эсрэг эргэвэл ЗӨВХӨН тэр талынхыг нь 0 болго.
#define LEFT_INVERT        1
#define RIGHT_INVERT       1

// =============================================================================
// ХУГАЦААНЫ ТОХИРГОО
// =============================================================================
#define START_DELAY_MS  5000
#define BRAKE_MS          60

// Ухралт: тогтмол хугацаа биш, ЦАГААНААС БҮРЭН САЛТАЛ ухарна.
#define BACK_MIN_MS      130  // хамгийн багадаа ийм хугацаа ухарна
#define BACK_MAX_MS      450  // хамгийн ихдээ (хамгаалалт — ард нь ирмэг байна)
#define BACK_CLEAR_MS     90  // цагаан алга болсноос хойш ийм удаан хүлээж баталгаажна

#define TURN_90_MS       360
#define TURN_180_MS      620
#define ESC_SETTLE_MS     40
#define ESC_FWD_MS       380  // эргэсний дараа төв рүү чигээрээ явах

// =============================================================================
// МЭДРЭГЧИЙН ТОХИРГОО
// =============================================================================
#define ENEMY_ACTIVE    HIGH
#define AUTO_CALIBRATE     1

// WHITE_FACTOR: босго = хар_дундаж * энэ утга.
// 0.30 нь ХЭТ ХАТУУ байсан — хар 800 бол босго 240 болж, QTR цагаан дээр
// 250-350 уншдаг тул хүрээ огт танигдахгүй. 0.55 нь найдвартай эхлэл.
// 'D' команд илгээж бодит утгаа хараад тааруул.
#define WHITE_FACTOR    0.55
#define MIN_BLACK_MARGIN 120
#define LINE_CONFIRM       5  // дараалсан цагаан уншилт (5 * 700us = 3.5 ms)
#define LINE_SAMPLE_US   700

// --- Серво -------------------------------------------------------------------
#define SERVO_START_ANGLE 90
#define SERVO_US_MIN     600
#define SERVO_US_MAX    2400
#define SERVO_SET_PULSES  35

// --- Дотоод төлвүүд ----------------------------------------------------------
bool running = false;

uint8_t strategy = STRAT_DEFAULT;
bool btStartReq = false;
bool btStopReq  = false;
bool dbgStream  = false;
uint32_t lastDbg = 0;

bool prevStarterGo = false;

uint16_t thrL = 450, thrR = 450;
uint8_t  hitL = 0, hitR = 0;
bool     lineL = false, lineR = false;
uint32_t lastSampleUs = 0;

bool eL90, eL45, eMid, eR45, eR90, eAny;
int8_t lastSeen = 1;

// ==========================================
// МОТОР УДИРДЛАГА
// ==========================================
static uint8_t scaleSpeed(int pwm) {
  long v = (long)(pwm * SPEED_SCALE);
  if (v > PWM_MAX) v = PWM_MAX;               // хатуу дээд хязгаар
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

// Ямар нэг цагаан уншилт байна уу (баталгаажаагүй ч)
static inline bool anyWhite() { return (hitL > 0) || (hitR > 0); }

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

#if BT_ENABLED
  Serial.print(F("CAL black=")); Serial.print(blackL); Serial.print('/'); Serial.print(blackR);
  Serial.print(F(" thr="));      Serial.print(thrL);   Serial.print('/'); Serial.println(thrR);
#endif
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
// BLUETOOTH
// ==========================================
const char* stratName() {
  if (strategy == STRAT_LEFT)  return "LEFT";
  if (strategy == STRAT_RIGHT) return "RIGHT";
  return "STRAIGHT";
}

void btReport() {
#if BT_ENABLED
  Serial.print(F("strat=")); Serial.print(stratName());
  Serial.print(F(" run="));  Serial.print(running ? 1 : 0);
  Serial.print(F(" thr="));  Serial.print(thrL); Serial.print('/'); Serial.println(thrR);
#endif
}

void btTask() {
#if BT_ENABLED
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': case 'l': case 'L': strategy = STRAT_LEFT;     btReport(); break;
      case '2': case 's': case 'S': strategy = STRAT_STRAIGHT; btReport(); break;
      case '3': case 'r': case 'R': strategy = STRAT_RIGHT;    btReport(); break;
      case 'g': case 'G': btStartReq = true;  break;
      case 'x': case 'X': btStopReq  = true;  break;
      case 'd': case 'D': dbgStream = !dbgStream; break;
      case '?':           btReport(); break;
      default: break;
    }
  }

  if (dbgStream && millis() - lastDbg >= 300) {
    lastDbg = millis();
    Serial.print(F("L=")); Serial.print(analogRead(lLine));
    Serial.print(F(" R=")); Serial.print(analogRead(rLine));
    Serial.print(F(" thr=")); Serial.print(thrL); Serial.print('/'); Serial.println(thrR);
  }
#endif
}

// ==========================================
// STARTER МОДУЛЬ / ТОВЧ
// ==========================================
bool starterGo() {
#if STARTER_ENABLED
  return digitalRead(STARTER_PIN) == STARTER_GO;
#else
  return false;
#endif
}

bool buttonDown() { return digitalRead(START_BUTTON) == LOW; }

bool buttonPressed() {
  if (!buttonDown()) return false;
  delay(25);
  if (!buttonDown()) return false;
  while (buttonDown()) { }
  delay(25);
  return true;
}

// Тулааны үед зогсоох дохио ирсэн үү
bool shouldStop() {
  btTask();
  if (btStopReq) { btStopReq = false; return true; }
  if (buttonDown()) return true;
#if STARTER_ENABLED
  if (!starterGo()) return true;               // модуль "ЗОГС" болголоо
#endif
  return false;
}

// ==========================================
// ХӨДӨЛГӨӨН
// ==========================================
// Зугтах маневрын үе шат: хугацаа дуустал заавал гүйцэтгэнэ
void driveFor(int left, int right, int pwm, uint16_t ms) {
  uint32_t t0 = millis();
  setMotors(left, right, pwm);
  while (millis() - t0 < ms) {
    lineTask();
    if (shouldStop()) { coastMotors(); running = false; return; }
  }
}

// Дайралт/нээлтийн үе шат: хүрээ эсвэл өрсөлдөгч илэрвэл тасална.
// true буцаавал таслагдсан гэсэн үг.
bool moveFor(int left, int right, int pwm, uint16_t ms, bool abortOnEnemy) {
  uint32_t t0 = millis();
  setMotors(left, right, pwm);
  while (millis() - t0 < ms) {
    lineTask();
    if (lineL || lineR) return true;                 // хүрээ — loop зугтаана
    if (shouldStop()) { coastMotors(); running = false; return true; }
    if (abortOnEnemy) { readEnemy(); if (eAny) return true; }
  }
  return false;
}

// Цагаанаас БҮРЭН салтал ухарна. Энэ нь тогтмол хугацаанаас найдвартай:
// эргэхээсээ өмнө хүрээнээс гарсан эсэхээ баталгаажуулна.
void reverseUntilClear() {
  uint32_t t0 = millis();
  uint32_t clearSince = 0;
  setMotors(-1, -1, SPD_BACK);

  while (millis() - t0 < BACK_MAX_MS) {
    lineTask();
    if (shouldStop()) { coastMotors(); running = false; return; }

    if (anyWhite()) {
      clearSince = 0;                                // дахиад цагаан харагдлаа
    } else if (clearSince == 0) {
      clearSince = millis();                         // цагаан алга болов
    }

    if (millis() - t0 >= BACK_MIN_MS &&
        clearSince && (millis() - clearSince) >= BACK_CLEAR_MS) {
      return;                                        // цэвэр хар дээр гарлаа
    }
  }
}

// ==========================================
// ХҮРЭЭНЭЭС ЗУГТАХ
// ==========================================
void escapeEdge(bool hitLeft, bool hitRight) {
  brakeMotors(); delay(BRAKE_MS);              // тас зуурах

  reverseUntilClear();                         // цагаанаас бүрэн салтал ухрах
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

  // Төв рүү чигээрээ явна (хүрээ/өрсөлдөгч илэрвэл тасарна)
  moveFor(1, 1, SPD_ESC_FWD, ESC_FWD_MS, true);
}

// ==========================================
// НЭЭЛТИЙН ТАКТИК (тулаан эхлэхэд нэг удаа)
// ==========================================
void openingMove() {
  if (strategy == STRAT_LEFT) {
    // Зүүн тийш эргэ -> хажуугаар давш -> төв рүү эргэж хар
    if (moveFor(-1,  1, SPD_OPEN_TURN, OPEN_TURN_MS, true)) return;
    if (moveFor( 1,  1, SPD_CHARGE,    OPEN_DASH_MS, true)) return;
    moveFor( 1, -1, SPD_OPEN_TURN, OPEN_BACK_MS, true);
    lastSeen = 1;                              // өрсөлдөгч баруун талд байх магадлалтай
  }
  else if (strategy == STRAT_RIGHT) {
    if (moveFor( 1, -1, SPD_OPEN_TURN, OPEN_TURN_MS, true)) return;
    if (moveFor( 1,  1, SPD_CHARGE,    OPEN_DASH_MS, true)) return;
    moveFor(-1,  1, SPD_OPEN_TURN, OPEN_BACK_MS, true);
    lastSeen = -1;
  }
  else {
    moveFor(1, 1, SPD_CHARGE, OPEN_STRAIGHT_MS, true);
  }
}

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
// ТУЛААН ЭХЛҮҮЛЭХ
// ==========================================
void startMatch() {
  calibrateLine();                             // робот ХАР талбай дээр байна
  servoGoTo(SERVO_START_ANGLE);                // серво 90°

#if BT_ENABLED
  Serial.print(F("START ")); Serial.println(stratName());
#endif

  uint32_t t0 = millis();                      // дүрмийн 5 сек
  while (millis() - t0 < START_DELAY_MS) {
    btTask();
    if (btStopReq) { btStopReq = false; return; }
    if (buttonDown()) { while (buttonDown()) {} return; }
#if STARTER_ENABLED
    if (!starterGo()) return;                  // модуль дуудлагаа буцаалаа
#endif
  }

  lineReset();
  readEnemy();
  running = true;

  openingMove();                               // сонгосон нээлтийн тактик
}

// ==========================================
// SETUP
// ==========================================
void setup() {
#if BT_ENABLED
  Serial.begin(BT_BAUD);
#endif

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  coastMotors();

  pinMode(left90, INPUT); pinMode(left45, INPUT); pinMode(mid, INPUT);
  pinMode(right45, INPUT); pinMode(right90, INPUT);
  // rLine (A1), lLine (A7) — зөвхөн analogRead

  pinMode(SERVO_PIN, OUTPUT); digitalWrite(SERVO_PIN, LOW);
  pinMode(START_BUTTON, INPUT_PULLUP);

#if STARTER_PULLUP
  pinMode(STARTER_PIN, INPUT_PULLUP);
#else
  pinMode(STARTER_PIN, INPUT);
#endif
  prevStarterGo = starterGo();

#if BT_ENABLED
  Serial.println(F("MINI SUMO ready. 1/2/3=strat G=go X=stop D=debug"));
#endif
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  btTask();

  // ---------------- ЗОГСОЛТ / ЭХЛҮҮЛЭХ ----------------
  if (!running) {
    coastMotors();

    bool go = starterGo();
    bool starterEdge = (go && !prevStarterGo);   // модуль дөнгөж "ЯВ" болов
    prevStarterGo = go;

    if (starterEdge || btStartReq || buttonPressed()) {
      btStartReq = false;
      startMatch();
    }
    return;
  }

  // ---------------- ТУЛААН ----------------
  // 1. Хүрээг шалгах (үргэлж хамгийн түрүүнд)
  lineTask();
  if (lineL || lineR) { escapeEdge(lineL, lineR); return; }

  // 2. Зогсоох дохио (BT 'X', товч, starter модуль)
  if (shouldStop()) {
    coastMotors();
    running = false;
    prevStarterGo = starterGo();
#if BT_ENABLED
    Serial.println(F("STOP"));
#endif
    return;
  }

  // 3. Өрсөлдөгчийн төлвөөр дайрах
  //    ТЭМДГИЙН КОНВЕНЦ: 1 = урагш, -1 = ухрах, 0 = тоормос
  readEnemy();

  if (eMid) {
    setMotors(1, 1, SPD_ATTACK);               // яг урд: хоёулаа урагшаа
  }
  else if (eL45) {
    setMotors(0, 1, SPD_CURVE);                // зүүн урд: зүүнээ түгжээд зүүн тийш
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
    // 4. Алдсан бол сүүлд харсан тал руугаа эргэж хайх
    if (lastSeen > 0) setMotors(1, 0, SPD_SEARCH);   // баруун тийш
    else              setMotors(0, 1, SPD_SEARCH);   // зүүн тийш
  }
}
