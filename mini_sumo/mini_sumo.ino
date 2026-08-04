/* =============================================================================
   MINI SUMO - БҮРЭН BLUETOOTH УДИРДЛАГА
   Чиглэл: 1 (Урагш), 0 (Тоормос), -1 (Ухрах)

   ТОВЧГҮЙ. Эхлүүлэх, зогсоох, тактик сонгох бүгд Bluetooth-оор.
   D12 сул болсон.

   ЧУХАЛ (Timer1): Nano дээр Servo сан Timer1-ийг эзэлж D9/D10-ийн analogWrite-ийг
   унтраадаг. PWMB = D9 тул Servo.h ашиглаж БОЛОХГҮЙ — серво импульсийг гараар
   үүсгэсэн, Timer1 чөлөөтэй үлдэнэ.

   ЧУХАЛ (A6/A7): lLine = A7 нь зөвхөн аналог орц. analogRead-аар л уншина.

   ЧУХАЛ (Bluetooth): HC-05/06 нь D0(RX)/D1(TX) дээр сууна. Модуль залгаастай
   үед код АЧААЛЛАХГҮЙ — upload хийхийн өмнө RX/TX-ийг салга.
   ============================================================================= */

#include <EEPROM.h>

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
#define STARTER_PIN 2         // starter модулийн дохио
// D12 — сул (товч хасагдсан)
// Bluetooth: D0 = RX, D1 = TX

// =============================================================================
// BLUETOOTH  — цорын ганц удирдлага
// =============================================================================
#define BT_ENABLED         1
#define BT_BAUD         9600  // HC-06 ихэвчлэн 9600, HC-05 нь 38400 байж болно

/*  BLUETOOTH КОМАНДУУД (терминалаас нэг үсэг илгээнэ):

      --- НЭЭЛТИЙН ТАКТИК СОНГОХ (эхлүүлэхээс өмнө) ---
      1 эсвэл S : ЧИГЭЭРЭЭ дайрах
      2 эсвэл L : ЗҮҮН талаар тойрч дайрах
      3 эсвэл R : БАРУУН талаар тойрч дайрах

      --- УДИРДЛАГА ---
      G : тулаан ЭХЛҮҮЛЭХ (сонгосон тактикаар)
      X : ЗОГСООХ
      D : QTR утга дамжуулахыг асаах/унтраах (босго тааруулахад)
      C : ЦАГААН КАЛИБРОВК — мэдрэгчээ ЦАГААН ХҮРЭЭН ДЭЭР тавьчихаад дар.
          EEPROM-д хадгална, нэг л удаа хийхэд хангалттай.
      ? : төлөв ба командын жагсаалт харах
*/

// =============================================================================
// STARTER МОДУЛЬ  (сонголтоор — тэмцээний дүрмийн дагуу)
// =============================================================================
#define STARTER_ENABLED    1  // модуль ЗАЛГААГҮЙ бол 0 болго
#define STARTER_GO   HIGH     // модуль "ЯВ" гэхэд өгөх түвшин (LOW бол сольж өг)
#define STARTER_PULLUP     1  // ил задгай коллектор гаралттай модульд 1

#if !BT_ENABLED && !STARTER_ENABLED
#error "Ehluuleh arga alga: BT_ENABLED esvel STARTER_ENABLED nэг нь 1 baih ystoi"
#endif

// =============================================================================
// НЭЭЛТИЙН ТАКТИК
// =============================================================================
#define STRAT_STRAIGHT 0
#define STRAT_LEFT     1
#define STRAT_RIGHT    2
#define STRAT_DEFAULT  STRAT_STRAIGHT   // BT-ээр сонгоогүй үеийн анхны тактик

#define OPEN_TURN_MS     180  // хажуу тийш эргэх (тойролтын 1-р шат)
#define OPEN_DASH_MS     420  // хажуугаар давших (2-р шат)
#define OPEN_BACK_MS     200  // төв рүү эргэж харах (3-р шат)
#define OPEN_STRAIGHT_MS 500  // чигээрээ дайрах хугацаа

// =============================================================================
// ХУРД (0..255)
// =============================================================================
#define SPEED_SCALE     0.60
#define PWM_MAX          100  // хатуу дээд хязгаар

#define SPD_CHARGE       200
#define SPD_ATTACK       200
#define SPD_CURVE        150
#define SPD_SPIN         140
#define SPD_SEARCH       135
#define SPD_OPEN_TURN    180
#define SPD_BACK         200
#define SPD_ESC_TURN     200
#define SPD_ESC_FWD      180

#define PWM_MIN           55

#define LEFT_INVERT        1
#define RIGHT_INVERT       1

// =============================================================================
// ХУГАЦАА  (зугталтыг ЗӨВХӨН эндээс зохицуулна)
//   Эргэлт дутуу -> TURN_*_MS ↑ | хэтэрнэ -> ↓
//   Хүрээнээс салахгүй -> BACK_MIN_MS ↑ | ард гарна -> BACK_MAX_MS ↓
// =============================================================================
#define START_DELAY_MS  5000
#define BRAKE_MS          20
#define BACK_MIN_MS       60
#define BACK_MAX_MS      260
#define BACK_CLEAR_MS     35
#define TURN_90_MS       180
#define TURN_180_MS      320
#define ESC_SETTLE_MS     10
#define ESC_FWD_MS       250

// =============================================================================
// МЭДРЭГЧ
// =============================================================================
#define ENEMY_ACTIVE    HIGH

/* ---- БОСГО ТОГТООХ ---------------------------------------------------------
   Зөвхөн хараас коэффициентээр таамаглах нь ажиллахгүй:
     хэт хатуу  -> хүрээгээ огт танихгүй
     хэт зөөлөн -> замын бартаа/зурагдсаныг цагаан гэж үзнэ
   Бартаа нь хараас цайвар боловч жинхэнэ цагаан хүрээнээс хамаагүй бараан.
   Тэднийг ялгахын тулд ЦАГААНАА хэмжинэ (BT 'C', EEPROM-д хадгалагдана).

   ЖИШЭЭ: цагаан 200, хар 800, CAL_K 0.30 -> босго 380.
          Бартаа ~500 уншина -> 500 > 380 тул ҮЛ ТООНО.
          Жинхэнэ хүрээ ~200 -> ажиллана.                                    */
#define CAL_K           0.30  // БАРТАА мэдэрвэл ↓ (0.22), хүрээгээ алдвал ↑ (0.45)
#define CAL_MIN_SPREAD    80  // хар/цагааны зөрүү үүнээс бага бол калибровк хүчингүй

// Цагааны калибровк ХИЙГЭЭГҮЙ үеийн нөөц арга (зөвхөн хараас):
#define WHITE_FACTOR    0.45  // 0.60 хэт зөөлөн байсан
#define MIN_BLACK_MARGIN 120

#define LINE_CONFIRM       9  // дараалсан цагаан уншилт (9 * 700us = 6.3 ms)
                              // 5 -> 9: богино бартаа шүүгдэнэ. 2 см хүрээ нь
                              // ~20 мс үргэлжлэх тул 6.3 мс нь хангалттай зай.
#define LINE_SAMPLE_US   700

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
bool btCalReq = false;

// Тулааныг хэн эхлүүлсэн бэ. Starter модуль зөвхөн ӨӨРӨӨ эхлүүлсэн тулааныг
// зогсооно. Ингэснээр модуль "ЗОГС" барьж байсан ч BT 'G' ажиллана — товчгүй
// болсон тул энэ нь ЗААВАЛ хэрэгтэй, эс бөгөөс эхлүүлэх ямар ч арга үлдэхгүй.
#define STARTED_BY_BT      0
#define STARTED_BY_STARTER 1
uint8_t startedBy = STARTED_BY_BT;

uint16_t thrL = 450, thrR = 450;
uint16_t whiteL = 0, whiteR = 0;      // EEPROM-д хадгалсан цагааны лавлагаа
bool     haveWhiteCal = false;
uint8_t  hitL = 0, hitR = 0;
bool     lineL = false, lineR = false;
uint32_t lastSampleUs = 0;

bool eL90, eL45, eMid, eR45, eR90, eAny;
int8_t lastSeen = 1;

// ==========================================
// МОТОР
// ==========================================
static uint8_t scaleSpeed(int pwm) {
  long v = (long)(pwm * SPEED_SCALE);
  if (v > PWM_MAX) v = PWM_MAX;
  if (v > 0 && v < PWM_MIN) v = PWM_MIN;
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

  if (l == 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255);
  } else {
    digitalWrite(AIN1, (l > 0) ? HIGH : LOW);
    digitalWrite(AIN2, (l > 0) ? LOW  : HIGH);
    analogWrite(PWMA, p);
  }

  if (r == 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255);
  } else {
    digitalWrite(BIN1, (r > 0) ? HIGH : LOW);
    digitalWrite(BIN2, (r > 0) ? LOW  : HIGH);
    analogWrite(PWMB, p);
  }
}

void brakeMotors() { setMotors(0, 0, 0); }

void coastMotors() {
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

static inline bool anyWhite() { return (hitL > 0) || (hitR > 0); }

// Хоёр QTR-ийг олон удаа уншиж дундажлана
void sampleQtr(uint16_t *outL, uint16_t *outR) {
  uint32_t sumL = 0, sumR = 0;
  const uint16_t N = 200;
  for (uint16_t i = 0; i < N; i++) {
    sumL += analogRead(lLine);
    sumR += analogRead(rLine);
    delay(2);
  }
  *outL = sumL / N;
  *outR = sumR / N;
}

// ---- EEPROM: цагааны лавлагаа ----
#define EE_MAGIC       0xA5C3
#define EE_ADDR_MAGIC  0
#define EE_ADDR_WHITE  2

void whiteCalLoad() {
  uint16_t magic = 0;
  EEPROM.get(EE_ADDR_MAGIC, magic);
  if (magic != EE_MAGIC) return;
  EEPROM.get(EE_ADDR_WHITE, whiteL);
  EEPROM.get(EE_ADDR_WHITE + 2, whiteR);
  haveWhiteCal = true;
}

void whiteCalSave() {
  uint16_t magic = EE_MAGIC;
  EEPROM.put(EE_ADDR_MAGIC, magic);
  EEPROM.put(EE_ADDR_WHITE, whiteL);
  EEPROM.put(EE_ADDR_WHITE + 2, whiteR);
  haveWhiteCal = true;
}

// BT 'C' — мэдрэгч ЦАГААН ХҮРЭЭН дээр байхад дуудна
void calibrateWhite() {
  sampleQtr(&whiteL, &whiteR);
  whiteCalSave();
#if BT_ENABLED
  Serial.print(F("WHITE CAL saved = ")); Serial.print(whiteL);
  Serial.print('/'); Serial.println(whiteR);
  Serial.println(F("Now put robot on BLACK and send G."));
#endif
  lineReset();
}

// Тулаан бүрийн эхэнд: хараа хэмжиж, босгыг цагаан ба хар хоёрын хооронд тавина
void calibrateLine() {
  uint16_t blackL, blackR;
  sampleQtr(&blackL, &blackR);

  bool ok = haveWhiteCal &&
            (blackL > whiteL + CAL_MIN_SPREAD) &&
            (blackR > whiteR + CAL_MIN_SPREAD);

  if (ok) {
    thrL = whiteL + (uint16_t)((blackL - whiteL) * CAL_K);
    thrR = whiteR + (uint16_t)((blackR - whiteR) * CAL_K);
  } else {
    long tL = (long)(blackL * WHITE_FACTOR);
    long tR = (long)(blackR * WHITE_FACTOR);
    if (blackL - tL < MIN_BLACK_MARGIN) tL = (long)blackL - MIN_BLACK_MARGIN;
    if (blackR - tR < MIN_BLACK_MARGIN) tR = (long)blackR - MIN_BLACK_MARGIN;
    thrL = (tL > 40) ? (uint16_t)tL : 450;
    thrR = (tR > 40) ? (uint16_t)tR : 450;
  }

#if BT_ENABLED
  Serial.print(F("CAL black=")); Serial.print(blackL); Serial.print('/'); Serial.print(blackR);
  if (ok) { Serial.print(F(" white=")); Serial.print(whiteL); Serial.print('/'); Serial.print(whiteR); }
  else    { Serial.print(F(" white=NONE")); }
  Serial.print(F(" thr="));  Serial.print(thrL); Serial.print('/'); Serial.println(thrR);
#endif
  lineReset();
}

// ==========================================
// ӨРСӨЛДӨГЧ
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
  if (strategy == STRAT_LEFT)  return "LEFT-CIRCLE";
  if (strategy == STRAT_RIGHT) return "RIGHT-CIRCLE";
  return "STRAIGHT";
}

void btReport() {
#if BT_ENABLED
  Serial.print(F("strat=")); Serial.print(stratName());
  Serial.print(F(" run="));  Serial.print(running ? 1 : 0);
  Serial.print(F(" thr="));  Serial.print(thrL); Serial.print('/'); Serial.println(thrR);
#endif
}

void btHelp() {
#if BT_ENABLED
  Serial.println(F("1/S=straight 2/L=left 3/R=right | G=go X=stop D=debug C=whitecal ?=help"));
  btReport();
#endif
}

void btTask() {
#if BT_ENABLED
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': case 's': case 'S': strategy = STRAT_STRAIGHT; btReport(); break;
      case '2': case 'l': case 'L': strategy = STRAT_LEFT;     btReport(); break;
      case '3': case 'r': case 'R': strategy = STRAT_RIGHT;    btReport(); break;
      case 'g': case 'G': btStartReq = true;  break;
      case 'x': case 'X': btStopReq  = true;  break;
      case 'd': case 'D': dbgStream = !dbgStream; break;
      case 'c': case 'C': btCalReq = true; break;
      case '?': case 'h': case 'H': btHelp(); break;
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
// STARTER МОДУЛЬ
// ==========================================
bool starterGo() {
#if STARTER_ENABLED
  return digitalRead(STARTER_PIN) == STARTER_GO;
#else
  return false;
#endif
}

// Тулааны үед зогсоох дохио ирсэн үү
bool shouldStop() {
  btTask();
  if (btStopReq) { btStopReq = false; return true; }
#if STARTER_ENABLED
  // Зөвхөн модуль өөрөө эхлүүлсэн тулааныг зогсооно
  if (startedBy == STARTED_BY_STARTER && !starterGo()) return true;
#endif
  return false;
}

// ==========================================
// ХӨДӨЛГӨӨН
// ==========================================
void driveFor(int left, int right, int pwm, uint16_t ms) {
  uint32_t t0 = millis();
  setMotors(left, right, pwm);
  while (millis() - t0 < ms) {
    lineTask();
    if (shouldStop()) { coastMotors(); running = false; return; }
  }
}

// Хүрээ эсвэл өрсөлдөгч илэрвэл тасална. true = таслагдсан.
bool moveFor(int left, int right, int pwm, uint16_t ms, bool abortOnEnemy) {
  uint32_t t0 = millis();
  setMotors(left, right, pwm);
  while (millis() - t0 < ms) {
    lineTask();
    if (lineL || lineR) return true;
    if (shouldStop()) { coastMotors(); running = false; return true; }
    if (abortOnEnemy) { readEnemy(); if (eAny) return true; }
  }
  return false;
}

// Цагаанаас БҮРЭН салтал ухарна
void reverseUntilClear() {
  uint32_t t0 = millis();
  uint32_t clearSince = 0;
  setMotors(-1, -1, SPD_BACK);

  while (millis() - t0 < BACK_MAX_MS) {
    lineTask();
    if (shouldStop()) { coastMotors(); running = false; return; }

    if (anyWhite()) {
      clearSince = 0;
    } else if (clearSince == 0) {
      clearSince = millis();
    }

    if (millis() - t0 >= BACK_MIN_MS &&
        clearSince && (millis() - clearSince) >= BACK_CLEAR_MS) {
      return;
    }
  }
}

// ==========================================
// ХҮРЭЭНЭЭС ЗУГТАХ
// ==========================================
void escapeEdge(bool hitLeft, bool hitRight) {
  brakeMotors(); delay(BRAKE_MS);

  reverseUntilClear();
  if (!running) return;

  if (hitLeft && hitRight) {
    driveFor(lastSeen > 0 ?  1 : -1,
             lastSeen > 0 ? -1 :  1, SPD_ESC_TURN, TURN_180_MS);
  } else if (hitLeft) {
    driveFor(1, -1, SPD_ESC_TURN, TURN_90_MS);   // баруун эргэх
    lastSeen = 1;
  } else {
    driveFor(-1, 1, SPD_ESC_TURN, TURN_90_MS);   // зүүн эргэх
    lastSeen = -1;
  }
  if (!running) return;

  brakeMotors(); delay(ESC_SETTLE_MS);
  lineReset();

  moveFor(1, 1, SPD_ESC_FWD, ESC_FWD_MS, true);  // төв рүү чигээрээ
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
    lastSeen = 1;
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
// СЕРВО
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
    if (btStopReq) { btStopReq = false; return; }   // BT 'X' — цуцлах
#if STARTER_ENABLED
    if (startedBy == STARTED_BY_STARTER && !starterGo()) return;
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

#if STARTER_PULLUP
  pinMode(STARTER_PIN, INPUT_PULLUP);
#else
  pinMode(STARTER_PIN, INPUT);
#endif
  prevStarterGo = starterGo();

  whiteCalLoad();                              // EEPROM-оос цагааны лавлагаа

#if BT_ENABLED
  Serial.println(F("MINI SUMO ready (BT only)."));
  if (haveWhiteCal) {
    Serial.print(F("white cal = ")); Serial.print(whiteL);
    Serial.print('/'); Serial.println(whiteR);
  } else {
    Serial.println(F("NO white cal - put sensors on WHITE line and send 'C'"));
  }
  btHelp();
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
    bool starterEdge = (go && !prevStarterGo);
    prevStarterGo = go;

    if (btCalReq) { btCalReq = false; calibrateWhite(); return; }

    if (starterEdge) {
      startedBy = STARTED_BY_STARTER;
      startMatch();
    } else if (btStartReq) {
      btStartReq = false;
      startedBy = STARTED_BY_BT;               // модуль энэ тулааныг зогсоохгүй
      startMatch();
    }
    return;
  }

  // ---------------- ТУЛААН ----------------
  // 1. Хүрээг шалгах (үргэлж хамгийн түрүүнд)
  lineTask();
  if (lineL || lineR) { escapeEdge(lineL, lineR); return; }

  // 2. Зогсоох дохио (BT 'X' эсвэл starter модуль)
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
  readEnemy();

  if (eMid) {
    setMotors(1, 1, SPD_ATTACK);               // яг урд
  }
  else if (eL45) {
    setMotors(0, 1, SPD_CURVE);                // зүүн урд
  }
  else if (eR45) {
    setMotors(1, 0, SPD_CURVE);                // баруун урд
  }
  else if (eL90) {
    setMotors(-1, 1, SPD_SPIN);                // зүүн зах
  }
  else if (eR90) {
    setMotors(1, -1, SPD_SPIN);                // баруун зах
  }
  else {
    // 4. Алдсан бол сүүлд харсан тал руугаа эргэж хайх
    if (lastSeen > 0) setMotors(1, 0, SPD_SEARCH);
    else              setMotors(0, 1, SPD_SEARCH);
  }
}
