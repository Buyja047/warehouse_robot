/*
  ==========================================================================
   MINI SUMO ROBOT - ESP32 + Bluepad32  (дахин бичсэн, сайжруулсан хувилбар)
  ==========================================================================

   Сан суулгах: "Arduino Core for ESP32 + Bluepad32"
   https://bluepad32.readthedocs.io/en/latest/plat_arduino/

   ӨӨРЧЛӨГДӨӨГҮЙ ЗҮЙЛС:
     - Бүх PIN хуучнаараа (GPIO нэг ч өөрчлөгдөөгүй)
     - Sensor board-ын UART протокол (0x55 + 9 байт + checksum, 230400)
     - NeoPixel LED, товчны UX, POWER_PIN-ий тэжээлийн логик
     - Bluepad32 холболт, PS gamepad-аас өөр контроллер татгалзах

   ГОЛ САЙЖРУУЛАЛТ:
     1) START МОДУЛЬ (GPIO15) = kill switch
          RISING (LOW->HIGH) -> эхлэх   |   FALLING (HIGH->LOW) -> ШУУД зогсох
          Модуль нэг удаа HIGH болсныг мэдэрсэн бол зогсоох эрхийг бүхэлд нь
          Start модуль эзэмшинэ (товч эхлүүлж чадахгүй).
     2) MANUAL горим хоцрогдолгүй:
          BP32.update() loop бүрт, gamepad утга кэшлэгдэнэ, hot path дотор
          Serial.printf БАЙХГҮЙ (энэ нь хуучин кодын гол хоцрогдол байсан),
          UART буфер бүрэн уншигдана, motor ramp хурдан.
     3) AUTOMAT горим цагаан зураас давахгүй:
          - Хар талбай дээр IDLE үед line сенсорын суурь (black reference)
            автоматаар калибровк хийгдэнэ
          - Хоёр босго (strong/weak) + ХУГАЦААНД суурилсан баталгаажуулалт:
            хар талбай дээрх нимгэн цагаан зураас/маажилт нь хэт богино
            тул MEDEREGDEHГҮЙ, харин 2 см-ийн ирмэгийн хүрээ найдвартай
            мэдрэгдэнэ
          - Ирмэг мэдрэгдмэгц ямар ч state-ийг таслан зогсоож (instant brake,
            ramp-ыг алгасна), ухрах -> дотогш эргэх -> хайх
          - Sensor board холбоо тасарсан бол автоматаар зогсоно (сохроор
            гүйхгүй)

   ХЯЛБАР ТОХИРУУЛГА: доорх "ТОХИРГОО" хэсгийн #define утгуудыг л өөрчил.
  ==========================================================================
*/

#include <Bluepad32.h>
#include <Adafruit_NeoPixel.h>

/* =========================== PIN (ӨӨРЧЛӨӨГҮЙ) =========================== */
#define PIN 2      // NeoPixel
#define NUMPIXELS 3

#define MOTOR_L_PWM_A 18
#define MOTOR_L_PWM_B 19
#define MOTOR_R_PWM_A 27
#define MOTOR_R_PWM_B 26

#define BUTTON1 0
#define BUTTON2 25
#define START_PIN 15  // хуучин `#define IR 15` - Start модулийн дохио
#define POWER_PIN 33

#define SERIAL2_RX 16
#define SERIAL2_TX 17
#define BAUD_RATE 230400

/* ==================== SENSOR BOARD-ЫН FRAME (ӨӨРЧЛӨӨГҮЙ) ================
   start  right_line left_line right90 right45 rightF  leftF   left45  left90  table  chksum
   0x55   0-255      0-255     0-255   0-255   0-255   0-255   0-255   0-255   0-63   0-255
   [0]    [1]        [2]       [3]     [4]     [5]     [6]     [7]     [8]     [9]    [10]
   checksum = [1]..[9]-ийн нийлбэр
   ======================================================================= */
#define FRAME_SIZE 11
#define FRAME_START_BYTE 0x55

/* ===================== GAMEPAD BIT MASK (ӨӨРЧЛӨӨГҮЙ) ==================== */
#define BUTTON_L1 0x0010
#define BUTTON_R1 0x0020
#define BUTTON_L2 0x0040
#define BUTTON_R2 0x0080
#define BUTTON_X 0x0001
#define BUTTON_O 0x0002
#define BUTTON_SQUARE 0x0004
#define BUTTON_TRIANGLE 0x0008

#define DPAD_UP 0x01
#define DPAD_DOWN 0x02
#define DPAD_RIGHT 0x04
#define DPAD_LEFT 0x08

/* ============================== ТОХИРГОО =============================== */

/* --- Start модуль (GPIO15) --- */
// Модуль push-pull драйвертай бол INPUT_PULLDOWN хамгийн аюулгүй:
// кабель салсан/тэжээлгүй үед оролт LOW = ЗОГССОН төлөв рүү унана.
// Хэрэв чиний модуль open-collector (гадна pull-up шаарддаг) бол
// энийг INPUT_PULLUP болго.
#define START_PIN_MODE INPUT_PULLDOWN
#define START_ACTIVE_HIGH 1  // 1: HIGH=RUN / LOW=STOP  (даалгаврын дагуу)
#define START_HIGH_STABLE_MS 15  // эхлүүлэхийн өмнөх тогтвортой HIGH хугацаа
#define START_LOW_STABLE_MS 3    // зогсоох хариу үйлдэл - маш хурдан
// Асаах агшинд дохио аль хэдийн HIGH байсан бол РОБОТ ЭХЛЭХГҮЙ. Эхлэхийн тулд
// заавал жинхэнэ LOW->HIGH шилжилт хэрэгтэй (санамсаргүй хөдлөхөөс хамгаална).
#define START_INIT_MS 200
#define START_GATE_APPLIES_TO_MANUAL 1  // kill switch manual горимд ч үйлчилнэ

// Mini sumo дүрмийн 5 секундын хүлээлт. Тэмцээний дүрэмгүй бол 0 болго.
#define START_COUNTDOWN_MS 5000

/* --- Line (ирмэг) мэдрэгч ---
   Уг протоколд ЦАГААН = БАГА ADC утга (хуучин код `< LINE_ADC_CUT` гэж
   шалгаж байсан). Хэрэв чиний борд эсрэгээр (цагаан = их) өгдөг бол
   LINE_WHITE_IS_LOW-г 0 болго.                                          */
#define LINE_WHITE_IS_LOW 1
#define LINE_ADC_CUT 100   // калибровк амжаагүй үеийн нөөц босго (хуучныг хэвээр)
#define LINE_STRONG_PCT 45  // black reference-ийн % : тод цагаан
#define LINE_WEAK_PCT 70    // black reference-ийн % : сул цагаан
#define LINE_STRONG_MS 2    // тод цагаан ийм хугацаа тогтвортой бол = ирмэг
#define LINE_WEAK_MS 7      // сул цагаан ийм хугацаа тогтвортой бол = ирмэг
#define LINE_RELEASE_MS 20  // ирмэгээс салсныг баталгаажуулах хугацаа
#define LINE_MIN_THRESHOLD 20   // босгын доод хязгаар (шуугианаас хамгаалах)
#define LINE_BLACK_MIN 60       // энэнээс бага бол "хар" гэж калибровк хийхгүй

/* --- Өрсөлдөгчийн мэдрэгч --- */
#define ADC_CUT 100  // (хуучныг хэвээр) энэнээс их бол объект байна

/* --- Sensor board-ын холбоо --- */
#define SENSOR_TIMEOUT_MS 200  // ийм хугацаанд frame ирэхгүй бол зогсоно
#define REQUIRE_SENSOR_BOARD 1 // 1: борд ажиллахгүй бол AUTO эхлэхгүй

/* --- Мотор --- */
#define MOTOR_PWM_FREQ 20000  // зүүн/баруун ижил давтамж (хуучин 30k/12k байсан)
#define MOTOR_PWM_BITS 10
#define MOTOR_MIN_DUTY 300    // хөдөлгөөн эхлэх доод duty (хуучныг хэвээр)
#define MOTOR_MAX_DUTY 1024
#define MOTOR_DEADBAND 10     // |утга| <= энэ бол тоормос

#define ACCEL_AUTO 14
#define DECEL_AUTO 30   // ирмэг дээр хурдан зогсохын тулд илүү өндөр
#define ACCEL_MANUAL 30 // гараар удирдахад бараг шууд хариу үйлдэл
#define DECEL_MANUAL 45

/* --- Гараар удирдах (gamepad) --- */
#define STICK_DEADZONE 40  // ±512-ийн хувьд
#define MANUAL_SPEED_NORMAL 70
#define MANUAL_SPEED_BOOST 100

/* --- Довтолгоо / хайлт --- */
#define ATTACK_SPEED 100
#define ATTACK_ARC_INNER 20
#define ATTACK_PIVOT 95
#define SEARCH_TURN 72
#define SEARCH_CREEP 62
#define SEARCH_TURN_MS 700
#define SEARCH_CREEP_MS 260

/* --- Ирмэгээс зугтах маневр --- */
#define EDGE_BRAKE_MS 25    // импульсийг таслах
#define EDGE_BACK_MS 190    // ухрах
#define EDGE_BACK_SPEED 92
#define EDGE_TURN_MS 165    // дотогш эргэх
#define EDGE_TURN_SPEED 100
#define EDGE_RETRY_MAX 4    // дараалан дахин ирмэг олдвол хамгаалалт

/* --- Бусад --- */
#define THREE_MIN 180000UL  // идэвхгүй байх хугацаа -> тэжээл унтраах
#define DEBUG_PRINT 1       // 1: IDLE үед телеметр хэвлэнэ (RUN үед ХЭЗЭЭ Ч хэвлэхгүй)

/* ============ ESP32 CORE 2.x / 3.x НИЙЦТЭЙ БОЛГОХ ДАВХАРГА =============
   Bluepad32-ийн Arduino core хувилбараас хамаарч LEDC болон timer-ийн API
   өөр байдаг тул доорх боодол функцүүдээр аль ч хувилбар дээр компайл
   болохоор хийв.                                                        */
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define CORE3_API 1
#else
#define CORE3_API 0
#endif

// Мотор гаралтын индекс 1..4 -> GPIO (сувгийн дугаар хуучинтай ижил утгатай)
static const int kMotorPin[5] = {-1, MOTOR_L_PWM_A, MOTOR_L_PWM_B,
                                 MOTOR_R_PWM_A, MOTOR_R_PWM_B};

static inline void pwmSetup() {
  for (uint8_t i = 1; i <= 4; i++) {
#if CORE3_API
    ledcAttachChannel(kMotorPin[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS, i);
#else
    ledcSetup(i, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(kMotorPin[i], i);
#endif
  }
}

static inline void pwmWrite(uint8_t idx, uint32_t duty) {
#if CORE3_API
  ledcWrite(kMotorPin[idx], duty);  // 3.x: PIN-ээр бичнэ
#else
  ledcWrite(idx, duty);             // 2.x: сувгаар бичнэ
#endif
}

/* ============================ ГЛОБАЛ ТӨЛӨВ ============================= */

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/* --- 100us hardware timer -> 1 ms control tick --- */
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t tick100us = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  tick100us++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

static inline void timerSetup100us() {
#if CORE3_API
  timer = timerBegin(1000000);  // 1 MHz = 1 tick / us
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 100, true, 0);
#else
  timer = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 tick / us
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, true);
  timerAlarmEnable(timer);
#endif
}

/* --------------------------- Урьдчилсан зарлалт ------------------------ */
void controlTick1ms();
void stopNow(bool toIdle);
void beginRun();

/* --- Робот төлөвийн машин --- */
enum RobotState : uint8_t {
  ST_IDLE = 0,     // зогссон, хүлээж байна
  ST_COUNTDOWN,    // start дохио ирсэн, 5 сек хүлээж байна
  ST_OPENING,      // сонгосон нээлтийн стратеги
  ST_SEARCH,       // өрсөлдөгч хайж байна
  ST_ATTACK,       // өрсөлдөгч рүү дайрч байна
  ST_EDGE_BRAKE,   // ирмэг мэдэрсэн - тоормос
  ST_EDGE_BACK,    // ухарч байна
  ST_EDGE_TURN     // дотогш эргэж байна
};

RobotState state = ST_IDLE;
uint32_t stateMs = 0;  // тухайн төлөвт байсан хугацаа (ms)

uint8_t battleMode = 2;   // 2..7 нээлтийн стратеги (LED өнгө)
uint8_t gameMode = 0;     // 0 = AUTOMAT, 1 = MANUAL
uint32_t shutDownCounter = 0;

/* --- Start модуль --- */
volatile bool startLevel = false;      // debounce хийсэн одоогийн түвшин
bool startModulePresent = false;       // нэг ч удаа HIGH болж байсан уу
bool startEventRise = false;
bool startEventFall = false;

/* --- UART frame --- */
uint8_t rxBuffer[FRAME_SIZE];
uint8_t rxIndex = 0;
uint8_t sensorBuffer[8];
uint32_t lastFrameMs = 0;
bool sensorBoardAlive = false;
uint32_t frameOkCount = 0, frameErrCount = 0;

/* --- Өрсөлдөгчийн мэдрэгчийн bit table --- */
uint8_t sensorTable = 0;
int8_t lastSeenDir = 1;  // +1 = зүүн тийш, -1 = баруун тийш сүүлд харагдсан

/* --- Line сенсорын фильтр --- */
struct LineFilter {
  uint16_t raw;
  uint16_t blackRef;     // хар талбайн суурь утга (калибровк)
  uint32_t whiteSince;   // цагаан руу орсон агшин (0 = цагаан биш)
  uint32_t lastWhiteMs;  // сүүлд цагаан харсан агшин
  bool active;           // фильтрлэсэн "ирмэг дээр байна"
  bool event;            // тасалдал (control loop уншаад цэвэрлэнэ)
};
LineFilter lineR, lineL;

uint8_t edgeSide = 0;      // 1 = баруун, 2 = зүүн, 3 = хоёулаа
uint8_t edgeRetry = 0;

/* --- Мотор --- */
int16_t motorL = 0, motorR = 0;   // -100..100 зорилтот утга
int16_t curL = 0, curR = 0;       // ramp хийсэн одоогийн утга

/* --- Gamepad кэш (loop бүрт шинэчлэгдэнэ) --- */
ControllerPtr myController = nullptr;
volatile bool padConnected = false;
int32_t padAxisY = 0, padAxisRY = 0, padThrottle = 0, padBrake = 0;
uint16_t padButtons = 0;
uint8_t padDpad = 0;

/* --- LED кэш (шаардлагагүй үед pixels.show() дуудахгүй) --- */
uint32_t ledCache[NUMPIXELS] = {0, 0, 0};

/* --- Товч --- */
uint16_t btn1HoldMs = 0;
int lastButtonState = HIGH;
bool buttonState = HIGH;
uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 50;
uint8_t tapCounter = 0;
bool flag1 = true, flag2 = false;
uint32_t presstime = 0, releasetime = 0;
uint32_t timediff = 0;

/* ============================ ТУСЛАХ ФУНКЦ ============================= */

static inline bool isRunning() {
  return state != ST_IDLE;
}

void setState(RobotState s) {
  state = s;
  stateMs = 0;
}

/* -------------------------------- LED --------------------------------- */
// pixels.show() нь ~90us тасалдал хаадаг тул зөвхөн өнгө өөрчлөгдсөн үед,
// бас нэг удаа л дуудна.
bool ledDirty = false;

void ledSet(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
  if (idx >= NUMPIXELS) return;
  uint32_t c = pixels.Color(r, g, b);
  if (ledCache[idx] == c) return;
  ledCache[idx] = c;
  pixels.setPixelColor(idx, c);
  ledDirty = true;
}

void ledStrategy(uint8_t colorSelect) {
  switch (colorSelect) {
    case 2: ledSet(0, 0, 50, 0); break;    // ногоон
    case 3: ledSet(0, 0, 0, 50); break;    // цэнхэр
    case 4: ledSet(0, 50, 0, 0); break;    // улаан
    case 5: ledSet(0, 50, 50, 0); break;   // шар
    case 6: ledSet(0, 50, 0, 50); break;   // ягаан
    case 7: ledSet(0, 0, 50, 50); break;   // цайвар цэнхэр
    default: ledSet(0, 0, 0, 0); break;
  }
}

void ledTask() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 20) return;
  last = now;

  bool blink = ((now / 150) & 1);

  // 0-р LED: сонгосон стратеги
  ledStrategy(battleMode);

  // 2-р LED: горим
  if (gameMode == 1) ledSet(2, 0, 0, 50);
  else ledSet(2, 0, 50, 0);

  // 1-р LED: төлөв
  if (!sensorBoardAlive && gameMode == 0) {
    ledSet(1, blink ? 60 : 0, 0, 0);  // борд холбоогүй - улаан анивчина
  } else {
    switch (state) {
      case ST_IDLE:
        // Start модуль байгаа ба зогсоосон бол сул улаан, эс бөгөөс сул цагаан
        if (startModulePresent && !startLevel) ledSet(1, 15, 0, 0);
        else ledSet(1, 8, 8, 8);
        break;
      case ST_COUNTDOWN: ledSet(1, blink ? 60 : 0, blink ? 40 : 0, 0); break;
      case ST_EDGE_BRAKE:
      case ST_EDGE_BACK:
      case ST_EDGE_TURN: ledSet(1, 60, 0, 0); break;
      default: ledSet(1, 0, 50, 0); break;
    }
  }

  if (ledDirty) {
    ledDirty = false;
    pixels.show();
  }
}

/* ------------------------------- МОТОР --------------------------------- */
static void motorWriteSide(int v, uint8_t chA, uint8_t chB) {
  if (v > MOTOR_DEADBAND) {
    pwmWrite(chA, map(v, MOTOR_DEADBAND, 100, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY));
    pwmWrite(chB, 0);
  } else if (v < -MOTOR_DEADBAND) {
    pwmWrite(chA, 0);
    pwmWrite(chB, map(v, -MOTOR_DEADBAND, -100, MOTOR_MIN_DUTY, MOTOR_MAX_DUTY));
  } else {  // тоормос (хоёр талд бүтэн duty = brake)
    pwmWrite(chA, MOTOR_MAX_DUTY);
    pwmWrite(chB, MOTOR_MAX_DUTY);
  }
}

// 1 ms тутам дуудагдана. Ramp хийж, PWM гаргана.
void motorUpdate(int16_t targetL, int16_t targetR, int accel, int decel) {
  targetL = constrain(targetL, -100, 100);
  targetR = constrain(targetR, -100, 100);

  curL = (targetL > curL) ? min(curL + accel, (int)targetL) : max(curL - decel, (int)targetL);
  curR = (targetR > curR) ? min(curR + accel, (int)targetR) : max(curR - decel, (int)targetR);
  curL = constrain(curL, -100, 100);
  curR = constrain(curR, -100, 100);

  motorWriteSide(curL, 1, 2);
  motorWriteSide(curR, 3, 4);
}

// Ramp-ыг алгасаж шууд утга онооно (ирмэг дээрх шуурхай тоормос).
void motorSnap(int16_t l, int16_t r) {
  curL = constrain(l, -100, 100);
  curR = constrain(r, -100, 100);
  motorWriteSide(curL, 1, 2);
  motorWriteSide(curR, 3, 4);
}

/* ================== SENSOR BOARD-ЫН UART УНШИЛТ ======================== */

void lineFilterReset(LineFilter &f) {
  f.whiteSince = 0;
  f.lastWhiteMs = 0;
  f.active = false;
  f.event = false;
}

// Нэг line сенсорын түүхий утгыг фильтрлэнэ.
// Хар талбай дээрх нимгэн цагаан маажилтыг ХУГАЦААГААР шүүж хаяна.
void lineFilterUpdate(LineFilter &f, uint16_t raw, uint32_t now, bool calibrate) {
  f.raw = raw;

  // --- Хар суурийг зөвхөн IDLE үед, хар мэт утгаар л шинэчилнэ ---
  uint16_t thrWeak, thrStrong;
  if (f.blackRef >= LINE_BLACK_MIN) {
#if LINE_WHITE_IS_LOW
    thrStrong = (uint32_t)f.blackRef * LINE_STRONG_PCT / 100;
    thrWeak = (uint32_t)f.blackRef * LINE_WEAK_PCT / 100;
    if (thrStrong < LINE_MIN_THRESHOLD) thrStrong = LINE_MIN_THRESHOLD;
    if (thrWeak <= thrStrong) thrWeak = thrStrong + 1;
#else
    thrStrong = 255 - (uint32_t)(255 - f.blackRef) * LINE_STRONG_PCT / 100;
    thrWeak = 255 - (uint32_t)(255 - f.blackRef) * LINE_WEAK_PCT / 100;
    if (thrWeak >= thrStrong) thrWeak = thrStrong - 1;
#endif
  } else {  // калибровк хийгдээгүй - нөөц босго
    thrStrong = LINE_ADC_CUT;
    thrWeak = LINE_ADC_CUT;
  }

#if LINE_WHITE_IS_LOW
  bool weak = (raw < thrWeak);
  bool strong = (raw < thrStrong);
  bool blackLike = (raw > thrWeak);
#else
  bool weak = (raw > thrWeak);
  bool strong = (raw > thrStrong);
  bool blackLike = (raw < thrWeak);
#endif

  if (calibrate && blackLike) {
    // EMA: blackRef = blackRef*7/8 + raw/8
    if (f.blackRef == 0) f.blackRef = raw;
    else f.blackRef = (uint16_t)(((uint32_t)f.blackRef * 7 + raw) / 8);
  }

  if (weak) {
    if (f.whiteSince == 0) f.whiteSince = now;
    f.lastWhiteMs = now;
    uint32_t dur = now - f.whiteSince;
    if ((strong && dur >= LINE_STRONG_MS) || dur >= LINE_WEAK_MS) {
      if (!f.active) f.event = true;  // ирмэг рүү орсон агшин
      f.active = true;
    }
  } else {
    f.whiteSince = 0;
    if (f.active && (now - f.lastWhiteMs) > LINE_RELEASE_MS) f.active = false;
  }
}

void usartPoll() {
  // Буферийг БҮРЭН уншина (хуучин код нэг loop-д 1 л байт уншиж frame алдаж
  // байсан). Нэг дуудлагад дээд тал нь 8 frame боловсруулна.
  uint16_t guard = FRAME_SIZE * 8;
  while (Serial2.available() && guard--) {
    uint8_t data = Serial2.read();

    if (rxIndex == 0 && data != FRAME_START_BYTE) continue;  // sync
    rxBuffer[rxIndex++] = data;
    if (rxIndex < FRAME_SIZE) continue;
    rxIndex = 0;

    uint8_t chk = 0;
    for (uint8_t i = 1; i <= 9; i++) chk += rxBuffer[i];
    if (chk != rxBuffer[10]) {
      frameErrCount++;
      continue;
    }
    frameOkCount++;

    uint32_t now = millis();
    lastFrameMs = now;
    sensorBoardAlive = true;

    // --- Ирмэгийн (line) сенсор ---
    bool calibrate = (state == ST_IDLE);
    lineFilterUpdate(lineR, rxBuffer[1], now, calibrate);
    lineFilterUpdate(lineL, rxBuffer[2], now, calibrate);

    // --- Өрсөлдөгчийн сенсор ---
    uint8_t t = 0;
    if (rxBuffer[3] > ADC_CUT) t |= 0x01;  // баруун 90
    if (rxBuffer[4] > ADC_CUT) t |= 0x02;  // баруун 45
    if (rxBuffer[5] > ADC_CUT) t |= 0x04;  // баруун урд
    if (rxBuffer[6] > ADC_CUT) t |= 0x08;  // зүүн урд
    if (rxBuffer[7] > ADC_CUT) t |= 0x10;  // зүүн 45
    if (rxBuffer[8] > ADC_CUT) t |= 0x20;  // зүүн 90
    sensorTable = t;

    for (uint8_t i = 0; i < 8; i++) sensorBuffer[i] = rxBuffer[i + 1];
  }

  if (sensorBoardAlive && (millis() - lastFrameMs) > SENSOR_TIMEOUT_MS) {
    sensorBoardAlive = false;
  }
}

/* ========================== START МОДУЛЬ (GPIO15) ======================= */
// 1 ms тутам дуудагдана.
void startModuleScan() {
  static uint16_t hiCnt = 0, loCnt = 0;
  static uint16_t initMs = 0;

  bool raw = (digitalRead(START_PIN) == HIGH);
#if !START_ACTIVE_HIGH
  raw = !raw;
#endif

  // Асаалтын эхний хэсэгт зөвхөн одоогийн түвшинг тогтооно - эвент үүсгэхгүй.
  if (initMs < START_INIT_MS) {
    initMs++;
    startLevel = raw;
    hiCnt = 0;
    loCnt = 0;
    return;
  }

  if (raw) {
    loCnt = 0;
    if (hiCnt < 1000) hiCnt++;
  } else {
    hiCnt = 0;
    if (loCnt < 1000) loCnt++;
  }

  if (!startLevel && hiCnt >= START_HIGH_STABLE_MS) {
    startLevel = true;
    startModulePresent = true;  // жинхэнэ шилжилт = модуль холбоотой
    startEventRise = true;
  } else if (startLevel && loCnt >= START_LOW_STABLE_MS) {
    startLevel = false;
    startModulePresent = true;  // жинхэнэ шилжилт = модуль холбоотой
    startEventFall = true;      // KILL SWITCH
  }
}

// Хөдөлгөөн зөвшөөрөгдсөн эсэх (Start модулийн хаалт)
static inline bool motionAllowed() {
  if (!startModulePresent) return true;  // модуль холбоогүй - тэвшин дээр тест
  return startLevel;
}

/* ============================ УДИРДЛАГА =============================== */

void stopNow(bool toIdle) {
  motorL = 0;
  motorR = 0;
  motorSnap(0, 0);  // ramp-гүй, шууд тоормос
  if (toIdle) setState(ST_IDLE);
}

void beginRun() {
  if (gameMode != 0) return;
  if (!motionAllowed()) return;
#if REQUIRE_SENSOR_BOARD
  if (!sensorBoardAlive) return;  // сенсоргүй бол сохроор гүйхгүй
#endif
  lineFilterReset(lineR);
  lineFilterReset(lineL);
  edgeRetry = 0;
  shutDownCounter = 0;
  motorSnap(0, 0);
#if START_COUNTDOWN_MS > 0
  setState(ST_COUNTDOWN);
#else
  setState(ST_OPENING);
#endif
}

/* --- Ирмэгийн эвент шалгах: ямар ч төлөвөөс дээгүүр эрхтэй --- */
bool edgeCheck() {
  bool r = lineR.event || lineR.active;
  bool l = lineL.event || lineL.active;
  lineR.event = false;
  lineL.event = false;
  if (!r && !l) return false;

  edgeSide = (r ? 1 : 0) | (l ? 2 : 0);
  lastSeenDir = (edgeSide == 1) ? 1 : -1;  // ирмэгээс эсрэг тийш эргэнэ
  return true;
}

/* --- Өрсөлдөгчийн байрлалыг bit-үүдээс тооцох ---
   Буцаах утга: pos10 (-30..+30). Сөрөг = баруун талд, эерэг = зүүн талд. */
static bool opponentPosition(uint8_t t, int16_t &pos10) {
  static const int8_t w[6] = {-3, -2, -1, +1, +2, +3};
  int16_t sum = 0;
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 6; i++) {
    if (t & (1 << i)) {
      sum += w[i];
      cnt++;
    }
  }
  if (cnt == 0) return false;
  pos10 = (int16_t)(sum * 10) / cnt;
  return true;
}

void doAttack() {
  int16_t pos10;
  if (!opponentPosition(sensorTable, pos10)) {
    setState(ST_SEARCH);
    return;
  }
  lastSeenDir = (pos10 > 0) ? 1 : -1;

  int16_t a = abs(pos10);
  if (a <= 12) {  // урдаа - бүрэн хүчээр түлхэх
    motorL = ATTACK_SPEED;
    motorR = ATTACK_SPEED;
  } else if (a <= 22) {  // 45 градус - нумаар дайрах
    if (pos10 < 0) {     // баруун талд
      motorL = ATTACK_SPEED;
      motorR = ATTACK_ARC_INNER;
    } else {  // зүүн талд
      motorL = ATTACK_ARC_INNER;
      motorR = ATTACK_SPEED;
    }
  } else {           // 90 градус - байран дээрээ эргэх
    if (pos10 < 0) {  // баруун тийш
      motorL = ATTACK_PIVOT;
      motorR = -ATTACK_PIVOT;
    } else {  // зүүн тийш
      motorL = -ATTACK_PIVOT;
      motorR = ATTACK_PIVOT;
    }
  }
}

void doSearch() {
  if (sensorTable) {
    setState(ST_ATTACK);
    return;
  }
  // Эргэлт <-> богино урагшлалт солигдож талбайг сканнердана
  uint32_t cycle = SEARCH_TURN_MS + SEARCH_CREEP_MS;
  uint32_t p = stateMs % cycle;
  if (p < SEARCH_TURN_MS) {
    if (lastSeenDir > 0) {  // зүүн тийш эргэх
      motorL = -SEARCH_TURN;
      motorR = SEARCH_TURN;
    } else {  // баруун тийш эргэх
      motorL = SEARCH_TURN;
      motorR = -SEARCH_TURN;
    }
  } else {
    motorL = SEARCH_CREEP;
    motorR = SEARCH_CREEP;
    if (p == SEARCH_TURN_MS) lastSeenDir = -lastSeenDir;  // дараагийн эргэлт нөгөө тийш
  }
}

/* --- Нээлтийн стратеги (хуучин runMode 2..7-той ижил хугацаанууд) --- */
void doOpening() {
#if 1
  // Урд талд өрсөлдөгч тод харагдвал нээлтийг таслаад шууд дайрна
  if (sensorTable & (0x04 | 0x08)) {
    setState(ST_ATTACK);
    return;
  }
#endif
  uint32_t t = stateMs;
  switch (battleMode) {
    case 2:  // ногоон: баруун тийш эргэх -> урагш -> зүүн тийш эргэх
      if (t < 50) { motorL = 100; motorR = -100; }
      else if (t < 80) { motorL = 0; motorR = 0; }
      else if (t < 280) { motorL = 100; motorR = 100; }
      else if (t < 310) { motorL = 0; motorR = 0; }
      else if (t < 450) { motorL = -100; motorR = 100; }
      else if (t < 500) { motorL = 0; motorR = 0; }
      else setState(ST_SEARCH);
      break;

    case 3:  // цэнхэр: зүүн тийш эргэх -> урагш -> баруун тийш эргэх
      if (t < 50) { motorL = -100; motorR = 100; }
      else if (t < 60) { motorL = 0; motorR = 0; }
      else if (t < 250) { motorL = 100; motorR = 100; }
      else if (t < 270) { motorL = 0; motorR = 0; }
      else if (t < 450) { motorL = 100; motorR = -100; }
      else if (t < 500) { motorL = 0; motorR = 0; }
      else setState(ST_SEARCH);
      break;

    case 4:  // улаан: нээлтгүй, шууд хайлт
      setState(ST_SEARCH);
      break;

    case 5:  // шар: баруун тийш эргэх -> ухрах
      if (t < 50) { motorL = 100; motorR = -100; }
      else if (t < 80) { motorL = 0; motorR = 0; }
      else if (t < 280) { motorL = -100; motorR = -100; }
      else if (t < 310) { motorL = 0; motorR = 0; }
      else setState(ST_SEARCH);
      break;

    case 6:  // ягаан: зүүн тийш эргэх -> ухрах
      if (t < 50) { motorL = -100; motorR = 100; }
      else if (t < 80) { motorL = 0; motorR = 0; }
      else if (t < 280) { motorL = -100; motorR = -100; }
      else if (t < 310) { motorL = 0; motorR = 0; }
      else setState(ST_SEARCH);
      break;

    case 7:  // цайвар цэнхэр: богино ухраад хүлээх (сөрөг довтолгоо)
      if (t < 200) { motorL = -100; motorR = -100; }
      else if (t < 1000) { motorL = 0; motorR = 0; }
      else setState(ST_SEARCH);
      break;

    default:
      setState(ST_SEARCH);
      break;
  }
}

/* --- Ирмэгээс зугтах --- */
void doEdge() {
  switch (state) {
    case ST_EDGE_BRAKE:
      motorL = 0;
      motorR = 0;
      if (stateMs >= EDGE_BRAKE_MS) setState(ST_EDGE_BACK);
      break;

    case ST_EDGE_BACK:
      motorL = -EDGE_BACK_SPEED;
      motorR = -EDGE_BACK_SPEED;
      // Хоёр сенсор зэрэг ажилласан бол урд тал бүхэлдээ ирмэг дээр байсан
      // тул арай удаан ухарна.
      if (stateMs >= (uint32_t)((edgeSide == 3) ? (EDGE_BACK_MS + 60) : EDGE_BACK_MS)) {
        setState(ST_EDGE_TURN);
      }
      break;

    case ST_EDGE_TURN: {
      uint32_t turnMs = (edgeSide == 3) ? (EDGE_TURN_MS * 2) : EDGE_TURN_MS;
      if (edgeSide == 1) {  // баруун ирмэг -> зүүн тийш эргэ
        motorL = -EDGE_TURN_SPEED;
        motorR = EDGE_TURN_SPEED;
      } else {  // зүүн ирмэг эсвэл хоёулаа -> баруун тийш эргэ
        motorL = EDGE_TURN_SPEED;
        motorR = -EDGE_TURN_SPEED;
      }
      if (stateMs >= turnMs) {
        edgeRetry = 0;
        setState(sensorTable ? ST_ATTACK : ST_SEARCH);
      }
      break;
    }

    default: break;
  }
}

/* --- AUTOMAT горимын нэг алхам (1 ms) --- */
void automatStep() {
  // 1) Sensor board амьд эсэх - сохроор гүйхгүй
#if REQUIRE_SENSOR_BOARD
  if (!sensorBoardAlive && isRunning()) {
    stopNow(true);
    return;
  }
#endif

  // 2) ИРМЭГ хамгийн өндөр эрхтэй
  bool edge = edgeCheck();
  if (edge && isRunning() && state != ST_COUNTDOWN) {
    bool alreadyEscaping =
        (state == ST_EDGE_BRAKE || state == ST_EDGE_BACK || state == ST_EDGE_TURN);
    if (!alreadyEscaping) {
      motorSnap(0, 0);  // импульсийг ШУУД таслах (ramp хүлээхгүй)
      setState(ST_EDGE_BRAKE);
    } else if (state == ST_EDGE_TURN && edgeRetry < EDGE_RETRY_MAX) {
      // Эргэж байхад дахин ирмэг олдвол дахин ухарна
      edgeRetry++;
      motorSnap(0, 0);
      setState(ST_EDGE_BRAKE);
    }
  }

  // 3) Төлөвийн машин
  switch (state) {
    case ST_IDLE:
      motorL = 0;
      motorR = 0;
      break;

    case ST_COUNTDOWN:
      motorL = 0;
      motorR = 0;
      if (stateMs >= START_COUNTDOWN_MS) setState(ST_OPENING);
      break;

    case ST_OPENING: doOpening(); break;
    case ST_SEARCH: doSearch(); break;
    case ST_ATTACK: doAttack(); break;

    case ST_EDGE_BRAKE:
    case ST_EDGE_BACK:
    case ST_EDGE_TURN: doEdge(); break;
  }
}

/* --- MANUAL горим (хоцрогдолгүй: кэшлэсэн утгаас шууд) --- */
static inline int16_t stickToSpeed(int32_t v, int16_t maxSpeed) {
  if (v > -STICK_DEADZONE && v < STICK_DEADZONE) return 0;
  int32_t s = ((int32_t)(-v) * maxSpeed) / 512;  // stick дээш = урагш
  return (int16_t)constrain(s, -maxSpeed, maxSpeed);
}

void manualStep() {
  if (!padConnected) {
    motorL = 0;
    motorR = 0;
    return;
  }

  int16_t maxSpeed = (padButtons & BUTTON_R1) ? MANUAL_SPEED_BOOST : MANUAL_SPEED_NORMAL;
  motorL = stickToSpeed(padAxisY, maxSpeed);
  motorR = stickToSpeed(padAxisRY, maxSpeed);

  if (padButtons & BUTTON_R2) {  // байрандаа баруун тийш
    int16_t v = (int16_t)(padThrottle / 10);
    motorL = v;
    motorR = -v;
  }
  if (padButtons & BUTTON_L2) {  // байрандаа зүүн тийш
    int16_t v = (int16_t)(padBrake / 10);
    motorL = -v;
    motorR = v;
  }
  if (padButtons & BUTTON_L1) {  // бүрэн урагш
    motorL = 100;
    motorR = 100;
  }
}

/* ========================= GAMEPAD (Bluepad32) ========================= */

void onConnectedController(ControllerPtr ctl) {
  ControllerProperties properties = ctl->getProperties();
  String model = ctl->getModelName();

  if (myController == nullptr && (model.startsWith("DualShock") || model.startsWith("DualSense"))) {
    Serial.println("PlayStation controller connected");
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                  model.c_str(), properties.vendor_id, properties.product_id);
    myController = ctl;
    padConnected = true;
    gameMode = 1;  // гар удирдлага руу шилжинэ
    stopNow(true);
  } else {
    Serial.println("Non-PlayStation controller attempted connection but was rejected.");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("Controller disconnected");
    myController = nullptr;
    padConnected = false;
    padButtons = 0;
    padDpad = 0;
    padAxisY = padAxisRY = padThrottle = padBrake = 0;
    gameMode = 0;   // автомат руу буцна
    stopNow(true);  // тасарсан агшинд мотор зогсоно
  }
}

// loop бүрт дуудагдана -> gamepad утга бараг агшин зуур шинэчлэгдэнэ
void padPoll() {
  if (!BP32.update()) return;
  if (myController && myController->isConnected()) {
    padConnected = true;
    padAxisY = myController->axisY();
    padAxisRY = myController->axisRY();
    padThrottle = myController->throttle();
    padBrake = myController->brake();
    padButtons = myController->buttons();
    padDpad = myController->dpad();
    shutDownCounter = 0;
  }
}

// 1 ms тутам: gamepad-ын товчнуудын логик
void padButtonsTask() {
  static uint16_t sqHold = 0, oHold = 0, r1Hold = 0;
  static uint16_t upHold = 0, downHold = 0;

  if (!padConnected) {
    sqHold = oHold = r1Hold = upHold = downHold = 0;
    return;
  }

  // SQUARE 200ms дарвал AUTO <-> MANUAL
  if (padButtons & BUTTON_SQUARE) {
    if (sqHold < 1000) sqHold++;
    if (sqHold == 200) {
      gameMode = (gameMode == 0) ? 1 : 0;
      stopNow(true);
      if (myController) myController->playDualRumble(0, 250, 0x80, 0x80);
    }
  } else sqHold = 0;

  if (gameMode != 0) return;

  // D-pad дээш/доош: стратеги солих
  if (padDpad & DPAD_UP) {
    if (upHold < 1000) upHold++;
    if (upHold == 5) {
      battleMode++;
      if (battleMode > 7) battleMode = 2;
    }
  } else upHold = 0;

  if (padDpad & DPAD_DOWN) {
    if (downHold < 1000) downHold++;
    if (downHold == 5) {
      battleMode--;
      if (battleMode < 2) battleMode = 7;
    }
  } else downHold = 0;

  // R1: эхлүүлэх (зөвхөн Start модуль ХОЛБООГҮЙ үед)
  if (padButtons & BUTTON_R1) {
    if (r1Hold < 1000) r1Hold++;
    if (r1Hold == 10 && !startModulePresent && state == ST_IDLE) beginRun();
  } else r1Hold = 0;

  // O: зогсоох
  if (padButtons & BUTTON_O) {
    if (oHold < 1000) oHold++;
    if (oHold == 10) stopNow(true);
  } else oHold = 0;
}

/* ============================== ТОВЧ =================================== */
void keyScan() {
  // ---- BUTTON1 (GPIO0) ----
  if (digitalRead(BUTTON1) == LOW) {
    if (btn1HoldMs < 60000) btn1HoldMs++;
    if (btn1HoldMs == 200) stopNow(true);          // 200 ms -> зогс
    if (btn1HoldMs == 500) digitalWrite(POWER_PIN, LOW);  // 500 ms -> тэжээл off
  } else {
    btn1HoldMs = 0;
  }

  // ---- BUTTON2 (GPIO25): tap / double tap / hold ----
  uint32_t now = millis();
  int reading = digitalRead(BUTTON2);
  if (reading != lastButtonState) lastDebounceTime = now;
  if ((now - lastDebounceTime) > debounceDelay && reading != buttonState) buttonState = reading;

  if (buttonState == LOW && !flag2) {
    presstime = now;
    flag1 = false;
    flag2 = true;
    tapCounter++;
  }
  if (buttonState == HIGH && !flag1) {
    releasetime = now;
    flag1 = true;
    flag2 = false;
    timediff = releasetime - presstime;
  }

  if ((now - presstime) > 400 && buttonState == HIGH && tapCounter > 0) {
    shutDownCounter = 0;
    switch (tapCounter) {
      case 1:
        if (timediff >= 400) {
          // Удаан дарах = эхлүүлэх/зогсоох.
          // Start модуль холбоотой бол зогсоох/эхлүүлэх эрхийг ЗӨВХӨӨН
          // тэр эзэмшинэ (даалгаврын шаардлага).
          if (state != ST_IDLE) {
            stopNow(true);
          } else if (!startModulePresent) {
            beginRun();
          }
        } else {
          battleMode++;
          if (battleMode > 7) battleMode = 2;
        }
        break;
      case 2:
        if (timediff < 400) {
          battleMode--;
          if (battleMode < 2) battleMode = 7;
        }
        break;
      case 3:
        shutDownCounter = THREE_MIN;  // тэжээл унтраах
        break;
      default: break;
    }
    tapCounter = 0;
  }
  lastButtonState = reading;
}

/* ============================ ТЕЛЕМЕТР ================================= */
#if DEBUG_PRINT
void telemetry() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 200) return;      // 5 Hz
  if (isRunning()) return;           // ТУЛААНЫ үед хэвлэхгүй (хоцрогдол үүсгэнэ)
  if (Serial.availableForWrite() < 100) return;  // блоклохоос сэргийлнэ
  last = now;

  Serial.printf("tab=%02X L=%3u R=%3u | refL=%3u refR=%3u | ir:%3u %3u %3u %3u %3u %3u | "
                "start=%d/%d board=%d ok=%lu err=%lu mode=%s strat=%u\n",
                sensorTable, lineL.raw, lineR.raw, lineL.blackRef, lineR.blackRef,
                sensorBuffer[2], sensorBuffer[3], sensorBuffer[4],
                sensorBuffer[5], sensorBuffer[6], sensorBuffer[7],
                (int)startLevel, (int)startModulePresent, (int)sensorBoardAlive,
                (unsigned long)frameOkCount, (unsigned long)frameErrCount,
                gameMode ? "MANUAL" : "AUTO", battleMode);
}
#endif

/* ============================== SETUP ================================== */
void setup() {
  Serial.begin(115200);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(START_PIN, START_PIN_MODE);
  pinMode(POWER_PIN, OUTPUT);

  // --- Тэжээл барих (хуучинтай ижил): BUTTON2 дарагдсан хэвээр байвал асна
  int debounce = 0;
  for (int a = 0; a < 100; a++) {
    debounce = (digitalRead(BUTTON2) == LOW) ? debounce + 1 : 0;
    delay(1);
  }
  bool forgetKeys = false;
  if (debounce > 50) {
    digitalWrite(POWER_PIN, HIGH);
    // BUTTON1-ийг мөн дарсан бол Bluetooth түлхүүрүүдийг устгана
    if (digitalRead(BUTTON1) == LOW) forgetKeys = true;
  }

  Serial2.setRxBufferSize(512);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
  Serial.println("Mini sumo start...");

  // --- NeoPixel ---
  pixels.begin();
  pixels.clear();
  pixels.show();
  for (int i = 0; i < NUMPIXELS; i++) {
    for (int a = 0; a < 50; a++) {
      pixels.setPixelColor(i, pixels.Color(a, a, a));
      pixels.show();
      delay(4);
    }
  }
  for (int i = 0; i < NUMPIXELS; i++) {
    for (int a = 50; a >= 0; a--) {
      pixels.setPixelColor(i, pixels.Color(a, a, a));
      pixels.show();
      delay(4);
    }
  }
  pixels.clear();
  pixels.show();

  // --- Мотор PWM (гаралтын индекс хуучнаараа 1..4) ---
  pwmSetup();
  motorSnap(0, 0);  // тоормослосон төлөв

  // --- 100 us timer ---
  timerSetup100us();

  // --- Bluepad32 ---
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  // Түлхүүрийг БҮР УДАА устгахгүй -> дахин холбогдох нь хурдан, найдвартай.
  // Устгах шаардлагатай бол асаахдаа BUTTON2 + BUTTON1-ийг хамт дар.
  if (forgetKeys) {
    Serial.println("Forgetting bluetooth keys...");
    BP32.forgetBluetoothKeys();
  }

  lineFilterReset(lineR);
  lineFilterReset(lineL);
  lineR.blackRef = 0;
  lineL.blackRef = 0;

  battleMode = 2;
  gameMode = 0;
  setState(ST_IDLE);

  // Асаахад дарж байсан BUTTON2 нь "нэг товшилт" болж стратеги солихоос
  // сэргийлж товчны төлөвийг эхлүүлнэ.
  lastButtonState = digitalRead(BUTTON2);
  buttonState = lastButtonState;
  lastDebounceTime = millis();
  presstime = releasetime = millis();
  timediff = 0;
  flag1 = true;
  flag2 = false;
  tapCounter = 0;
}

/* =============================== LOOP ================================== */
void loop() {
  // 1) Хамгийн хурдан ажиллах ёстой зүйлс - loop бүрт
  padPoll();    // gamepad хоцрогдолгүй байх үндэс
  usartPoll();  // sensor board-ын бүх байт

  // 2) 1 ms-ийн тогтмол алхам (алдагдсан tick-ийг нөхнө)
  static uint16_t tickAccum = 0;
  uint16_t t;
  portENTER_CRITICAL(&timerMux);
  t = tick100us;
  tick100us = 0;
  portEXIT_CRITICAL(&timerMux);
  tickAccum += t;

  uint8_t guard = 5;  // нэг loop-д дээд тал нь 5 ms нөхнө
  while (tickAccum >= 10 && guard--) {
    tickAccum -= 10;
    controlTick1ms();
  }

  // 3) Удаан ажлууд
  ledTask();
#if DEBUG_PRINT
  telemetry();
#endif
}

/* --------------------- 1 ms тутамд ажиллах цөм ------------------------ */
void controlTick1ms() {
  stateMs++;

  /* --- START МОДУЛЬ: kill switch (хамгийн түрүүнд) --- */
  startModuleScan();

  if (startEventFall) {  // HIGH -> LOW : ШУУД ЗОГС
    startEventFall = false;
    startEventRise = false;
    stopNow(true);
    Serial.println("START module: FALLING -> STOP");
  }
  if (startEventRise) {  // LOW -> HIGH : эхлэх
    startEventRise = false;
    shutDownCounter = 0;
    if (gameMode == 0 && state == ST_IDLE) {
      Serial.println("START module: RISING -> RUN");
      beginRun();
    }
  }

  /* --- Товч --- */
  keyScan();
  padButtonsTask();

  /* --- Горим --- */
  if (gameMode == 0) {
    automatStep();
  } else {
    manualStep();
  }

  /* --- Хөдөлгөөний хаалт (Start модуль) --- */
  bool blocked = false;
  if (gameMode == 0) {
    blocked = !motionAllowed();
  } else {
#if START_GATE_APPLIES_TO_MANUAL
    blocked = !motionAllowed();
#endif
  }
  if (blocked) {
    motorL = 0;
    motorR = 0;
    if (gameMode == 0 && isRunning()) setState(ST_IDLE);
  }

  /* --- Мотор гаргах --- */
  if (gameMode == 1) {
    motorUpdate(motorL, motorR, ACCEL_MANUAL, DECEL_MANUAL);
  } else {
    motorUpdate(motorL, motorR, ACCEL_AUTO, DECEL_AUTO);
  }

  /* --- Тэжээлийн автомат унтраалт --- */
  if (isRunning() || padConnected) shutDownCounter = 0;
  else shutDownCounter++;
  if (shutDownCounter > THREE_MIN) digitalWrite(POWER_PIN, LOW);
}
