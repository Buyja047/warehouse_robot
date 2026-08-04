/* =============================================================================
   PWM ШАЛГАХ SKETCH  —  Arduino Nano + TB6612FNG

   ЗОРИЛГО: analogWrite(PWMA/PWMB, ...) нь моторын хурдыг ҮНЭХЭЭР удирдаж
   байгаа эсэхийг тодорхойлох. Сумо кодод хурдны тоо өөрчлөхөд юу ч болохгүй
   байвал асуудал кодод биш, PWM УТСАНД байж болно.

   >>> РОБОТОО ӨРГӨЖ БАРЬ эсвэл тавцан дээр тавь — дугуй чөлөөтэй эргэх ёстой.
       Ширээн дээр тавьбал доош унана.

   ХОЛБОЛТ: сумо роботынхтой яг ижил пин. Өөр юу ч холбох шаардлагагүй.

   ХАРАХ: Serial Monitor 9600 baud.  (Bluetooth модуль D0/D1 дээр байгаа бол
          USB-ээр харахын тулд түүнийг САЛГА. Эсвэл BT терминалаараа хараарай —
          BT ч мөн 9600 дээр ижил мессежийг хүлээж авна.)

   ҮР ДҮНГ ХЭРХЭН УНШИХ:
     Шат ахих тусам дугуй ХУРДСАЖ байвал  -> PWM ЗӨВ ажиллаж байна.
        => Сумо кодод SPEED_SCALE / PWM_MAX-аар хурдаа тааруулж болно.
     Бүх шат ИЖИЛ хурдтай байвал          -> PWM удирдлага ХҮРЭХГҮЙ байна.
        => PWMA (D3) / PWMB (D9) утас драйверт очихгүй байна, эсвэл драйвер
           дээрх PWM пин VCC рүү холбоостой/jumper-тэй. Ямар ч тоо өөрчилсөн
           хурд буухгүй — утсаа шалга.
     Эхний 1-2 шат огт эргэхгүй бол       -> хэвийн (үхмэл бүс). Сумо кодын
           PWM_MIN-ийг тэр босгоос дээш тавь.
     Нэг мотор эргээд нөгөө нь эргэхгүй   -> тэр талын утас/драйверын суваг.
   ============================================================================= */

// ==========================================
// ПИН (сумо роботынхтой ижил)
// ==========================================
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 8
#define BIN1 6
#define BIN2 7
#define PWMB 9

// ==========================================
// ТОХИРГОО
// ==========================================
#define STEP_MS       2500    // шат тус бүрийн хугацаа
#define PAUSE_MS       800    // шат хоорондын завсар
#define PHASE_PAUSE   1500    // үе шат хоорондын завсар

const int steps[] = { 60, 100, 150, 200, 255 };
const uint8_t NSTEPS = sizeof(steps) / sizeof(steps[0]);

// ==========================================
// МОТОРЫН ТУСЛАХУУД
// ==========================================
// dir: 1 = нэг чиг, -1 = эсрэг, 0 = тоормос.  Энэ тестэд чиглэл хамаагүй.
void motorA(int dir, int pwm) {
  if (dir == 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 255); return; }
  digitalWrite(AIN1, (dir > 0) ? HIGH : LOW);
  digitalWrite(AIN2, (dir > 0) ? LOW  : HIGH);
  analogWrite(PWMA, pwm);
}

void motorB(int dir, int pwm) {
  if (dir == 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 255); return; }
  digitalWrite(BIN1, (dir > 0) ? HIGH : LOW);
  digitalWrite(BIN2, (dir > 0) ? LOW  : HIGH);
  analogWrite(PWMB, pwm);
}

void allOff() {                                  // чөлөөтэй зогсолт
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

// ==========================================
// ҮЕ ШАТУУД
// ==========================================
// which: 'A' = зөвхөн A мотор, 'B' = зөвхөн B, '2' = хоёул
void rampPhase(const char *title, char which) {
  Serial.println();
  Serial.print(F("=== ")); Serial.print(title); Serial.println(F(" ==="));

  for (uint8_t i = 0; i < NSTEPS; i++) {
    int p = steps[i];

    Serial.print(F("  PWM = ")); Serial.print(p);
    Serial.print(F("  (")); Serial.print((int)(p * 100L / 255)); Serial.println(F("%)"));

    if (which == 'A' || which == '2') motorA(1, p); else motorA(0, 0);
    if (which == 'B' || which == '2') motorB(1, p); else motorB(0, 0);

    delay(STEP_MS);
    allOff();
    delay(PAUSE_MS);
  }
}

void directionPhase() {
  Serial.println();
  Serial.println(F("=== 4. CHIGLEL SHALGAH (PWM 200) ==="));

  Serial.println(F("  dir = +1"));
  motorA(1, 200); motorB(1, 200);
  delay(2000);
  allOff(); delay(PAUSE_MS);

  Serial.println(F("  dir = -1  (esreg tijsh ergeh yostoi)"));
  motorA(-1, 200); motorB(-1, 200);
  delay(2000);
  allOff(); delay(PAUSE_MS);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);                      // драйверыг сэрээнэ
  allOff();

  delay(1500);
  Serial.println(F("================================"));
  Serial.println(F(" PWM TEST - ROBOTOO ORGOJ BARI!"));
  Serial.println(F("================================"));
  Serial.println(F("Shat ahih tusam hurdsval -> PWM ZOV"));
  Serial.println(F("Bugd ijil hurdtai bol    -> PWM UTAS HUREHGUI"));
  delay(2000);
}

// ==========================================
// LOOP — мөчлөгөөр давтана
// ==========================================
void loop() {
  rampPhase("1. HOYUL MOTOR", '2');
  delay(PHASE_PAUSE);

  rampPhase("2. ZOVHON A MOTOR (PWMA=D3)", 'A');
  delay(PHASE_PAUSE);

  rampPhase("3. ZOVHON B MOTOR (PWMB=D9)", 'B');
  delay(PHASE_PAUSE);

  directionPhase();

  Serial.println();
  Serial.println(F("--- niit mochlog duuslaa, 5 sek daraa davtana ---"));
  allOff();
  delay(5000);
}
