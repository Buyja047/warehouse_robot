#include <Arduino.h>

// ===================== PINS (your mapping) =====================
// PUSH
static const int PUSH_INA = 1;
static const int PUSH_INB = 2;
static const int PUSH_PWM = 42;
static const int FRONT_LIMIT = 17;
static const int BACK_LIMIT  = 16;

// LIFT
static const int LIFT_INA = 41;
static const int LIFT_INB = 40;
static const int LIFT_PWM = 39;

// LEFT WHEEL
static const int L_INA = 38;
static const int L_INB = 37;
static const int L_PWM = 36;
static const int ENC_L = 5;

// RIGHT WHEEL
static const int R_INA = 35;
static const int R_INB = 0;
static const int R_PWM = 45;
static const int ENC_R = 4;

// IR sensors
static const int IR_R = 6;
static const int IR_M = 7;
static const int IR_L = 15;

// ===================== CONFIG =====================
static const bool IR_ACTIVE_LOW    = true;  // HW-871 ихэнхдээ хар дээр LOW
static const bool LIMIT_ACTIVE_LOW = true;  // INPUT_PULLUP => pressed LOW

static const int PWM_FREQ = 20000;
static const int PWM_RES  = 10;
static const int PWM_MAX  = (1 << PWM_RES) - 1;

static int DUTY = 500;       // default duty (0..1023)
static int LIFT_DUTY = 1023; //lift speed
static int TURN_DUTY = 450;

// Motor invert toggles (if direction is opposite)
static bool INV_L = false;
static bool INV_R = false;
static bool INV_PUSH = false;
static bool INV_LIFT = false;

// ===================== ENCODERS =====================
volatile long encL = 0;
volatile long encR = 0;
void IRAM_ATTR isrEncL() { encL++; }
void IRAM_ATTR isrEncR() { encR++; }

static inline void resetEnc() {
  noInterrupts();
  encL = 0; encR = 0;
  interrupts();
}

static inline bool readActive(int pin, bool activeLow) {
  int v = digitalRead(pin);
  return activeLow ? (v == LOW) : (v == HIGH);
}

static inline bool frontPressed() { return readActive(FRONT_LIMIT, LIMIT_ACTIVE_LOW); }
static inline bool backPressed()  { return readActive(BACK_LIMIT,  LIMIT_ACTIVE_LOW); }

static inline int clampDuty(int d) {
  if (d < 0) return 0;
  if (d > PWM_MAX) return PWM_MAX;
  return d;
}

static inline int applyInv(bool inv, int v) { return inv ? -v : v; }

// ===================== MOTOR DRIVE (pin-based LEDC) =====================
static void motorDrive(int ina, int inb, int pwmPin, int dutySigned, bool invFlag) {
  dutySigned = applyInv(invFlag, dutySigned);

  int d = dutySigned;
  if (d > 0) {
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);
  } else if (d < 0) {
    digitalWrite(ina, LOW);
    digitalWrite(inb, HIGH);
    d = -d;
  } else {
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);
  }
  ledcWrite(pwmPin, clampDuty(d));
}

static void wheels(int leftDuty, int rightDuty) {
  motorDrive(L_INA, L_INB, L_PWM, leftDuty,  INV_L);
  motorDrive(R_INA, R_INB, R_PWM, rightDuty, INV_R);
}

static void push(int duty) {
  // safety by limits
  if (duty > 0 && frontPressed()) duty = 0;
  if (duty < 0 && backPressed())  duty = 0;
  motorDrive(PUSH_INA, PUSH_INB, PUSH_PWM, duty, INV_PUSH);
}

static void lift(int duty) {
  motorDrive(LIFT_INA, LIFT_INB, LIFT_PWM, duty, INV_LIFT);
}

static void allStop() {
  wheels(0,0);
  push(0);
  lift(0);
}

// ===================== STATUS =====================
static void printStatus() {
  bool L = readActive(IR_L, IR_ACTIVE_LOW);
  bool M = readActive(IR_M, IR_ACTIVE_LOW);
  bool R = readActive(IR_R, IR_ACTIVE_LOW);

  noInterrupts();
  long el = encL, er = encR;
  interrupts();

  Serial.printf("IR:%d%d%d FL:%d BL:%d EncL:%ld EncR:%ld DUTY:%d\n",
                L,M,R, frontPressed(), backPressed(), el, er, DUTY);
}

static void help() {
  Serial.println("\nCommands:");
  Serial.println("i          -> print sensors");
  Serial.println("w/s/a/d    -> wheels forward/stop/spinL/spinR");
  Serial.println("p/o        -> push fwd/back (limit safe)");
  Serial.println("u/j        -> lift up/down");
  Serial.println("x          -> ALL stop");
  Serial.println("e          -> reset encoders");
  Serial.println("v N        -> set duty 0..1023 (ex: v 500)");
  Serial.println();
}

// ===================== SERIAL CMD =====================
static String line;

static void handleCmd(String cmd) {
  cmd.trim();
  if (cmd.length()==0) return;

  if (cmd == "h") { help(); return; }
  if (cmd == "i") { printStatus(); return; }
  if (cmd == "x") { allStop(); Serial.println("STOP"); return; }
  if (cmd == "e") { resetEnc(); Serial.println("Enc reset"); return; }

  if (cmd == "w") { wheels(DUTY, DUTY); Serial.println("WHEELS FWD"); return; }
  if (cmd == "s") { wheels(0,0); Serial.println("WHEELS STOP"); return; }
  if (cmd == "a") { wheels(-TURN_DUTY, TURN_DUTY); Serial.println("SPIN LEFT"); return; }
  if (cmd == "d") { wheels(TURN_DUTY, -TURN_DUTY); Serial.println("SPIN RIGHT"); return; }

  if (cmd == "p") { push(DUTY); Serial.println("PUSH FWD"); return; }
  if (cmd == "o") { push(-DUTY); Serial.println("PUSH BACK"); return; }

  if (cmd == "u") { lift(LIFT_DUTY); Serial.println("LIFT UP"); return; }
  if (cmd == "j") { lift(-LIFT_DUTY); Serial.println("LIFT DOWN"); return; }

  if (cmd.startsWith("v ")) {
    int v = cmd.substring(2).toInt();
    DUTY = constrain(v, 0, PWM_MAX);
    Serial.printf("DUTY=%d\n", DUTY);
    return;
  }

  Serial.println("Unknown. Type h");
}

// ===================== SETUP/LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(PUSH_INA, OUTPUT); pinMode(PUSH_INB, OUTPUT);
  pinMode(LIFT_INA, OUTPUT); pinMode(LIFT_INB, OUTPUT);
  pinMode(L_INA, OUTPUT);    pinMode(L_INB, OUTPUT);
  pinMode(R_INA, OUTPUT);    pinMode(R_INB, OUTPUT);

  // pin-based LEDC attach (core 3.x style, matches your working code)
  bool ok1 = ledcAttach(L_PWM,    PWM_FREQ, PWM_RES);
  bool ok2 = ledcAttach(R_PWM,    PWM_FREQ, PWM_RES);
  bool ok3 = ledcAttach(PUSH_PWM, PWM_FREQ, PWM_RES);
  bool ok4 = ledcAttach(LIFT_PWM, PWM_FREQ, PWM_RES);

  Serial.printf("LEDC attach: L=%d R=%d PUSH=%d LIFT=%d\n", ok1, ok2, ok3, ok4);

  pinMode(IR_L, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_R, INPUT);

  pinMode(FRONT_LIMIT, INPUT_PULLUP);
  pinMode(BACK_LIMIT,  INPUT_PULLUP);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrEncR, RISING);

  resetEnc();
  allStop();
  help();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r') {
      String cmd = line; line = "";
      handleCmd(cmd);
    } else {
      line += c;
      if (line.length() > 80) line = "";
    }
  }

  static uint32_t t0=0;
  if (millis()-t0 > 300) {
    t0 = millis();
    printStatus();
  }
}
