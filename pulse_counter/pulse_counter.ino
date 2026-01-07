#include <Arduino.h>
#include "esp_cpu.h"   // esp_cpu_get_cycle_count()

// ======= ТА ЭНДЭЭС ӨӨРЧИЛНӨ =======
#define ENC_PIN  5
const uint32_t MIN_PULSE_US = 150;     // bounce их бол 150~500 болгож өсгөнө
const float GEAR_RATIO = 3700.0 / 440.0;
// ==================================

volatile uint32_t pulseCount = 0;
volatile uint32_t lastCycle = 0;
volatile bool measuring = false;

uint32_t minCycles = 0;  // MIN_PULSE_US -> CPU cycles

void IRAM_ATTR encISR() {
  uint32_t now = esp_cpu_get_cycle_count();    // ISR-safe
  if ((uint32_t)(now - lastCycle) >= minCycles) {
    lastCycle = now;
    if (measuring) pulseCount++;
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("=== Encoder PPR Measure ===");
  Serial.println("b = begin measuring");
  Serial.println("e = end measuring and compute PPR");
  Serial.println("r = reset counter");
  Serial.println("h = help");
  Serial.println("Заавар: b -> дугуйгаа N бүтэн эргүүл -> e -> N (ж: 10) гэж бичээд Enter");
  Serial.println();
}

float waitFloatFromSerial() {
  // Watchdog-д ээлтэй (blocking үед yield хийнэ)
  while (!Serial.available()) { delay(10); }
  return Serial.parseFloat();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // MIN_PULSE_US-ийг cycle болгон хөрвүүлэх
  // F_CPU = Hz, 1us = F_CPU/1e6 cycles
  minCycles = (uint32_t)((uint64_t)F_CPU * MIN_PULSE_US / 1000000ULL);

  pinMode(ENC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encISR, RISING);

  Serial.println("ESP32-S3 Encoder PPR manual measure (ISR-safe)");
  Serial.print("F_CPU="); Serial.print(F_CPU);
  Serial.print("  MIN_PULSE_US="); Serial.print(MIN_PULSE_US);
  Serial.print("  minCycles="); Serial.println(minCycles);

  printHelp();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'h') {
      printHelp();
    } else if (c == 'r') {
      noInterrupts();
      pulseCount = 0;
      interrupts();
      Serial.println("Counter reset to 0.");
    } else if (c == 'b') {
      noInterrupts();
      pulseCount = 0;
      lastCycle = esp_cpu_get_cycle_count();
      measuring = true;
      interrupts();
      Serial.println("MEASURE STARTED. Дугуйгаа N бүтэн эргүүлээд 'e' дар.");
    } else if (c == 'e') {
      measuring = false;
      delay(30);

      uint32_t total;
      noInterrupts();
      total = pulseCount;
      interrupts();

      Serial.print("Total pulses counted = ");
      Serial.println(total);

      Serial.println("Одоо хэдэн БҮТЭН дугуйн эргэлт хийснээ оруул (ж: 10) Enter:");
      float wheelRevs = waitFloatFromSerial();

      if (wheelRevs <= 0.0f) {
        Serial.println("Invalid wheelRevs (0/negative). Дахин 'b' хийгээд зөв N оруул.");
        return;
      }

      float pulsesPerWheelRev = (float)total / wheelRevs;
      Serial.print("Pulses per WHEEL revolution (1ch RISING) = ");
      Serial.println(pulsesPerWheelRev, 3);

      float estMotorPPR = pulsesPerWheelRev / GEAR_RATIO;
      Serial.print("Estimated pulses per MOTOR revolution (1ch RISING) = ");
      Serial.println(estMotorPPR, 3);

      Serial.println("Дахин хэмжих бол 'b'.");
    }
  }

  // Live count
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    uint32_t c;
    noInterrupts();
    c = pulseCount;
    interrupts();
    Serial.print("count="); Serial.print(c);
    Serial.println(measuring ? " (measuring)" : " (idle)");
  }
}
