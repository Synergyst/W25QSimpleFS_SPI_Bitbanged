// Auto-detect which Pico GPIO is wired to ATtiny PB3 (BUS_ACTIVE).
// It asserts PA0 low and scans all GPIOs for a HIGH transition.
//
// Wiring per your swap-in harness (you already have this):
//  - ATtiny PA0..PA7 connected to Pico GP16,17,22,11,18,19,20,21 respectively.
//  - ATtiny PB3 is wired to some Pico GPIO (unknown) — we will auto-find it.

#define CS0_PIN 16  // PA0 <- GP16 (we will assert this one)
#define CS1_PIN 17
#define CS2_PIN 22
#define CS3_PIN 11
#define CS4_PIN 18
#define CS5_PIN 19
#define CS6_PIN 20
#define CS7_PIN 21

#include <Arduino.h>

// Treat CS lines as open-drain: OUTPUT LOW = assert, INPUT = release
void cs_release_all() {
  pinMode(CS0_PIN, INPUT);
  pinMode(CS1_PIN, INPUT);
  pinMode(CS2_PIN, INPUT);
  pinMode(CS3_PIN, INPUT);
  pinMode(CS4_PIN, INPUT);
  pinMode(CS5_PIN, INPUT);
  pinMode(CS6_PIN, INPUT);
  pinMode(CS7_PIN, INPUT);
}
void cs0_assert() {
  pinMode(CS0_PIN, OUTPUT);
  digitalWrite(CS0_PIN, LOW);
}

// Return true if this pin number is one of the CS pins (we won’t scan those)
bool is_cs_pin(int p) {
  return p == CS0_PIN || p == CS1_PIN || p == CS2_PIN || p == CS3_PIN || p == CS4_PIN || p == CS5_PIN || p == CS6_PIN || p == CS7_PIN;
}

void setup() {
  Serial.begin(115200);
  delay(4000);
  Serial.println(F("[BUS-DETECT] Releasing all CS lines (Hi-Z)…"));
  cs_release_all();
  delay(20);  // let external pull-ups settle

  // Configure all non-CS GPIOs as INPUT and sample baseline
  const int MAX_GPIO = 29;  // RP2040 GP0..GP28
  uint8_t base[30] = { 0 }, after[30] = { 0 };

  for (int p = 0; p <= MAX_GPIO; ++p) {
    if (is_cs_pin(p)) continue;
    pinMode(p, INPUT);
    delayMicroseconds(5);
    base[p] = (uint8_t)digitalRead(p);
  }

  Serial.println(F("[BUS-DETECT] Asserting PA0 low (OUTPUT LOW)…"));
  cs0_assert();
  delay(20);  // plenty for the ATtiny loop to update PB3

  // Resample after assertion
  for (int p = 0; p <= MAX_GPIO; ++p) {
    if (is_cs_pin(p)) continue;
    after[p] = (uint8_t)digitalRead(p);
  }

  // Report pins that toggled LOW->HIGH (candidate BUS lines)
  Serial.println(F("[BUS-DETECT] Candidates (LOW->HIGH on assert):"));
  bool foundAny = false;
  for (int p = 0; p <= MAX_GPIO; ++p) {
    if (is_cs_pin(p)) continue;
    if (base[p] == 0 && after[p] == 1) {
      Serial.print(F("  GP"));
      Serial.println(p);
      foundAny = true;
    }
  }
  if (!foundAny) {
    Serial.println(F("  None found — PB3 not at any scanned pin or ATtiny not asserting."));
    Serial.println(F("  Hints:"));
    Serial.println(F("   - Ensure the 74HC32 emulator is running and powered"));
    Serial.println(F("   - Confirm PB3 is actually wired to a Pico GPIO (common choices: GP8, GP18, GP22)"));
    Serial.println(F("   - If PB3 is jumpered to the Arbiter’s BUS0 net (PA2), try GP22"));
  }

  // Cleanup
  cs_release_all();
}

void loop() {}