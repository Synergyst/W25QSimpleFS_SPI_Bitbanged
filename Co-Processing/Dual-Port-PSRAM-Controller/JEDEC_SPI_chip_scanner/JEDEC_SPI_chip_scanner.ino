#include <Arduino.h>
#include "UnifiedSPIMem.h"
#include "UnifiedSPIMemTests.h"

// SPI pins (adjust to your board/wiring)
static const uint8_t PIN_SCK = 10;
static const uint8_t PIN_MOSI = 11;
static const uint8_t PIN_MISO = 12;
static const int8_t PIN_WP = -1;
static const int8_t PIN_HOLD = -1;

// CS pins to scan
static const uint8_t CS_PINS[] = { 9, 14, 15 };
static const size_t CS_COUNT = sizeof(CS_PINS) / sizeof(CS_PINS[0]);

static const bool RUN_SAFE_AT_BOOT = true;
static const bool RUN_DESTRUCTIVE_AT_BOOT = true;  // WARNING: alters tail region on each device

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(50); }
  delay(200);

  for (size_t i = 0; i < CS_COUNT; ++i) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }

  UnifiedSpiMem::SuiteConfig cfg;
  cfg.pinSCK = PIN_SCK;
  cfg.pinMOSI = PIN_MOSI;
  cfg.pinMISO = PIN_MISO;
  cfg.pinWP = PIN_WP;
  cfg.pinHOLD = PIN_HOLD;
  cfg.csList = CS_PINS;
  cfg.csCount = CS_COUNT;
  cfg.console = &Serial;
  cfg.printDeviceTable = true;

  // Destructive suite options (generic, driver-only)
  cfg.regionBytes = 0;      // auto: eraseSize, else pageSize, else 4 KiB
  cfg.tryTailRegions = 4;   // try a few trailing regions if the last one fails
  cfg.alignToErase = true;  // align to erase boundary when available
  cfg.restoreAfter = true;  // try to restore after verification
  cfg.restoreFill = 0x00;   // for devices without erase (PSRAM)

  if (RUN_SAFE_AT_BOOT) {
    Serial.println("\nRunning SAFE (non-destructive) test suite...");
    bool ok = UnifiedSpiMem::RunSafeSuite(cfg);
    Serial.println(ok ? "SAFE SUITE: ALL TESTS PASSED" : "SAFE SUITE: SOME TESTS FAILED");
  }

  if (RUN_DESTRUCTIVE_AT_BOOT) {
    Serial.println("\nRunning DESTRUCTIVE (generic) test suite...");
    bool ok = UnifiedSpiMem::RunDestructiveSuite(cfg);
    Serial.println(ok ? "DESTRUCTIVE SUITE: ALL TESTS PASSED" : "DESTRUCTIVE SUITE: SOME TESTS FAILED");
  }
}

void loop() {
  delay(1000);
}