#include "BusArbiterWithISP.h"

void setup() {
  delay(500);
  Serial.begin();
  while (!Serial) { delay(500); }
  delay(500);
  Serial.println("Booting system..");
  ArbiterISP::initTestPins();
  bool ok = ArbiterISP::runTestSuiteOnce();
  ArbiterISP::cleanupToResetState();
  if (!ok) {
    Serial.println("P.O.S.T. failed!\nEntering ISP mode, please exit serial console and reprogram MCU!");
    ArbiterISP::enterISPMode();
    while (!BOOTSEL) ArbiterISP::serviceISPOnce();  // inside loop
    delay(2000);
    ArbiterISP::exitISPMode();
    Serial.println("Exited ISP mode..");
  } else {
    Serial.println("P.O.S.T. success!");
  }
}
void loop() {
  if (BOOTSEL) {
    Serial.println("Rebooting now..");
    delay(1500);
    yield();
    watchdog_reboot(0, 0, 1500);
    while (true) tight_loop_contents();
  }
}