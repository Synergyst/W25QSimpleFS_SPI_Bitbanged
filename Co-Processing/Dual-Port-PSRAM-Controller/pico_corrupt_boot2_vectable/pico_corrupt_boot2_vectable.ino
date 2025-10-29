#include <Arduino.h>
extern "C" {
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
}

static void __not_in_flash_func(corrupt_erase_first_sector)() {
  uint32_t irq = save_and_disable_interrupts();
  // Offset 0 = very start of your flash image (boot2 + vector table live here)
  flash_range_erase(0x00000000u, 4096);
  restore_interrupts(irq);

  // Reboot; the device won't boot your app anymore.
  delay(20);
  watchdog_reboot(0, 0, 0);
  while (true) {}
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(20); }
  Serial.println("About to ERASE first 4KB of internal flash in 5.5s...");
  delay(500);
  Serial.println("About to ERASE first 4KB of internal flash in 5s...");
  for (int i = 5; i > 0; --i) {
    Serial.printf(" %d\n", i);
    delay(1000);
  }
  corrupt_erase_first_sector();
}

void loop() {}