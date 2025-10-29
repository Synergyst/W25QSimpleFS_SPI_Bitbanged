// ATtiny861A BUS_ACTIVE Aggregator v0.1
// BUS_ACTIVE_OUT = HIGH if any CS_n input on PA0..PA7 is LOW
// Test now: drive PA0 from Pico GP22; connect PB3 -> Arbiter Tiny PA2.
#include <Arduino.h>

// Inputs: CS_n lines on Port A (PA0..PA7)
const uint8_t CS_MASK = 0xFF;  // use PA0..PA7

// Output: BUS_ACTIVE (active-HIGH)
const uint8_t PIN_BUS_OUT = PB3;

inline void setOutputB(uint8_t bit) {
  DDRB |= _BV(bit);
}
inline void setHighB(uint8_t bit) {
  PORTB |= _BV(bit);
}
inline void setLowB(uint8_t bit) {
  PORTB &= ~_BV(bit);
}
void setup() {
  // PA as inputs, no pull-ups (external drivers like MCP/Pico will drive them)
  DDRA = 0x00;
  PORTA = 0x00;

  setOutputB(PIN_BUS_OUT);
  setLowB(PIN_BUS_OUT);  // idle = LOW (no CS active)
}
void loop() {
  uint8_t pa = PINA;
  // Any CS_n low? -> (~pa & CS_MASK) != 0
  bool anyActive = ((~pa) & CS_MASK) != 0;
  if (anyActive) setHighB(PIN_BUS_OUT);
  else setLowB(PIN_BUS_OUT);

  // Optional tiny glitch filter (comment out if not needed)
  // delayMicroseconds(1);
}