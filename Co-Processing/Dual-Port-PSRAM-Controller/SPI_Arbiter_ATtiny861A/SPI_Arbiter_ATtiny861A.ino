// ATtiny861A Arbiter v1.3 (PA/PB direct, avoids PB0/PB1/PB2/PB7)
// 8 MHz internal, BOD ≈ 3.0 V. Program via PB0/PB1/PB2, RESET=PB7.
// Inputs:  REQ_A=PA0, REQ_B=PA1, BUS0=PA2 (BUS1 optional)
// Outputs: SEL=PB3, OE=PB4, OWNER_A=PA4, OWNER_B=PA5, PREV_A=PA6, PREV_B=PA7, IRQ_A=PB5, IRQ_B=PB6

#include <Arduino.h>
#include <avr/wdt.h>

// ===== Pin map (edit if needed) =====
#define REQ_A_BIT PA0
#define REQ_B_BIT PA1
#define BUS0_BIT PA2
#define USE_BUS1 0
#define BUS1_BIT PA3

#define SEL_BIT PB3
#define OE_BIT PB4
#define OWNER_A_BIT PA4
#define OWNER_B_BIT PA5
#define PREV_A_BIT PA6
#define PREV_B_BIT PA7
#define IRQ_A_BIT PB5
#define IRQ_B_BIT PB6

// Polarity/config
const bool REQ_ACTIVE_LOW = true;
const bool BUS_ACTIVE_HIGH = true;

// Timing
const uint16_t DEAD_US = 5;
const uint16_t IRQ_PULSE_MS = 2;
const uint32_t FORCE_TIMEOUT_MS = 0;  // 0=off

// ===== Owner constants (no custom types in signatures) =====
#define OWN_NONE 0u
#define OWN_A 1u
#define OWN_B 2u

// ===== Bit ops =====
#define SBI(reg, bit) ((reg) |= _BV(bit))
#define CBI(reg, bit) ((reg) &= ~_BV(bit))
#define TST(reg, bit) (((reg)&_BV(bit)) != 0)

// Fast I/O helpers
inline void setOutputA(uint8_t bit) {
  SBI(DDRA, bit);
}
inline void setInputA_PU(uint8_t bit) {
  CBI(DDRA, bit);
  SBI(PORTA, bit);
}
inline void setHighA(uint8_t bit) {
  SBI(PORTA, bit);
}
inline void setLowA(uint8_t bit) {
  CBI(PORTA, bit);
}
inline bool readA(uint8_t bit) {
  return TST(PINA, bit);
}

inline void setOutputB(uint8_t bit) {
  SBI(DDRB, bit);
}
inline void setInputB_PU(uint8_t bit) {
  CBI(DDRB, bit);
  SBI(PORTB, bit);
}
inline void setHighB(uint8_t bit) {
  SBI(PORTB, bit);
}
inline void setLowB(uint8_t bit) {
  CBI(PORTB, bit);
}
inline bool readB(uint8_t bit) {
  return TST(PINB, bit);
}

// ===== State =====
uint8_t owner = OWN_NONE;
uint8_t prevOwner = OWN_B;  // first tie grants A
bool lastBusActive = false;
uint32_t busActiveSince = 0;

// ===== Signals =====
inline bool readReqA() {
  bool v = readA(REQ_A_BIT);
  return REQ_ACTIVE_LOW ? !v : v;
}
inline bool readReqB() {
  bool v = readA(REQ_B_BIT);
  return REQ_ACTIVE_LOW ? !v : v;
}
inline bool readBusActive() {
  bool a = readA(BUS0_BIT);
#if USE_BUS1
  bool b = readA(BUS1_BIT);
  bool raw = a || b;
#else
  bool raw = a;
#endif
  return BUS_ACTIVE_HIGH ? raw : !raw;
}

// SN74CBTLV3257 control (OE active-low)
inline void setOE(bool enable) {
  if (enable) setLowB(OE_BIT);
  else setHighB(OE_BIT);
}
inline void setSEL(uint8_t o) {
  (o == OWN_B) ? setHighB(SEL_BIT) : setLowB(SEL_BIT);
}

inline void driveOwnerLEDs() {
  (owner == OWN_A) ? setHighA(OWNER_A_BIT) : setLowA(OWNER_A_BIT);
  (owner == OWN_B) ? setHighA(OWNER_B_BIT) : setLowA(OWNER_B_BIT);
  (prevOwner == OWN_A) ? setHighA(PREV_A_BIT) : setLowA(PREV_A_BIT);
  (prevOwner == OWN_B) ? setHighA(PREV_B_BIT) : setLowA(PREV_B_BIT);
}
inline void pulseIRQ_B(uint8_t bit) {
  SBI(PORTB, bit);
  delay(IRQ_PULSE_MS);
  CBI(PORTB, bit);
}

inline void disconnectBus() {
  setOE(false);
}
inline void breakBeforeMakeTo(uint8_t next) {
  setOE(false);
  delayMicroseconds(DEAD_US);
  setSEL(next);
  delayMicroseconds(DEAD_US);
  setOE(true);
}

inline uint8_t chooseGrant(bool reqA, bool reqB) {
  if (reqA && !reqB) return OWN_A;
  if (reqB && !reqA) return OWN_B;
  if (reqA && reqB) return (prevOwner == OWN_A) ? OWN_B : OWN_A;
  return OWN_NONE;
}
inline void grant(uint8_t o) {
  breakBeforeMakeTo(o);
  owner = o;
  driveOwnerLEDs();
  (o == OWN_A) ? pulseIRQ_B(IRQ_A_BIT) : pulseIRQ_B(IRQ_B_BIT);
}
inline void releaseToNone() {
  disconnectBus();
  prevOwner = owner;
  owner = OWN_NONE;
  driveOwnerLEDs();
  pulseIRQ_B(IRQ_A_BIT);
  pulseIRQ_B(IRQ_B_BIT);
}

// ===== Setup/Loop =====
void setup() {
  // Inputs
  setInputA_PU(REQ_A_BIT);
  setInputA_PU(REQ_B_BIT);
  CBI(PORTA, BUS0_BIT);
  CBI(DDRA, BUS0_BIT);
#if USE_BUS1
  CBI(PORTA, BUS1_BIT);
  CBI(DDRA, BUS1_BIT);
#endif

  // Outputs
  setOutputB(SEL_BIT);
  setOutputB(OE_BIT);
  setOutputA(OWNER_A_BIT);
  setOutputA(OWNER_B_BIT);
  setOutputA(PREV_A_BIT);
  setOutputA(PREV_B_BIT);
  setOutputB(IRQ_A_BIT);
  setOutputB(IRQ_B_BIT);
  CBI(PORTB, IRQ_A_BIT);
  CBI(PORTB, IRQ_B_BIT);

  // Safe defaults
  setSEL(OWN_A);    // arbitrary
  disconnectBus();  // OE high
  owner = OWN_NONE;
  driveOwnerLEDs();

  wdt_enable(WDTO_120MS);

  lastBusActive = readBusActive();
  busActiveSince = millis();
}

void loop() {
  wdt_reset();

  const bool reqA = readReqA();
  const bool reqB = readReqB();
  const bool busAct = readBusActive();

  if (busAct != lastBusActive) {
    lastBusActive = busAct;
    busActiveSince = millis();
  }

  if (FORCE_TIMEOUT_MS > 0 && owner != OWN_NONE && busAct) {
    if (millis() - busActiveSince > FORCE_TIMEOUT_MS) {
      disconnectBus();
      owner = OWN_NONE;
      driveOwnerLEDs();
      pulseIRQ_B(IRQ_A_BIT);
      pulseIRQ_B(IRQ_B_BIT);
    }
  }

  switch (owner) {
    case OWN_NONE:
      {
        uint8_t g = chooseGrant(reqA, reqB);
        if (g != OWN_NONE) grant(g);
      }
      break;

    case OWN_A:
      {
        if (!reqA && !busAct) releaseToNone();
      }
      break;

    case OWN_B:
      {
        if (!reqB && !busAct) releaseToNone();
      }
      break;
  }

  delayMicroseconds(50);
}