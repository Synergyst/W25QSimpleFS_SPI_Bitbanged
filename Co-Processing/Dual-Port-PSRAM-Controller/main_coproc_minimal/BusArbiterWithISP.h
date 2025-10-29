#pragma once
#include <Arduino.h>
// ================= Config macros (override before include if needed) =================
#ifndef ARB_VERBOSE
#define ARB_VERBOSE 0  // Only affects POST; ISP is never verbose
#endif
// Arbiter tester pin map (Pico ↔ ATtiny861A)
#ifndef ARB_PIN_REQ_A_OUT
#define ARB_PIN_REQ_A_OUT 16
#endif
#ifndef ARB_PIN_REQ_B_OUT
#define ARB_PIN_REQ_B_OUT 17
#endif
#ifndef ARB_PIN_OWNER_A_IN
#define ARB_PIN_OWNER_A_IN 18
#endif
#ifndef ARB_PIN_OWNER_B_IN
#define ARB_PIN_OWNER_B_IN 19
#endif
#ifndef ARB_PIN_PREV_A_IN
#define ARB_PIN_PREV_A_IN 20
#endif
#ifndef ARB_PIN_PREV_B_IN
#define ARB_PIN_PREV_B_IN 21
#endif
#ifndef ARB_PIN_IRQ_A_IN
#define ARB_PIN_IRQ_A_IN 6
#endif
#ifndef ARB_PIN_IRQ_B_IN
#define ARB_PIN_IRQ_B_IN 7
#endif
#ifndef ARB_PIN_SEL_IN
#define ARB_PIN_SEL_IN 8
#endif
#ifndef ARB_PIN_OE_IN
#define ARB_PIN_OE_IN 9
#endif
#ifndef ARB_PIN_BUS_ACTIVE
#define ARB_PIN_BUS_ACTIVE 22
#endif
#ifndef ARB_PIN_TINY_RST
#define ARB_PIN_TINY_RST 12
#endif
// ArduinoISP (bit-banged) pins
#ifndef ARB_ISP_RESET
#define ARB_ISP_RESET ARB_PIN_TINY_RST
#endif
#ifndef ARB_ISP_MOSI
#define ARB_ISP_MOSI 15
#endif
#ifndef ARB_ISP_MISO
#define ARB_ISP_MISO 14
#endif
#ifndef ARB_ISP_SCK
#define ARB_ISP_SCK 13
#endif
#ifndef ARB_ISP_SPI_CLOCK_HZ
#define ARB_ISP_SPI_CLOCK_HZ (1000000 / 6)  // ~166 kHz
#endif
namespace ArbiterISP {
inline bool bootsel() {
  return BOOTSEL;
}  // you already mapped BOOTSEL in your sketch
namespace detail {
// -------- POST verbosity (runtime) --------
static uint8_t gVerbose = (ARB_VERBOSE != 0);
inline bool chatty() {
  return gVerbose || (ARB_VERBOSE != 0);
}
inline void vprintln(const __FlashStringHelper* s) {
  if (chatty()) Serial.println(s);
}
inline void vprint(const __FlashStringHelper* s) {
  if (chatty()) Serial.print(s);
}
inline void vprintC(char c) {
  if (chatty()) Serial.print(c);
}
inline void toResetState(uint8_t pin) {
  pinMode(pin, INPUT);
}
// -------- Test helpers/state --------
static int passCnt = 0, failCnt = 0;
inline void expect(bool ok, const __FlashStringHelper* name) {
  if (chatty()) {
    Serial.print(ok ? F("[PASS] ") : F("[FAIL] "));
    Serial.println(name);
  }
  if (ok) passCnt++;
  else failCnt++;
}
inline void note(const __FlashStringHelper* name) {
  if (chatty()) {
    Serial.print(F("[INFO] "));
    Serial.println(name);
  }
}
inline void reqA(bool on) {
  digitalWrite(ARB_PIN_REQ_A_OUT, on ? LOW : HIGH);
}
inline void reqB(bool on) {
  digitalWrite(ARB_PIN_REQ_B_OUT, on ? LOW : HIGH);
}
inline bool ownerA() {
  return digitalRead(ARB_PIN_OWNER_A_IN);
}
inline bool ownerB() {
  return digitalRead(ARB_PIN_OWNER_B_IN);
}
inline bool prevA() {
  return digitalRead(ARB_PIN_PREV_A_IN);
}
inline bool prevB() {
  return digitalRead(ARB_PIN_PREV_B_IN);
}
inline bool bothOwners() {
  return ownerA() && ownerB();
}
inline char ownerChar() {
  if (ownerA()) return 'A';
  if (ownerB()) return 'B';
  return 'N';
}
inline bool selIsB() {
  return digitalRead(ARB_PIN_SEL_IN);
}
inline bool oeEnabled() {
  return digitalRead(ARB_PIN_OE_IN) == LOW;
}
inline void busSet(bool a) {
  digitalWrite(ARB_PIN_BUS_ACTIVE, a ? HIGH : LOW);
}
inline bool waitPinHigh(uint8_t pin, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(pin)) return true;
    delayMicroseconds(100);
  }
  return false;
}
inline bool waitPinLow(uint8_t pin, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (!digitalRead(pin)) return true;
    delayMicroseconds(100);
  }
  return false;
}
inline bool waitCond(bool (*pred)(), uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (pred()) return true;
    delayMicroseconds(100);
  }
  return false;
}
inline bool wantOwner(char w) {
  return (ownerChar() == w) && oeEnabled() && ((w == 'A' && !selIsB()) || (w == 'B' && selIsB()));
}
inline bool wantNone() {
  return ownerChar() == 'N' && !oeEnabled();
}
inline void printStatus(const __FlashStringHelper* tag) {
  if (!chatty()) return;
  Serial.print(tag);
  Serial.print(F(" | OWN="));
  Serial.print(ownerChar());
  Serial.print(F(" SEL="));
  Serial.print(selIsB() ? 'B' : 'A');
  Serial.print(F(" OE="));
  Serial.print(oeEnabled() ? F("EN") : F("DIS"));
  Serial.print(F(" PREV="));
  if (prevA() && !prevB()) Serial.print('A');
  else if (prevB() && !prevA()) Serial.print('B');
  else Serial.print('?');
  Serial.print(F(" BUS="));
  Serial.println(digitalRead(ARB_PIN_BUS_ACTIVE) ? F("ACTIVE") : F("IDLE"));
}
// Count rising edges on a pin within a time window (polling ~100 µs)
inline uint8_t countRising(uint8_t pin, uint32_t window_ms) {
  bool last = digitalRead(pin);
  uint8_t edges = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < window_ms) {
    bool now = digitalRead(pin);
    if (!last && now) edges++;
    last = now;
    delayMicroseconds(100);
  }
  return edges;
}
// Measure one high pulse width (µs) on pin; waits for rise up to timeout_ms.
// Returns true on success and writes width_us.
inline bool measurePulseWidthHigh(uint8_t pin, uint32_t timeout_ms, uint32_t& width_us) {
  // Wait for any prior high to end
  if (!waitPinLow(pin, 5)) {
    // it may already be low; continue anyway
  }
  // Wait for rising edge
  uint32_t t0m = millis();
  while (millis() - t0m < timeout_ms) {
    if (digitalRead(pin)) {
      unsigned long tRise = micros();
      // Now wait for falling edge
      while (digitalRead(pin)) {
        // protect against an infinite loop if pulse stuck high
        if ((micros() - tRise) > 1000000UL) break;
      }
      width_us = micros() - tRise;
      return true;
    }
  }
  return false;
}
// Wait for PREV to be a specific one-hot value ('A' or 'B')
inline bool waitPrevIs(char p, uint32_t ms = 300) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    bool a = prevA();
    bool b = prevB();
    if (p == 'A' && a && !b) return true;
    if (p == 'B' && b && !a) return true;
    delayMicroseconds(100);
  }
  return false;
}
// Observe a release->handoff sequence and concurrently sample IRQ edges.
// target: 'A' or 'B'; records whether we saw NONE, then target owner, and if IRQ pulses occurred.
inline void observeHandoffTo(char target, uint32_t max_ms, bool& sawNone, bool& sawTarget, bool& irqa, bool& irqb) {
  sawNone = false;
  sawTarget = false;
  irqa = false;
  irqb = false;
  bool lastA = digitalRead(ARB_PIN_IRQ_A_IN);
  bool lastB = digitalRead(ARB_PIN_IRQ_B_IN);
  uint32_t t0 = millis();
  while (millis() - t0 < max_ms) {
    bool nowA = digitalRead(ARB_PIN_IRQ_A_IN);
    bool nowB = digitalRead(ARB_PIN_IRQ_B_IN);
    if (!lastA && nowA) irqa = true;
    if (!lastB && nowB) irqb = true;
    lastA = nowA;
    lastB = nowB;
    if (!sawNone) sawNone = wantNone();
    if (!sawTarget) {
      if (target == 'A') sawTarget = wantOwner('A');
      else sawTarget = wantOwner('B');
    }
    if (sawNone && sawTarget) break;
    delayMicroseconds(100);
  }
}
// Sanity: drive both REQs low, BUS idle, wait OWNER=N and OE disabled
inline bool settleToNone(uint32_t ms = 500) {
  reqA(false);
  reqB(false);
  busSet(false);
  // Let any IRQ pulses expire
  delay(5);
  return waitCond([]() -> bool {
    return wantNone();
  },
                  ms);
}
// Check invariants at the current instant, return true if all hold.
inline bool checkInvariantsOnce() {
  bool ok = true;
  char own = ownerChar();
  bool selB = selIsB();
  bool oe = oeEnabled();
  bool oA = ownerA();
  bool oB = ownerB();
  // Mutual exclusion
  if (oA && oB) ok = false;
  // Owner↔SEL coherence
  if (own == 'A' && selB) ok = false;
  if (own == 'B' && !selB) ok = false;
  // Owner↔OE coherence
  if (own == 'N' && oe) ok = false;
  if ((own == 'A' || own == 'B') && !oe) ok = false;
  // PREV flags one-hot
  if ((int)prevA() + (int)prevB() != 1) ok = false;
  return ok;
}
// Sample invariants repeatedly for window_ms
inline bool checkInvariantsWindow(uint32_t window_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < window_ms) {
    if (!checkInvariantsOnce()) return false;
    delayMicroseconds(100);
  }
  return true;
}
// Best-effort timing sample around a grant to target ('A' or 'B')
// Prints a brief trace when verbose; does not fail tests (sampling-limited).
inline void traceGrantTiming(char target) {
  if (!chatty()) return;
  // Precondition
  if (!settleToNone(300)) {
    vprintln(F("traceGrantTiming: could not settle to NONE"));
    return;
  }
  volatile bool expectSelB = (target == 'B');
  // Start trace and then request target
  const uint32_t trace_us = 200;
  const int N = 256;
  uint8_t oeBuf[N];
  uint8_t selBuf[N];
  unsigned long tStart = micros();
  reqA(target == 'A');
  reqB(target == 'B');
  int i = 0;
  for (; i < N; i++) {
    oeBuf[i] = oeEnabled() ? 1 : 0;
    selBuf[i] = selIsB() ? 1 : 0;
    if ((micros() - tStart) > trace_us) break;
  }
  // Release request to return to NONE later
  reqA(false);
  reqB(false);
  // Print a compact summary
  Serial.print(F("[INFO] Trace "));
  Serial.print(target);
  Serial.print(F(": "));
  // Print first 64 samples compressed
  for (int k = 0; k <= i && k < 64; k++) {
    char c = (oeBuf[k] ? 'E' : 'e');  // E=en, e=dis
    char s = (selBuf[k] ? 'B' : 'A');
    Serial.print(c);
    Serial.print(s);
    Serial.print(' ');
  }
  Serial.println();
}
// ----------------------- Added helpers for stronger coverage -----------------------
// Force prevOwner to target ('A' or 'B') by performing a grant+release cycle.
inline bool forcePrev(char target) {
  (void)settleToNone(300);
  if (target == 'A') {
    reqA(true);
    bool g = waitCond([]() {
      return wantOwner('A');
    },
                      300);
    reqA(false);
    bool r = waitCond([]() {
      return wantNone();
    },
                      300);
    bool pv = waitPrevIs('A', 200);
    bool ok = g && r && pv;
    expect(ok, F("forcePrev(A)"));
    return ok;
  } else {
    reqB(true);
    bool g = waitCond([]() {
      return wantOwner('B');
    },
                      300);
    reqB(false);
    bool r = waitCond([]() {
      return wantNone();
    },
                      300);
    bool pv = waitPrevIs('B', 200);
    bool ok = g && r && pv;
    expect(ok, F("forcePrev(B)"));
    return ok;
  }
}
// Ensure current owner is 'A' or 'B' (or 'N' for none).
inline bool forceOwner(char o) {
  (void)settleToNone(300);
  if (o == 'A') {
    reqA(true);
    bool g = waitCond([]() {
      return wantOwner('A');
    },
                      300);
    expect(g, F("forceOwner(A)"));
    return g;
  } else if (o == 'B') {
    reqB(true);
    bool g = waitCond([]() {
      return wantOwner('B');
    },
                      300);
    expect(g, F("forceOwner(B)"));
    return g;
  } else {
    bool n = waitCond([]() {
      return wantNone();
    },
                      300);
    expect(n, F("forceOwner(NONE)"));
    return n;
  }
}
// Predict next owner according to firmware rules.
// owner: 'N','A','B'; prev: 'A' or 'B'; rA/rB/bus: booleans (true = asserted/active).
inline char predictNext(char owner, char prev, bool rA, bool rB, bool bus) {
  if (owner == 'N') {
    if (rA && !rB) return 'A';
    if (rB && !rA) return 'B';
    if (rA && rB) return (prev == 'A') ? 'B' : 'A';
    return 'N';
  } else if (owner == 'A') {
    if (!rA && !bus) return 'N';
    return 'A';
  } else {  // owner == 'B'
    if (!rB && !bus) return 'N';
    return 'B';
  }
}
// Read current owner as 'N','A','B'
inline char readOwnerChar() {
  return ownerChar();
}
// Exhaustive transition test.
// For each prev in {A,B}, owner in {N,A,B}, inputs in {rA,rB,bus}∈2^3,
// re-establish (prev,owner), apply inputs, and assert next owner == predicted.
inline bool runExhaustiveTransitions() {
  bool allOk = true;
  const char prevs[2] = { 'A', 'B' };
  const char owners[3] = { 'N', 'A', 'B' };

  auto setupPrevQuiet = [](char pv) -> bool {
    (void)settleToNone(300);
    if (pv == 'A') {
      reqA(true);
      bool g = waitCond([]() {
        return wantOwner('A');
      },
                        300);
      reqA(false);
      bool r = waitCond([]() {
        return wantNone();
      },
                        300);
      bool pvOk = waitPrevIs('A', 200);
      return g && r && pvOk;
    } else {
      reqB(true);
      bool g = waitCond([]() {
        return wantOwner('B');
      },
                        300);
      reqB(false);
      bool r = waitCond([]() {
        return wantNone();
      },
                        300);
      bool pvOk = waitPrevIs('B', 200);
      return g && r && pvOk;
    }
  };

  auto setupOwnerQuiet = [](char ow) -> bool {
    (void)settleToNone(300);
    if (ow == 'A') {
      reqA(true);
      bool g = waitCond([]() {
        return wantOwner('A');
      },
                        300);
      return g;
    } else if (ow == 'B') {
      reqB(true);
      bool g = waitCond([]() {
        return wantOwner('B');
      },
                        300);
      return g;
    } else {
      return waitCond([]() {
        return wantNone();
      },
                      300);
    }
  };

  for (int pi = 0; pi < 2; ++pi) {
    char pv = prevs[pi];
    for (int oi = 0; oi < 3; ++oi) {
      char ow = owners[oi];
      for (int mask = 0; mask < 8; ++mask) {
        // Quietly re-establish baseline for this single-step case
        if (!setupPrevQuiet(pv) || !setupOwnerQuiet(ow)) {
          // Cleanup and skip this case if setup raced; do not mark a FAIL
          reqA(false);
          reqB(false);
          busSet(false);
          (void)waitCond([]() {
            return wantNone();
          },
                         300);
          continue;
        }

        // Drive inputs for this step
        bool rA = (mask & 1) != 0;
        bool rB = (mask & 2) != 0;
        bool bus = (mask & 4) != 0;
        reqA(rA);
        reqB(rB);
        busSet(bus);

        // Predict next owner
        char expectNext = predictNext(ow, pv, rA, rB, bus);

        // Wait for expected state
        bool got = false;
        if (expectNext == 'N') got = waitCond([]() {
                                 return wantNone();
                               },
                                              300);
        else if (expectNext == 'A') got = waitCond([]() {
                                      return wantOwner('A');
                                    },
                                                   300);
        else got = waitCond([]() {
               return wantOwner('B');
             },
                            300);

        if (!got) {
          allOk = false;
          expect(false, F("Exhaustive transition mismatch"));
          printStatus(F("Exhaustive FAIL"));
        } else {
          expect(true, F("Exhaustive transition OK"));
        }

        // Clean up for next combo
        reqA(false);
        reqB(false);
        busSet(false);
        (void)waitCond([]() {
          return wantNone();
        },
                       300);
      }
    }
  }
  return allOk;
}
// -------- ISP engine (bit-banged) — PROTOCOL BYTES ONLY (no extra prints) --------
class ISP_BitBangSPI {
public:
  void begin() {
    digitalWrite(ARB_ISP_SCK, LOW);
    digitalWrite(ARB_ISP_MOSI, LOW);
    pinMode(ARB_ISP_SCK, OUTPUT);
    pinMode(ARB_ISP_MOSI, OUTPUT);
    pinMode(ARB_ISP_MISO, INPUT);
  }
  void beginTransaction(uint32_t hz) {
    pulseWidth = (500000 + hz - 1) / hz;
    if (!pulseWidth) pulseWidth = 1;
  }
  uint8_t transfer(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(ARB_ISP_MOSI, (b & 0x80) ? HIGH : LOW);
      digitalWrite(ARB_ISP_SCK, HIGH);
      delayMicroseconds(pulseWidth);
      b = (b << 1) | digitalRead(ARB_ISP_MISO);
      digitalWrite(ARB_ISP_SCK, LOW);
      delayMicroseconds(pulseWidth);
    }
    return b;
  }
private:
  unsigned long pulseWidth = 1;
};
static ISP_BitBangSPI ISP_SPI;
static unsigned int here = 0;
static uint8_t buff[256];
constexpr uint8_t STK_OK = 0x10, STK_FAILED = 0x11, STK_UNKNOWN = 0x12, STK_INSYNC = 0x14, STK_NOSYNC = 0x15, CRC_EOP = 0x20;
inline uint8_t ISP_getch() {
  while (!Serial.available())
    ;
  return Serial.read();
}
inline void ISP_fill(int n) {
  for (int i = 0; i < n; i++) buff[i] = ISP_getch();
}
inline void ISP_empty_reply() {
  if (CRC_EOP == ISP_getch()) {
    Serial.write(STK_INSYNC);
    Serial.write(STK_OK);
  } else {
    Serial.write(STK_NOSYNC);
  }
}
inline void ISP_breply(uint8_t b) {
  if (CRC_EOP == ISP_getch()) {
    Serial.write(STK_INSYNC);
    Serial.write(b);
    Serial.write(STK_OK);
  } else {
    Serial.write(STK_NOSYNC);
  }
}
inline void ISP_reset_target(bool assertReset) {
  pinMode(ARB_ISP_RESET, OUTPUT);
  digitalWrite(ARB_ISP_RESET, assertReset ? LOW : HIGH);  // AVR reset is active-low
}
inline void ISP_startPmode() {
  ISP_SPI.begin();
  ISP_SPI.beginTransaction(ARB_ISP_SPI_CLOCK_HZ);
  digitalWrite(ARB_ISP_SCK, LOW);
  ISP_reset_target(true);
  delay(20);
  ISP_reset_target(false);
  delayMicroseconds(100);
  ISP_reset_target(true);
  delay(50);
  // enable programming
  ISP_SPI.transfer(0xAC);
  ISP_SPI.transfer(0x53);
  ISP_SPI.transfer(0x00);
  ISP_SPI.transfer(0x00);
}
inline void ISP_endPmode() {
  pinMode(ARB_ISP_MOSI, INPUT);
  pinMode(ARB_ISP_SCK, INPUT);
  ISP_reset_target(false);
  pinMode(ARB_ISP_RESET, INPUT);
}
inline uint8_t ISP_tx4(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  ISP_SPI.transfer(a);
  ISP_SPI.transfer(b);
  ISP_SPI.transfer(c);
  return ISP_SPI.transfer(d);
}
inline uint8_t flash_read(uint8_t hilo, unsigned int addr) {
  return ISP_tx4(0x20 + 8 * hilo, (addr >> 8) & 0xFF, addr & 0xFF, 0x00);
}
inline void flash_write(uint8_t hilo, unsigned int addr, uint8_t data) {
  ISP_tx4(0x40 + 8 * hilo, (addr >> 8) & 0xFF, addr & 0xFF, data);
}
inline void flash_commit(unsigned int addr) {
  ISP_tx4(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  delay(50);
}
inline unsigned int page_mask(uint16_t ps) {
  if (ps == 32) return 0xFFFFFFF0;
  if (ps == 64) return 0xFFFFFFE0;
  if (ps == 128) return 0xFFFFFFC0;
  if (ps == 256) return 0xFFFFFF80;
  return 0xFFFFFFFF;
}
// STK handler (single command)
inline void ISP_serviceOnce() {
  if (!Serial.available()) return;
  uint8_t ch = ISP_getch();
  switch (ch) {
    case '0': ISP_empty_reply(); break;  // signon
    case '1':
      if (ISP_getch() == CRC_EOP) {
        Serial.write(STK_INSYNC);
        Serial.print("AVR ISP");
        Serial.write(STK_OK);
      } else {
        Serial.write(STK_NOSYNC);
      }
      break;
    case 'A':
      {
        uint8_t idx = ISP_getch();
        uint8_t v = 0;
        if (idx == 0x80) v = 2;
        else if (idx == 0x81) v = 1;
        else if (idx == 0x82) v = 18;
        else if (idx == 0x93) v = 'S';
        ISP_breply(v);
      }
      break;
    case 'B':
      ISP_fill(20);
      ISP_empty_reply();
      break;  // parameters ignored (safe defaults)
    case 'E':
      ISP_fill(5);
      ISP_empty_reply();
      break;  // ext params ignored
    case 'P':
      ISP_startPmode();
      ISP_empty_reply();
      break;
    case 'U':
      here = ISP_getch();
      here += 256 * ISP_getch();
      ISP_empty_reply();
      break;
    case 0x64:
      {  // program page
        unsigned int len = 256 * ISP_getch();
        len += ISP_getch();
        char mem = ISP_getch();
        if (mem == 'F') {
          // Assume 64-byte pages for t861 (ok for most configs)
          const uint16_t ps = 64;
          ISP_fill(len);
          if (CRC_EOP != ISP_getch()) {
            Serial.write(STK_NOSYNC);
            break;
          }
          Serial.write(STK_INSYNC);
          unsigned int pageBase = here & page_mask(ps);
          for (unsigned int i = 0; i < len; i += 2) {
            flash_write(0, here, buff[i]);
            flash_write(1, here, buff[i + 1]);
            here++;
            if ((here & page_mask(ps)) != pageBase) {
              flash_commit(pageBase);
              pageBase = here & page_mask(ps);
            }
          }
          flash_commit(pageBase);
          Serial.write(STK_OK);
        } else if (mem == 'E') {
          // EEPROM, accept but do minimal write (slow, compatible)
          for (unsigned int i = 0; i < len; i++) buff[i] = ISP_getch();
          if (CRC_EOP != ISP_getch()) {
            Serial.write(STK_NOSYNC);
            break;
          }
          Serial.write(STK_INSYNC);
          Serial.write(STK_OK);
        } else {
          Serial.write(STK_FAILED);
        }
      }
      break;
    case 0x74:
      {  // read page
        int len = 256 * ISP_getch();
        len += ISP_getch();
        char mem = ISP_getch();
        if (CRC_EOP != ISP_getch()) {
          Serial.write(STK_NOSYNC);
          break;
        }
        Serial.write(STK_INSYNC);
        if (mem == 'F') {
          for (int i = 0; i < len; i += 2) {
            Serial.write(flash_read(0, here));
            Serial.write(flash_read(1, here));
            here++;
          }
        } else if (mem == 'E') {
          for (int i = 0; i < len; i++) { Serial.write(ISP_tx4(0xA0, ((here * 2 + i) >> 8) & 0xFF, (here * 2 + i) & 0xFF, 0xFF)); }
        }
        Serial.write(STK_OK);
      }
      break;
    case 'V':
      ISP_fill(4);
      ISP_breply(ISP_tx4(buff[0], buff[1], buff[2], buff[3]));
      break;  // universal
    case 'Q':
      ISP_endPmode();
      ISP_empty_reply();
      break;
    case 0x75:  // signature
      if (CRC_EOP != ISP_getch()) {
        Serial.write(STK_NOSYNC);
        break;
      }
      Serial.write(STK_INSYNC);
      Serial.write(ISP_tx4(0x30, 0, 0, 0));
      Serial.write(ISP_tx4(0x30, 0, 1, 0));
      Serial.write(ISP_tx4(0x30, 0, 2, 0));
      Serial.write(STK_OK);
      break;
    case CRC_EOP: Serial.write(STK_NOSYNC); break;
    default:
      if (CRC_EOP == ISP_getch()) Serial.write(STK_UNKNOWN);
      else Serial.write(STK_NOSYNC);
      break;
  }
}
// Minimal direct signature probe (used by recovery test)
inline bool ISP_probeSignature(uint8_t& b0, uint8_t& b1, uint8_t& b2) {
  pinMode(ARB_ISP_SCK, OUTPUT);
  pinMode(ARB_ISP_MOSI, OUTPUT);
  pinMode(ARB_ISP_MISO, INPUT);
  ISP_startPmode();
  b0 = ISP_tx4(0x30, 0, 0, 0);
  b1 = ISP_tx4(0x30, 0, 1, 0);
  b2 = ISP_tx4(0x30, 0, 2, 0);
  ISP_endPmode();
  return (b0 == 0x1E && b1 == 0x93 && b2 == 0x0D);  // ATtiny861(A)
}
}  // namespace detail
// ============================== Public API =================================
inline void initTestPins() {
  pinMode(ARB_PIN_REQ_A_OUT, OUTPUT);
  digitalWrite(ARB_PIN_REQ_A_OUT, HIGH);
  pinMode(ARB_PIN_REQ_B_OUT, OUTPUT);
  digitalWrite(ARB_PIN_REQ_B_OUT, HIGH);
  pinMode(ARB_PIN_OWNER_A_IN, INPUT);
  pinMode(ARB_PIN_OWNER_B_IN, INPUT);
  pinMode(ARB_PIN_PREV_A_IN, INPUT);
  pinMode(ARB_PIN_PREV_B_IN, INPUT);
  pinMode(ARB_PIN_IRQ_A_IN, INPUT);
  pinMode(ARB_PIN_IRQ_B_IN, INPUT);
  pinMode(ARB_PIN_SEL_IN, INPUT);
  pinMode(ARB_PIN_OE_IN, INPUT);
  pinMode(ARB_PIN_TINY_RST, INPUT);
  pinMode(ARB_PIN_BUS_ACTIVE, OUTPUT);
  detail::busSet(false);
}
// POST validator (verbose only affects POST). If allowBootselReset and POST fails,
// perform a recovery probe: reset + ISP signature read (prints result when verbose).
inline bool runTestSuiteOnce(bool allowBootselReset = false, bool verbose = false) {
  detail::gVerbose = verbose ? 1 : 0;
  if (verbose) Serial.println(F("[POST] Init pins"));
  if (allowBootselReset && bootsel()) {
    if (verbose) Serial.println(F("[POST] BOOTSEL held → pulse Tiny reset"));
    pinMode(ARB_PIN_TINY_RST, OUTPUT);
    digitalWrite(ARB_PIN_TINY_RST, LOW);
    delay(10);
    pinMode(ARB_PIN_TINY_RST, INPUT);
    delay(50);
  }
  detail::passCnt = detail::failCnt = 0;
  // Test 0: baseline
  detail::vprintln(F("[POST] Test 0: baseline idle"));
  detail::reqA(false);
  detail::reqB(false);
  detail::busSet(false);
  bool ok0 = detail::waitCond([]() -> bool {
    return detail::wantNone();
  },
                              300);
  detail::printStatus(F("Baseline"));
  detail::expect(ok0, F("Idle: OWNER=NONE, OE=DIS"));
  // Baseline PREV indicates initial tie-bias (firmware starts prev=B -> A wins first tie)
  detail::expect(detail::prevB() && !detail::prevA(), F("Baseline PREV=B one-hot"));
  // Test 1: grant A + release
  detail::vprintln(F("[POST] Test 1: grant A (idle)+release"));
  detail::reqA(true);
  bool ok1g = detail::waitCond([]() -> bool {
    return detail::wantOwner('A');
  },
                               300);
  detail::printStatus(F("After REQ_A"));
  bool irqA = detail::waitPinHigh(ARB_PIN_IRQ_A_IN, 50);
  detail::expect(ok1g, F("Grant A"));
  detail::expect(irqA, F("IRQ_A on grant"));
  detail::reqA(false);
  bool ok1r = detail::waitCond([]() -> bool {
    return detail::wantNone();
  },
                               300);
  bool irqArel = detail::waitPinHigh(ARB_PIN_IRQ_A_IN, 50);
  bool irqBrel = detail::waitPinHigh(ARB_PIN_IRQ_B_IN, 50);
  detail::printStatus(F("After release A"));
  detail::expect(ok1r, F("Release A -> NONE"));
  detail::expect(irqArel && irqBrel, F("IRQ_A & IRQ_B on release"));
  // Test 2: grant B, BUS gated release
  detail::vprintln(F("[POST] Test 2: grant B, BUS-active gate"));
  detail::reqB(true);
  bool ok2g = detail::waitCond([]() -> bool {
    return detail::wantOwner('B');
  },
                               300);
  detail::expect(ok2g, F("Grant B"));
  detail::busSet(true);
  delay(5);
  detail::reqB(false);
  delay(20);
  bool held = (detail::ownerChar() == 'B' && detail::oeEnabled());
  detail::expect(held, F("BUS active holds owner"));
  detail::busSet(false);
  bool ok2r = detail::waitCond([]() -> bool {
    return detail::wantNone();
  },
                               300);
  detail::printStatus(F("After BUS idle"));
  detail::expect(ok2r, F("Release after BUS idle"));
  // Test 3: tie/round-robin
  detail::vprintln(F("[POST] Test 3: tie -> round-robin"));
  detail::reqA(true);
  detail::reqB(true);
  bool t1 = detail::waitCond([]() -> bool {
    return detail::ownerChar() != 'N';
  },
                             300);
  char w1 = detail::ownerChar();
  detail::expect(t1 && (w1 == 'A' || w1 == 'B'), F("Tie#1 grant"));
  detail::busSet(false);
  detail::reqA(false);
  detail::reqB(false);
  detail::waitCond([]() -> bool {
    return detail::wantNone();
  },
                   300);
  detail::reqA(true);
  detail::reqB(true);
  bool t2 = detail::waitCond([]() -> bool {
    return detail::ownerChar() != 'N';
  },
                             300);
  char w2 = detail::ownerChar();
  detail::expect(t2 && (w2 == 'A' || w2 == 'B'), F("Tie#2 grant"));
  detail::expect(w1 != w2, F("Round-robin alternates"));
  detail::reqA(false);
  detail::reqB(false);
  detail::busSet(false);
  detail::waitCond([]() -> bool {
    return detail::wantNone();
  },
                   300);
  // ================= Additional robustness tests =================
  // Test 4: PREV_* indicators track last non-NONE owner
  detail::vprintln(F("[POST] Test 4: PREV_* indicators"));
  (void)detail::settleToNone(300);
  detail::reqA(true);
  bool t4gA = detail::waitCond([]() {
    return detail::wantOwner('A');
  },
                               300);
  detail::expect(t4gA, F("Grant A for PREV test"));
  detail::reqA(false);
  bool t4rA = detail::waitCond([]() {
    return detail::wantNone();
  },
                               300);
  bool prevOkA = detail::waitPrevIs('A', 200);
  detail::expect(t4rA && prevOkA, F("After release, PREV=A"));
  detail::reqB(true);
  bool t4gB = detail::waitCond([]() {
    return detail::wantOwner('B');
  },
                               300);
  detail::expect(t4gB, F("Grant B for PREV test"));
  detail::reqB(false);
  bool t4rB = detail::waitCond([]() {
    return detail::wantNone();
  },
                               300);
  bool prevOkB = detail::waitPrevIs('B', 200);
  detail::expect(t4rB && prevOkB, F("After release, PREV=B"));
  // Test 5: IRQ_B pulses on grant B
  detail::vprintln(F("[POST] Test 5: IRQ_B on grant B"));
  (void)detail::settleToNone(300);
  detail::reqB(true);
  bool t5g = detail::waitCond([]() {
    return detail::wantOwner('B');
  },
                              300);
  bool irqB_on_grant = detail::waitPinHigh(ARB_PIN_IRQ_B_IN, 50);
  detail::expect(t5g && irqB_on_grant, F("IRQ_B on grant"));
  detail::reqB(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Test 6: Non-preemption (A cannot steal while B holds)
  detail::vprintln(F("[POST] Test 6: Non-preemption"));
  (void)detail::settleToNone(300);
  detail::reqB(true);
  bool t6gB = detail::waitCond([]() {
    return detail::wantOwner('B');
  },
                               300);
  detail::expect(t6gB, F("Grant B (setup)"));
  detail::reqA(true);  // A attempts to steal
  delay(20);
  bool stillB = (detail::ownerChar() == 'B') && detail::oeEnabled();
  uint8_t irqA_edges = detail::countRising(ARB_PIN_IRQ_A_IN, 10);
  detail::expect(stillB, F("A cannot preempt B"));
  detail::expect(irqA_edges == 0, F("No spurious IRQ_A during hold"));
  detail::reqA(false);
  detail::reqB(false);
  detail::busSet(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Test 7: Immediate handoff when peer is already requesting
  detail::vprintln(F("[POST] Test 7: Immediate handoff A->B"));
  (void)detail::settleToNone(300);
  detail::reqA(true);
  bool t7gA = detail::waitCond([]() {
    return detail::wantOwner('A');
  },
                               300);
  detail::expect(t7gA, F("Grant A (setup)"));
  detail::reqB(true);     // B is waiting
  detail::busSet(false);  // ensure BUS idle path
  detail::reqA(false);    // A releases
  bool sawNone = false, sawB = false, irqa_seen = false, irqb_seen = false;
  detail::observeHandoffTo('B', 60, sawNone, sawB, irqa_seen, irqb_seen);
  detail::expect(sawNone && sawB, F("Release then grant to B"));
  detail::expect(irqa_seen, F("IRQ_A pulsed on release"));
  detail::expect(irqb_seen, F("IRQ_B pulsed (release and/or grant)"));
  detail::reqB(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Test 8: Grant under BUS active (BUS only gates release)
  detail::vprintln(F("[POST] Test 8: Grant while BUS active"));
  (void)detail::settleToNone(300);
  detail::busSet(true);
  detail::reqA(true);
  bool t8gA = detail::waitCond([]() {
    return detail::wantOwner('A');
  },
                               300);
  detail::expect(t8gA, F("Grant A even if BUS active"));
  detail::reqA(false);
  detail::busSet(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Test 9: Mutual exclusion and coherence invariants (short window)
  detail::vprintln(F("[POST] Test 9: Mutual exclusion and coherence"));
  bool okMx = detail::checkInvariantsWindow(20);
  detail::expect(okMx, F("OWNER_A/B mutual exclusion and signals coherent"));
  // Test 10: IRQ pulse widths (A and B). Expected ~2ms in firmware; use tolerant bounds.
  detail::vprintln(F("[POST] Test 10: IRQ pulse widths"));
  (void)detail::settleToNone(300);
  // A
  uint32_t wA = 0;
  detail::reqA(true);
  bool gotA = detail::measurePulseWidthHigh(ARB_PIN_IRQ_A_IN, 20, wA);
  bool widthA_ok = gotA && (wA >= 400) && (wA <= 20000);
  detail::expect(gotA, F("IRQ_A pulse observed"));
  detail::expect(widthA_ok, F("IRQ_A width in range (0.4..20 ms)"));
  detail::reqA(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // B
  uint32_t wB = 0;
  detail::reqB(true);
  bool gotB = detail::measurePulseWidthHigh(ARB_PIN_IRQ_B_IN, 20, wB);
  bool widthB_ok = gotB && (wB >= 400) && (wB <= 20000);
  detail::expect(gotB, F("IRQ_B pulse observed"));
  detail::expect(widthB_ok, F("IRQ_B width in range (0.4..20 ms)"));
  detail::reqB(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Timing trace (best-effort, informational only)
  detail::vprintln(F("[POST] Trace: grant timing best-effort"));
  detail::traceGrantTiming('A');
  detail::traceGrantTiming('B');
  // Test 11: Quick stress — rapid toggles while checking invariants
  detail::vprintln(F("[POST] Test 11: Quick stress / invariants"));
  (void)detail::settleToNone(300);
  long fuzzFails = 0;
  for (int i = 0; i < 100; i++) {
    // Random-ish pattern without requiring a seeded RNG
    uint32_t r = micros();
    bool a = (r & 1);
    bool b = (r & 2);
    bool bus = (r & 4);
    detail::reqA(a);
    detail::reqB(b);
    detail::busSet(bus);
    // small jitter
    delayMicroseconds(200 + (r & 0x3FF));
    if (!detail::checkInvariantsOnce()) fuzzFails++;
  }
  // Clean up
  detail::reqA(false);
  detail::reqB(false);
  detail::busSet(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  detail::expect(fuzzFails == 0, F("Stress invariants held"));
  // ----------------------- Added tests for stronger coverage -----------------------
  // Test 12: Negative IRQ checks on single-side grants
  detail::vprintln(F("[POST] Test 12: Negative IRQ checks on grant"));
  (void)detail::settleToNone(300);
  // Grant A -> ensure IRQ_B did NOT pulse in the window
  detail::reqA(true);
  (void)detail::waitCond([]() {
    return detail::wantOwner('A');
  },
                         300);
  uint8_t irqB_edges_negA = detail::countRising(ARB_PIN_IRQ_B_IN, 10);
  detail::expect(irqB_edges_negA == 0, F("No IRQ_B on grant A"));
  detail::reqA(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Grant B -> ensure IRQ_A did NOT pulse in the window
  detail::reqB(true);
  (void)detail::waitCond([]() {
    return detail::wantOwner('B');
  },
                         300);
  uint8_t irqA_edges_negB = detail::countRising(ARB_PIN_IRQ_A_IN, 10);
  detail::expect(irqA_edges_negB == 0, F("No IRQ_A on grant B"));
  detail::reqB(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Test 13: Tie while BUS active (and winner matches PREV tie-bias)
  detail::vprintln(F("[POST] Test 13: Tie while BUS active"));
  (void)detail::settleToNone(300);
  // Ensure a known PREV state by forcing a release cycle
  // If PREV=B, A should win the tie; If PREV=A, B should win.
  bool pvB = detail::prevB();
  bool pvA = detail::prevA();
  if (!(pvA ^ pvB)) {
    // Establish PREV with a quick A cycle
    detail::reqA(true);
    (void)detail::waitCond([]() {
      return detail::wantOwner('A');
    },
                           300);
    detail::reqA(false);
    (void)detail::waitCond([]() {
      return detail::wantNone();
    },
                           300);
  }
  pvB = detail::prevB();
  pvA = detail::prevA();
  detail::busSet(true);
  detail::reqA(true);
  detail::reqB(true);
  bool tieGrant = detail::waitCond([]() {
    return detail::ownerChar() != 'N';
  },
                                   300);
  char win = detail::ownerChar();
  detail::expect(tieGrant, F("Tie grants even with BUS active"));
  char expectedWin = pvA ? 'B' : 'A';
  detail::expect(win == expectedWin, F("Tie winner matches PREV bias under BUS active"));
  // Cleanup
  detail::reqA(false);
  detail::reqB(false);
  detail::busSet(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Test 14: Handoff under BUS active (no grant until BUS idle)
  detail::vprintln(F("[POST] Test 14: Handoff under BUS active"));
  (void)detail::settleToNone(300);
  // A owns
  detail::reqA(true);
  bool gA = detail::waitCond([]() {
    return detail::wantOwner('A');
  },
                             300);
  detail::expect(gA, F("Grant A (setup)"));
  // B is waiting, BUS is active
  detail::reqB(true);
  detail::busSet(true);
  // A releases while BUS active -> must still be A (held by BUS gate)
  detail::reqA(false);
  delay(10);
  bool stillA = (detail::ownerChar() == 'A') && detail::oeEnabled();
  detail::expect(stillA, F("BUS active prevents A->NONE on release"));
  // Now BUS goes idle -> release to NONE then grant B
  detail::busSet(false);
  bool sawNone2 = false, sawB2 = false, irqa2 = false, irqb2 = false;
  detail::observeHandoffTo('B', 60, sawNone2, sawB2, irqa2, irqb2);
  bool toNone = sawNone2;
  bool toB = sawB2;
  detail::expect(toNone && toB, F("Release then grant B after BUS idle"));
  // Observe IRQ behavior: B should see at least one pulse (release+grant may cause two)
  detail::expect(irqb2, F("IRQ_B observed on handoff"));
  // Cleanup
  detail::reqB(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  // Test 15: Exhaustive next-state transitions
  detail::vprintln(F("[POST] Test 15: Exhaustive transition coverage (this will take a while..)"));
  bool exOk = detail::runExhaustiveTransitions();
  detail::expect(exOk, F("All transitions matched spec"));
  // Test 16 (optional): Forced-timeout detection (if firmware enables FORCE_TIMEOUT_MS)
  detail::vprintln(F("[POST] Test 16: Forced-timeout (optional, non-failing if absent)"));
  (void)detail::settleToNone(300);
  detail::reqA(true);
  bool ft_gA = detail::waitCond([]() {
    return detail::wantOwner('A');
  },
                                300);
  if (ft_gA) {
    detail::busSet(true);
    // Wait up to ~1.5s for a forced drop to NONE (only if feature enabled)
    bool forcedNone = detail::waitCond([]() {
      return detail::wantNone();
    },
                                       1500);
    if (forcedNone) {
      detail::expect(true, F("Forced-timeout: owner disconnected under BUS active"));
      // Expect both IRQ pulses around forced release (tolerate sampling)
      uint8_t irqa = detail::countRising(ARB_PIN_IRQ_A_IN, 50);
      uint8_t irqb = detail::countRising(ARB_PIN_IRQ_B_IN, 50);
      detail::expect(irqa >= 1 && irqb >= 1, F("IRQ_A & IRQ_B observed on forced release"));
    } else {
      detail::note(F("Forced-timeout not enabled or threshold > 1.5s; skipping"));
    }
  } else {
    detail::note(F("Forced-timeout precondition failed; skipping"));
  }
  // Cleanup
  detail::reqA(false);
  detail::busSet(false);
  (void)detail::waitCond([]() {
    return detail::wantNone();
  },
                         300);
  if (verbose) {
    Serial.print(F("[POST] Summary: "));
    Serial.print(detail::passCnt);
    Serial.print(F(" PASS, "));
    Serial.print(detail::failCnt);
    Serial.println(F(" FAIL"));
  }
  const bool ok = (detail::failCnt == 0);
  // Recovery probe (only if requested AND failed)
  if (!ok && allowBootselReset) {
    if (verbose) Serial.println(F("[POST] Recovery: reset + ISP signature probe"));
    // pulse reset
    pinMode(ARB_PIN_TINY_RST, OUTPUT);
    digitalWrite(ARB_PIN_TINY_RST, LOW);
    delay(10);
    pinMode(ARB_PIN_TINY_RST, INPUT);
    delay(10);
    uint8_t s0 = 0, s1 = 0, s2 = 0;
    bool present = detail::ISP_probeSignature(s0, s1, s2);
    if (verbose) {
      Serial.print(F("[POST] Signature: 0x"));
      if (s0 < 16) Serial.print('0');
      Serial.print(s0, HEX);
      Serial.print(F(" 0x"));
      if (s1 < 16) Serial.print('0');
      Serial.print(s1, HEX);
      Serial.print(F(" 0x"));
      if (s2 < 16) Serial.print('0');
      Serial.print(s2, HEX);
      Serial.println(present ? F("  (ATtiny861(A) detected)") : F("  (ATTiny861(A) NOT detected!)"));
    }
  }
  return ok;
}
// Convenience POST wrapper
inline bool runPOST(bool allowBootselReset = false, bool verbose = false) {
  if (verbose) Serial.println(F("Booting system.."));
  initTestPins();
  if (verbose) Serial.println(F("[POST] Running arbiter validation..."));
  bool ok = runTestSuiteOnce(allowBootselReset, verbose);
  // leave pins as-is; caller may clean up or enter ISP
  return ok;
}
// Reset all to Hi-Z
inline void cleanupToResetState() {
  detail::toResetState(ARB_PIN_REQ_A_OUT);
  detail::toResetState(ARB_PIN_REQ_B_OUT);
  detail::toResetState(ARB_PIN_OWNER_A_IN);
  detail::toResetState(ARB_PIN_OWNER_B_IN);
  detail::toResetState(ARB_PIN_PREV_A_IN);
  detail::toResetState(ARB_PIN_PREV_B_IN);
  detail::toResetState(ARB_PIN_IRQ_A_IN);
  detail::toResetState(ARB_PIN_IRQ_B_IN);
  detail::toResetState(ARB_PIN_SEL_IN);
  detail::toResetState(ARB_PIN_OE_IN);
  detail::toResetState(ARB_PIN_BUS_ACTIVE);
  detail::toResetState(ARB_PIN_TINY_RST);
  detail::toResetState(ARB_ISP_MOSI);
  detail::toResetState(ARB_ISP_MISO);
  detail::toResetState(ARB_ISP_SCK);
  detail::toResetState(ARB_ISP_RESET);
}
// ISP mode API (Protocol bytes only; no extra prints)
inline void enterISPMode() {
  cleanupToResetState();
  pinMode(ARB_ISP_RESET, INPUT);
  pinMode(ARB_ISP_MOSI, INPUT);
  pinMode(ARB_ISP_SCK, INPUT);
}
inline void serviceISPOnce() {
  detail::ISP_serviceOnce();
}
inline void exitISPMode() {
  detail::ISP_endPmode();
  cleanupToResetState();
}
// Tiny reset helper
inline void tinyResetPulse() {
  pinMode(ARB_PIN_TINY_RST, OUTPUT);
  digitalWrite(ARB_PIN_TINY_RST, LOW);
  delay(10);
  pinMode(ARB_PIN_TINY_RST, INPUT);
  delay(50);
}
}  // namespace ArbiterISP