// BusArbiterWithISP.h single-header-library

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

// -------- Test helpers/state (unchanged logic) --------
static int passCnt = 0, failCnt = 0;
inline void expect(bool ok, const __FlashStringHelper* name) {
  if (chatty()) {
    Serial.print(ok ? F("[PASS] ") : F("[FAIL] "));
    Serial.println(name);
  }
  if (ok) passCnt++;
  else failCnt++;
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
  Serial.print(F(" BUS="));
  Serial.println(digitalRead(ARB_PIN_BUS_ACTIVE) ? F("ACTIVE") : F("IDLE"));
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
      Serial.println(present ? F("  (ATtiny861(A) detected)") : F("  (NOT detected)"));
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