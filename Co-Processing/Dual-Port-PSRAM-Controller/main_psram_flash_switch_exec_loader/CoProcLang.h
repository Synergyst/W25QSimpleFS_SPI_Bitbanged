#pragma once
/*
  CoProcLang.h (safe, no-heap, no-static-init, low-footprint)
  Minimal line-oriented scripting language interpreter for RP2040/RP2350 Arduino builds.
  Goals in this revision:
    - Zero global/static constructors. Header declares no globals and does not allocate at include-time.
    - No dynamic allocation during run() by default (operates in-place on a mutable script buffer).
      Define COPROCLANG_COPY_INPUT=1 before including this header to enable a copying mode if your buffer is const.
    - Bounded RAM usage: internal tables are small and configurable at compile time.
    - Robust label handling (LABEL: at start of statement; code may follow ':' on same line).
    - Arduino-native I/O and delays; timeout + cancel-flag honored.
  Script features:
    - Registers: R0..R15 (int32). Args preload R0..R(N-1).
    - Labels: "NAME:" at statement start.
    - Flow:  GOTO <label>
             IF Rn <op> <expr> GOTO <label>   where <op> in {==, !=, <, >, <=, >=}
    - Math:  LET Rn <expr>, ADD Rn <expr>, SUB Rn <expr>, MOV Rn Rm
    - I/O:   PINMODE <pin> <IN|OUT|INPUT|OUTPUT|INPU|INPD|PULLUP|PULLDOWN|num>
             DWRITE  <pin> <expr>   (expr may be 0/1 or LOW/HIGH/FALSE/TRUE)
             DREAD   <pin> Rn
             AWRITE  <pin> <expr>   (0..255)
             AREAD   <pin> Rn
    - Time:  DELAY <ms>, DELAY_US <us>
    - Mailbox: MBCLR, MBAPP "text", PRINT "text"
    - Return: RET <expr>
    - Comments: '#' or '//' to end of statement.
    - Statements separated by newline or ';'
  Notes:
    - Default parsing operates IN-PLACE: the interpreter writes '\0' to split statements and strip comments.
      Ensure the buffer passed to run() is writable (your pipeline already allocates g_script, so it's fine).
      If you need to keep the input immutable, define COPROCLANG_COPY_INPUT=1 before including this file.
    - To minimize RAM while supporting larger scripts, defaults use increased but fixed tables; override before including if needed:
        #define COPROCLANG_MAX_LINES  8192
        #define COPROCLANG_MAX_LABELS 2048
*/
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
// ---------- Configurable bounds ----------
#ifndef COPROCLANG_MAX_LINES
#define COPROCLANG_MAX_LINES 8192
#endif
#ifndef COPROCLANG_MAX_LABELS
#define COPROCLANG_MAX_LABELS 2048
#endif
namespace CoProcLang {
// ---------- Small ASCII helpers (no <ctype.h>) ----------
static inline bool is_ws(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}
static inline bool is_alpha(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}
static inline bool is_digit(char c) {
  return (c >= '0' && c <= '9');
}
static inline bool is_xdigit(char c) {
  return is_digit(c) || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}
static inline char to_lower(char c) {
  return (c >= 'A' && c <= 'Z') ? char(c - 'A' + 'a') : c;
}
static inline char to_upper(char c) {
  return (c >= 'a' && c <= 'z') ? char(c - 'a' + 'A') : c;
}
static inline bool is_ident_ch0(char c) {
  return (c == '_') || is_alpha(c);
}
static inline bool is_ident_ch(char c) {
  return (c == '_') || is_alpha(c) || is_digit(c);
}
static inline void skip_ws(const char*& p) {
  while (*p && is_ws(*p)) ++p;
}
static inline int stricmp_l(const char* a, const char* b, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    char ca = to_lower(a[i]), cb = to_lower(b[i]);
    if (ca != cb) return (int)((unsigned char)ca) - (int)((unsigned char)cb);
  }
  return 0;
}
static inline bool strieq_full(const char* a, const char* b) {
  while (*a && *b) {
    if (to_lower(*a) != to_lower(*b)) return false;
    ++a;
    ++b;
  }
  return (*a == 0 && *b == 0);
}
// ---------- Execution env ----------
struct Env {
  volatile uint8_t* mailbox;            // optional mailbox (null-terminated), may be nullptr
  uint32_t mailbox_max;                 // bytes
  volatile const uint8_t* cancel_flag;  // optional cancel flag pointer (1 => cancel)
  Env()
    : mailbox(nullptr), mailbox_max(0), cancel_flag(nullptr) {}
};
// ---------- VM ----------
struct VM {
  // Registers
  int32_t R[16];
  // In-place script storage view
  char* base;    // points to mutable script buffer (owned by caller)
  uint32_t len;  // bytes available in base
  // Tables (bounded, no heap)
  const char* lines[COPROCLANG_MAX_LINES];
  uint32_t line_count;
  struct Label {
    const char* name;  // pointer inside base (null-terminated)
    uint32_t idx;      // line index of first statement following the label
  };
  Label labels[COPROCLANG_MAX_LABELS];
  uint32_t label_count;
  // Mailbox and control
  Env env;
  uint32_t mb_w;
#if COPROCLANG_MAX_LINES < 8 || COPROCLANG_MAX_LABELS < 8
#error "COPROCLANG_MAX_* too small"
#endif
  VM()
    : base(nullptr), len(0), line_count(0), label_count(0), mb_w(0) {
    for (int i = 0; i < 16; ++i) R[i] = 0;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LINES; ++i) lines[i] = nullptr;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LABELS; ++i) {
      labels[i].name = nullptr;
      labels[i].idx = 0;
    }
  }
  // ----- Mailbox -----
  void mbClear() {
    if (!env.mailbox || env.mailbox_max == 0) return;
    mb_w = 0;
    env.mailbox[0] = 0;
  }
  void mbAppend(const char* s, size_t n) {
    if (!env.mailbox || env.mailbox_max == 0 || !s || n == 0) return;
    uint32_t cap = env.mailbox_max;
    while (n && (mb_w + 1) < cap) {
      env.mailbox[mb_w++] = (uint8_t)(*s++);
      --n;
    }
    env.mailbox[mb_w] = 0;
  }
  // ----- Lex helpers -----
  static bool parseIdent(const char*& p, const char*& outStart, size_t& outLen) {
    skip_ws(p);
    const char* s = p;
    if (!is_ident_ch0(*s)) return false;
    const char* b = s++;
    while (is_ident_ch(*s)) ++s;
    outStart = b;
    outLen = (size_t)(s - b);
    p = s;
    return true;
  }
  static bool parseString(const char*& p, const char*& outStart, size_t& outLen) {
    skip_ws(p);
    if (*p != '"') return false;
    ++p;
    const char* s = p;
    while (*s && *s != '"') ++s;
    if (*s != '"') return false;
    outStart = p;
    outLen = (size_t)(s - p);
    p = s + 1;
    return true;
  }
  static bool parseNumber(const char*& p, int32_t& out) {
    skip_ws(p);
    const char* s = p;
    bool neg = false;
    if (*s == '+' || *s == '-') {
      neg = (*s == '-');
      ++s;
    }
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      s += 2;
      if (!is_xdigit(*s)) return false;
      int32_t v = 0;
      while (is_xdigit(*s)) {
        char c = *s++;
        int d = (c >= '0' && c <= '9') ? (c - '0') : (c >= 'a' && c <= 'f') ? (c - 'a' + 10)
                                                   : (c >= 'A' && c <= 'F') ? (c - 'A' + 10)
                                                                            : -1;
        if (d < 0) return false;
        v = (v << 4) | d;
      }
      out = neg ? -v : v;
      p = s;
      return true;
    } else {
      if (!is_digit(*s)) return false;
      int32_t v = 0;
      while (is_digit(*s)) {
        v = v * 10 + (*s - '0');
        ++s;
      }
      out = neg ? -v : v;
      p = s;
      return true;
    }
  }
  static bool parseReg(const char*& p, int& r) {
    skip_ws(p);
    const char* s = p;
    if (*s != 'R' && *s != 'r') return false;
    ++s;
    if (!is_digit(*s)) return false;
    int v = 0;
    while (is_digit(*s)) {
      v = v * 10 + (*s - '0');
      ++s;
    }
    if (v < 0 || v > 15) return false;
    r = v;
    p = s;
    return true;
  }
  bool parseExpr(const char*& p, int32_t& out) {
    const char* save = p;
    int r = -1;
    if (parseReg(p, r)) {
      out = R[r];
      return true;
    }
    // identifiers: HIGH/LOW/TRUE/FALSE
    const char* id;
    size_t n;
    const char* s2 = p;
    if (parseIdent(s2, id, n)) {
      if ((n == 4 && stricmp_l(id, "HIGH", 4) == 0) || (n == 4 && stricmp_l(id, "TRUE", 4) == 0)) {
        p = s2;
        out = 1;
        return true;
      }
      if ((n == 3 && stricmp_l(id, "LOW", 3) == 0) || (n == 5 && stricmp_l(id, "FALSE", 5) == 0)) {
        p = s2;
        out = 0;
        return true;
      }
    }
    if (parseNumber(p, out)) return true;
    p = save;
    return false;
  }
  enum CmpOp { OP_EQ,
               OP_NE,
               OP_LT,
               OP_GT,
               OP_LE,
               OP_GE,
               OP_BAD };
  static CmpOp parseCmpOp(const char*& p) {
    skip_ws(p);
    if (p[0] == '=' && p[1] == '=') {
      p += 2;
      return OP_EQ;
    }
    if (p[0] == '!' && p[1] == '=') {
      p += 2;
      return OP_NE;
    }
    if (p[0] == '<' && p[1] == '=') {
      p += 2;
      return OP_LE;
    }
    if (p[0] == '>' && p[1] == '=') {
      p += 2;
      return OP_GE;
    }
    if (p[0] == '<') {
      ++p;
      return OP_LT;
    }
    if (p[0] == '>') {
      ++p;
      return OP_GT;
    }
    return OP_BAD;
  }
  static bool evalCmp(int32_t a, CmpOp op, int32_t b) {
    switch (op) {
      case OP_EQ: return a == b;
      case OP_NE: return a != b;
      case OP_LT: return a < b;
      case OP_GT: return a > b;
      case OP_LE: return a <= b;
      case OP_GE: return a >= b;
      default: return false;
    }
  }
  static bool parsePinModeToken(const char*& p, int& modeOut) {
    const char* id;
    size_t n;
    const char* save = p;
    if (parseIdent(p, id, n)) {
      // lowercase compare
      if ((n == 2 && stricmp_l(id, "in", 2) == 0) || (n == 5 && stricmp_l(id, "input", 5) == 0)) {
        modeOut = INPUT;
        return true;
      }
      if ((n == 3 && stricmp_l(id, "out", 3) == 0) || (n == 6 && stricmp_l(id, "output", 6) == 0)) {
        modeOut = OUTPUT;
        return true;
      }
#if defined(INPUT_PULLUP)
      if ((n == 4 && stricmp_l(id, "inpu", 4) == 0) || (n == 6 && stricmp_l(id, "pullup", 6) == 0)) {
        modeOut = INPUT_PULLUP;
        return true;
      }
#endif
#if defined(INPUT_PULLDOWN)
      if ((n == 4 && stricmp_l(id, "inpd", 4) == 0) || (n == 8 && stricmp_l(id, "pulldown", 8) == 0)) {
        modeOut = INPUT_PULLDOWN;
        return true;
      }
#endif
      // not matched; fall through to numeric
      p = save;
    }
    int32_t v = 0;
    if (parseNumber(p, v)) {
      modeOut = (int)v;
      return true;
    }
    return false;
  }
  // ----- Preprocess: split into statements and collect labels (IN-PLACE) -----
  void stripCommentInPlace(char* s) {
    if (!s) return;
    for (; *s; ++s) {
      if (*s == '#') {
        *s = 0;
        return;
      }
      if (s[0] == '/' && s[1] == '/') {
        s[0] = 0;
        return;
      }
    }
  }
  bool buildTablesInPlace() {
    line_count = 0;
    label_count = 0;
    if (!base || len == 0) return false;
    char* cur = base;
    char* end = base + len;
    auto trim = [](char*& a) {
      while (*a && is_ws(*a)) ++a;
      // trailing trim after we know endpoint; we will set '\0' already
    };
    while (cur < end) {
      // Get one physical line up to '\n' or end
      char* line = cur;
      while (cur < end && *cur != '\n') ++cur;
      char* after = cur;
      if (cur < end && *cur == '\n') {
        *cur = 0;
        ++cur;
      }
      // Now split by ';' into statements
      char* stmt = line;
      while (stmt && *stmt) {
        char* semi = strchr(stmt, ';');
        if (semi) *semi = 0;
        // Strip comments and trim spaces
        stripCommentInPlace(stmt);
        // trim leading
        char* s = stmt;
        while (*s && is_ws(*s)) ++s;
        // trim trailing
        char* t = s + strlen(s);
        while (t > s && is_ws(t[-1])) --t;
        *t = 0;
        if (*s) {
          // Label detection: IDENT: at start
          const char* p = s;
          skip_ws(p);
          const char* id;
          size_t idn;
          const char* p0 = p;
          if (parseIdent(p, id, idn)) {
            if (*p == ':') {
              // register label name by zero-terminating at ':'
              char* nameStart = (char*)id;
              char* colon = (char*)p;
              *colon = 0;  // terminate label name
              if (label_count >= COPROCLANG_MAX_LABELS) return false;
              labels[label_count].name = nameStart;
              labels[label_count].idx = line_count;
              ++label_count;
              // any code after ':' becomes a statement
              ++p;  // move past '\0' we just wrote, now points to char after colon in original text
              while (*p && is_ws(*p)) ++p;
              if (*p) {
                if (line_count >= COPROCLANG_MAX_LINES) return false;
                lines[line_count++] = p;
              }
            } else {
              // normal statement
              if (line_count >= COPROCLANG_MAX_LINES) return false;
              lines[line_count++] = s;
            }
          } else {
            // not an identifier
            if (line_count >= COPROCLANG_MAX_LINES) return false;
            lines[line_count++] = s;
          }
        }
        if (!semi) break;
        stmt = semi + 1;
      }
      (void)after;
    }
    return true;
  }
  int findLabel(const char* name) const {
    if (!name || !*name) return -1;
    for (uint32_t i = 0; i < label_count; ++i) {
      if (labels[i].name && strieq_full(labels[i].name, name)) return (int)labels[i].idx;
    }
    return -1;
  }
  // ----- Execute one statement -----
  bool execLine(uint32_t idx, int32_t& retVal, bool& didReturn, int& outJumpIdx) {
    outJumpIdx = -1;
    const char* s = lines[idx];
    if (!s || !*s) return true;
    const char* p = s;
    // command
    const char* cmd;
    size_t cmdn;
    if (!parseIdent(p, cmd, cmdn)) return true;
    // Normalize a short token for branching
    char tok[24];
    size_t L = (cmdn < sizeof(tok) - 1) ? cmdn : (sizeof(tok) - 1);
    for (size_t i = 0; i < L; ++i) tok[i] = to_upper(cmd[i]);
    tok[L] = 0;
    // Math/move
    if (!strcmp(tok, "LET")) {
      int r;
      if (!parseReg(p, r)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      R[r] = v;
      return true;
    }
    if (!strcmp(tok, "ADD")) {
      int r;
      if (!parseReg(p, r)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      R[r] += v;
      return true;
    }
    if (!strcmp(tok, "SUB")) {
      int r;
      if (!parseReg(p, r)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      R[r] -= v;
      return true;
    }
    if (!strcmp(tok, "MOV")) {
      int rd;
      if (!parseReg(p, rd)) return true;
      int rs;
      if (!parseReg(p, rs)) return true;
      R[rd] = R[rs];
      return true;
    }
    // I/O
    if (!strcmp(tok, "PINMODE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int mode;
      if (!parsePinModeToken(p, mode)) return true;
      pinMode((uint8_t)pin, mode);
      return true;
    }
    if (!strcmp(tok, "DWRITE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      digitalWrite((uint8_t)pin, v ? HIGH : LOW);
      return true;
    }
    if (!strcmp(tok, "DREAD")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int r;
      if (!parseReg(p, r)) return true;
      R[r] = digitalRead((uint8_t)pin);
      return true;
    }
    if (!strcmp(tok, "AWRITE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      analogWrite((uint8_t)pin, (int)v);
      return true;
    }
    if (!strcmp(tok, "AREAD")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int r;
      if (!parseReg(p, r)) return true;
      R[r] = analogRead((uint8_t)pin);
      return true;
    }
    // SHIFTOUT: SHIFTOUT <dataPin> <clockPin> <latchPin> <expr> [bits] [MSBFIRST|LSBFIRST]
    if (!strcmp(tok, "SHIFTOUT")) {
      int32_t dataPin, clockPin, latchPin;
      if (!parseNumber(p, dataPin)) return true;
      if (!parseNumber(p, clockPin)) return true;
      if (!parseNumber(p, latchPin)) return true;
      int32_t val;
      if (!parseExpr(p, val)) return true;
      // optional bit count
      const char* save_p = p;
      int bitCount = 8;
      int32_t tmp;
      if (parseNumber(p, tmp)) {
        if (tmp > 0 && tmp <= 32) bitCount = tmp;
        else bitCount = 8;
      } else {
        p = save_p;
      }
      // Optional bit order token
      const char* id;
      size_t idn;
      bool msbFirst = true;
      const char* save2 = p;
      if (parseIdent(p, id, idn)) {
        if (idn == 8 && stricmp_l(id, "LSBFIRST", 8) == 0) msbFirst = false;
        else if (idn == 8 && stricmp_l(id, "MSBFIRST", 8) == 0) msbFirst = true;
        else p = save2;  // unrecognized token; rewind
      }
      // Ensure pins are outputs (safe)
      pinMode((uint8_t)dataPin, OUTPUT);
      pinMode((uint8_t)clockPin, OUTPUT);
      pinMode((uint8_t)latchPin, OUTPUT);
      // Latch low, shift bits, latch high
      digitalWrite((uint8_t)latchPin, LOW);
      // Shift bitCount bits (cap to 32)
      int bits = (bitCount > 32) ? 32 : bitCount;
      for (int i = 0; i < bits; ++i) {
        int bitIndex = msbFirst ? (bits - 1 - i) : i;
        int b = (((uint32_t)val >> bitIndex) & 1) ? HIGH : LOW;
        digitalWrite((uint8_t)dataPin, b);
        // Clock pulse
        digitalWrite((uint8_t)clockPin, HIGH);
        delayMicroseconds(1);  // tweak if you need slower/faster
        digitalWrite((uint8_t)clockPin, LOW);
        delayMicroseconds(1);
      }
      digitalWrite((uint8_t)latchPin, HIGH);
      return true;
    }
    // Timing
    if (!strcmp(tok, "DELAY")) {
      int32_t ms;
      if (!parseExpr(p, ms)) return true;
      if (ms < 0) ms = 0;
      delay((uint32_t)ms);
      return true;
    }
    if (!strcmp(tok, "DELAY_US")) {
      int32_t us;
      if (!parseExpr(p, us)) return true;
      if (us < 0) us = 0;
      delayMicroseconds((uint32_t)us);
      return true;
    }
    // Mailbox
    if (!strcmp(tok, "MBCLR")) {
      mbClear();
      return true;
    }
    if (!strcmp(tok, "MBAPP") || !strcmp(tok, "PRINT")) {
      const char* t;
      size_t tn;
      if (!parseString(p, t, tn)) return true;
      mbAppend(t, tn);
      return true;
    }
    // Flow
    if (!strcmp(tok, "RET")) {
      int32_t v = 0;
      (void)parseExpr(p, v);
      retVal = v;
      didReturn = true;
      return false;
    }
    if (!strcmp(tok, "GOTO")) {
      const char* id;
      size_t idn;
      if (!parseIdent(p, id, idn)) return true;
      char name[64];
      size_t m = (idn < sizeof(name) - 1) ? idn : (sizeof(name) - 1);
      memcpy(name, id, m);
      name[m] = 0;
      int li = findLabel(name);
      if (li >= 0) outJumpIdx = li;
      return true;
    }
    if (!strcmp(tok, "IF")) {
      // IF Rn <op> <expr> GOTO <label>
      int r;
      if (!parseReg(p, r)) return true;
      CmpOp op = parseCmpOp(p);
      if (op == OP_BAD) return true;
      int32_t rhs;
      if (!parseExpr(p, rhs)) return true;
      const char* g;
      size_t gn;
      if (!parseIdent(p, g, gn)) return true;
      // Expect GOTO
      if (!(gn == 4 && stricmp_l(g, "GOTO", 4) == 0)) return true;
      const char* id;
      size_t idn;
      if (!parseIdent(p, id, idn)) return true;
      char name[64];
      size_t m = (idn < sizeof(name) - 1) ? idn : (sizeof(name) - 1);
      memcpy(name, id, m);
      name[m] = 0;
      if (evalCmp(R[r], op, rhs)) {
        int li = findLabel(name);
        if (li >= 0) outJumpIdx = li;
      }
      return true;
    }
    // Unknown token: ignore
    return true;
  }
  // ----- Public run API -----
  // By default, operates IN-PLACE: modifies 'buf' by inserting '\0' terminators.
  // Ensure 'buf' is writable. Your transport allocates a script buffer, so this matches well.
  bool run(uint8_t* buf, uint32_t buflen, const int32_t* args, uint32_t argc, uint32_t timeout_ms, int32_t& retVal) {
#if COPROCLANG_COPY_INPUT
    // Copying mode (if you need const input). Allocates once per run (heap).
    char* copy = (char*)malloc(buflen + 1);
    if (!copy) return false;
    memcpy(copy, buf, buflen);
    copy[buflen] = 0;
    bool ok = runInPlaceInternal(copy, buflen, args, argc, timeout_ms, retVal);
    free(copy);
    return ok;
#else
    // In-place mode: cast to char* and run
    return runInPlaceInternal((char*)buf, buflen, args, argc, timeout_ms, retVal);
#endif
  }
private:
  bool runInPlaceInternal(char* buf, uint32_t buflen, const int32_t* args, uint32_t argc, uint32_t timeout_ms, int32_t& retVal) {
    // init
    base = buf;
    len = buflen;
    for (int i = 0; i < 16; ++i) R[i] = 0;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LINES; ++i) lines[i] = nullptr;
    for (uint32_t i = 0; i < COPROCLANG_MAX_LABELS; ++i) {
      labels[i].name = nullptr;
      labels[i].idx = 0;
    }
    line_count = 0;
    label_count = 0;
    mb_w = 0;
    for (uint32_t i = 0; i < argc && i < 16; ++i) R[i] = args[i];
    if (env.mailbox && env.mailbox_max) env.mailbox[0] = 0;
    // Build tables
    if (!buildTablesInPlace()) return false;
    // Execute
    retVal = 0;
    uint32_t t0 = millis();
    bool didReturn = false;
    for (uint32_t pc = 0; pc < line_count;) {
      if (timeout_ms && (millis() - t0) > timeout_ms) return false;
      if (env.cancel_flag && *env.cancel_flag) return false;
      int jump = -1;
      if (!execLine(pc, retVal, didReturn, jump)) break;
      if (didReturn) break;
      if (jump >= 0 && (uint32_t)jump < line_count) pc = (uint32_t)jump;
      else ++pc;
      // lightweight yield
      tight_loop_contents();
      yield();
    }
    return true;
  }
};
}  // namespace CoProcLang