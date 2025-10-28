#pragma once
/*
  CoProcLang.h
  Minimal line-oriented scripting language interpreter for RP2040/RP2350 Arduino builds.
  - No STL; header-only; uses Arduino pin APIs and delays.
  - Designed to run inside the co-processor sketch after a script payload is loaded/CRC-checked.
  - Supports:
      Registers: R0..R15 (signed 32-bit). Args populate R0..R(N-1) at start.
      Labels: "LABEL:" at start of a line.
      Branching: GOTO <label>, IF Rn <op> <expr> GOTO <label>, ops: == != < > <= >=
      Math: LET Rn <expr>, ADD Rn <expr>, SUB Rn <expr>, MOV Rn Rm
      I/O: PINMODE <pin> <IN|OUT|INPUT|OUTPUT|INPU|INPD|PULLUP|PULLDOWN|<num>>
           DWRITE <pin> <0|1>, DREAD <pin> Rn
           AWRITE <pin> <0..255>, AREAD <pin> Rn
      Delays: DELAY <ms>, DELAY_US <us>
      Mailbox: MBCLR, MBAPP "text"
      Print: PRINT "text"  (alias for MBAPP)
      Ret: RET <expr>  (sets return value and stops)
    - Comments: '#' or '//' to end of line.
    - Tokens separated by whitespace; strings in double quotes.
*/

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>

namespace CoProcLang {

struct Env {
  volatile uint8_t* mailbox;            // optional mailbox (null-terminated), may be nullptr
  uint32_t mailbox_max;                 // capacity of mailbox in bytes
  volatile const uint8_t* cancel_flag;  // optional cancel flag pointer (1 => cancel)
  Env()
    : mailbox(nullptr), mailbox_max(0), cancel_flag(nullptr) {}
};

struct VM {
  // Config
  Env env;
  // Registers
  int32_t R[16];
  // Script storage
  char* text;       // owned copy, null-terminated
  size_t text_len;  // bytes in text (excluding trailing '\0')
  // Lines and labels
  static constexpr uint32_t MAX_LINES = 2048;
  static constexpr uint32_t MAX_LABELS = 512;
  const char* lines[MAX_LINES];
  uint32_t line_count;
  struct Label {
    const char* name;
    uint32_t idx;
  };
  Label labels[MAX_LABELS];
  uint32_t label_count;
  // Mailbox write head
  uint32_t mb_w;

  VM()
    : text(nullptr), text_len(0), line_count(0), label_count(0), mb_w(0) {
    for (int i = 0; i < 16; ++i) R[i] = 0;
    for (uint32_t i = 0; i < MAX_LINES; ++i) lines[i] = nullptr;
    for (uint32_t i = 0; i < MAX_LABELS; ++i) labels[i] = { nullptr, 0 };
  }
  ~VM() {
    if (text) free(text);
    text = nullptr;
  }

  // Helpers
  static inline bool is_ws(char c) {
    return (c == ' ' || c == '\t' || c == '\r' || c == '\n');
  }
  static inline void skip_ws(const char*& p) {
    while (*p && is_ws(*p)) ++p;
  }

  static inline bool is_ident_ch0(char c) {
    return (c == '_' || isalpha((unsigned char)c));
  }
  static inline bool is_ident_ch(char c) {
    return (c == '_' || isalnum((unsigned char)c));
  }

  static inline bool strieq(const char* a, const char* b) {
    while (*a && *b) {
      char ca = *a, cb = *b;
      if (ca >= 'A' && ca <= 'Z') ca = char(ca - 'A' + 'a');
      if (cb >= 'A' && cb <= 'Z') cb = char(cb - 'A' + 'a');
      if (ca != cb) return false;
      ++a;
      ++b;
    }
    return (*a == 0 && *b == 0);
  }

  // Parse number: decimal or 0xHEX
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
      int32_t v = 0;
      if (!isxdigit((unsigned char)*s)) return false;
      while (isxdigit((unsigned char)*s)) {
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
      if (!isdigit((unsigned char)*s)) return false;
      int32_t v = 0;
      while (isdigit((unsigned char)*s)) {
        v = v * 10 + (*s - '0');
        ++s;
      }
      out = neg ? -v : v;
      p = s;
      return true;
    }
  }

  // Parse register token: R0..R15
  static bool parseReg(const char*& p, int& regIndex) {
    skip_ws(p);
    const char* s = p;
    if (*s != 'R' && *s != 'r') return false;
    ++s;
    if (!isdigit((unsigned char)*s)) return false;
    int v = 0;
    while (isdigit((unsigned char)*s)) {
      v = v * 10 + (*s - '0');
      ++s;
    }
    if (v < 0 || v > 15) return false;
    regIndex = v;
    p = s;
    return true;
  }

  // Parse identifier (word)
  static bool parseIdent(const char*& p, const char*& outStart, size_t& outLen) {
    skip_ws(p);
    const char* s = p;
    if (!is_ident_ch0(*s)) return false;
    const char* b = s;
    ++s;
    while (is_ident_ch(*s)) ++s;
    outStart = b;
    outLen = size_t(s - b);
    p = s;
    return true;
  }

  // Parse quoted string "..."
  static bool parseString(const char*& p, const char*& outStart, size_t& outLen) {
    skip_ws(p);
    if (*p != '"') return false;
    ++p;
    const char* s = p;
    while (*s && *s != '"') ++s;
    if (*s != '"') return false;
    outStart = p;
    outLen = size_t(s - p);
    p = s + 1;
    return true;
  }

  // Comments: '#' or '//' to end of line
  static void stripComment(char* line) {
    if (!line) return;
    for (char* s = line; *s; ++s) {
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

  // Mailbox helpers
  void mbClear() {
    if (!env.mailbox || env.mailbox_max == 0) return;
    mb_w = 0;
    env.mailbox[0] = 0;
  }
  void mbAppend(const char* s, size_t n) {
    if (!env.mailbox || env.mailbox_max == 0 || !s || n == 0) return;
    uint32_t cap = env.mailbox_max;
    if (cap == 0) return;
    // keep space for final '\0'
    while (n && mb_w + 1 < cap) {
      env.mailbox[mb_w++] = (uint8_t)(*s++);
      --n;
      if (mb_w + 1 >= cap) break;
    }
    env.mailbox[mb_w] = 0;
  }
  void mbAppendStr(const char* s) {
    if (s) mbAppend(s, strlen(s));
  }

  // Build lines[] and labels[]
  bool buildLinesAndLabels() {
    line_count = 0;
    label_count = 0;
    if (!text) return false;
    // Split by newline or ';'
    char* s = text;
    while (*s) {
      // skip leading whitespace/newlines/semicolons
      while (*s && (is_ws(*s) || *s == ';')) {
        if (*s == '\n') { /*just skip*/
        }
        ++s;
      }
      if (!*s) break;
      if (line_count >= MAX_LINES) return false;
      char* line = s;
      // find end
      while (*s && *s != '\n') ++s;
      char* e = s;
      if (*s == '\n') {
        *s = 0;
        ++s;
      }  // terminate and advance
      // Also split by ';' inside this line into sub-statements
      char* cur = line;
      while (cur && *cur) {
        // find semicolon or end
        char* semi = strchr(cur, ';');
        if (semi) *semi = 0;
        // trim trailing spaces
        char* t = cur + strlen(cur);
        while (t > cur && is_ws(t[-1])) --t;
        *t = 0;
        // remove leading spaces
        while (*cur && is_ws(*cur)) ++cur;
        if (*cur) {
          // strip comments in-place
          stripComment(cur);
          // trim again
          t = cur + strlen(cur);
          while (t > cur && is_ws(t[-1])) --t;
          *t = 0;
          if (*cur) lines[line_count++] = cur;
        }
        if (!semi) break;
        cur = semi + 1;
      }
      (void)e;
    }
    // Collect labels: NAME: at start
    for (uint32_t i = 0; i < line_count; ++i) {
      const char* p = lines[i];
      skip_ws(p);
      // label if first token ends with ':'
      const char* id;
      size_t len;
      const char* p0 = p;
      if (parseIdent(p, id, len)) {
        if (id && len && id[len - 1] == ':') {
          if (label_count >= MAX_LABELS) return false;
          // register label name without trailing ':'
          char* nm = (char*)malloc(len);
          if (!nm) return false;
          memcpy(nm, id, len - 1);
          nm[len - 1] = 0;
          // rewrite line to empty (label-only line)
          lines[i] = (const char*)p;  // keep parser after label; rest of line considered statement if any
          labels[label_count++] = { nm, i };
        } else {
          // not a label; keep
          (void)p0;
        }
      }
    }
    return true;
  }

  int findLabel(const char* name) const {
    for (uint32_t i = 0; i < label_count; ++i) {
      if (labels[i].name && strieq(labels[i].name, name)) return (int)labels[i].idx;
    }
    return -1;
  }

  // Expression: either a literal number or a register
  bool parseExpr(const char*& p, int32_t& out) {
    const char* save = p;
    int ri = -1;
    if (parseReg(p, ri)) {
      out = R[ri];
      return true;
    }
    p = save;
    if (parseNumber(p, out)) return true;
    return false;
  }

  // Operators for IF
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
      p += 1;
      return OP_LT;
    }
    if (p[0] == '>') {
      p += 1;
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

  // Map PINMODE second arg token to Arduino pinMode values
  static bool parsePinMode(const char*& p, int& mode) {
    const char* id;
    size_t n;
    const char* save = p;
    if (parseIdent(p, id, n)) {
      // Copy to temp lower-case
      char tmp[16];
      size_t m = (n < sizeof(tmp) - 1) ? n : (sizeof(tmp) - 1);
      for (size_t i = 0; i < m; ++i) tmp[i] = (char)tolower((unsigned char)id[i]);
      tmp[m] = 0;
      if (!strcmp(tmp, "in") || !strcmp(tmp, "input")) {
        mode = INPUT;
        return true;
      }
      if (!strcmp(tmp, "out") || !strcmp(tmp, "output")) {
        mode = OUTPUT;
        return true;
      }
#ifdef INPUT_PULLUP
      if (!strcmp(tmp, "inpu") || !strcmp(tmp, "pullup")) {
        mode = INPUT_PULLUP;
        return true;
      }
#endif
#ifdef INPUT_PULLDOWN
      if (!strcmp(tmp, "inpd") || !strcmp(tmp, "pulldown")) {
        mode = INPUT_PULLDOWN;
        return true;
      }
#endif
      // else fall through and try number
      p = save;
    }
    int32_t v = 0;
    if (parseNumber(p, v)) {
      mode = (int)v;
      return true;
    }
    return false;
  }

  // Execute one line; returns true to continue; false to stop; may set outJumpIdx for GOTO
  bool execLine(uint32_t idx, int32_t& retVal, bool& didReturn, int& outJumpIdx) {
    outJumpIdx = -1;
    const char* p = lines[idx];
    if (!p || !*p) return true;  // empty

    // Strip leading whitespace
    skip_ws(p);
    if (!*p) return true;

    // Parse first token (command or label-remainder)
    const char* cmd;
    size_t clen;
    if (!parseIdent(p, cmd, clen)) {
      // may be label-only remainder -> skip
      return true;
    }

    // Upper-case compare by copying a short token
    char tok[24];
    size_t L = clen < sizeof(tok) - 1 ? clen : sizeof(tok) - 1;
    for (size_t i = 0; i < L; ++i) tok[i] = (char)toupper((unsigned char)cmd[i]);
    tok[L] = 0;

    // Commands
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
      int rDst;
      if (!parseReg(p, rDst)) return true;
      int rSrc;
      if (!parseReg(p, rSrc)) return true;
      R[rDst] = R[rSrc];
      return true;
    }

    if (!strcmp(tok, "PINMODE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int mode;
      if (!parsePinMode(p, mode)) return true;
      pinMode((uint8_t)pin, mode);
      return true;
    }
    if (!strcmp(tok, "DWRITE")) {
      int32_t pin;
      if (!parseNumber(p, pin)) return true;
      int32_t v;
      if (!parseExpr(p, v)) return true;
      digitalWrite((uint8_t)pin, (v ? HIGH : LOW));
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
    if (!strcmp(tok, "MBCLR")) {
      mbClear();
      return true;
    }
    if (!strcmp(tok, "MBAPP") || !strcmp(tok, "PRINT")) {
      const char* s;
      size_t n;
      if (!parseString(p, s, n)) return true;
      mbAppend(s, n);
      return true;
    }
    if (!strcmp(tok, "RET")) {
      int32_t v;
      if (!parseExpr(p, v)) v = 0;
      retVal = v;
      didReturn = true;
      return false;  // stop
    }
    if (!strcmp(tok, "GOTO")) {
      const char* id;
      size_t n;
      if (!parseIdent(p, id, n)) return true;
      char name[64];
      size_t m = n < sizeof(name) - 1 ? n : sizeof(name) - 1;
      memcpy(name, id, m);
      name[m] = 0;
      int li = findLabel(name);
      if (li >= 0) { outJumpIdx = li; }
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
      char gtok[8];
      size_t k = gn < sizeof(gtok) - 1 ? gn : sizeof(gtok) - 1;
      for (size_t i = 0; i < k; ++i) gtok[i] = (char)toupper((unsigned char)g[i]);
      gtok[k] = 0;
      if (strcmp(gtok, "GOTO") != 0) return true;
      const char* id;
      size_t n;
      if (!parseIdent(p, id, n)) return true;
      char name[64];
      size_t m = n < sizeof(name) - 1 ? n : sizeof(name) - 1;
      memcpy(name, id, m);
      name[m] = 0;
      if (evalCmp(R[r], op, rhs)) {
        int li = findLabel(name);
        if (li >= 0) outJumpIdx = li;
      }
      return true;
    }

    // Unknown token: ignore line
    return true;
  }

  // Run the script with timeout and cancel support.
  // args[0..argc-1] preload R0..R(N-1).
  // retVal receives RET value (default 0 if not returned explicitly).
  bool run(const uint8_t* buf, uint32_t len, const int32_t* args, uint32_t argc, uint32_t timeout_ms, int32_t& retVal) {
    if (!buf || len == 0) return false;
    if (text) {
      free(text);
      text = nullptr;
    }
    text = (char*)malloc(len + 1);
    if (!text) return false;
    memcpy(text, buf, len);
    text[len] = 0;
    text_len = len;

    // Initialize registers
    for (int i = 0; i < 16; ++i) R[i] = 0;
    for (uint32_t i = 0; i < argc && i < 16; ++i) R[i] = args[i];

    // Mailbox begin
    mb_w = 0;
    if (env.mailbox && env.mailbox_max) env.mailbox[0] = 0;

    // Build lines / labels
    if (!buildLinesAndLabels()) return false;

    // Main loop
    retVal = 0;
    bool didReturn = false;
    uint32_t t0 = millis();

    for (uint32_t pc = 0; pc < line_count;) {
      // Timeout/cancel
      if (timeout_ms && (millis() - t0) > timeout_ms) return false;
      if (env.cancel_flag && *env.cancel_flag) return false;

      int jumpIdx = -1;
      if (!execLine(pc, retVal, didReturn, jumpIdx)) {
        // execLine returned false => RET or hard-stop
        break;
      }
      if (didReturn) break;

      if (jumpIdx >= 0 && (uint32_t)jumpIdx < line_count) pc = (uint32_t)jumpIdx;
      else pc++;

      // yield to scheduler
      tight_loop_contents();
      yield();
    }

    return true;
  }
};

}  // namespace CoProcLang