// MCCompiler.h
// Extremely simplified, single-header "compiler" for ARM Cortex-M0+ (RP2040).
// It parses a tiny C-like fragment:
//   int main() { return <expr>; }
// Where <expr> supports:
//   - 32-bit signed integer literals (decimal or 0x... hex)
//   - binary operators: +, -, *
//   - unary minus: -x
//   - parentheses: ( ... )
// It emits flat raw ARM Thumb-1 machine code bytes (no ELF, no headers).
// Output is a callable function body with a minimal prologue/epilogue:
//
//   push {lr}
//   ... compute expression into r0 using a small temp stack on SP ...
//   pop  {pc}
//
// Result is machine code intended for Cortex-M0+ (Thumb-only). No FPU, no
// external helper calls, no division, no variables, no function calls.
//
// Usage example:
//
//   #include "MCCompiler.h"
//   MCCompiler comp;
//   uint8_t out[256];
//   size_t outSize = 0;
//   const char* src = "int main(){ return (12 + 34) * 5; }";
//   MCCompiler::Result res = comp.compile(src, strlen(src), out, sizeof(out), &outSize);
//   if (!res.ok) {
//     Serial.print("Compile error at pos "); Serial.println(res.errorPos);
//     Serial.println(res.errorMsg ? res.errorMsg : "Unknown error");
//   } else {
//     // 'out' now holds raw Thumb-1 opcodes for a function that returns r0.
//     // NOTE: Executing generated code is not provided here.
//   }
//
// Notes:
// - Generates a literal pool at the end for constants > 255 using LDR (literal).
// - Very small operator stack and output RPN buffer (configurable).
// - No heap allocation; all fixed-size scratch buffers.
// - All instructions are 16-bit Thumb encodings suitable for Cortex-M0+.
// - This header is self-contained (C++), Arduino-friendly.

#ifndef MCCOMPILER_H_
#define MCCOMPILER_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>

class MCCompiler {
public:
  struct Result {
    bool ok = false;
    const char* errorMsg = nullptr;
    size_t errorPos = 0;
    size_t outSize = 0;
  };

  // Compile 'src[0..srcLen)' into raw Thumb-1 machine code in 'outBuf'.
  // - outBuf receives: push{lr} ... body ... pop{pc} ... literal-pool
  // - outCap: capacity of outBuf in bytes
  // - outSize: actual number of bytes written
  // Returns Result with ok=false on error.
  Result compile(const char* src, size_t srcLen, uint8_t* outBuf, size_t outCap, size_t* outSize) {
    initError();
    if (!src || !outBuf || !outSize) return error("null arg", 0);

    reset();

    _src = src;
    _srcLen = srcLen;

    if (!parseProgram()) return makeErr();

    // Convert to postfix (RPN) using Shunting-Yard on the recorded expression slice
    if (!toRPN()) return makeErr();

    // Emit machine code into outBuf
    _out = outBuf;
    _outCap = outCap;
    _outSz = 0;

    // Prologue: push {lr}
    if (!emit16(0xB500)) return error("output buffer too small", _tokPosStart);

    // Generate code for RPN
    if (!emitRPN()) return makeErr();

    // Final result should be exactly one stack item: pop -> r0
    if (_stackDepth != 1) return error("internal stack depth mismatch", _tokPosEnd);
    if (!emitPopR(0)) return error("output buffer too small", _tokPosEnd);

    // Epilogue now (return BEFORE literal pool)
    if (!emit16(0xBD00)) return error("output buffer too small", _tokPosEnd);  // POP {PC}

    // Align to 4 for the literal pool storage
    while ((_outSz & 0x3) != 0) {
      if (!emit16(0xBF00)) return error("output buffer too small", _tokPosEnd);  // NOP
    }

    // Append literal pool and fixup LDR literal imm8
    if (!fixupAndEmitLiteralPool()) return makeErr();

    *outSize = _outSz;
    Result r;
    r.ok = true;
    r.outSize = _outSz;
    return r;
  }

  // Limits (you can tweak for your environment)
  static const int MAX_TOKENS = 256;    // max tokens in expression
  static const int MAX_RPN = 256;       // max RPN items
  static const int MAX_OPSTACK = 64;    // max operator stack
  static const int MAX_LITERALS = 128;  // max unique literal constants

private:
  // Lexer/Parser support (extremely small)
  enum TokKind : uint8_t {
    TK_EOF = 0,
    TK_NUM,
    TK_PLUS,
    TK_MINUS,
    TK_MUL,
    TK_LPAREN,
    TK_RPAREN,
    TK_INT,
    TK_MAIN,
    TK_RETURN,
    TK_LBRACE,
    TK_RBRACE,
    TK_SEMI
  };
  struct Token {
    TokKind kind;
    int32_t ival;  // for TK_NUM
    size_t pos;    // error location
  };

  // RPN item
  enum RPNKind : uint8_t { RPN_NUM = 0,
                           RPN_ADD,
                           RPN_SUB,
                           RPN_MUL,
                           RPN_NEG };
  struct RPNItem {
    RPNKind k;
    int32_t v;  // for numbers
    size_t pos;
  };

  // Fixup for LDR literal instruction imm8
  struct LdrFixup {
    size_t instrOffset;  // byte offset where the 16-bit LDR literal is
    int32_t value;       // constant to load
  };

  // Internal state
  const char* _src = nullptr;
  size_t _srcLen = 0;
  size_t _idx = 0;

  // expression capture bounds in source (after "return")
  size_t _exprStart = 0;
  size_t _exprEnd = 0;
  size_t _tokPosStart = 0;
  size_t _tokPosEnd = 0;

  Token _toks[MAX_TOKENS];
  int _ntok = 0;

  // RPN
  RPNItem _rpn[MAX_RPN];
  int _nrpn = 0;

  // Output buffer
  uint8_t* _out = nullptr;
  size_t _outCap = 0;
  size_t _outSz = 0;

  // For constant pool and fixups
  int32_t _literals[MAX_LITERALS];
  int _nlit = 0;
  LdrFixup _fixups[MAX_LITERALS];
  int _nfix = 0;

  // virtual eval stack depth (push {r0}, pop {r0}/{r1})
  int _stackDepth = 0;

  // Error tracking
  const char* _errMsg = nullptr;
  size_t _errPos = 0;

  void reset() {
    _idx = 0;
    _ntok = 0;
    _nrpn = 0;
    _nlit = 0;
    _nfix = 0;
    _out = nullptr;
    _outCap = 0;
    _outSz = 0;
    _stackDepth = 0;
    _exprStart = _exprEnd = 0;
    _tokPosStart = _tokPosEnd = 0;
    _errMsg = nullptr;
    _errPos = 0;
  }

  void initError() {
    _errMsg = nullptr;
    _errPos = 0;
  }
  Result makeErr() {
    Result r;
    r.ok = false;
    r.errorMsg = _errMsg;
    r.errorPos = _errPos;
    r.outSize = 0;
    return r;
  }
  Result error(const char* m, size_t p) {
    _errMsg = m;
    _errPos = p;
    return makeErr();
  }
  inline bool fail(const char* m, size_t p) {
    _errMsg = m;
    _errPos = p;
    return false;
  }

  static bool isSpace(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
  }
  static bool isDigit(char c) {
    return c >= '0' && c <= '9';
  }
  static bool isHex(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
  }
  static int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return 0;
  }

  void skipWs() {
    while (_idx < _srcLen && isSpace(_src[_idx])) _idx++;
  }

  bool matchKW(const char* kw) {
    size_t klen = strlen(kw);
    if (_idx + klen > _srcLen) return false;
    if (memcmp(_src + _idx, kw, klen) == 0) {
      // ensure boundary
      char after = (_idx + klen < _srcLen) ? _src[_idx + klen] : '\0';
      if ((after >= 'a' && after <= 'z') || (after >= 'A' && after <= 'Z') || (after == '_') || (after >= '0' && after <= '9')) {
        return false;
      }
      _idx += klen;
      return true;
    }
    return false;
  }

  bool parseProgram() {
    // Two modes:
    // 1) Full tiny form: "int main() { return <expr>; }"
    // 2) Just an expression: "<expr>"
    size_t save = _idx;
    skipWs();
    bool asTinyC = false;
    size_t startPos = _idx;

    if (matchKW("int")) {
      skipWs();
      if (matchKW("main")) {
        skipWs();
        if (_idx < _srcLen && _src[_idx] == '(') {
          _idx++;
          skipWs();
        } else return fail("expected '(' after main", _idx);

        if (_idx < _srcLen && _src[_idx] == ')') {
          _idx++;
          skipWs();
        } else return fail("expected ')'", _idx);

        if (_idx < _srcLen && _src[_idx] == '{') {
          _idx++;
          skipWs();
        } else return fail("expected '{'", _idx);

        if (!matchKW("return")) return fail("expected 'return'", _idx);
        skipWs();
        asTinyC = true;
      } else {
        // "int" appeared but not tiny main - fallback to expr with the leading 'int' is illegal
        _idx = save;
      }
    }

    _exprStart = _idx;
    if (!lexExpressionTokens()) return false;  // fills _toks[]
    _exprEnd = _idx;

    if (asTinyC) {
      skipWs();
      if (!consume(TK_SEMI)) return fail("expected ';' after return expr", _idx);
      skipWs();
      if (_idx >= _srcLen || _src[_idx] != '}') return fail("expected '}'", _idx);
      _idx++;
      skipWs();
      if (_idx != _srcLen) {
        // trailing stuff allowed? We'll allow whitespace only.
        for (size_t i = _idx; i < _srcLen; i++)
          if (!isSpace(_src[i])) return fail("trailing characters", i);
      }
    } else {
      // Expression-only mode: ensure we consumed entire input (whitespace allowed)
      skipWs();
      if (_idx != _srcLen) return fail("unexpected trailing characters", _idx);
    }

    if (_ntok <= 0) return fail("empty expression", startPos);
    _tokPosStart = _toks[0].pos;
    _tokPosEnd = _toks[_ntok - 1].pos;
    return true;
  }

  bool lexExpressionTokens() {
    // produce tokens until we hit ';', '}' or EOF
    _ntok = 0;
    bool expectUnary = true;  // at start of an expression, '-' is unary

    while (_idx < _srcLen) {
      skipWs();
      if (_idx >= _srcLen) break;

      char c = _src[_idx];
      size_t pos = _idx;

      if (isDigit(c)) {
        // number literal decimal or hex
        int32_t val = 0;
        if (c == '0' && (_idx + 1 < _srcLen) && (_src[_idx + 1] == 'x' || _src[_idx + 1] == 'X')) {
          _idx += 2;
          if (_idx >= _srcLen || !isHex(_src[_idx])) return fail("malformed hex literal", pos);
          val = 0;
          while (_idx < _srcLen && isHex(_src[_idx])) {
            val = (val << 4) | hexVal(_src[_idx]);
            _idx++;
          }
        } else {
          val = 0;
          while (_idx < _srcLen && isDigit(_src[_idx])) {
            val = val * 10 + (_src[_idx] - '0');
            _idx++;
          }
        }
        if (!emitTokNum(val, pos)) return false;
        expectUnary = false;
        continue;
      }

      // punctuation or end of expr
      if (c == ';' || c == '}') {
        // do not consume here; caller may consume
        break;
      }

      if (c == '(') {
        if (!emitTok(TK_LPAREN, pos)) return false;
        _idx++;
        expectUnary = true;
        continue;
      }
      if (c == ')') {
        if (!emitTok(TK_RPAREN, pos)) return false;
        _idx++;
        expectUnary = false;
        continue;
      }
      if (c == '*') {
        if (!emitTok(TK_MUL, pos)) return false;
        _idx++;
        expectUnary = true;
        continue;
      }

      if (c == '+') {
        if (!emitTok(TK_PLUS, pos)) return false;
        _idx++;
        expectUnary = true;
        continue;
      }

      if (c == '-') {
        // distinguished later in toRPN
        if (!emitTok(TK_MINUS, pos)) return false;
        _idx++;
        expectUnary = true;
        continue;
      }

      // If we see "return" during lex expr (tiny mode), it means caller didn't cut input correctly
      if (matchKW("return")) return fail("unexpected 'return' in expression", pos);

      // Otherwise it's invalid char
      return fail("invalid character in expression", pos);
    }

    return (_ntok > 0);
  }

  bool consume(TokKind want) {
    skipWs();
    if (_idx >= _srcLen) return false;
    char c = _src[_idx];
    if (want == TK_SEMI && c == ';') {
      _idx++;
      return true;
    }
    if (want == TK_RBRACE && c == '}') {
      _idx++;
      return true;
    }
    return false;
  }

  bool emitTok(TokKind k, size_t pos) {
    if (_ntok >= MAX_TOKENS) return fail("too many tokens", pos);
    _toks[_ntok].kind = k;
    _toks[_ntok].ival = 0;
    _toks[_ntok].pos = pos;
    _ntok++;
    return true;
  }
  bool emitTokNum(int32_t v, size_t pos) {
    if (_ntok >= MAX_TOKENS) return fail("too many tokens", pos);
    _toks[_ntok].kind = TK_NUM;
    _toks[_ntok].ival = v;
    _toks[_ntok].pos = pos;
    _ntok++;
    return true;
  }

  // Operator precedence/associativity
  // We will treat unary minus (NEG) as higher precedence than MUL, and MUL higher than PLUS/MINUS.
  struct OpInfo {
    uint8_t prec;
    bool rightAssoc;
    bool unary;
    RPNKind rpn;
  };

  static OpInfo opInfoFor(TokKind k, bool unaryMinus) {
    if (k == TK_MINUS && unaryMinus) {
      OpInfo o;
      o.prec = 3;
      o.rightAssoc = true;
      o.unary = true;
      o.rpn = RPN_NEG;
      return o;
    }
    if (k == TK_MUL) {
      OpInfo o;
      o.prec = 2;
      o.rightAssoc = false;
      o.unary = false;
      o.rpn = RPN_MUL;
      return o;
    }
    if (k == TK_PLUS) {
      OpInfo o;
      o.prec = 1;
      o.rightAssoc = false;
      o.unary = false;
      o.rpn = RPN_ADD;
      return o;
    }
    if (k == TK_MINUS) {
      OpInfo o;
      o.prec = 1;
      o.rightAssoc = false;
      o.unary = false;
      o.rpn = RPN_SUB;
      return o;
    }
    OpInfo z;
    z.prec = 0;
    z.rightAssoc = false;
    z.unary = false;
    z.rpn = RPN_ADD;
    return z;
  }

  bool toRPN() {
    _nrpn = 0;
    _opTop = 0;
    bool expectUnary = true;

    for (int i = 0; i < _ntok; i++) {
      TokKind k = _toks[i].kind;
      size_t pos = _toks[i].pos;

      if (k == TK_NUM) {
        if (_nrpn >= MAX_RPN) return fail("RPN overflow", pos);
        _rpn[_nrpn++] = RPNItem{ RPN_NUM, _toks[i].ival, pos };
        expectUnary = false;
        continue;
      }

      if (k == TK_LPAREN) {
        if (!pushLParen(pos)) return false;
        expectUnary = true;
        continue;
      }

      if (k == TK_RPAREN) {
        // pop until '(' sentinel
        bool matched = false;
        while (_opTop > 0) {
          if (_opPrec[_opTop - 1] == 0xFF && !_opAssoc[_opTop - 1]) {  // '('
            matched = true;
            _opTop--;
            break;
          }
          if (!flushTopOpToRPN(_opPos[_opTop - 1])) return false;
          _opTop--;
        }
        if (!matched) return fail("mismatched ')'", pos);
        expectUnary = false;
        continue;
      }

      // Operators: +, -, *
      if (k == TK_PLUS || k == TK_MINUS || k == TK_MUL) {
        bool isUnaryMinus = (k == TK_MINUS && expectUnary);
        OpInfo oi = opInfoFor(k, isUnaryMinus);
        if (!opPush(oi, pos)) return false;
        expectUnary = true;
        continue;
      }

      return fail("invalid token in expression", pos);
    }

    // Flush operators
    while (_opTop > 0) {
      if (_opPrec[_opTop - 1] == 0xFF && !_opAssoc[_opTop - 1]) {
        return fail("mismatched '('", (_ntok > 0) ? _toks[_ntok - 1].pos : 0);
      }
      if (!flushTopOpToRPN(_opPos[_opTop - 1])) return false;
      _opTop--;
    }

    if (_nrpn <= 0) return fail("empty expression", (_ntok > 0) ? _toks[0].pos : 0);
    return true;
  }

  // Tiny operator stack implementation (to support associativity and precedence cleanly)
  uint8_t _opPrec[MAX_OPSTACK];  // precedence
  bool _opRight[MAX_OPSTACK];    // right-assoc
  bool _opUnary[MAX_OPSTACK];    // unary?
  RPNKind _opRPN[MAX_OPSTACK];   // corresponding RPN op kind
  size_t _opPos[MAX_OPSTACK];    // position for error context
  bool _opAssoc[MAX_OPSTACK];    // occupied marker (true = operator; false = '(')
  int _opTop = 0;

  bool opPush(const OpInfo& oi, size_t pos) {
    // Pop while top has higher prec, or equal prec and left-assoc
    while (_opTop > 0 && _opAssoc[_opTop - 1]) {
      uint8_t tp = _opPrec[_opTop - 1];
      if (tp == 0xFF) break;  // '(' sentinel (shouldn't be marked as assoc=true, but guard anyway)
      if (tp > oi.prec || (tp == oi.prec && !oi.rightAssoc)) {
        if (!flushTopOpToRPN(_opPos[_opTop - 1])) return false;
        _opTop--;
      } else break;
    }

    if (_opTop >= MAX_OPSTACK) return fail("operator stack overflow", pos);
    _opPrec[_opTop] = oi.prec;
    _opRight[_opTop] = oi.rightAssoc;
    _opUnary[_opTop] = oi.unary;
    _opRPN[_opTop] = oi.rpn;
    _opPos[_opTop] = pos;
    _opAssoc[_opTop] = true;
    _opTop++;
    return true;
  }

  bool flushTopOpToRPN(size_t pos) {
    if (_opTop <= 0) return fail("operator stack underflow", pos);
    if (_nrpn >= MAX_RPN) return fail("RPN overflow", pos);
    // push op to RPN
    _rpn[_nrpn++] = RPNItem{ _opRPN[_opTop - 1], 0, pos };
    return true;
  }

  // Place a '(' sentinel in operator stack
  bool pushLParen(size_t pos) {
    if (_opTop >= MAX_OPSTACK) return fail("operator stack overflow", pos);
    _opPrec[_opTop] = 0xFF;  // sentinel
    _opRight[_opTop] = false;
    _opUnary[_opTop] = false;
    _opRPN[_opTop] = RPN_ADD;
    _opPos[_opTop] = pos;
    _opAssoc[_opTop] = false;  // mark as '('
    _opTop++;
    return true;
  }

  // Emitters: 16-bit Thumb instructions (little-endian)
  bool emit16(uint16_t hw) {
    if (_outSz + 2 > _outCap) return false;
    _out[_outSz + 0] = (uint8_t)(hw & 0xFF);
    _out[_outSz + 1] = (uint8_t)(hw >> 8);
    _outSz += 2;
    return true;
  }

  bool emitPushR(uint8_t r) {  // r in 0..7
    if (r > 7) return false;
    return emit16((uint16_t)(0xB400 | (1u << r)));  // PUSH {r}
  }
  bool emitPopR(uint8_t r) {
    if (r > 7) return false;
    return emit16((uint16_t)(0xBC00 | (1u << r)));  // POP {r}
  }

  bool emitADD_r0_r0_r1() {
    // ADD (register) T1: 0001100 Rm(3) Rn(3) Rd(3)
    // r0 = r0 + r1 => Rd=r0, Rn=r0, Rm=r1 => 0x1800 | (1<<6) | (0<<3) | 0
    return emit16(0x1800 | (1u << 6));
  }
  bool emitSUB_r0_r0_r1() {
    // SUB (register) T1: 0001101 Rm Rn Rd
    // r0 = r0 - r1 => Rd=r0, Rn=r0, Rm=r1 => 0x1A00 | (1<<6)
    return emit16(0x1A00 | (1u << 6));
  }
  bool emitMULS_r0_r0_r1() {
    // MULS Rd, Rm  (010000 1101 Rm Rdn) with Rdn=r0, Rm=r1 => 0x4340 | (1<<3) | 0
    return emit16(0x4340 | (1u << 3));
  }

  bool emitMOVS_r0_imm8(uint8_t imm) {
    // MOVS (immediate) T1: 00100 Rd imm8 => Rd=r0, 0x2000 | imm8
    return emit16(0x2000 | imm);
  }

  // LDR literal T1: 01001 Rt imm8 => LDR Rt, [PC, imm*4], base is aligned(PC,4)
  // We'll emit with imm8=0 for now and fixup later once pool address is known.
  bool emitLDR_lit_r0_andFixup(int32_t value) {
    if (_nfix >= MAX_LITERALS) return fail("too many literal fixups", _tokPosEnd);
    size_t instrOff = _outSz;
    if (!emit16(0x4800 /* Rt=r0, imm8=0; fixup later */)) return false;

    // track fixup and ensure literal uniqueness (dedupe)
    int litIndex = findOrAddLiteral(value);
    if (litIndex < 0) return fail("literal pool full", _tokPosEnd);

    _fixups[_nfix].instrOffset = instrOff;
    _fixups[_nfix].value = value;  // we will map to index on final emit
    _nfix++;
    return true;
  }

  int findLiteralIndex(int32_t v) const {
    for (int i = 0; i < _nlit; i++)
      if (_literals[i] == v) return i;
    return -1;
  }
  int addLiteral(int32_t v) {
    if (_nlit >= MAX_LITERALS) return -1;
    _literals[_nlit++] = v;
    return _nlit - 1;
  }
  int findOrAddLiteral(int32_t v) {
    int idx = findLiteralIndex(v);
    if (idx >= 0) return idx;
    return addLiteral(v);
  }

  bool loadImmToR0(int32_t v, size_t /*pos*/) {
    // small imm: [0..255]
    if (v >= 0 && v <= 255) return emitMOVS_r0_imm8((uint8_t)v);
    // negative or large => LDR literal (fixup)
    return emitLDR_lit_r0_andFixup(v);
  }

  bool emitRPN() {
    _stackDepth = 0;
    for (int i = 0; i < _nrpn; i++) {
      const RPNItem& it = _rpn[i];
      switch (it.k) {
        case RPN_NUM:
          {
            if (!loadImmToR0(it.v, it.pos)) return fail("emit number failed", it.pos);
            if (!emitPushR(0)) return fail("output buffer too small", it.pos);
            _stackDepth++;
            break;
          }
        case RPN_ADD:
          {
            if (!binaryOpPrep(it.pos)) return false;
            if (!emitADD_r0_r0_r1()) return fail("output buffer too small", it.pos);
            if (!emitPushR(0)) return fail("output buffer too small", it.pos);
            _stackDepth++;
            break;
          }
        case RPN_SUB:
          {
            if (!binaryOpPrep(it.pos)) return false;
            if (!emitSUB_r0_r0_r1()) return fail("output buffer too small", it.pos);
            if (!emitPushR(0)) return fail("output buffer too small", it.pos);
            _stackDepth++;
            break;
          }
        case RPN_MUL:
          {
            if (!binaryOpPrep(it.pos)) return false;
            if (!emitMULS_r0_r0_r1()) return fail("output buffer too small", it.pos);
            if (!emitPushR(0)) return fail("output buffer too small", it.pos);
            _stackDepth++;
            break;
          }
        case RPN_NEG:
          {
            // unary minus: pop r1 (x), movs r0,#0; sub r0,r0,r1; push r0
            if (!ensureStack(1, it.pos)) return false;
            if (!emitPopR(1)) return fail("output buffer too small", it.pos);  // r1 <- x
            _stackDepth--;
            if (!emitMOVS_r0_imm8(0)) return fail("output buffer too small", it.pos);
            if (!emitSUB_r0_r0_r1()) return fail("output buffer too small", it.pos);
            if (!emitPushR(0)) return fail("output buffer too small", it.pos);
            _stackDepth++;
            break;
          }
        default:
          return fail("unsupported RPN op", it.pos);
      }
    }
    return true;
  }

  bool ensureStack(int need, size_t pos) {
    if (_stackDepth < need) return fail("stack underflow", pos);
    return true;
  }

  bool binaryOpPrep(size_t pos) {
    // pop b -> r1, pop a -> r0, compute (a op b), push r0
    if (!ensureStack(2, pos)) return false;
    if (!emitPopR(1)) return fail("output buffer too small", pos);
    if (!emitPopR(0)) return fail("output buffer too small", pos);
    _stackDepth -= 2;
    return true;
  }

  bool fixupAndEmitLiteralPool() {
    if (_nlit == 0) return true;  // no pool

    // pool starts at current _outSz (already 4-aligned)
    size_t poolBase = _outSz;

    // Emit all literals in order they were added
    for (int i = 0; i < _nlit; i++) {
      int32_t v = _literals[i];
      if (_outSz + 4 > _outCap) return fail("output buffer too small for literal pool", _tokPosEnd);
      _out[_outSz + 0] = (uint8_t)(v & 0xFF);
      _out[_outSz + 1] = (uint8_t)((v >> 8) & 0xFF);
      _out[_outSz + 2] = (uint8_t)((v >> 16) & 0xFF);
      _out[_outSz + 3] = (uint8_t)((v >> 24) & 0xFF);
      _outSz += 4;
    }

    // Now fixup all LDR literal imm8 fields
    for (int fi = 0; fi < _nfix; fi++) {
      const LdrFixup& fx = _fixups[fi];
      int litIdx = findLiteralIndex(fx.value);
      if (litIdx < 0) return fail("internal: literal not found", fx.instrOffset);

      size_t instrAddr = fx.instrOffset;  // byte offset
      size_t pcAtInstr = instrAddr + 4;   // PC for Thumb is instr address + 4
      size_t pcAligned = pcAtInstr & ~((size_t)3);

      size_t litAddr = poolBase + (size_t)litIdx * 4;
      if (litAddr < pcAligned) return fail("literal pool placed before reference (unexpected)", instrAddr);
      size_t byteDiff = litAddr - pcAligned;
      if ((byteDiff % 4) != 0) return fail("literal pool misalignment", instrAddr);
      size_t imm8 = byteDiff / 4;
      if (imm8 > 255) return fail("literal too far (imm8 overflow)", instrAddr);

      // LDR literal T1 encoding at instrOffset:
      // opcode 01001 Rt imm8 => base halfword = 0x4800 | (Rt<<8) | imm8
      // Here Rt==r0, so we need to set low 8 bits to imm8.
      uint16_t hw = (uint16_t)(_out[instrAddr] | (_out[instrAddr + 1] << 8));
      hw = (uint16_t)((hw & 0xFF00) | (uint16_t)imm8);
      _out[instrAddr] = (uint8_t)(hw & 0xFF);
      _out[instrAddr + 1] = (uint8_t)(hw >> 8);
    }

    return true;
  }
};

#endif  // MCCOMPILER_H_