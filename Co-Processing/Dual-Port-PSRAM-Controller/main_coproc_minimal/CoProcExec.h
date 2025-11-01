/*
  CoProcExec.h
  Consolidated executor, script handling, and named function dispatch for RP2350/RP2040 co-processor.
  Notes:
  - This header assumes the following are provided by the including sketch:
      - #include "CoProcProto.h"
      - #include "CoProcLang.h"
      - A definition of BLOB_MAILBOX_MAX (macro)
      - A definition of the symbol BLOB_MAILBOX with external C linkage:
          extern "C" __attribute__((aligned(4)))
          uint8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX];
      - A definition of volatile uint8_t g_cancel_flag;
      - Optional DBG(...) macro for debug logging; if not defined, a no-op is used.
*/
#pragma once
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "CoProcProto.h"
#include "CoProcLang.h"
#ifndef DBG
#define DBG(...) \
  do { \
  } while (0)
#endif
#ifndef MAX_EXEC_ARGS
#define MAX_EXEC_ARGS 64
#endif
// These must be provided by the including sketch (definitions), we only declare them here.
extern "C" uint8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX];
extern volatile uint8_t g_cancel_flag;
// Helper: call entry in "Thumb" mode with up to MAX_EXEC_ARGS int32 args.
static inline int call_with_args_thumb(void* entryThumb, uint32_t argc, const int32_t* args) {
  if (!entryThumb) return 0;
  int32_t a64[MAX_EXEC_ARGS] = { 0 };
  uint32_t n = (argc > MAX_EXEC_ARGS) ? MAX_EXEC_ARGS : argc;
  for (uint32_t i = 0; i < n; ++i) a64[i] = args[i];
  using fvar_t = int (*)(...);
  fvar_t f = reinterpret_cast<fvar_t>(entryThumb);
  return f(
    a64[0], a64[1], a64[2], a64[3], a64[4], a64[5], a64[6], a64[7],
    a64[8], a64[9], a64[10], a64[11], a64[12], a64[13], a64[14], a64[15],
    a64[16], a64[17], a64[18], a64[19], a64[20], a64[21], a64[22], a64[23],
    a64[24], a64[25], a64[26], a64[27], a64[28], a64[29], a64[30], a64[31],
    a64[32], a64[33], a64[34], a64[35], a64[36], a64[37], a64[38], a64[39],
    a64[40], a64[41], a64[42], a64[43], a64[44], a64[45], a64[46], a64[47],
    a64[48], a64[49], a64[50], a64[51], a64[52], a64[53], a64[54], a64[55],
    a64[56], a64[57], a64[58], a64[59], a64[60], a64[61], a64[62], a64[63]);
}
class CoProcExec {
public:
  using FuncN = int32_t (*)(const int32_t* argv, uint32_t argc);
  CoProcExec()
    : g_blob(nullptr), g_blob_len(0), g_blob_cap(0), g_blob_crc(0),
      g_script(nullptr), g_script_len(0), g_script_cap(0), g_script_crc(0),
      g_exec_state(CoProc::EXEC_IDLE) {
    memset(&g_job, 0, sizeof(g_job));
    clearFuncRegistry();
  }
  void begin() {
    memset(&g_job, 0, sizeof(g_job));
    memset(BLOB_MAILBOX, 0, BLOB_MAILBOX_MAX);
    g_cancel_flag = 0;
    g_exec_state = CoProc::EXEC_IDLE;
  }
  // To be called from core1 loop
  void workerPoll() {
    if (!g_job.active) {
      tight_loop_contents();
      return;
    }
    int32_t status = 0, result = 0;
    if ((g_job.code == 0) || (g_job.size & 1u)) {
      status = CoProc::ST_PARAM;
    } else {
      BLOB_MAILBOX[0] = 0;
      void* entryThumb = (void*)(g_job.code | 1u);
      result = call_with_args_thumb(entryThumb, g_job.argc, g_job.args);
    }
    g_job.result = result;
    g_job.status = status;
    g_job.active = 0;
    DBG("[COPROC] EXEC done status=%d result=%d\n", (int)status, (int)result);
  }
  // Command handlers (build payload into out resp buffer)
  int32_t cmdHELLO(bool ispActive, uint8_t* out, size_t cap, size_t& off) {
    int32_t status = CoProc::ST_OK;
    int32_t version = (int32_t)CoProc::VERSION;
    uint32_t flags = 0;
    if (g_blob_len) flags |= 1u;
    if (g_job.active) flags |= 2u;
    if (BLOB_MAILBOX[0]) flags |= 4u;
    if (ispActive) flags |= (1u << 8);
    int32_t features = (int32_t)flags;
    DBG("[DBG] HELLO -> ver=%d features=0x%08X\n", version, (unsigned)features);
    CoProc::writePOD(out, cap, off, status);
    CoProc::writePOD(out, cap, off, version);
    CoProc::writePOD(out, cap, off, features);
    return status;
  }
  int32_t cmdINFO(bool ispActive, uint8_t* out, size_t cap, size_t& off) {
    CoProc::Info info{};
    info.impl_flags = 0;
    if (g_blob_len) info.impl_flags |= 1u;
    if (g_job.active) info.impl_flags |= 2u;
    if (BLOB_MAILBOX[0]) info.impl_flags |= 4u;
    if (ispActive) info.impl_flags |= (1u << 8);
    info.blob_len = g_blob_len;
    info.mailbox_max = BLOB_MAILBOX_MAX;
    DBG("[DBG] INFO flags=0x%08X blob_len=%u mbox=%u\n",
        (unsigned)info.impl_flags, (unsigned)info.blob_len, (unsigned)info.mailbox_max);
    int32_t status = CoProc::ST_OK;
    CoProc::writePOD(out, cap, off, status);
    CoProc::writePOD(out, cap, off, info);
    return status;
  }
  int32_t cmdLOAD_BEGIN(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    size_t p = 0;
    uint32_t total = 0;
    if (!CoProc::readPOD(in, len, p, total)) return writeStatus(out, cap, off, CoProc::ST_PARAM);
    if (total == 0 || (total & 1u)) return writeStatus(out, cap, off, CoProc::ST_SIZE);
    freeBlob();
    g_blob = (uint8_t*)malloc(total);
    if (!g_blob) return writeStatus(out, cap, off, CoProc::ST_NOMEM);
    g_blob_len = 0;
    g_blob_cap = total;
    g_blob_crc = 0;
    g_exec_state = CoProc::EXEC_LOADED;
    DBG("[DBG] LOAD_BEGIN total=%u\n", (unsigned)total);
    return writeStatus(out, cap, off, CoProc::ST_OK);
  }
  int32_t cmdLOAD_DATA(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    if (!g_blob || g_blob_len >= g_blob_cap) return writeStatus(out, cap, off, CoProc::ST_STATE);
    uint32_t can = g_blob_cap - g_blob_len;
    uint32_t take = (len < can) ? len : can;
    if (take) {
      memcpy(g_blob + g_blob_len, in, take);
      g_blob_len += take;
      g_blob_crc = CoProc::crc32_ieee(in, take, g_blob_crc);
    }
    int32_t status = (take == len) ? CoProc::ST_OK : CoProc::ST_SIZE;
    CoProc::writePOD(out, cap, off, status);
    CoProc::writePOD(out, cap, off, g_blob_len);
    DBG("[DBG] LOAD_DATA len=%u take=%u total=%u\n",
        (unsigned)len, (unsigned)take, (unsigned)g_blob_len);
    return status;
  }
  int32_t cmdLOAD_END(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    size_t p = 0;
    uint32_t expected = 0;
    if (!CoProc::readPOD(in, len, p, expected)) return writeStatus(out, cap, off, CoProc::ST_PARAM);
    int32_t status = (expected == g_blob_crc) ? CoProc::ST_OK : CoProc::ST_CRC;
    CoProc::writePOD(out, cap, off, status);
    CoProc::writePOD(out, cap, off, g_blob_len);
    DBG("[DBG] LOAD_END crc_exp=0x%08X crc_have=0x%08X st=%d\n",
        (unsigned)expected, (unsigned)g_blob_crc, (int)status);
    return status;
  }
  int32_t cmdEXEC(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    if (!g_blob || (g_blob_len & 1u)) return writeStatus2(out, cap, off, CoProc::ST_STATE, 0);
    if (g_job.active) return writeStatus2(out, cap, off, CoProc::ST_STATE, 0);
    size_t p = 0;
    uint32_t argc = 0, timeout_ms = 0;
    if (!CoProc::readPOD(in, len, p, argc)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    for (uint32_t i = 0; i < argc; ++i) {
      int32_t v = 0;
      if (!CoProc::readPOD(in, len, p, v)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
      g_job.args[i] = v;
    }
    (void)CoProc::readPOD(in, len, p, timeout_ms);
    g_job.code = (uintptr_t)g_blob;
    g_job.size = g_blob_len;
    g_job.argc = argc;
    g_job.result = 0;
    g_job.status = 0;
    g_job.active = 1;
    g_exec_state = CoProc::EXEC_RUNNING;
    DBG("[DBG] EXEC start argc=%u timeout=%u\n", (unsigned)argc, (unsigned)timeout_ms);
    uint32_t t0 = millis();
    while (g_job.active) {
      if (timeout_ms && (millis() - t0) > timeout_ms) {
        mailboxSetCancel(1);
        CoProc::writePOD(out, cap, off, (int32_t)CoProc::ST_TIMEOUT);
        CoProc::writePOD(out, cap, off, (int32_t)0);
        g_exec_state = CoProc::EXEC_DONE;
        DBG("[DBG] EXEC timeout\n");
        return CoProc::ST_TIMEOUT;
      }
      tight_loop_contents();
    }
    mailboxSetCancel(0);
    g_exec_state = CoProc::EXEC_DONE;
    CoProc::writePOD(out, cap, off, (int32_t)g_job.status);
    CoProc::writePOD(out, cap, off, (int32_t)g_job.result);
    DBG("[DBG] EXEC reply status=%d result=%d\n", (int)g_job.status, (int)g_job.result);
    return g_job.status;
  }
  int32_t cmdSTATUS(uint8_t* out, size_t cap, size_t& off) {
    int32_t status = CoProc::ST_OK;
    uint32_t es = g_exec_state;
    CoProc::writePOD(out, cap, off, status);
    CoProc::writePOD(out, cap, off, es);
    DBG("[DBG] STATUS es=%u\n", (unsigned)es);
    return status;
  }
  int32_t cmdMAILBOX_RD(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    size_t p = 0;
    uint32_t maxb = 0;
    if (!CoProc::readPOD(in, len, p, maxb)) return writeStatus(out, cap, off, CoProc::ST_PARAM);
    if (maxb > BLOB_MAILBOX_MAX) maxb = BLOB_MAILBOX_MAX;
    uint32_t n = 0;
    for (; n < maxb; ++n)
      if (BLOB_MAILBOX[n] == 0) break;
    CoProc::writePOD(out, cap, off, (int32_t)CoProc::ST_OK);
    CoProc::writePOD(out, cap, off, n);
    CoProc::writeBytes(out, cap, off, BLOB_MAILBOX, n);
    DBG("[DBG] MAILBOX_RD n=%u\n", (unsigned)n);
    return CoProc::ST_OK;
  }
  int32_t cmdCANCEL(uint8_t* out, size_t cap, size_t& off) {
    mailboxSetCancel(1);
    int32_t st = CoProc::ST_OK;
    CoProc::writePOD(out, cap, off, st);
    DBG("[DBG] CANCEL set\n");
    return st;
  }
  // Script pipeline
  int32_t cmdSCRIPT_BEGIN(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    size_t p = 0;
    uint32_t total = 0;
    if (!CoProc::readPOD(in, len, p, total)) return writeStatus(out, cap, off, CoProc::ST_PARAM);
    if (total == 0) return writeStatus(out, cap, off, CoProc::ST_SIZE);
    freeScript();
    g_script = (uint8_t*)malloc(total + 1);
    if (!g_script) return writeStatus(out, cap, off, CoProc::ST_NOMEM);
    g_script_len = 0;
    g_script_cap = total;
    g_script_crc = 0;
    DBG("[DBG] SCRIPT_BEGIN total=%u\n", (unsigned)total);
    return writeStatus(out, cap, off, CoProc::ST_OK);
  }
  int32_t cmdSCRIPT_DATA(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    if (!g_script || g_script_len >= g_script_cap) return writeStatus(out, cap, off, CoProc::ST_STATE);
    uint32_t can = g_script_cap - g_script_len;
    uint32_t take = (len < can) ? len : can;
    if (take) {
      memcpy(g_script + g_script_len, in, take);
      g_script_len += take;
      g_script_crc = CoProc::crc32_ieee(in, take, g_script_crc);
    }
    int32_t status = (take == len) ? CoProc::ST_OK : CoProc::ST_SIZE;
    CoProc::writePOD(out, cap, off, status);
    CoProc::writePOD(out, cap, off, g_script_len);
    DBG("[DBG] SCRIPT_DATA len=%u take=%u total=%u\n",
        (unsigned)len, (unsigned)take, (unsigned)g_script_len);
    return status;
  }
  int32_t cmdSCRIPT_END(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    size_t p = 0;
    uint32_t expected = 0;
    if (!CoProc::readPOD(in, len, p, expected)) return writeStatus(out, cap, off, CoProc::ST_PARAM);

    // Null-terminate for interpreter; does not affect CRC input.
    if (g_script) {
      uint32_t nulpos = (g_script_len <= g_script_cap) ? g_script_len : g_script_cap;
      g_script[nulpos] = 0;
    }

    // Canonical, one-shot CRC over the stored bytes.
    const uint32_t have = (g_script && g_script_len)
                            ? CoProc::crc32_ieee(g_script, g_script_len)
                            : 0;

    const int32_t status = (expected == have) ? CoProc::ST_OK : CoProc::ST_CRC;

    // Respond with: status, length, and have-CRC (extra 4 bytes for robustness).
    CoProc::writePOD(out, cap, off, status);
    CoProc::writePOD(out, cap, off, g_script_len);
    CoProc::writePOD(out, cap, off, have);

    DBG("[DBG] SCRIPT_END crc_exp=0x%08X crc_have=0x%08X st=%d\n",
        (unsigned)expected, (unsigned)have, (int)status);
    return status;
  }
  int32_t cmdSCRIPT_EXEC(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    if (!g_script || g_script_len == 0) return writeStatus2(out, cap, off, CoProc::ST_STATE, 0);
    size_t p = 0;
    uint32_t argc = 0, timeout_ms = 0;
    if (!CoProc::readPOD(in, len, p, argc)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    int32_t argv[MAX_EXEC_ARGS] = { 0 };
    if (argc == 0) {
      if (!CoProc::readPOD(in, len, p, timeout_ms)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    } else {
      if (p + sizeof(uint32_t) <= len) {
        uint32_t marker = 0;
        memcpy(&marker, in + p, sizeof(marker));
        if (marker == 0xFFFFFFFFu) {
          p += sizeof(uint32_t);
          DBG("[DBG] SCRIPT_EXEC: alternate ASCII-args mode detected argc=%u\n", (unsigned)argc);
          for (uint32_t i = 0; i < argc; ++i) {
            uint32_t slen = 0;
            if (!CoProc::readPOD(in, len, p, slen)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            if (slen == 0 || p + slen > len) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            if (slen > 1024) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            uint8_t tmp[1025];
            memcpy(tmp, in + p, slen);
            p += slen;
            int32_t parsed = 0;
            if (!parseAsciiInt(tmp, slen, parsed)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            argv[i] = parsed;
            DBG("[DBG] SCRIPT_EXEC arg[%u] ascii='%.*s' -> %d (0x%08X)\n",
                (unsigned)i, (int)slen, (const char*)tmp, (int)parsed, (unsigned)parsed);
          }
          if (!CoProc::readPOD(in, len, p, timeout_ms)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
        } else {
          DBG("[DBG] SCRIPT_EXEC: classic int32-args mode argc=%u\n", (unsigned)argc);
          for (uint32_t i = 0; i < argc; ++i) {
            int32_t v = 0;
            if (!CoProc::readPOD(in, len, p, v)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            argv[i] = v;
          }
          if (!CoProc::readPOD(in, len, p, timeout_ms)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
        }
      } else {
        return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
      }
    }
    if (!s_vm) {
      s_vm = new CoProcLang::VM();
      if (!s_vm) return writeStatus2(out, cap, off, CoProc::ST_NOMEM, 0);
    }
    s_vm->env.mailbox = BLOB_MAILBOX;
    s_vm->env.mailbox_max = BLOB_MAILBOX_MAX;
    s_vm->env.cancel_flag = &g_cancel_flag;
    g_exec_state = CoProc::EXEC_RUNNING;
    int32_t retVal = 0;
    bool ok = s_vm->run(g_script, g_script_len, argv, argc, timeout_ms, retVal);
    mailboxSetCancel(0);
    g_exec_state = CoProc::EXEC_DONE;
    int32_t st = ok ? (int32_t)CoProc::ST_OK : (int32_t)CoProc::ST_EXEC;
    CoProc::writePOD(out, cap, off, st);
    CoProc::writePOD(out, cap, off, (int32_t)retVal);
    return st;
  }
  // Named function registry and dispatcher (CMD_FUNC)
  // Register a function by name with fixed argc.
  // expected_argc: >=0 for exact match; -1 to allow any argc.
  bool registerFunc(const char* name, int expected_argc, FuncN fn) {
    if (!name || !fn) return false;
    // Update if already present
    for (uint32_t i = 0; i < m_funcCount; ++i) {
      if (strcmp(m_funcs[i].name, name) == 0) {
        m_funcs[i].expected_argc = expected_argc;
        m_funcs[i].fn = fn;
        return true;
      }
    }
    if (m_funcCount >= MAX_FUNCS) return false;
    m_funcs[m_funcCount].name = name;  // assume lifetime >= registry
    m_funcs[m_funcCount].expected_argc = expected_argc;
    m_funcs[m_funcCount].fn = fn;
    ++m_funcCount;
    return true;
  }
  int32_t cmdFUNC(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
    size_t p = 0;
    uint32_t nameLen = 0;
    if (!CoProc::readPOD(in, len, p, nameLen)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    if (nameLen == 0 || nameLen > 128 || p + nameLen > len) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    char nameBuf[129];
    memcpy(nameBuf, in + p, nameLen);
    nameBuf[(nameLen < 128) ? nameLen : 128] = '\0';
    p += nameLen;
    uint32_t argc = 0;
    if (!CoProc::readPOD(in, len, p, argc)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    int32_t argv[MAX_EXEC_ARGS] = { 0 };
    // Support classic int32 args or alternate ASCII-args mode with marker 0xFFFFFFFF
    if (argc == 0) {
      // nothing more
    } else {
      if (p + sizeof(uint32_t) <= len) {
        uint32_t marker = 0;
        memcpy(&marker, in + p, sizeof(marker));
        if (marker == 0xFFFFFFFFu) {
          p += sizeof(uint32_t);
          DBG("[DBG] FUNC '%s': alternate ASCII-args mode argc=%u\n", nameBuf, (unsigned)argc);
          for (uint32_t i = 0; i < argc; ++i) {
            uint32_t slen = 0;
            if (!CoProc::readPOD(in, len, p, slen)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            if (slen == 0 || p + slen > len) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            if (slen > 1024) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            uint8_t tmp[1025];
            memcpy(tmp, in + p, slen);
            p += slen;
            int32_t parsed = 0;
            if (!parseAsciiInt(tmp, slen, parsed)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            argv[i] = parsed;
            DBG("[DBG] FUNC arg[%u] ascii='%.*s' -> %d (0x%08X)\n",
                (unsigned)i, (int)slen, (const char*)tmp, (int)parsed, (unsigned)parsed);
          }
        } else {
          DBG("[DBG] FUNC '%s': classic int32-args mode argc=%u\n", nameBuf, (unsigned)argc);
          for (uint32_t i = 0; i < argc; ++i) {
            int32_t v = 0;
            if (!CoProc::readPOD(in, len, p, v)) return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
            argv[i] = v;
          }
        }
      } else {
        return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
      }
    }
    const FuncEntry* fe = findFunc(nameBuf);
    if (!fe) {
      DBG("[DBG] FUNC '%s' not found\n", nameBuf);
      return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    }
    if (fe->expected_argc >= 0 && (uint32_t)fe->expected_argc != argc) {
      DBG("[DBG] FUNC '%s' argc mismatch exp=%d got=%u\n", nameBuf, fe->expected_argc, (unsigned)argc);
      return writeStatus2(out, cap, off, CoProc::ST_PARAM, 0);
    }
    int32_t rv = fe->fn(argv, argc);
    CoProc::writePOD(out, cap, off, (int32_t)CoProc::ST_OK);
    CoProc::writePOD(out, cap, off, rv);
    return CoProc::ST_OK;
  }
  // Accessors (if needed by the sketch)
  uint32_t getExecState() const {
    return g_exec_state;
  }
  uint32_t getBlobLen() const {
    return g_blob_len;
  }
  bool isJobActive() const {
    return g_job.active != 0;
  }
private:
  struct ExecJob {
    uintptr_t code;
    uint32_t size;
    uint32_t argc;
    int32_t args[MAX_EXEC_ARGS];
    volatile uint8_t active;
    volatile int32_t result;
    volatile int32_t status;
  };
  ExecJob g_job;
  uint8_t* g_blob;
  uint32_t g_blob_len;
  uint32_t g_blob_cap;
  uint32_t g_blob_crc;
  uint8_t* g_script;
  uint32_t g_script_len;
  uint32_t g_script_cap;
  uint32_t g_script_crc;
  volatile uint32_t g_exec_state;
  inline static CoProcLang::VM* s_vm = nullptr;
  // Function registry
  struct FuncEntry {
    const char* name;
    int expected_argc;  // -1 = any
    FuncN fn;
  };
  static constexpr uint32_t MAX_FUNCS = 32;
  FuncEntry m_funcs[MAX_FUNCS];
  uint32_t m_funcCount = 0;
  void clearFuncRegistry() {
    m_funcCount = 0;
    for (uint32_t i = 0; i < MAX_FUNCS; ++i) {
      m_funcs[i].name = nullptr;
      m_funcs[i].expected_argc = -1;
      m_funcs[i].fn = nullptr;
    }
  }
  const FuncEntry* findFunc(const char* name) const {
    for (uint32_t i = 0; i < m_funcCount; ++i) {
      if (m_funcs[i].name && strcmp(m_funcs[i].name, name) == 0) {
        return &m_funcs[i];
      }
    }
    return nullptr;
  }
  static inline void mailboxSetCancel(uint8_t v) {
    g_cancel_flag = v;
  }
  static bool parseAsciiInt(const uint8_t* s, size_t n, int32_t& out) {
    if (!s || n == 0) return false;
    const char* p = (const char*)s;
    size_t i = 0;
    while (i < n && (p[i] == ' ' || p[i] == '\t' || p[i] == '\r' || p[i] == '\n')) ++i;
    if (i >= n) return false;
    bool neg = false;
    if (p[i] == '+' || p[i] == '-') {
      neg = (p[i] == '-');
      ++i;
    }
    if (i >= n) return false;
    if (i + 1 < n && p[i] == '0' && (p[i + 1] == 'x' || p[i + 1] == 'X')) {
      i += 2;
      if (i >= n) return false;
      uint32_t v = 0;
      bool saw = false;
      while (i < n) {
        char c = p[i];
        int d = -1;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else break;
        saw = true;
        v = (v << 4) | (uint32_t)d;
        ++i;
      }
      if (!saw) return false;
      out = neg ? -(int32_t)v : (int32_t)v;
      return true;
    }
    int32_t v = 0;
    bool saw = false;
    while (i < n) {
      char c = p[i];
      if (c < '0' || c > '9') break;
      saw = true;
      v = v * 10 + (c - '0');
      ++i;
    }
    if (!saw) return false;
    out = neg ? -v : v;
    return true;
  }
  void freeBlob() {
    if (g_blob) free(g_blob);
    g_blob = nullptr;
    g_blob_len = 0;
    g_blob_cap = 0;
    g_blob_crc = 0;
    g_exec_state = CoProc::EXEC_IDLE;
    DBG("[COPROC] blob freed\n");
  }
  void freeScript() {
    if (g_script) free(g_script);
    g_script = nullptr;
    g_script_len = 0;
    g_script_cap = 0;
    g_script_crc = 0;
    DBG("[COPROC] script freed\n");
  }
  static int32_t writeStatus(uint8_t* out, size_t cap, size_t& off, int32_t st) {
    CoProc::writePOD(out, cap, off, st);
    return st;
  }
  static int32_t writeStatus2(uint8_t* out, size_t cap, size_t& off, int32_t st, int32_t v) {
    CoProc::writePOD(out, cap, off, st);
    CoProc::writePOD(out, cap, off, v);
    return st;
  }
};