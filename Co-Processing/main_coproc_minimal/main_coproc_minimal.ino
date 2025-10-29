/*
  main_coproc_softserial.ino
  RP2350 / RP2040 Co-Processor (software-serial "bitbanged UART") — PoC transport + blob executor + script interpreter
  --------------------------------------------------------------------------------------------------------------------
  - Software serial (SoftwareSerial) transport: 8N1 framing over two GPIOs.
  - Core0: transport + protocol using a framed request/response over TX/RX pins.
  - Core1: blob executor (setup1/loop1 pattern).
  - Adds CoProcLang script pipeline (SCRIPT_BEGIN/DATA/END/EXEC) alongside binary blob pipeline.
  Pins (as requested):
    RX = GP0
    TX = GP1
  Notes:
    - This replaces the previous SPI slave transport with a byte-stream serial link.
    - Framing is the same as CoProcProto.h: 24B header + payload + CRC32(payload).
    - The code scans the serial stream for MAGIC+VERSION and then reads a full frame.
    - Baud rate is configurable (SOFT_BAUD). Default: 230400.
*/
#include <Arduino.h>

#include "BusArbiterWithISP.h"
#include "PSRAMMulti.h"
#include "rp_selfupdate.h"

#include <SoftwareSerial.h>
#include "CoProcProto.h"
#define COPROCLANG_MAX_LINES 512
#define COPROCLANG_MAX_LABELS 128
#include "CoProcLang.h"
// ------- Debug toggle -------
#ifndef COPROC_DEBUG
#define COPROC_DEBUG 1
#endif
#if COPROC_DEBUG
#define DBG(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(...) \
  do { \
  } while (0)
#endif
// ------- Pins (as requested) -------
static const uint8_t PIN_RX = 0;  // GP0  (co-processor RX)
static const uint8_t PIN_TX = 1;  // GP1  (co-processor TX)
// ------- Software serial config -------
#ifndef SOFT_BAUD
#define SOFT_BAUD 230400
#endif
static SoftwareSerial link(PIN_RX, PIN_TX, false);  // RX, TX, non-inverted
// ------- Mailbox / cancel (shared with blob + script) -------
#ifndef BLOB_MAILBOX_MAX
#define BLOB_MAILBOX_MAX 256
#endif
extern "C" __attribute__((aligned(4)))
uint8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };
static volatile uint8_t g_cancel_flag = 0;
// ------- Executor plumbing (core1) -------
#ifndef MAX_EXEC_ARGS
#define MAX_EXEC_ARGS 64
#endif
struct ExecJob {
  uintptr_t code;
  uint32_t size;
  uint32_t argc;
  int32_t args[MAX_EXEC_ARGS];
  volatile uint8_t active;
  volatile int32_t result;
  volatile int32_t status;
};
static ExecJob g_job;
// call_with_args_thumb same as before
extern "C" int call_with_args_thumb(void* entryThumb, uint32_t argc, const int32_t* args) {
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
static void core1WorkerPoll() {
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
// ------- Blob buffer / state -------
static uint8_t* g_blob = nullptr;
static uint32_t g_blob_len = 0;
static uint32_t g_blob_cap = 0;
static uint32_t g_blob_crc = 0;
static volatile uint32_t g_exec_state = CoProc::EXEC_IDLE;
// ------- Script buffer / state -------
static uint8_t* g_script = nullptr;
static uint32_t g_script_len = 0;
static uint32_t g_script_cap = 0;
static uint32_t g_script_crc = 0;
// ------- Transport / protocol buffers -------
static CoProc::Frame g_reqHdr, g_respHdr;
static uint8_t* g_reqBuf = nullptr;
static uint8_t* g_respBuf = nullptr;
static const uint32_t RESP_MAX = 8192;
static const uint32_t REQ_MAX = 8192;
// ------- Helpers -------
static inline void mailboxSetCancel(uint8_t v) {
  g_cancel_flag = v;
}
static inline uint8_t mailboxGetCancel() {
  return g_cancel_flag;
}
static void freeBlob() {
  if (g_blob) free(g_blob);
  g_blob = nullptr;
  g_blob_len = 0;
  g_blob_cap = 0;
  g_blob_crc = 0;
  g_exec_state = CoProc::EXEC_IDLE;
  DBG("[COPROC] blob freed\n");
}
static void freeScript() {
  if (g_script) free(g_script);
  g_script = nullptr;
  g_script_len = 0;
  g_script_cap = 0;
  g_script_crc = 0;
  DBG("[COPROC] script freed\n");
}
// Read a single byte with timeout (ms). Returns true if a byte was read.
static bool readByte(uint8_t& b, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) <= timeoutMs) {
    int a = link.available();
    if (a > 0) {
      int v = link.read();
      if (v >= 0) {
        b = (uint8_t)v;
        return true;
      }
    }
    tight_loop_contents();
    yield();
  }
  return false;
}
// Read exactly n bytes with timeout per byte. Returns true on success.
static bool readExact(uint8_t* dst, size_t n, uint32_t timeoutPerByteMs) {
  for (size_t i = 0; i < n; ++i) {
    if (!readByte(dst[i], timeoutPerByteMs)) return false;
  }
  return true;
}
// Write all bytes with a global timeout. Returns true on success.
static bool writeAll(const uint8_t* src, size_t n, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t off = 0;
  while (off < n) {
    size_t wrote = link.write(src + off, n - off);
    if (wrote > 0) {
      off += wrote;
      continue;
    }
    if ((millis() - start) > timeoutMs) return false;
    yield();
  }
  link.flush();  // ensure pushed out
  return true;
}
// Small helper: parse ASCII decimal or 0xHEX into int32
static bool parseAsciiInt(const uint8_t* s, size_t n, int32_t& out) {
  if (!s || n == 0) return false;
  const char* p = (const char*)s;
  size_t i = 0;
  // skip leading whitespace
  while (i < n && (p[i] == ' ' || p[i] == '\t' || p[i] == '\r' || p[i] == '\n')) ++i;
  if (i >= n) return false;
  bool neg = false;
  if (p[i] == '+' || p[i] == '-') {
    neg = (p[i] == '-');
    ++i;
  }
  if (i >= n) return false;
  // hex?
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
  // decimal
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
// Handlers (build response payloads starting with status)
static int32_t handleHELLO(uint8_t* out, size_t cap, size_t& off) {
  int32_t status = CoProc::ST_OK;
  int32_t version = (int32_t)CoProc::VERSION;
  uint32_t flags = 0;
  if (g_blob_len) flags |= 1u;
  if (g_job.active) flags |= 2u;
  if (BLOB_MAILBOX[0]) flags |= 4u;
  int32_t features = (int32_t)flags;
#if COPROC_DEBUG
  DBG("[DBG] HELLO -> ver=%d features=0x%08X\n", version, (unsigned)features);
#endif
  CoProc::writePOD(out, cap, off, status);
  CoProc::writePOD(out, cap, off, version);
  CoProc::writePOD(out, cap, off, features);
  return status;
}
static int32_t handleINFO(uint8_t* out, size_t cap, size_t& off) {
  CoProc::Info info{};
  info.impl_flags = 0;
  if (g_blob_len) info.impl_flags |= 1u;
  if (g_job.active) info.impl_flags |= 2u;
  if (BLOB_MAILBOX[0]) info.impl_flags |= 4u;
  info.blob_len = g_blob_len;
  info.mailbox_max = BLOB_MAILBOX_MAX;
#if COPROC_DEBUG
  DBG("[DBG] INFO flags=0x%08X blob_len=%u mbox=%u\n", (unsigned)info.impl_flags, (unsigned)info.blob_len, (unsigned)info.mailbox_max);
#endif
  int32_t status = CoProc::ST_OK;
  CoProc::writePOD(out, cap, off, status);
  CoProc::writePOD(out, cap, off, info);
  return status;
}
static int32_t handleLOAD_BEGIN(const uint8_t* in, size_t len) {
  size_t off = 0;
  uint32_t total = 0;
  if (!CoProc::readPOD(in, len, off, total)) return CoProc::ST_PARAM;
  if (total == 0 || (total & 1u)) return CoProc::ST_SIZE;
  freeBlob();
  g_blob = (uint8_t*)malloc(total);
  if (!g_blob) return CoProc::ST_NOMEM;
  g_blob_len = 0;
  g_blob_cap = total;
  g_blob_crc = 0;
  g_exec_state = CoProc::EXEC_LOADED;
#if COPROC_DEBUG
  DBG("[DBG] LOAD_BEGIN total=%u\n", (unsigned)total);
#endif
  return CoProc::ST_OK;
}
static int32_t handleLOAD_DATA(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  if (!g_blob || g_blob_len >= g_blob_cap) return CoProc::ST_STATE;
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
#if COPROC_DEBUG
  DBG("[DBG] LOAD_DATA len=%u take=%u total=%u\n", (unsigned)len, (unsigned)take, (unsigned)g_blob_len);
#endif
  return status;
}
static int32_t handleLOAD_END(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  size_t p = 0;
  uint32_t expected = 0;
  if (!CoProc::readPOD(in, len, p, expected)) return CoProc::ST_PARAM;
  int32_t status = (expected == g_blob_crc) ? CoProc::ST_OK : CoProc::ST_CRC;
  CoProc::writePOD(out, cap, off, status);
  CoProc::writePOD(out, cap, off, g_blob_len);
#if COPROC_DEBUG
  DBG("[DBG] LOAD_END crc_exp=0x%08X crc_have=0x%08X st=%d\n", (unsigned)expected, (unsigned)g_blob_crc, (int)status);
#endif
  return status;
}
static int32_t handleEXEC(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  if (!g_blob || (g_blob_len & 1u)) return CoProc::ST_STATE;
  if (g_job.active) return CoProc::ST_STATE;
  size_t p = 0;
  uint32_t argc = 0, timeout_ms = 0;
  if (!CoProc::readPOD(in, len, p, argc)) return CoProc::ST_PARAM;
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  for (uint32_t i = 0; i < argc; ++i) {
    int32_t v = 0;
    if (!CoProc::readPOD(in, len, p, v)) return CoProc::ST_PARAM;
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
#if COPROC_DEBUG
  DBG("[DBG] EXEC start argc=%u timeout=%u\n", (unsigned)argc, (unsigned)timeout_ms);
#endif
  uint32_t t0 = millis();
  while (g_job.active) {
    if (timeout_ms && (millis() - t0) > timeout_ms) {
      mailboxSetCancel(1);
      CoProc::writePOD(out, cap, off, (int32_t)CoProc::ST_TIMEOUT);
      CoProc::writePOD(out, cap, off, (int32_t)0);
      g_exec_state = CoProc::EXEC_DONE;
#if COPROC_DEBUG
      DBG("[DBG] EXEC timeout\n");
#endif
      return CoProc::ST_TIMEOUT;
    }
    tight_loop_contents();
  }
  mailboxSetCancel(0);
  g_exec_state = CoProc::EXEC_DONE;
  CoProc::writePOD(out, cap, off, (int32_t)g_job.status);
  CoProc::writePOD(out, cap, off, (int32_t)g_job.result);
#if COPROC_DEBUG
  DBG("[DBG] EXEC reply status=%d result=%d\n", (int)g_job.status, (int)g_job.result);
#endif
  return g_job.status;
}
static int32_t handleSTATUS(uint8_t* out, size_t cap, size_t& off) {
  int32_t status = CoProc::ST_OK;
  uint32_t es = g_exec_state;
  CoProc::writePOD(out, cap, off, status);
  CoProc::writePOD(out, cap, off, es);
#if COPROC_DEBUG
  DBG("[DBG] STATUS es=%u\n", (unsigned)es);
#endif
  return status;
}
static int32_t handleMAILBOX_RD(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  size_t p = 0;
  uint32_t maxb = 0;
  if (!CoProc::readPOD(in, len, p, maxb)) return CoProc::ST_PARAM;
  if (maxb > BLOB_MAILBOX_MAX) maxb = BLOB_MAILBOX_MAX;
  uint32_t n = 0;
  for (; n < maxb; ++n)
    if (BLOB_MAILBOX[n] == 0) break;
  CoProc::writePOD(out, cap, off, (int32_t)CoProc::ST_OK);
  CoProc::writePOD(out, cap, off, n);
  CoProc::writeBytes(out, cap, off, BLOB_MAILBOX, n);
#if COPROC_DEBUG
  DBG("[DBG] MAILBOX_RD n=%u\n", (unsigned)n);
#endif
  return CoProc::ST_OK;
}
static int32_t handleCANCEL(uint8_t* out, size_t cap, size_t& off) {
  mailboxSetCancel(1);
  int32_t st = CoProc::ST_OK;
  CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
  DBG("[DBG] CANCEL set\n");
#endif
  return st;
}
static int32_t handleRESET() {
#if COPROC_DEBUG
  DBG("[DBG] RESET\n");
#endif
  delay(10);
  rp2040.reboot();
  return CoProc::ST_OK;
}
// ------- Script handlers -------
static int32_t handleSCR_LOAD_BEGIN(const uint8_t* in, size_t len) {
  size_t off = 0;
  uint32_t total = 0;
  if (!CoProc::readPOD(in, len, off, total)) return CoProc::ST_PARAM;
  if (total == 0) return CoProc::ST_SIZE;
  freeScript();
  g_script = (uint8_t*)malloc(total + 1);  // +1 for safety NUL
  if (!g_script) return CoProc::ST_NOMEM;
  g_script_len = 0;
  g_script_cap = total;
  g_script_crc = 0;
#if COPROC_DEBUG
  DBG("[DBG] SCRIPT_BEGIN total=%u\n", (unsigned)total);
#endif
  return CoProc::ST_OK;
}
static int32_t handleSCR_LOAD_DATA(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  if (!g_script || g_script_len >= g_script_cap) return CoProc::ST_STATE;
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
#if COPROC_DEBUG
  DBG("[DBG] SCRIPT_DATA len=%u take=%u total=%u\n", (unsigned)len, (unsigned)take, (unsigned)g_script_len);
#endif
  return status;
}
static int32_t handleSCR_LOAD_END(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  size_t p = 0;
  uint32_t expected = 0;
  if (!CoProc::readPOD(in, len, p, expected)) return CoProc::ST_PARAM;
  int32_t status = (expected == g_script_crc) ? CoProc::ST_OK : CoProc::ST_CRC;
  // Ensure NUL-termination for script buffer (safety for in-place parser)
  if (g_script) {
    uint32_t nulpos = (g_script_len <= g_script_cap) ? g_script_len : g_script_cap;
    g_script[nulpos] = 0;
  }
  CoProc::writePOD(out, cap, off, status);
  CoProc::writePOD(out, cap, off, g_script_len);
#if COPROC_DEBUG
  DBG("[DBG] SCRIPT_END crc_exp=0x%08X crc_have=0x%08X st=%d\n", (unsigned)expected, (unsigned)g_script_crc, (int)status);
#endif
  return status;
}
static int32_t handleSCR_EXEC(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  if (!g_script || g_script_len == 0) return CoProc::ST_STATE;
  size_t p = 0;
  uint32_t argc = 0, timeout_ms = 0;
  if (!CoProc::readPOD(in, len, p, argc)) return CoProc::ST_PARAM;
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;

  int32_t argv[MAX_EXEC_ARGS] = { 0 };

  // Heuristic/mode detection:
  // - If the next uint32 is 0xFFFFFFFF, interpret as "alternate" mode:
  //     uint32 marker = 0xFFFFFFFF;
  //     for each arg: uint32 len; bytes[len] (ASCII string for numeric token)
  //     uint32 timeout_ms;
  // - Otherwise, fallback to classic mode: int32 argv[argc] followed by uint32 timeout_ms.
  if (argc == 0) {
    // No args: just read timeout and run
    if (!CoProc::readPOD(in, len, p, timeout_ms)) return CoProc::ST_PARAM;
  } else {
    // peek next 4 bytes if available
    if (p + sizeof(uint32_t) <= len) {
      uint32_t marker = 0;
      // safe copy
      memcpy(&marker, in + p, sizeof(marker));
      if (marker == 0xFFFFFFFFu) {
        // Alternate encoding
        p += sizeof(uint32_t);  // consume marker
#if COPROC_DEBUG
        DBG("[DBG] SCRIPT_EXEC: alternate ASCII-args mode detected argc=%u\n", (unsigned)argc);
#endif
        for (uint32_t i = 0; i < argc; ++i) {
          uint32_t slen = 0;
          if (!CoProc::readPOD(in, len, p, slen)) return CoProc::ST_PARAM;
          if (slen == 0 || p + slen > len) return CoProc::ST_PARAM;
          // safety limit: prevent huge allocations; use stack buffer up to 1024 bytes
          if (slen > 1024) return CoProc::ST_PARAM;
          uint8_t tmp[1025];
          memcpy(tmp, in + p, slen);
          p += slen;
          int32_t parsed = 0;
          if (!parseAsciiInt(tmp, slen, parsed)) return CoProc::ST_PARAM;
          argv[i] = parsed;
#if COPROC_DEBUG
          DBG("[DBG] SCRIPT_EXEC arg[%u] ascii='%.*s' -> %d (0x%08X)\n", (unsigned)i, (int)slen, (const char*)tmp, (int)parsed, (unsigned)parsed);
#endif
        }
        if (!CoProc::readPOD(in, len, p, timeout_ms)) return CoProc::ST_PARAM;
      } else {
        // Classic encoding: int32 per arg
#if COPROC_DEBUG
        DBG("[DBG] SCRIPT_EXEC: classic int32-args mode argc=%u\n", (unsigned)argc);
#endif
        for (uint32_t i = 0; i < argc; ++i) {
          int32_t v = 0;
          if (!CoProc::readPOD(in, len, p, v)) return CoProc::ST_PARAM;
          argv[i] = v;
        }
        if (!CoProc::readPOD(in, len, p, timeout_ms)) return CoProc::ST_PARAM;
      }
    } else {
      // Not enough bytes to peek marker -> error
      return CoProc::ST_PARAM;
    }
  }

  // Lazily allocate VM once; avoid stack usage and global constructors
  static CoProcLang::VM* s_vm = nullptr;
  if (!s_vm) {
    s_vm = new CoProcLang::VM();
    if (!s_vm) {
      CoProc::writePOD(out, cap, off, (int32_t)CoProc::ST_NOMEM);
      CoProc::writePOD(out, cap, off, (int32_t)0);
      return CoProc::ST_NOMEM;
    }
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
// Command names (debug)
static const char* cmdName(uint16_t c) {
  switch (c) {
    case CoProc::CMD_HELLO: return "HELLO";
    case CoProc::CMD_INFO: return "INFO";
    case CoProc::CMD_LOAD_BEGIN: return "LOAD_BEGIN";
    case CoProc::CMD_LOAD_DATA: return "LOAD_DATA";
    case CoProc::CMD_LOAD_END: return "LOAD_END";
    case CoProc::CMD_EXEC: return "EXEC";
    case CoProc::CMD_STATUS: return "STATUS";
    case CoProc::CMD_MAILBOX_RD: return "MAILBOX_RD";
    case CoProc::CMD_CANCEL: return "CANCEL";
    case CoProc::CMD_RESET: return "RESET";
    case CoProc::CMD_SCRIPT_BEGIN: return "SCRIPT_BEGIN";
    case CoProc::CMD_SCRIPT_DATA: return "SCRIPT_DATA";
    case CoProc::CMD_SCRIPT_END: return "SCRIPT_END";
    case CoProc::CMD_SCRIPT_EXEC: return "SCRIPT_EXEC";
  }
  return "UNKNOWN";
}
// Build response from request
static void processRequest(const CoProc::Frame& hdr, const uint8_t* payload,
                           CoProc::Frame& respH, uint8_t* respBuf, uint32_t& respLen) {
  size_t off = 0;
  int32_t st = CoProc::ST_BAD_CMD;
  respLen = 0;
#if COPROC_DEBUG
  DBG("[DBG] CMD=%s (0x%02X) seq=%u len=%u\n", cmdName(hdr.cmd), (unsigned)hdr.cmd, (unsigned)hdr.seq, (unsigned)hdr.len);
#endif
  switch (hdr.cmd) {
    // Binary pipeline
    case CoProc::CMD_HELLO: st = handleHELLO(respBuf, RESP_MAX, off); break;
    case CoProc::CMD_INFO: st = handleINFO(respBuf, RESP_MAX, off); break;
    case CoProc::CMD_LOAD_BEGIN:
      st = handleLOAD_BEGIN(payload, hdr.len);
      CoProc::writePOD(respBuf, RESP_MAX, off, st);
      break;
    case CoProc::CMD_LOAD_DATA: st = handleLOAD_DATA(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_LOAD_END: st = handleLOAD_END(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_EXEC: st = handleEXEC(payload, hdr.len, respBuf, RESP_MAX, off); break;
    // Housekeeping
    case CoProc::CMD_STATUS: st = handleSTATUS(respBuf, RESP_MAX, off); break;
    case CoProc::CMD_MAILBOX_RD: st = handleMAILBOX_RD(payload, hdr.len, respBuf, RESP_MAX, off); break;
    case CoProc::CMD_CANCEL: st = handleCANCEL(respBuf, RESP_MAX, off); break;
    case CoProc::CMD_RESET: st = handleRESET(); break;
    // Script pipeline
    case CoProc::CMD_SCRIPT_BEGIN:
      st = handleSCR_LOAD_BEGIN(payload, hdr.len);
      CoProc::writePOD(respBuf, RESP_MAX, off, st);
      break;
    case CoProc::CMD_SCRIPT_DATA:
      st = handleSCR_LOAD_DATA(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_SCRIPT_END:
      st = handleSCR_LOAD_END(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_SCRIPT_EXEC:
      st = handleSCR_EXEC(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    default:
      st = CoProc::ST_BAD_CMD;
      CoProc::writePOD(respBuf, RESP_MAX, off, st);
      break;
  }
  respLen = (uint32_t)off;
  uint32_t crc = (respLen ? CoProc::crc32_ieee(respBuf, respLen) : 0);
  CoProc::makeResponseHeader(respH, hdr.cmd, hdr.seq, respLen, crc);
#if COPROC_DEBUG
  DBG("[DBG] RESP len=%u crc=0x%08X\n", (unsigned)respLen, (unsigned)crc);
#endif
}
// ------- Serial protocol helpers -------
static bool readFramedRequest(CoProc::Frame& hdr, uint8_t* payloadBuf) {
  // Sliding-window scan for MAGIC (little-endian bytes: 'C','P','R','0')
  const uint8_t magicBytes[4] = {
    (uint8_t)('C'), (uint8_t)('P'), (uint8_t)('R'), (uint8_t)('0')
  };
  uint8_t w[4] = { 0, 0, 0, 0 };
  uint32_t lastActivity = millis();
  // Scan indefinitely until MAGIC+VERSION found (yields to keep system responsive)
  for (;;) {
    // Try to read one byte (with some idle timeout so we can yield)
    uint8_t b;
    if (readByte(b, 50)) {
      w[0] = w[1];
      w[1] = w[2];
      w[2] = w[3];
      w[3] = b;
      lastActivity = millis();
      if (w[0] == magicBytes[0] && w[1] == magicBytes[1] && w[2] == magicBytes[2] && w[3] == magicBytes[3]) {
        // Found magic. Read rest of header.
        union {
          CoProc::Frame f;
          uint8_t bytes[sizeof(CoProc::Frame)];
        } u;
        memcpy(u.bytes, w, 4);
        // Read remaining header bytes
        if (!readExact(u.bytes + 4, sizeof(CoProc::Frame) - 4, 200)) {
          DBG("[COPROC] header tail timeout\n");
          return false;
        }
        hdr = u.f;
        // Validate version/magic fields
        if ((hdr.magic != CoProc::MAGIC) || (hdr.version != CoProc::VERSION)) {
          DBG("[COPROC] bad header magic=0x%08X ver=0x%04X\n", (unsigned)hdr.magic, (unsigned)hdr.version);
          return false;
        }
        // Validate payload length
        if (hdr.len > REQ_MAX) {
          DBG("[COPROC] req too large (%u > %u), draining\n", (unsigned)hdr.len, (unsigned)REQ_MAX);
          // Drain the advertised payload to realign stream, but respond with ST_SIZE after.
          uint8_t sink[64];
          uint32_t left = hdr.len;
          while (left) {
            uint32_t chunk = (left > sizeof(sink)) ? sizeof(sink) : left;
            if (!readExact(sink, chunk, 200)) {
              DBG("[COPROC] drain timeout\n");
              return false;
            }
            left -= chunk;
          }
          // Build and send size error immediately
          size_t off = 0;
          int32_t st = CoProc::ST_SIZE;
          CoProc::writePOD(g_respBuf, RESP_MAX, off, st);
          uint32_t crc = CoProc::crc32_ieee(g_respBuf, off);
          CoProc::makeResponseHeader(g_respHdr, hdr.cmd, hdr.seq, (uint32_t)off, crc);
          // Send response
          writeAll(reinterpret_cast<const uint8_t*>(&g_respHdr), sizeof(CoProc::Frame), 2000);
          if (off) writeAll(g_respBuf, off, 5000);
          return false;
        }
        // Read payload (if any)
        if (hdr.len) {
          if (!readExact(payloadBuf, hdr.len, 200)) {
            DBG("[COPROC] payload read timeout\n");
            return false;
          }
          // CRC check
          uint32_t crc = CoProc::crc32_ieee(payloadBuf, hdr.len);
          if (crc != hdr.crc32) {
            DBG("[COPROC] CRC mismatch exp=0x%08X got=0x%08X\n", (unsigned)hdr.crc32, (unsigned)crc);
            // Send ST_CRC error response
            size_t off = 0;
            int32_t st = CoProc::ST_CRC;
            CoProc::writePOD(g_respBuf, RESP_MAX, off, st);
            uint32_t rcrc = CoProc::crc32_ieee(g_respBuf, off);
            CoProc::makeResponseHeader(g_respHdr, hdr.cmd, hdr.seq, (uint32_t)off, rcrc);
            writeAll(reinterpret_cast<const uint8_t*>(&g_respHdr), sizeof(CoProc::Frame), 2000);
            if (off) writeAll(g_respBuf, off, 5000);
            return false;
          }
        }
        // Completed a good frame
        return true;
      }
    } else {
      // idle; opportunistically run worker-friendly hints
      if ((millis() - lastActivity) > 1000) {
        lastActivity = millis();
      }
      tight_loop_contents();
      yield();
    }
  }
}
// ------- Core0: serial protocol loop -------
static void protocolLoop() {
  for (;;) {
    // Receive full request frame
    if (!readFramedRequest(g_reqHdr, g_reqBuf)) {
      // Failed to parse a full valid frame; continue scanning
      continue;
    }
    // Build response
    uint32_t respLen = 0;
    processRequest(g_reqHdr, g_reqBuf, g_respHdr, g_respBuf, respLen);
    // Send response: header then payload
    if (!writeAll(reinterpret_cast<const uint8_t*>(&g_respHdr), sizeof(CoProc::Frame), 2000)) {
      DBG("[COPROC] write header failed\n");
      continue;
    }
    if (respLen) {
      if (!writeAll(g_respBuf, respLen, 10000)) {
        DBG("[COPROC] write payload failed\n");
        continue;
      }
    }
    if (BOOTSEL) {
      Serial.println("Rebooting now..");
      delay(1500);
      yield();
      watchdog_reboot(0, 0, 1500);
      while (true) tight_loop_contents();
    }
  }
}
// ------- rp_selfupdate -------
static const uint8_t PIN_MISO = 12;   // pin from RP to PSRAM pin-5
static const uint8_t PIN_MOSI = 11;   // pin from RP to PSRAM pin-2
static const uint8_t PIN_SCK = 10;    // pin from RP to PSRAM pin-6
static const uint8_t PIN_DEC_EN = 9;  // 74HC138 G2A/G2B tied together via transistor or directly; active LOW
static const uint8_t PIN_DEC_A0 = 8;  // 74HC138 address bit 0
static const uint8_t PIN_DEC_A1 = 7;  // 74HC138 address bit 1
static const uint8_t BANKS = 4;
static const uint32_t PER_CHIP_BYTES = 8u * 1024u * 1024u;
PSRAMAggregateDevice psram(PIN_MISO, PIN_MOSI, PIN_SCK, PER_CHIP_BYTES);
PSRAMSimpleFS_Multi fs(psram, /*capacityBytes=*/PER_CHIP_BYTES* BANKS);
const uint8_t PIN_UPDATE = 22;
volatile bool doUpdate = false;
void onTrig() {
  doUpdate = true;
}
// ========== Setup and main ==========
void setup() {
  delay(500);
  Serial.begin();
  while (!Serial) { delay(500); }
  delay(500);

  // Configure decoder selection for 4 banks via 74HC138
  /*psram.configureDecoder138(BANKS, PIN_DEC_EN, PIN_DEC_A0, PIN_DEC_A1);
  psram.begin();
  // Mount and auto-format if empty (or call fs.format()/fs.wipeChip() explicitly)
  fs.mount(true);
  pinMode(PIN_UPDATE, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_UPDATE), onTrig, RISING);
  Serial.println("Ready. Put update.bin into PSRAM FS, then pull GP22 HIGH to flash.");*/
  // Example: to fully wipe PSRAM FS area for staging:
  // rpupdfs_wipe_chip(fs);

  Serial.println("CoProc (soft-serial) booting...");

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

  // Allocate protocol buffers
  g_reqBuf = (uint8_t*)malloc(REQ_MAX);
  g_respBuf = (uint8_t*)malloc(RESP_MAX);
  // Initialize software serial link
  link.begin(SOFT_BAUD);
  link.listen();  // ensure active receiver
  // initialize executor
  memset(&g_job, 0, sizeof(g_job));
  memset(BLOB_MAILBOX, 0, BLOB_MAILBOX_MAX);
  g_cancel_flag = 0;
  g_exec_state = CoProc::EXEC_IDLE;
  Serial.printf("CoProc ready (soft-serial %u bps). RX=GP%u TX=GP%u\n", (unsigned)SOFT_BAUD, (unsigned)PIN_RX, (unsigned)PIN_TX);
}
void loop() {
  protocolLoop();  // never returns
}
// setup1/loop1 for core1 (Philhower)
void setup1() { /* nothing */
}
void loop1() {
  core1WorkerPoll();
  tight_loop_contents();
}