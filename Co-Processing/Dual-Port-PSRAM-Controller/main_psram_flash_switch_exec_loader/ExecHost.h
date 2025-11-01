// ExecHost.h
#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>
#include "ConsolePrint.h"
#include "CoProcProto.h"
#include "blob_mailbox_config.h"
#ifndef MAX_EXEC_ARGS
#define MAX_EXEC_ARGS 64
#endif
// Simple FS function pointer table injected by the sketch from activeFs binding.
struct ExecFSTable {
  // The subset needed for exec/coprocessor operations
  bool (*exists)(const char*) = nullptr;
  bool (*getFileSize)(const char*, uint32_t&) = nullptr;
  uint32_t (*readFile)(const char*, uint8_t*, uint32_t) = nullptr;
  uint32_t (*readFileRange)(const char*, uint32_t, uint8_t*, uint32_t) = nullptr;
  bool (*createFileSlot)(const char*, uint32_t, const uint8_t*, uint32_t) = nullptr;
  bool (*writeFile)(const char*, const uint8_t*, uint32_t, int) = nullptr;
  bool (*writeFileInPlace)(const char*, const uint8_t*, uint32_t, bool) = nullptr;
  bool (*getFileInfo)(const char*, uint32_t&, uint32_t&, uint32_t&) = nullptr;
};
// Thumb-call helper; mirrors prior behavior.
static inline int exechost_call_with_args_thumb(void* entryThumb, uint32_t argc, const int32_t* args) {
  if (!entryThumb) return 0;
  constexpr uint32_t N = MAX_EXEC_ARGS;
  int32_t a64[N] = { 0 };
  uint32_t n = argc;
  if (n > N) n = N;
  if (args && n) {
    for (uint32_t i = 0; i < n; ++i) a64[i] = args[i];
  }
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
class ExecHost {
public:
  ExecHost()
    : _console(nullptr), _fs{}, _fsValid(false), _link(nullptr), _coproc_seq(1),
      _timeout_override_ms(0), _bg_raw(nullptr), _bg_buf(nullptr), _bg_sz(0),
      _bg_active(false), _bg_start_ms(0), _bg_timeout_ms(0), _bg_cancel_requested(false) {
    _bg_fname[0] = '\0';
    _job.code = 0;
    _job.size = 0;
    _job.argc = 0;
    _result = 0;
    _status = 0;
    _job_flag = 0u;
  }
  // Console to be used for prints
  void attachConsole(ConsolePrint* c) {
    _console = c;
  }
  // Provide FS function table (call after bindActiveFs or when backend changes)
  void attachFS(const ExecFSTable& t) {
    _fs = t;
    _fsValid = (_fs.getFileSize && _fs.readFile);
  }
  // Attach and initialize co-processor link (SoftwareSerial)
  void attachCoProc(SoftwareSerial* link, uint32_t baud) {
    _link = link;
    if (_link) {
      _link->begin(baud);
      _link->listen();
    }
  }
  // Timeout override management
  void setTimeoutOverride(uint32_t ms) {
    _timeout_override_ms = ms;
  }
  uint32_t getTimeoutOverride() const {
    return _timeout_override_ms;
  }
  uint32_t timeout(uint32_t defaultMs) const {
    return _timeout_override_ms ? _timeout_override_ms : defaultMs;
  }
  // Core1 worker (call from setup1/loop1)
  void core1Setup() {
    _job_flag = 0u;
  }
  void core1Poll() {
    if (_job_flag != 1u) return;
    __asm volatile("dsb" ::
                     : "memory");
    __asm volatile("isb" ::
                     : "memory");
    _job_flag = 2u;
    ExecJob local{};
    local.code = _job.code;
    local.size = _job.size;
    local.argc = _job.argc;
    if (local.argc > MAX_EXEC_ARGS) local.argc = MAX_EXEC_ARGS;
    for (uint32_t i = 0; i < local.argc; ++i) local.args[i] = _job.args[i];
    int32_t rv = 0;
    int32_t st = 0;
    if (local.code == 0 || (local.size & 1u)) {
      st = -1;
    } else {
      void* entryThumb = (void*)(local.code | 1u);  // set Thumb bit
      rv = exechost_call_with_args_thumb(entryThumb, local.argc, local.args);
    }
    _result = rv;
    _status = st;
    __asm volatile("dsb" ::
                     : "memory");
    __asm volatile("isb" ::
                     : "memory");
    _job_flag = 3u;
  }
  // Mailbox helpers (shared with blobs)
  void mailboxClearFirstByte() {
    volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
    mb[0] = 0;
  }
  void mailboxPrintIfAny() {
    if (!_console) return;
    const volatile char* p = (const volatile char*)(uintptr_t)BLOB_MAILBOX_ADDR;
    if (p[0] == '\0') return;
    _console->print(" Info=\"");
    for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) {
      char c = p[i];
      if (!c) break;
      _console->print(c);
    }
    _console->println("\"");
  }
  void mailboxSetCancelFlag(uint8_t v) {
    volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
    mb[0] = v;
  }
  void mailboxClearCancelFlag() {
    volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
    mb[0] = 0;
  }
  uint8_t mailboxGetCancelFlag() {
    volatile uint8_t* mb = (volatile uint8_t*)(uintptr_t)BLOB_MAILBOX_ADDR;
    return mb[0];
  }
  // Load a file into an aligned exec buffer (malloc rawOut, returns aligned pointer)
  bool loadFileToExecBuf(const char* fname, void*& rawOut, uint8_t*& alignedBuf, uint32_t& szOut) {
    rawOut = nullptr;
    alignedBuf = nullptr;
    szOut = 0;
    if (!_fsValid || !_fs.getFileSize || !_fs.readFile) {
      if (_console) _console->println("load: FS not attached");
      return false;
    }
    uint32_t sz = 0;
    if (!_fs.getFileSize(fname, sz) || sz == 0) {
      if (_console) _console->println("load: missing/empty");
      return false;
    }
    if (sz & 1u) {
      if (_console) _console->println("load: odd-sized blob (Thumb requires 16-bit alignment)");
      return false;
    }
    void* raw = malloc(sz + 4);
    if (!raw) {
      if (_console) _console->println("load: malloc failed");
      return false;
    }
    uint8_t* buf = (uint8_t*)((((uintptr_t)raw) + 3) & ~((uintptr_t)3));
    if (_fs.readFile(fname, buf, sz) != sz) {
      if (_console) _console->println("load: read failed");
      free(raw);
      return false;
    }
    rawOut = raw;
    alignedBuf = buf;
    szOut = sz;
    return true;
  }
  // Foreground exec: load from FS and run on core1; prints return + mailbox
  bool execBlobForeground(const char* fname, int argc, const int32_t* argv, int& retVal) {
    if (argc < 0) argc = 0;
    if (argc > (int)MAX_EXEC_ARGS) argc = (int)MAX_EXEC_ARGS;
    void* raw = nullptr;
    uint8_t* buf = nullptr;
    uint32_t sz = 0;
    if (!loadFileToExecBuf(fname, raw, buf, sz)) return false;
    mailboxClearFirstByte();
    uintptr_t code = (uintptr_t)buf;
    if (_console) {
      _console->print("Calling entry on core1 at 0x");
      _console->println((uintptr_t)buf, HEX);
    }
    bool ok = runOnCore1(code, sz, (uint32_t)argc, argv, retVal, timeout(100000));
    if (!ok) {
      if (_console) _console->println("exec: core1 run failed");
      free(raw);
      return false;
    }
    if (_console) {
      _console->print("Return=");
      _console->println(retVal);
    }
    mailboxPrintIfAny();
    free(raw);
    return true;
  }
  // Background job submission; caller must have loaded raw/buf/sz via loadFileToExecBuf
  bool submitBackground(const char* fname, void* raw, uint8_t* alignedBuf, uint32_t sz,
                        uint32_t argc, const int32_t* argv, uint32_t timeoutMs) {
    if (!fname || !raw || !alignedBuf || sz == 0) return false;
    if (_job_flag != 0u) {
      if (_console) _console->println("core1 busy");
      return false;
    }
    _bg_cancel_requested = false;
    _bg_raw = raw;
    _bg_buf = alignedBuf;
    _bg_sz = sz;
    _bg_active = true;
    _bg_start_ms = millis();
    _bg_timeout_ms = timeoutMs;
    strncpy(_bg_fname, fname, sizeof(_bg_fname) - 1);
    _bg_fname[sizeof(_bg_fname) - 1] = '\0';
    _job.code = (uintptr_t)alignedBuf;
    _job.size = sz;
    _job.argc = (argc > MAX_EXEC_ARGS) ? MAX_EXEC_ARGS : argc;
    for (uint32_t i = 0; i < (uint32_t)_job.argc; ++i) _job.args[i] = (int32_t)argv[i];
    mailboxClearCancelFlag();
    __asm volatile("dsb" ::
                     : "memory");
    __asm volatile("isb" ::
                     : "memory");
    _job_flag = 1u;
    if (_console) {
      _console->printf("Background job '%s' started on core1 at 0x%08X (sz=%u)\n",
                       _bg_fname, (unsigned)_job.code, (unsigned)_job.size);
    }
    return true;
  }
  // Poll background exec completion/timeout; call each main loop
  void pollBackground() {
    if (!_bg_active) {
      if (_job_flag == 3u) {
        __asm volatile("dsb" ::
                         : "memory");
        __asm volatile("isb" ::
                         : "memory");
        _job_flag = 0u;
      }
      return;
    }
    if (_job_flag == 3u) {
      int32_t result = (int32_t)_result;
      int32_t status = (int32_t)_status;
      if (_bg_cancel_requested) {
        if (_console) _console->printf("Background job '%s' finished on core1 but was canceled — result ignored\n", _bg_fname);
      } else {
        if (_console) {
          _console->printf("Background job '%s' completed: Return=%d\n", _bg_fname, (int)result);
          if (status != 0) _console->printf(" Background job status=%d\n", (int)status);
        }
        mailboxPrintIfAny();
      }
      mailboxClearCancelFlag();
      if (_bg_raw) free((void*)_bg_raw);
      _bg_raw = nullptr;
      _bg_buf = nullptr;
      _bg_sz = 0;
      _bg_active = false;
      _bg_start_ms = 0;
      _bg_timeout_ms = 0;
      _bg_fname[0] = '\0';
      _bg_cancel_requested = false;
      __asm volatile("dsb" ::
                       : "memory");
      __asm volatile("isb" ::
                       : "memory");
      _job_flag = 0u;
      return;
    }
    uint32_t timeoutMs = _bg_timeout_ms ? _bg_timeout_ms : 0;
    if (_bg_active && timeoutMs != 0) {
      uint32_t elapsed = (uint32_t)(millis() - _bg_start_ms);
      if (elapsed > timeoutMs) {
        if (_console) _console->println("core1 timeout (background job)");
        _bg_cancel_requested = true;
        mailboxSetCancelFlag(1);
        const uint32_t graceMs = 50;
        uint32_t waited = 0;
        while (waited < graceMs) {
          if (_job_flag == 3u) {
            return;
          }
          delay(1);
          ++waited;
        }
        if (_bg_raw) free((void*)_bg_raw);
        _bg_raw = nullptr;
        _bg_buf = nullptr;
        _bg_sz = 0;
        _bg_active = false;
        _bg_start_ms = 0;
        _bg_timeout_ms = 0;
        _bg_fname[0] = '\0';
        __asm volatile("dsb" ::
                         : "memory");
        __asm volatile("isb" ::
                         : "memory");
        _job_flag = 0u;
        return;
      }
    }
  }
  // Query background status (for console)
  bool bgActive() const {
    return _bg_active;
  }
  const char* bgName() const {
    return _bg_fname;
  }
  uint32_t bgElapsedMs() const {
    return _bg_active ? (millis() - _bg_start_ms) : 0;
  }
  uint32_t bgTimeoutMs() const {
    return _bg_timeout_ms;
  }
  uint32_t core1JobFlag() const {
    return _job_flag;
  }
  // Cancel helpers used by console
  bool cancelQueuedBackgroundIfAny() {
    if (_job_flag == 1u) {
      __asm volatile("dsb" ::
                       : "memory");
      __asm volatile("isb" ::
                       : "memory");
      _job_flag = 0u;
      void* raw = (void*)_bg_raw;
      if (raw) free(raw);
      _bg_raw = nullptr;
      _bg_buf = nullptr;
      _bg_sz = 0;
      _bg_active = false;
      _bg_start_ms = 0;
      _bg_timeout_ms = 0;
      _bg_fname[0] = '\0';
      _bg_cancel_requested = false;
      return true;
    }
    return false;
  }
  bool requestCancelRunningBackground() {
    if (_job_flag == 2u) {
      _bg_cancel_requested = true;
      mailboxSetCancelFlag(1);
      return true;
    }
    return false;
  }
  // Low-level run when already in RAM/aligned
  bool runOnCore1(uintptr_t codeAligned, uint32_t sz, uint32_t argc, const int32_t* argv, int& retVal, uint32_t timeoutMs) {
    if (_job_flag != 0u) {
      if (_console) _console->println("core1 busy");
      return false;
    }
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    _job.code = codeAligned;
    _job.size = sz;
    _job.argc = argc;
    for (uint32_t i = 0; i < argc; ++i) _job.args[i] = argv[i];
    _status = 0;
    _result = 0;
    __asm volatile("dsb" ::
                     : "memory");
    __asm volatile("isb" ::
                     : "memory");
    _job_flag = 1u;
    uint32_t start = millis();
    while (_job_flag != 3u) {
      tight_loop_contents();
      if ((millis() - start) > timeoutMs) {
        if (_job_flag == 3u) break;
        if (_console) _console->println("core1 timeout");
        _job_flag = 0u;
        return false;
      }
    }
    if (_status != 0) {
      if (_console) {
        _console->print("core1 error status=");
        _console->println((int)_status);
      }
      _job_flag = 0u;
      return false;
    }
    retVal = (int)_result;
    _job_flag = 0u;
    return true;
  }
  // ---------------- Co-processor RPC (SoftwareSerial) ----------------
  bool coprocHello() {
    CoProc::Frame rh;
    uint8_t buf[16];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_HELLO, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
    if (st != CoProc::ST_OK || rl < 12) return false;
    int32_t version = 0, features = 0;
    memcpy(&version, buf + 4, 4);
    memcpy(&features, buf + 8, 4);
    if (_console) _console->printf("CoProc HELLO: version=%d features=0x%08X\n", version, (unsigned)features);
    return true;
  }
  bool coprocInfo() {
    CoProc::Frame rh;
    uint8_t buf[32];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_INFO, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
    if (st != CoProc::ST_OK || rl < (int32_t)(4 + sizeof(CoProc::Info))) return false;
    CoProc::Info inf{};
    memcpy(&inf, buf + 4, sizeof(inf));
    if (_console)
      _console->printf("CoProc INFO: flags=0x%08X blob_len=%u mailbox_max=%u\n",
                       (unsigned)inf.impl_flags, (unsigned)inf.blob_len, (unsigned)inf.mailbox_max);
    return true;
  }
  bool coprocLoadBuffer(const uint8_t* data, uint32_t len) {
    uint8_t b4[4];
    memcpy(b4, &len, 4);
    CoProc::Frame rh;
    uint8_t rbuf[8];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_LOAD_BEGIN, b4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK) {
      if (_console) _console->printf("LOAD_BEGIN failed st=%d\n", st);
      return false;
    }
    const uint32_t CHUNK = 512;
    uint32_t sent = 0;
    uint32_t rollingCrc = 0;
    while (sent < len) {
      uint32_t n = (len - sent > CHUNK) ? CHUNK : (len - sent);
      if (!coprocRequest(CoProc::CMD_LOAD_DATA, data + sent, n, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
      if (st != CoProc::ST_OK && st != CoProc::ST_SIZE) {
        if (_console) _console->printf("LOAD_DATA st=%d\n", st);
        return false;
      }
      rollingCrc = CoProc::crc32_ieee(data + sent, n, rollingCrc);
      sent += n;
    }
    uint8_t crc4[4];
    memcpy(crc4, &rollingCrc, 4);
    rl = 0;
    if (!coprocRequest(CoProc::CMD_LOAD_END, crc4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK) {
      if (_console) _console->printf("LOAD_END failed st=%d\n", st);
      return false;
    }
    if (_console) _console->printf("CoProc LOAD OK (%u bytes)\n", (unsigned)len);
    return true;
  }
  bool coprocLoadFile(const char* fname) {
    if (!_fsValid || !_fs.getFileSize || !_fs.readFileRange) {
      if (_console) _console->println("coproc: FS not attached");
      return false;
    }
    if (!_fs.exists || !_fs.exists(fname)) {
      if (_console) _console->println("coproc: file not found");
      return false;
    }
    uint32_t size = 0;
    if (!_fs.getFileSize(fname, size) || size == 0) {
      if (_console) _console->println("coproc: empty file");
      return false;
    }
    if (size & 1u) {
      if (_console) _console->println("coproc: odd-sized blob (Thumb needs even)");
      return false;
    }
    uint8_t b4[4];
    memcpy(b4, &size, 4);
    CoProc::Frame rh;
    uint8_t rbuf[8];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_LOAD_BEGIN, b4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK) {
      if (_console) _console->printf("LOAD_BEGIN failed st=%d\n", st);
      return false;
    }
    const uint32_t CHUNK = 512;
    uint8_t buf[CHUNK];
    uint32_t sent = 0, offset = 0;
    uint32_t rollingCrc = 0;
    while (offset < size) {
      uint32_t n = (size - offset > CHUNK) ? CHUNK : (size - offset);
      uint32_t got = _fs.readFileRange(fname, offset, buf, n);
      if (got != n) {
        if (_console) _console->println("coproc: read error");
        return false;
      }
      rl = 0;
      if (!coprocRequest(CoProc::CMD_LOAD_DATA, buf, n, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
      if (st != CoProc::ST_OK && st != CoProc::ST_SIZE) {
        if (_console) _console->printf("LOAD_DATA st=%d\n", st);
        return false;
      }
      rollingCrc = CoProc::crc32_ieee(buf, n, rollingCrc);
      offset += n;
      sent += n;
    }
    uint8_t crc4[4];
    memcpy(crc4, &rollingCrc, 4);
    rl = 0;
    if (!coprocRequest(CoProc::CMD_LOAD_END, crc4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK) {
      if (_console) _console->printf("LOAD_END failed st=%d\n", st);
      return false;
    }
    if (_console) _console->printf("CoProc LOAD OK (%u bytes)\n", (unsigned)size);
    return true;
  }
  bool coprocExec(const int32_t* argv, uint32_t argc) {
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    uint32_t timeoutMs = timeout(100000);
    uint8_t payload[4 + MAX_EXEC_ARGS * 4 + 4];
    size_t off = 0;
    memcpy(payload + off, &argc, 4);
    off += 4;
    for (uint32_t i = 0; i < argc; ++i) {
      memcpy(payload + off, &argv[i], 4);
      off += 4;
    }
    memcpy(payload + off, &timeoutMs, 4);
    off += 4;
    CoProc::Frame rh;
    uint8_t rbuf[16];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (rl < 8) {
      if (_console) _console->println("coproc: short EXEC response");
      return false;
    }
    int32_t retCode = 0;
    memcpy(&st, rbuf + 0, 4);
    memcpy(&retCode, rbuf + 4, 4);
    if (_console) _console->printf("CoProc EXEC: status=%d return=%d\n", (int)st, (int)retCode);
    return true;
  }
  bool coprocExecTokens(const char* tokens[], uint32_t argc) {
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    uint32_t timeoutMs = timeout(100000);
    bool needAscii = false;
    for (uint32_t i = 0; i < argc; ++i) {
      const char* s = tokens[i];
      if (!s || !*s) {
        needAscii = true;
        break;
      }
      if ((s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))) {
        needAscii = true;
        break;
      }
      for (const char* p = s; *p; ++p) {
        char c = *p;
        if ((c >= '0' && c <= '9') || c == '+' || c == '-') continue;
        needAscii = true;
        break;
      }
      if (needAscii) break;
    }
    if (!needAscii) {
      int32_t argv[MAX_EXEC_ARGS];
      for (uint32_t i = 0; i < argc; ++i) argv[i] = (int32_t)strtol(tokens[i], nullptr, 0);
      return coprocExec(argv, argc);
    }
    uint32_t total = 4 + 4;  // argc + marker
    for (uint32_t i = 0; i < argc; ++i) total += 4 + (uint32_t)strlen(tokens[i]);
    total += 4;  // timeout
    if (total > 8192) {
      if (_console) _console->println("coproc exec: args too large");
      return false;
    }
    uint8_t* payload = (uint8_t*)malloc(total);
    if (!payload) {
      if (_console) _console->println("coproc exec: malloc failed");
      return false;
    }
    size_t off = 0;
    memcpy(payload + off, &argc, 4);
    off += 4;
    uint32_t marker = 0xFFFFFFFFu;
    memcpy(payload + off, &marker, 4);
    off += 4;
    for (uint32_t i = 0; i < argc; ++i) {
      uint32_t sl = (uint32_t)strlen(tokens[i]);
      memcpy(payload + off, &sl, 4);
      off += 4;
      memcpy(payload + off, tokens[i], sl);
      off += sl;
    }
    memcpy(payload + off, &timeoutMs, 4);
    off += 4;
    CoProc::Frame rh;
    uint8_t rbuf[16];
    uint32_t rl = 0;
    int32_t st = 0;
    bool ok = coprocRequest(CoProc::CMD_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st);
    free(payload);
    if (!ok) return false;
    if (rl < 8) {
      if (_console) _console->println("coproc: short EXEC response");
      return false;
    }
    int32_t retCode = 0;
    memcpy(&st, rbuf + 0, 4);
    memcpy(&retCode, rbuf + 4, 4);
    if (_console) _console->printf("CoProc EXEC: status=%d return=%d\n", (int)st, (int)retCode);
    return true;
  }
  bool coprocExecFile(const char* fname, const int32_t* argv, uint32_t argc) {
    if (!fname || !*fname) {
      if (_console) _console->println("coproc exec: missing file name");
      return false;
    }
    if (!coprocLoadFile(fname)) {
      if (_console) _console->println("coproc exec: LOAD_* failed");
      return false;
    }
    if (!coprocExec(argv, argc)) {
      if (_console) _console->println("coproc exec: EXEC failed");
      return false;
    }
    return true;
  }
  bool coprocStatus() {
    CoProc::Frame rh;
    uint8_t buf[8];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_STATUS, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
    if (st != CoProc::ST_OK || rl < 8) return false;
    uint32_t es = 0;
    memcpy(&es, buf + 4, 4);
    if (_console) _console->printf("CoProc STATUS: exec_state=%u\n", (unsigned)es);
    return true;
  }
  bool coprocCancel() {
    CoProc::Frame rh;
    uint8_t buf[4];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_CANCEL, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
    if (_console) _console->printf("CoProc CANCEL: status=%d\n", (int)st);
    return true;
  }
  bool coprocMailboxRead(uint32_t maxBytes) {
    uint8_t in[4];
    memcpy(in, &maxBytes, 4);
    CoProc::Frame rh;
    uint8_t out[512];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_MAILBOX_RD, in, 4, rh, out, sizeof(out), rl, &st)) return false;
    if (st != CoProc::ST_OK || rl < 8) return false;
    uint32_t n = 0;
    memcpy(&n, out + 4, 4);
    if (_console) {
      _console->print("CoProc MAILBOX: ");
      for (uint32_t i = 0; i < n; ++i) _console->print((char)out[8 + i]);
      _console->println();
    }
    return true;
  }
  bool coprocReset() {
    CoProc::Frame rh;
    uint8_t buf[4];
    uint32_t rl = 0;
    int32_t st = 0;
    bool ok = coprocRequest(CoProc::CMD_RESET, nullptr, 0, rh, buf, sizeof(buf), rl, &st);
    if (_console) _console->println("CoProc RESET issued");
    return ok;
  }
  bool coprocIspEnter() {
    CoProc::Frame rh;
    uint8_t buf[8];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_ISP_ENTER, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
    if (_console) _console->printf("CoProc ISP_ENTER: status=%d\n", (int)st);
    return true;
  }
  bool coprocIspExit() {
    CoProc::Frame rh;
    uint8_t buf[8];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_ISP_EXIT, nullptr, 0, rh, buf, sizeof(buf), rl, &st)) return false;
    if (_console) _console->printf("CoProc ISP_EXIT: status=%d\n", (int)st);
    return true;
  }
  bool coprocScriptLoadFile(const char* fname) {
    if (!_fsValid || !_fs.getFileSize || !_fs.readFileRange) {
      if (_console) _console->println("coproc script: FS not attached");
      return false;
    }
    if (!_fs.exists || !_fs.exists(fname)) {
      if (_console) _console->println("coproc script: file not found");
      return false;
    }
    uint32_t size = 0;
    if (!_fs.getFileSize(fname, size) || size == 0) {
      if (_console) _console->println("coproc script: empty file");
      return false;
    }

    // SCRIPT_BEGIN
    uint8_t b4[4];
    memcpy(b4, &size, 4);
    CoProc::Frame rh;
    uint8_t rbuf[32];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_SCRIPT_BEGIN, b4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (st != CoProc::ST_OK) {
      if (_console) _console->printf("SCRIPT_BEGIN failed st=%d\n", st);
      return false;
    }

    // Stream DATA; we don't need to compute a host CRC anymore
    const uint32_t CHUNK = 512;
    uint8_t buf[CHUNK];
    uint32_t offset = 0;
    while (offset < size) {
      uint32_t n = (size - offset > CHUNK) ? CHUNK : (size - offset);
      uint32_t got = _fs.readFileRange(fname, offset, buf, n);
      if (got != n) {
        if (_console) _console->println("coproc script: read error");
        return false;
      }
      rl = 0;
      if (!coprocRequest(CoProc::CMD_SCRIPT_DATA, buf, n, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
      if (st != CoProc::ST_OK && st != CoProc::ST_SIZE) {
        if (_console) _console->printf("SCRIPT_DATA st=%d\n", st);
        return false;
      }
      offset += n;
    }

    // First attempt: send sentinel to accept device-computed CRC
    uint32_t sentinel = 0xFFFFFFFFu;
    uint8_t crc4[4];
    memcpy(crc4, &sentinel, 4);
    rl = 0;
    if (!coprocRequest(CoProc::CMD_SCRIPT_END, crc4, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;

    if (st == CoProc::ST_OK) {
      if (_console) {
        uint32_t have = 0;
        if (rl >= 12) memcpy(&have, rbuf + 8, 4);
        _console->printf("CoProc SCRIPT LOAD OK (%u bytes) crc=0x%08X\n", (unsigned)size, (unsigned)have);
      }
      return true;
    }

    // Backward compatibility: older firmware didn’t accept sentinel.
    // Fall back to computing canonical CRC locally and retry END once.
    if (st == CoProc::ST_CRC || st == CoProc::ST_PARAM) {
      // Compute canonical CRC across chunks (seed=0xFFFFFFFF, undo final XOR between chunks)
      uint32_t crc_state = 0xFFFFFFFFu;
      uint32_t final_crc = 0;
      offset = 0;
      while (offset < size) {
        uint32_t n = (size - offset > CHUNK) ? CHUNK : (size - offset);
        uint32_t got = _fs.readFileRange(fname, offset, buf, n);
        if (got != n) {
          if (_console) _console->println("coproc script: read error (CRC pass)");
          return false;
        }
        uint32_t ret = CoProc::crc32_ieee(buf, n, crc_state);
        crc_state = ret ^ 0xFFFFFFFFu;
        final_crc = ret;
        offset += n;
      }
      uint8_t crc4_retry[4];
      memcpy(crc4_retry, &final_crc, 4);
      rl = 0;
      if (!coprocRequest(CoProc::CMD_SCRIPT_END, crc4_retry, 4, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
      if (st == CoProc::ST_OK) {
        if (_console) _console->printf("CoProc SCRIPT LOAD OK after compat END (%u bytes)\n", (unsigned)size);
        return true;
      }
      if (_console) _console->printf("SCRIPT_END failed st=%d\n", st);
      return false;
    }

    if (_console) _console->printf("SCRIPT_END failed st=%d\n", st);
    return false;
  }
  bool coprocScriptExec(const int32_t* argv, uint32_t argc) {
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    uint32_t timeoutMs = timeout(100000);
    uint8_t payload[4 + MAX_EXEC_ARGS * 4 + 4];
    size_t off = 0;
    memcpy(payload + off, &argc, 4);
    off += 4;
    for (uint32_t i = 0; i < argc; ++i) {
      memcpy(payload + off, &argv[i], 4);
      off += 4;
    }
    memcpy(payload + off, &timeoutMs, 4);
    off += 4;
    CoProc::Frame rh;
    uint8_t rbuf[16];
    uint32_t rl = 0;
    int32_t st = 0;
    if (!coprocRequest(CoProc::CMD_SCRIPT_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st)) return false;
    if (rl < 8) {
      if (_console) _console->println("coproc: short SCRIPT_EXEC response");
      return false;
    }
    int32_t retCode = 0;
    memcpy(&st, rbuf + 0, 4);
    memcpy(&retCode, rbuf + 4, 4);
    if (_console) _console->printf("CoProc SCRIPT EXEC: status=%d return=%d\n", (int)st, (int)retCode);
    return true;
  }
  bool coprocScriptExecTokens(const char* tokens[], uint32_t argc) {
    if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
    uint32_t timeoutMs = timeout(100000);
    bool needAscii = false;
    for (uint32_t i = 0; i < argc; ++i) {
      const char* s = tokens[i];
      if (!s || !*s) {
        needAscii = true;
        break;
      }
      if ((s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))) {
        needAscii = true;
        break;
      }
      for (const char* p = s; *p; ++p) {
        char c = *p;
        if ((c >= '0' && c <= '9') || c == '+' || c == '-') continue;
        needAscii = true;
        break;
      }
      if (needAscii) break;
    }
    if (!needAscii) {
      int32_t argv[MAX_EXEC_ARGS];
      for (uint32_t i = 0; i < argc; ++i) argv[i] = (int32_t)strtol(tokens[i], nullptr, 0);
      return coprocScriptExec(argv, argc);
    }
    uint32_t total = 4 + 4;
    for (uint32_t i = 0; i < argc; ++i) total += 4 + (uint32_t)strlen(tokens[i]);
    total += 4;
    if (total > 8192) {
      if (_console) _console->println("coproc script exec: args too large");
      return false;
    }
    uint8_t* payload = (uint8_t*)malloc(total);
    if (!payload) {
      if (_console) _console->println("coproc script exec: malloc failed");
      return false;
    }
    size_t off = 0;
    memcpy(payload + off, &argc, 4);
    off += 4;
    uint32_t marker = 0xFFFFFFFFu;
    memcpy(payload + off, &marker, 4);
    off += 4;
    for (uint32_t i = 0; i < argc; ++i) {
      uint32_t sl = (uint32_t)strlen(tokens[i]);
      memcpy(payload + off, &sl, 4);
      off += 4;
      memcpy(payload + off, tokens[i], sl);
      off += sl;
    }
    memcpy(payload + off, &timeoutMs, 4);
    off += 4;
    CoProc::Frame rh;
    uint8_t rbuf[16];
    uint32_t rl = 0;
    int32_t st = 0;
    bool ok = coprocRequest(CoProc::CMD_SCRIPT_EXEC, payload, (uint32_t)off, rh, rbuf, sizeof(rbuf), rl, &st);
    free(payload);
    if (!ok) return false;
    if (rl < 8) {
      if (_console) _console->println("coproc: short SCRIPT_EXEC response");
      return false;
    }
    int32_t retCode = 0;
    memcpy(&st, rbuf + 0, 4);
    memcpy(&retCode, rbuf + 4, 4);
    if (_console) _console->printf("CoProc SCRIPT EXEC: status=%d return=%d\n", (int)st, (int)retCode);
    return true;
  }
  bool coprocScriptExecFileTokens(const char* fname, const char* tokens[], uint32_t argc) {
    if (!coprocScriptLoadFile(fname)) return false;
    return coprocScriptExecTokens(tokens, argc);
  }
  bool coprocScriptExecFile(const char* fname, const int32_t* argv, uint32_t argc) {
    if (!coprocScriptLoadFile(fname)) return false;
    return coprocScriptExec(argv, argc);
  }
private:
  // Serial helpers
  bool linkReadByte(uint8_t& b, uint32_t timeoutMs) {
    if (!_link) return false;
    uint32_t start = millis();
    while ((millis() - start) <= timeoutMs) {
      if (_link->available() > 0) {
        int v = _link->read();
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
  bool linkReadExact(uint8_t* dst, size_t n, uint32_t timeoutPerByteMs) {
    for (size_t i = 0; i < n; ++i) {
      if (!linkReadByte(dst[i], timeoutPerByteMs)) return false;
    }
    return true;
  }
  bool linkWriteAll(const uint8_t* src, size_t n, uint32_t timeoutMs) {
    if (!_link) return false;
    uint32_t start = millis();
    size_t off = 0;
    while (off < n) {
      size_t wrote = _link->write(src + off, n - off);
      if (wrote > 0) {
        off += wrote;
        continue;
      }
      if ((millis() - start) > timeoutMs) return false;
      yield();
    }
    _link->flush();
    return true;
  }
  bool readResponseHeader(CoProc::Frame& rh, uint32_t overallTimeoutMs = 120000) {
    const uint8_t magicBytes[4] = { (uint8_t)('C'), (uint8_t)('P'), (uint8_t)('R'), (uint8_t)('0') };
    uint8_t w[4] = { 0, 0, 0, 0 };
    uint32_t start = millis();
    while ((millis() - start) <= overallTimeoutMs) {
      uint8_t b;
      if (!linkReadByte(b, 50)) {
        yield();
        continue;
      }
      w[0] = w[1];
      w[1] = w[2];
      w[2] = w[3];
      w[3] = b;
      if (w[0] == magicBytes[0] && w[1] == magicBytes[1] && w[2] == magicBytes[2] && w[3] == magicBytes[3]) {
        union {
          CoProc::Frame f;
          uint8_t bytes[sizeof(CoProc::Frame)];
        } u;
        memcpy(u.bytes, w, 4);
        if (!linkReadExact(u.bytes + 4, sizeof(CoProc::Frame) - 4, 200)) return false;
        rh = u.f;
        return true;
      }
    }
    return false;
  }
  bool coprocRequest(uint16_t cmd,
                     const uint8_t* payload, uint32_t len,
                     CoProc::Frame& respHdr,
                     uint8_t* respBuf, uint32_t respCap, uint32_t& respLen,
                     int32_t* statusOut = nullptr) {
    if (!_link) {
      if (_console) _console->println("coproc(serial): link not attached");
      return false;
    }
    CoProc::Frame req{};
    req.magic = CoProc::MAGIC;
    req.version = CoProc::VERSION;
    req.cmd = cmd;
    req.seq = _coproc_seq++;
    req.len = len;
    req.crc32 = (len ? CoProc::crc32_ieee(payload, len) : 0);
    if (!linkWriteAll(reinterpret_cast<const uint8_t*>(&req), sizeof(req), 2000)) {
      if (_console) _console->println("coproc(serial): write header failed");
      return false;
    }
    if (len && payload) {
      if (!linkWriteAll(payload, len, 10000)) {
        if (_console) _console->println("coproc(serial): write payload failed");
        return false;
      }
    }
    memset(&respHdr, 0, sizeof(respHdr));
    if (!readResponseHeader(respHdr, 180000)) {
      if (_console) _console->println("coproc(serial): read resp header timeout");
      return false;
    }
    if (respHdr.magic != CoProc::MAGIC || respHdr.version != CoProc::VERSION) {
      if (_console) _console->println("coproc(serial): bad resp header (magic/version)");
      return false;
    }
    if (respHdr.cmd != (uint16_t)(cmd | 0x80)) {
      if (_console) _console->printf("coproc(serial): bad resp cmd 0x%02X (expected 0x%02X)\n", (unsigned)respHdr.cmd, (unsigned)(cmd | 0x80));
      return false;
    }
    respLen = 0;
    if (respHdr.len) {
      if (respHdr.len > respCap) {
        if (_console) _console->printf("coproc(serial): resp too large %u > cap %u\n", (unsigned)respHdr.len, (unsigned)respCap);
        return false;
      }
      if (!linkReadExact(respBuf, respHdr.len, 200)) {
        if (_console) _console->println("coproc(serial): resp payload timeout");
        return false;
      }
      respLen = respHdr.len;
      if (respHdr.crc32) {
        uint32_t c = CoProc::crc32_ieee(respBuf, respLen);
        if (c != respHdr.crc32) {
          if (_console) _console->printf("coproc(serial): resp CRC mismatch calc=0x%08X hdr=0x%08X\n",
                                         (unsigned)c, (unsigned)respHdr.crc32);
          return false;
        }
      }
    }
    if (statusOut) {
      if (respLen >= 4) {
        int32_t st = 0;
        memcpy(&st, respBuf, 4);
        *statusOut = st;
      } else {
        *statusOut = (int32_t)CoProc::ST_OK;
      }
    }
    return true;
  }
private:
  struct ExecJob {
    uintptr_t code;
    uint32_t size;
    uint32_t argc;
    int32_t args[MAX_EXEC_ARGS];
  };
  ConsolePrint* _console;
  ExecFSTable _fs;
  bool _fsValid;
  SoftwareSerial* _link;
  uint32_t _coproc_seq;
  uint32_t _timeout_override_ms;
  // core1 shared state
  volatile ExecJob _job;
  volatile int32_t _result;
  volatile int32_t _status;
  volatile uint32_t _job_flag;  // 0=idle,1=ready,2=running,3=done
  // background job
  volatile void* _bg_raw;
  volatile uint8_t* _bg_buf;
  volatile uint32_t _bg_sz;
  volatile bool _bg_active;
  volatile uint32_t _bg_start_ms;
  volatile uint32_t _bg_timeout_ms;
  char _bg_fname[32 + 1];  // filename max = 32 per original ActiveFS::MAX_NAME
  volatile bool _bg_cancel_requested;
};