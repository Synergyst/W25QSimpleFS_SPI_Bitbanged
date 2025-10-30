/*
  main_psram_flash_switch_exec_loader.ino
  DEFAULT CONFIGURATION (golden/known-good-configs) PSRAM CS SCHEME: Single 74HC138 (MC74HC138AN), 4 PSRAM chips
*/
// ------- Shared HW SPI (FLASH + PSRAM) -------
#include "ConsolePrint.h"
#include <SPI.h>
// FLASH on HW-SPI via W25QBitbang's HW path
#define W25Q_USE_HW_SPI 1
#define W25Q_SPI_INSTANCE SPI1
#define W25Q_SPI_CLOCK_HZ 20000000UL
#include "W25QBitbang.h"
#include "W25QSimpleFS.h"
// PSRAM transport
#define USE_PSRAM_HW_SPI 1
#if USE_PSRAM_HW_SPI
#define PSRAM_SPI_INSTANCE SPI1
#include "SPIHWAdapter.h"
#define PSRAM_AGGREGATE_BUS SPIHWAdapter
#else
#include "PSRAMBitbang.h"
#endif
#include "PSRAMMulti.h"
// ------- Co-Processor over Software Serial (framed RPC) -------
#include <SoftwareSerial.h>
#include "CoProcProto.h"
#define COPROCLANG_MAX_LINES 512
#define COPROCLANG_MAX_LABELS 128
#include "CoProcLang.h"
#ifndef COPROC_BAUD
#define COPROC_BAUD 230400
#endif
#ifndef PIN_COPROC_RX
#define PIN_COPROC_RX 0  // GP0 (main RX)
#endif
#ifndef PIN_COPROC_TX
#define PIN_COPROC_TX 1  // GP1 (main TX)
#endif
static SoftwareSerial coprocLink(PIN_COPROC_RX, PIN_COPROC_TX, false);  // RX, TX, non-inverted
// Store ASCII scripts as raw multi-line strings, expose pointer + length.
#ifndef DECLARE_ASCII_SCRIPT
#define DECLARE_ASCII_SCRIPT(name, literal) \
  static const char name##_ascii[] = literal; \
  const uint8_t* name = reinterpret_cast<const uint8_t*>(name##_ascii); \
  const unsigned int name##_len = (unsigned int)(sizeof(name##_ascii) - 1)
#endif
struct BlobReg;               // Forward declaration
static ConsolePrint Console;  // Console wrapper
// ------- MC Compiiler -------
#include "MCCompiler.h"

// ------- User preference configuration -------
#include "config.h"

// ========== ActiveFS structure ==========
struct ActiveFS {
  bool (*mount)(bool) = nullptr;
  bool (*format)() = nullptr;
  bool (*wipeChip)() = nullptr;
  bool (*exists)(const char*) = nullptr;
  bool (*createFileSlot)(const char*, uint32_t, const uint8_t*, uint32_t) = nullptr;
  bool (*writeFile)(const char*, const uint8_t*, uint32_t, int /*mode*/) = nullptr;
  bool (*writeFileInPlace)(const char*, const uint8_t*, uint32_t, bool) = nullptr;
  uint32_t (*readFile)(const char*, uint8_t*, uint32_t) = nullptr;
  uint32_t (*readFileRange)(const char*, uint32_t, uint8_t*, uint32_t) = nullptr;
  bool (*getFileSize)(const char*, uint32_t&) = nullptr;
  bool (*getFileInfo)(const char*, uint32_t&, uint32_t&, uint32_t&) = nullptr;
  bool (*deleteFile)(const char*) = nullptr;
  void (*listFilesToSerial)() = nullptr;
  uint32_t (*nextDataAddr)() = nullptr;
  uint32_t (*capacity)() = nullptr;
  uint32_t (*dataRegionStart)() = nullptr;
  static constexpr uint32_t SECTOR_SIZE = FS_SECTOR_SIZE;
  static constexpr uint32_t PAGE_SIZE = 256;
  static constexpr size_t MAX_NAME = 32;
} activeFs;

static void bindActiveFs(StorageBackend backend) {
  if (backend == StorageBackend::Flash) {
    activeFs.mount = [](bool b) {
      return fsFlash.mount(b);
    };
    activeFs.format = []() {
      return fsFlash.format();
    };
    activeFs.wipeChip = []() {
      return fsFlash.wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return fsFlash.exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsFlash.createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsFlash.writeFile(n, d, s, static_cast<W25QSimpleFS::WriteMode>(m));
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsFlash.writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsFlash.readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return fsFlash.readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return fsFlash.getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsFlash.getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return fsFlash.deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      fsFlash.listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return fsFlash.nextDataAddr();
    };
    activeFs.capacity = []() {
      return fsFlash.capacity();
    };
    activeFs.dataRegionStart = []() {
      return fsFlash.dataRegionStart();
    };
  } else {
    activeFs.mount = [](bool b) {
      return fsPSRAM.mount(b);
    };
    activeFs.format = []() {
      return fsPSRAM.format();
    };
    activeFs.wipeChip = []() {
      return fsPSRAM.wipeChip();
    };
    activeFs.exists = [](const char* n) {
      return fsPSRAM.exists(n);
    };
    activeFs.createFileSlot = [](const char* n, uint32_t r, const uint8_t* d, uint32_t s) {
      return fsPSRAM.createFileSlot(n, r, d, s);
    };
    activeFs.writeFile = [](const char* n, const uint8_t* d, uint32_t s, int m) {
      return fsPSRAM.writeFile(n, d, s, m);
    };
    activeFs.writeFileInPlace = [](const char* n, const uint8_t* d, uint32_t s, bool a) {
      return fsPSRAM.writeFileInPlace(n, d, s, a);
    };
    activeFs.readFile = [](const char* n, uint8_t* b, uint32_t sz) {
      return fsPSRAM.readFile(n, b, sz);
    };
    activeFs.readFileRange = [](const char* n, uint32_t off, uint8_t* b, uint32_t l) {
      return fsPSRAM.readFileRange(n, off, b, l);
    };
    activeFs.getFileSize = [](const char* n, uint32_t& s) {
      return fsPSRAM.getFileSize(n, s);
    };
    activeFs.getFileInfo = [](const char* n, uint32_t& a, uint32_t& s, uint32_t& c) {
      return fsPSRAM.getFileInfo(n, a, s, c);
    };
    activeFs.deleteFile = [](const char* n) {
      return fsPSRAM.deleteFile(n);
    };
    activeFs.listFilesToSerial = []() {
      fsPSRAM.listFilesToSerial();
    };
    activeFs.nextDataAddr = []() {
      return fsPSRAM.nextDataAddr();
    };
    activeFs.capacity = []() {
      return fsPSRAM.capacity();
    };
    activeFs.dataRegionStart = []() {
      return fsPSRAM.dataRegionStart();
    };
  }
}

// ========== Blob registry ==========
struct BlobReg {
  const char* id;
  const uint8_t* data;
  unsigned int len;
};
static const BlobReg g_blobs[] = {
  { FILE_RET42, blob_ret42, blob_ret42_len },
  { FILE_ADD2, blob_add2, blob_add2_len },
  { FILE_PWMC, blob_pwmc, blob_pwmc_len },
  { FILE_PWMC2350, blob_pwmc, blob_pwmc2350_len },
  { FILE_RETMIN, blob_retmin, blob_retmin_len },
  { FILE_BLINKSCRIPT, blob_blinkscript, blob_blinkscript_len },
  { FILE_ONSCRIPT, blob_onscript, blob_onscript_len },
};
static const size_t g_blobs_count = sizeof(g_blobs) / sizeof(g_blobs[0]);

// ========== Return mailbox reservation (Scratch) ==========
extern "C" __scratch_x("blob_mailbox") __attribute__((aligned(4)))
int8_t BLOB_MAILBOX[BLOB_MAILBOX_MAX] = { 0 };

// ================= ExecHost header =================
#include "ExecHost.h"
static ExecHost Exec;

// ================= FSHelpers header =================
#include "FSHelpers.h"

// ================= CompilerHelpers header =================
#include "CompilerHelpers.h"

// ===================== Co-Processor RPC helpers (for CMD_FUNC) =====================
static uint32_t g_coproc_seq = 1;
static bool coprocWriteAll(const uint8_t* src, size_t n, uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t off = 0;
  while (off < n) {
    size_t w = coprocLink.write(src + off, n - off);
    if (w > 0) {
      off += w;
      continue;
    }
    if ((millis() - start) > timeoutMs) return false;
    yield();
  }
  coprocLink.flush();
  return true;
}
static bool coprocReadByte(uint8_t& b, uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((millis() - start) <= timeoutMs) {
    int a = coprocLink.available();
    if (a > 0) {
      int v = coprocLink.read();
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
static bool coprocReadExact(uint8_t* dst, size_t n, uint32_t timeoutPerByteMs) {
  for (size_t i = 0; i < n; ++i) {
    if (!coprocReadByte(dst[i], timeoutPerByteMs)) return false;
  }
  return true;
}
static bool coprocCallFunc(const char* name, const int32_t* argv, uint32_t argc, int32_t& outResult) {
  // Send CMD_FUNC with classic int32 argv[argc]. On success returns true and sets outResult.
  if (!name) return false;
  uint32_t nameLen = (uint32_t)strlen(name);
  if (nameLen == 0 || nameLen > 128) {
    Console.println("coproc func: name length invalid (1..128)");
    return false;
  }
  if (argc > MAX_EXEC_ARGS) argc = MAX_EXEC_ARGS;
  // Build payload: [uint32 nameLen][bytes name][uint32 argc][int32 argv[argc]]
  const uint32_t payloadLen = 4 + nameLen + 4 + 4 * argc;
  uint8_t payload[4 + 128 + 4 + 4 * MAX_EXEC_ARGS];
  size_t off = 0;
  CoProc::writePOD(payload, sizeof(payload), off, nameLen);
  CoProc::writeBytes(payload, sizeof(payload), off, name, nameLen);
  CoProc::writePOD(payload, sizeof(payload), off, argc);
  for (uint32_t i = 0; i < argc; ++i) {
    CoProc::writePOD(payload, sizeof(payload), off, argv[i]);
  }
  // Build request header
  CoProc::Frame req{};
  req.magic = CoProc::MAGIC;
  req.version = CoProc::VERSION;
  req.cmd = CoProc::CMD_FUNC;
  req.seq = g_coproc_seq++;
  req.len = payloadLen;
  req.crc32 = (payloadLen ? CoProc::crc32_ieee(payload, payloadLen) : 0);
  // Send request
  if (!coprocWriteAll(reinterpret_cast<const uint8_t*>(&req), sizeof(req), 2000)) {
    Console.println("coproc func: write header failed");
    return false;
  }
  if (payloadLen) {
    if (!coprocWriteAll(payload, payloadLen, 10000)) {
      Console.println("coproc func: write payload failed");
      return false;
    }
  }
  // Read response: scan for magic
  const uint8_t magicBytes[4] = { (uint8_t)('C'), (uint8_t)('P'), (uint8_t)('R'), (uint8_t)('0') };
  uint8_t w[4] = { 0, 0, 0, 0 };
  for (;;) {
    uint8_t b = 0;
    if (!coprocReadByte(b, 5000)) {
      Console.println("coproc func: response timeout");
      return false;
    }
    w[0] = w[1];
    w[1] = w[2];
    w[2] = w[3];
    w[3] = b;
    if (w[0] == magicBytes[0] && w[1] == magicBytes[1] && w[2] == magicBytes[2] && w[3] == magicBytes[3]) {
      // We have magic, read rest of header
      union {
        CoProc::Frame f;
        uint8_t bytes[sizeof(CoProc::Frame)];
      } u;
      memcpy(u.bytes, w, 4);
      if (!coprocReadExact(u.bytes + 4, sizeof(CoProc::Frame) - 4, 200)) {
        Console.println("coproc func: resp header tail timeout");
        return false;
      }
      CoProc::Frame resp = u.f;
      if ((resp.magic != CoProc::MAGIC) || (resp.version != CoProc::VERSION)) {
        Console.println("coproc func: resp bad magic/version");
        return false;
      }
      if (resp.cmd != (uint16_t)(CoProc::CMD_FUNC | 0x80)) {
        Console.print("coproc func: unexpected resp cmd=0x");
        Console.println(resp.cmd, HEX);
        return false;
      }
      // Read payload if any
      uint8_t buf[16];
      if (resp.len > sizeof(buf)) {
        Console.println("coproc func: resp too large");
        uint32_t left = resp.len;
        uint8_t sink[32];
        while (left) {
          uint32_t chunk = (left > sizeof(sink)) ? sizeof(sink) : left;
          if (!coprocReadExact(sink, chunk, 200)) break;
          left -= chunk;
        }
        return false;
      }
      if (resp.len) {
        if (!coprocReadExact(buf, resp.len, 200)) {
          Console.println("coproc func: resp payload timeout");
          return false;
        }
        uint32_t crc = CoProc::crc32_ieee(buf, resp.len);
        if (crc != resp.crc32) {
          Console.print("coproc func: resp CRC mismatch exp=0x");
          Console.print(resp.crc32, HEX);
          Console.print(" got=0x");
          Console.println(crc, HEX);
          return false;
        }
      }
      // Parse response: int32 status, int32 result
      size_t rp = 0;
      int32_t st = CoProc::ST_BAD_CMD;
      int32_t rv = 0;
      if (resp.len >= sizeof(int32_t)) {
        CoProc::readPOD(buf, resp.len, rp, st);
      }
      if (resp.len >= 2 * sizeof(int32_t)) {
        CoProc::readPOD(buf, resp.len, rp, rv);
      }
      if (st == CoProc::ST_OK) {
        outResult = rv;
        return true;
      } else {
        Console.print("coproc func: remote error status=");
        Console.println(st);
        return false;
      }
    }
  }
}

// ----------------- Nano-like editor integration -----------------
#include "TextEditor.h"

// ========== Serial console / command handling ==========
static char lineBuf[FS_SECTOR_SIZE];
static bool readLine() {
  // Cooked console line input with echo, backspace handling, and CR/LF normalization.
  // This fixes lack of echo in terminal emulators and supports Windows CRLF.
  static size_t pos = 0;
  while (Serial.available()) {
    int ic = Serial.read();
    if (ic < 0) break;
    char c = (char)ic;

    // Handle Windows CRLF and Unix LF
    if (c == '\r' || c == '\n') {
      // Swallow paired LF after CR for Windows
      if (c == '\r') {
        if (Serial.available()) {
          int p = Serial.peek();
          if (p == '\n') Serial.read();
        }
      }
      // Echo newline to terminal
      Serial.write('\r');
      Serial.write('\n');
      lineBuf[pos] = 0;
      pos = 0;
      return true;
    }

    // Backspace / Delete
    if ((c == 0x08) || (c == 0x7F)) {
      if (pos > 0) {
        pos--;
        // Erase char visually: backspace, space, backspace
        Serial.write('\b');
        Serial.write(' ');
        Serial.write('\b');
      }
      continue;
    }

    // Printable ASCII
    if (c >= 32 && c <= 126) {
      if (pos + 1 < sizeof(lineBuf)) {
        lineBuf[pos++] = c;
        Serial.write(c);  // echo
      } else {
        // Optional: bell on overflow
        Serial.write('\a');
      }
      continue;
    }

    // Ignore other control bytes
  }
  return false;
}
static int nextToken(char*& p, char*& tok) {
  while (*p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  if (!*p) {
    tok = nullptr;
    return 0;
  }
  tok = p;
  while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
  if (*p) {
    *p = 0;
    ++p;
  }
  return 1;
}
static void printHelp() {
  Console.println("Commands (filename max 32 chars):");
  Console.println("  help                         - this help");
  Console.println("  storage                      - show active storage");
  Console.println("  storage flash|psram          - switch storage backend");
  Console.println("  mode                         - show mode (dev/prod)");
  Console.println("  mode dev|prod                - set dev or prod mode");
  Console.println("  persist read                 - read persist file from active storage");
  Console.println("  persist write                - write persist file to active storage");
  Console.println("  blobs                        - list compiled-in blobs");
  Console.println("  autogen                      - auto-create enabled blobs if missing");
  Console.println("  files                        - list files in FS");
  Console.println("  info <file>                  - show file addr/size/cap");
  Console.println("  dump <file> <nbytes>         - hex dump head of file");
  Console.println("  mkSlot <file> <reserve>      - create sector-aligned slot");
  Console.println("  writeblob <file> <blobId>    - create/update file from blob");
  Console.printf("  exec <file> [a0..aN] [&]     - execute blob with 0..%d int args on core1; use '&' to background\n", (int)MAX_EXEC_ARGS);
  Console.println("                                (foreground blocks until completion; background returns immediately)");
  Console.println("  del <file>                   - delete a file");
  Console.println("  format                       - format active FS");
  Console.println("  wipe                         - erase active chip (DANGEROUS to FS)");
  Console.println("  wipereboot                   - erase chip then reboot (DANGEROUS to FS)");
  Console.println("  wipebootloader               - erase chip then reboot to bootloader (DANGEROUS to FS)");
  Console.println("  meminfo                      - show heap/stack info");
  Console.println("  psramsmoketest               - safe, non-destructive PSRAM test (prefers FS-free region)");
  Console.println("  bg [status|query]            - query background job status");
  Console.println("  bg kill|cancel [force]       - cancel background job; 'force' may reset core1 (platform-dependent)");
  Console.println("  reboot                       - reboot the MCU");
  Console.println("  timeout [ms]                 - show or set core1 timeout override (0=defaults)");
  Console.println("  puthex <file> <hex>          - upload binary as hex string (single line) to FS");
  Console.println("  putb64 <file> <base64>       - upload binary as base64 (single line) to FS");
  Console.println("  cc <src> <dst>               - compile Tiny-C source file to raw ARM Thumb-1 binary");
  Console.println();
  Console.println("Editor (Nano-style) commands:");
  Console.println("  nano <file>                  - open Nano-like editor for <file> (ActiveFS-backed)");
  Console.println("  edit <file>                  - alias for 'nano'");
  Console.println("    Keys in editor:");
  Console.println("      Arrows: move   Backspace/Delete: edit   Enter: newline");
  Console.println("      Ctrl+C: show cursor position   Ctrl+X: prompt to save and exit");
  Console.println();
  Console.println("Co-Processor (serial RPC) commands:");
  Console.println("  coproc ping                  - HELLO (version/features)");
  Console.println("  coproc info                  - INFO (flags, blob_len, mailbox_max)");
  Console.println("  coproc exec <file> [a0..aN]  - Load <file> from active FS to co-proc and EXEC with args");
  Console.println("                                 Timeout is taken from 'timeout' setting on the master");
  Console.println("  coproc sexec <file> [a0..aN] - Load a script file to co-proc and execute once");
  Console.println("                                 Uses master 'timeout' setting; script may use DELAY/DELAY_US, PINMODE, DWRITE, etc.");
  Console.println("  coproc func <name> [a0..aN]  - Call a named function registered on the co-processor");
  Console.println("  coproc status                - STATUS (exec_state)");
  Console.println("  coproc mbox [n]              - MAILBOX_RD (read up to n bytes; default 128)");
  Console.println("  coproc cancel                - CANCEL (sets cancel flag)");
  Console.println("  coproc reset                 - RESET (remote reboot)");
  Console.println("  coproc isp enter|exit        - request co-processor to enter or exit ISP mode (device-specific)");
  Console.println();
  Console.println("Linux hints:");
  Console.println("  # Base64 (single line, no wraps)  base64 -w0 your.bin");
  Console.println("  # Hex (single line, no spaces)    xxd -p -c 999999 your.bin | tr -d '\\n'");
  Console.println("Example:");
  Console.println("  putb64 myprog <paste_output_of_base64>");
  Console.println("  puthex myprog <paste_output_of_xxd>");
  Console.println("  cc myprog.c myprog.bin");
  Console.println("  coproc sexec myscript [a0..aN]");
  Console.println("Notes:");
  Console.println("  - For blobs you intend to exec, even byte length is recommended (Thumb).");
  Console.println("  - Background jobs: only one background job supported at a time. Use 'bg' to query or cancel.");
  Console.println();
}
static void handleCommand(char* line) {
  char* p = line;
  char* t0;
  if (!nextToken(p, t0)) return;

  if (!strcmp(t0, "help")) {
    printHelp();

  } else if (!strcmp(t0, "nano") || !strcmp(t0, "edit")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: nano <file>");
      return;
    }
    if (!checkNameLen(fn)) return;
    Console.printf("Opening editor for '%s'...\n", fn);
    runNanoEditor(fn);
    Console.println("Returned to console.");

  } else if (!strcmp(t0, "storage")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.print("Active storage: ");
      Console.println(g_storage == StorageBackend::Flash ? "flash" : "psram");
      return;
    }
    if (!strcmp(tok, "flash")) {
      g_storage = StorageBackend::Flash;
      bindActiveFs(g_storage);
      updateExecFsTable();
      bool ok = activeFs.mount(true);
      Console.println("Switched active storage to FLASH");
      Console.println(ok ? "Mounted FLASH (auto-format if empty)" : "Mount failed (FLASH)");
    } else if (!strcmp(tok, "psram")) {
      g_storage = StorageBackend::PSRAM_BACKEND;
      bindActiveFs(g_storage);
      updateExecFsTable();
      bool ok = activeFs.mount(false);
      Console.println("Switched active storage to PSRAM");
      Console.println(ok ? "Mounted PSRAM (no auto-format)" : "Mount failed (PSRAM)");
    } else {
      Console.println("usage: storage [flash|psram]");
    }

  } else if (!strcmp(t0, "mode")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.print("Mode: ");
      Console.println(g_dev_mode ? "dev" : "prod");
      return;
    }
    if (!strcmp(tok, "dev")) {
      g_dev_mode = true;
      Console.println("mode=dev");
    } else if (!strcmp(tok, "prod")) {
      g_dev_mode = false;
      Console.println("mode=prod");
    } else Console.println("usage: mode [dev|prod]");

  } else if (!strcmp(t0, "persist")) {
    char* tok;
    if (!nextToken(p, tok)) {
      Console.println("usage: persist [read|write]");
      return;
    }
    if (!strcmp(tok, "read")) {
      const char* pf = ".persist";
      if (!activeFs.exists(pf)) {
        Console.println("persist read: not found");
        return;
      }
      uint32_t sz = 0;
      activeFs.getFileSize(pf, sz);
      uint8_t buf[PERSIST_LEN];
      memset(buf, 0, PERSIST_LEN);
      uint32_t got = activeFs.readFile(pf, buf, PERSIST_LEN);
      Console.printf("persist read: %d bytes: ", got);
      for (size_t i = 0; i < got; ++i) {
        if (i) Console.print(' ');
        if (buf[i] < 0x10) Console.print('0');
        Console.print(buf[i], HEX);
      }
      Console.println();
    } else if (!strcmp(tok, "write")) {
      const char* pf = ".persist";
      uint8_t buf[PERSIST_LEN];
      for (size_t i = 0; i < PERSIST_LEN; ++i) buf[i] = (uint8_t)(i ^ (g_dev_mode ? 0xA5 : 0x5A));
      if (!activeFs.exists(pf)) {
        if (!activeFs.createFileSlot(pf, ActiveFS::SECTOR_SIZE, buf, PERSIST_LEN)) {
          Console.println("persist write: create failed");
          return;
        }
      } else {
        if (!activeFs.writeFileInPlace(pf, buf, PERSIST_LEN, true)) {
          if (!activeFs.writeFile(pf, buf, PERSIST_LEN, static_cast<int>(W25QSimpleFS::WriteMode::ReplaceIfExists))) {
            Console.println("persist write: write failed");
            return;
          }
        }
      }
      Console.println("persist write: OK");
    } else {
      Console.println("usage: persist [read|write]");
    }

  } else if (!strcmp(t0, "autogen")) {
    autogenBlobWrites();

  } else if (!strcmp(t0, "files")) {
    activeFs.listFilesToSerial();

  } else if (!strcmp(t0, "info")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: info <file>");
      return;
    }
    uint32_t a, s, c;
    if (activeFs.getFileInfo(fn, a, s, c)) {
      Console.print(fn);
      Console.print(": addr=0x");
      Console.print(a, HEX);
      Console.print(" size=");
      Console.print(s);
      Console.print(" cap=");
      Console.println(c);
    } else Console.println("not found");

  } else if (!strcmp(t0, "dump")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Console.println("usage: dump <file> <nbytes>");
      return;
    }
    dumpFileHead(fn, (uint32_t)strtoul(nstr, nullptr, 0));

  } else if (!strcmp(t0, "mkSlot")) {
    char* fn;
    char* nstr;
    if (!nextToken(p, fn) || !nextToken(p, nstr)) {
      Console.println("usage: mkSlot <file> <reserve>");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint32_t res = (uint32_t)strtoul(nstr, nullptr, 0);
    if (activeFs.createFileSlot(fn, res, nullptr, 0)) Console.println("slot created");
    else Console.println("mkSlot failed");

  } else if (!strcmp(t0, "writeblob")) {
    char* fn;
    char* bid;
    if (!nextToken(p, fn) || !nextToken(p, bid)) {
      Console.println("usage: writeblob <file> <blobId>");
      return;
    }
    if (!checkNameLen(fn)) return;
    const BlobReg* br = findBlob(bid);
    if (!br) {
      Console.println("unknown blobId; use 'blobs'");
      return;
    }
    if (ensureBlobFile(fn, br->data, br->len)) Console.println("writeblob OK");
    else Console.println("writeblob failed");

  } else if (!strcmp(t0, "exec")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: exec <file> [a0 ... aN] [&]");
      return;
    }
    auto strip_trailing_amp = [](char* s) -> bool {
      if (!s) return false;
      size_t L = strlen(s);
      if (L == 0) return false;
      if (s[L - 1] == '&') {
        s[L - 1] = '\0';
        return true;
      }
      return false;
    };
    char* toks[MAX_EXEC_ARGS + 2];
    int tcount = 0;
    char* tok = nullptr;
    while (tcount < (int)MAX_EXEC_ARGS + 1 && nextToken(p, tok)) toks[tcount++] = tok;
    bool background = false;
    if (tcount > 0 && strcmp(toks[tcount - 1], "&") == 0) {
      background = true;
      --tcount;
    } else if (tcount == 0) {
      if (strip_trailing_amp(fn)) background = true;
    } else {
      if (strip_trailing_amp(toks[tcount - 1])) {
        background = true;
        if (toks[tcount - 1][0] == '\0') --tcount;
      }
    }
    static int32_t argvN[MAX_EXEC_ARGS];
    int argc = 0;
    for (int i = 0; i < tcount && argc < (int)MAX_EXEC_ARGS; ++i) argvN[argc++] = (int32_t)strtol(toks[i], nullptr, 0);
    if (!background) {
      int rv = 0;
      if (!Exec.execBlobForeground(fn, argc, argvN, rv)) { Console.println("exec failed"); }
    } else {
      void* raw = nullptr;
      uint8_t* buf = nullptr;
      uint32_t sz = 0;
      if (!Exec.loadFileToExecBuf(fn, raw, buf, sz)) {
        Console.println("exec: load failed");
        return;
      }
      uint32_t timeout = Exec.timeout(100000);
      if (!Exec.submitBackground(fn, raw, buf, sz, (uint32_t)argc, (const int32_t*)argvN, timeout)) {
        free(raw);
      }
    }

  } else if (!strcmp(t0, "bg")) {
    char* tok;
    if (!nextToken(p, tok)) {
      if (!Exec.bgActive()) Console.println("No active background job");
      else {
        uint32_t elapsed = Exec.bgElapsedMs();
        Console.printf("BG job: '%s'\n", Exec.bgName());
        Console.printf("  elapsed=%u ms  timeout=%u ms  job_flag=%u\n", (unsigned)elapsed, (unsigned)Exec.bgTimeoutMs(), (unsigned)Exec.core1JobFlag());
      }
      return;
    }
    if (!strcmp(tok, "status") || !strcmp(tok, "query")) {
      if (!Exec.bgActive()) Console.println("No active background job");
      else {
        uint32_t elapsed = Exec.bgElapsedMs();
        Console.printf("BG job: '%s'  elapsed=%u ms  timeout=%u ms  job_flag=%u\n",
                       Exec.bgName(), (unsigned)elapsed, (unsigned)Exec.bgTimeoutMs(), (unsigned)Exec.core1JobFlag());
      }
    } else if (!strcmp(tok, "kill") || !strcmp(tok, "cancel")) {
      char* sub = nullptr;
      nextToken(p, sub);
      if (!Exec.bgActive()) {
        if (Exec.core1JobFlag() == 1u) {
          if (Exec.cancelQueuedBackgroundIfAny()) {
            Console.println("Cleared queued core1 job");
          } else {
            Console.println("No active background job to cancel");
          }
        } else {
          Console.println("No active background job to cancel");
        }
        return;
      }
      if (Exec.core1JobFlag() == 1u) {
        if (Exec.cancelQueuedBackgroundIfAny()) Console.println("Background job canceled (was queued)");
        else Console.println("No queued job to cancel");
        return;
      }
      if (Exec.core1JobFlag() == 2u) {
        if (Exec.requestCancelRunningBackground()) {
          Console.println("Cancellation requested (mailbox flag set). If blob cooperates, it will stop shortly.");
        } else {
          Console.println("No running BG job to cancel");
        }
        return;
      }
      Console.println("Cancellation requested (best-effort).");
    } else {
      Console.println("bg: usage: bg [status|query] | bg kill|cancel [force]");
    }

  } else if (!strcmp(t0, "coproc")) {
    char* sub;
    if (!nextToken(p, sub)) {
      Console.println("coproc cmds:");
      Console.println("  coproc ping");
      Console.println("  coproc info");
      Console.println("  coproc exec <file> [a0..aN]");
      Console.println("  coproc sexec <file> [a0..aN]");
      Console.println("  coproc func <name> [a0..aN]");
      Console.println("  coproc status");
      Console.println("  coproc mbox [n]");
      Console.println("  coproc cancel");
      Console.println("  coproc reset");
      Console.println("  coproc isp enter|exit         - request co-processor to enter or exit ISP mode (device-specific)");
      return;
    }
    if (!strcmp(sub, "ping")) {
      if (!Exec.coprocHello()) Console.println("coproc ping failed");
    } else if (!strcmp(sub, "info")) {
      if (!Exec.coprocInfo()) Console.println("coproc info failed");
    } else if (!strcmp(sub, "sexec")) {
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc sexec <file> [a0..aN]");
        return;
      }
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) tokens[argc++] = tok;
      if (!Exec.coprocScriptLoadFile(fname)) {
        Console.println("coproc sexec failed: load failed");
        return;
      }
      if (!Exec.coprocScriptExecTokens(tokens, argc)) Console.println("coproc sexec failed");
    } else if (!strcmp(sub, "exec")) {
      char* fname = nullptr;
      if (!nextToken(p, fname)) {
        Console.println("usage: coproc exec <file> [a0..aN]");
        return;
      }
      const char* tokens[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) tokens[argc++] = tok;
      if (!Exec.coprocLoadFile(fname)) {
        Console.println("coproc exec failed: load failed");
        return;
      }
      if (!Exec.coprocExecTokens(tokens, argc)) Console.println("coproc exec failed");
    } else if (!strcmp(sub, "func")) {
      char* name = nullptr;
      if (!nextToken(p, name)) {
        Console.println("usage: coproc func <name> [a0..aN]");
        return;
      }
      static int32_t argvN[MAX_EXEC_ARGS];
      uint32_t argc = 0;
      char* tok = nullptr;
      while (nextToken(p, tok) && argc < MAX_EXEC_ARGS) {
        argvN[argc++] = (int32_t)strtol(tok, nullptr, 0);
      }
      int32_t result = 0;
      if (coprocCallFunc(name, argvN, argc, result)) {
        Console.print("coproc func OK, result=");
        Console.println(result);
      } else {
        Console.println("coproc func failed");
      }
    } else if (!strcmp(sub, "status")) {
      if (!Exec.coprocStatus()) Console.println("coproc status failed");
    } else if (!strcmp(sub, "mbox")) {
      char* nstr;
      uint32_t n = 128;
      if (nextToken(p, nstr)) n = (uint32_t)strtoul(nstr, nullptr, 0);
      if (!Exec.coprocMailboxRead(n)) Console.println("coproc mbox failed");
    } else if (!strcmp(sub, "cancel")) {
      if (!Exec.coprocCancel()) Console.println("coproc cancel failed");
    } else if (!strcmp(sub, "reset")) {
      if (!Exec.coprocReset()) Console.println("coproc reset failed (transport)");
    } else if (!strcmp(sub, "isp")) {
      char* arg = nullptr;
      if (!nextToken(p, arg)) {
        Console.println("usage: coproc isp enter|exit");
        return;
      }
      if (!strcmp(arg, "enter")) {
        if (!Exec.coprocIspEnter()) Console.println("coproc isp enter failed");
      } else if (!strcmp(arg, "exit")) {
        if (!Exec.coprocIspExit()) Console.println("coproc isp exit failed");
      } else {
        Console.println("usage: coproc isp enter|exit");
      }
    } else {
      Console.println("usage: coproc ping|info|exec <file> [a0..aN]|sexec <file> [a0..aN]|func <name> [a0..aN]|status|mbox [n]|cancel|reset");
    }

  } else if (!strcmp(t0, "blobs")) {
    listBlobs();

  } else if (!strcmp(t0, "timeout")) {
    char* msStr;
    if (!nextToken(p, msStr)) {
      Console.print("timeout override = ");
      Console.print((uint32_t)Exec.getTimeoutOverride());
      Console.println(" ms (0 = use per-call defaults)");
    } else {
      uint32_t ms = (uint32_t)strtoul(msStr, nullptr, 0);
      Exec.setTimeoutOverride(ms);
      Console.print("timeout override set to ");
      Console.print(ms);
      Console.println(" ms");
    }

  } else if (!strcmp(t0, "meminfo")) {
    Console.println();
    Console.printf("Total Heap:        %d bytes\n", rp2040.getTotalHeap());
    Console.printf("Free Heap:         %d bytes\n", rp2040.getFreeHeap());
    Console.printf("Used Heap:         %d bytes\n", rp2040.getUsedHeap());
    Console.printf("Total PSRAM Heap:  %d bytes\n", rp2040.getTotalPSRAMHeap());
    Console.printf("Free PSRAM Heap:   %d bytes\n", rp2040.getFreePSRAMHeap());
    Console.printf("Used PSRAM Heap:   %d bytes\n", rp2040.getUsedPSRAMHeap());
    Console.printf("Free Stack:        %d bytes\n", rp2040.getFreeStack());
    psramBB.printCapacityReport();

  } else if (!strcmp(t0, "psramsmoketest")) {
    psramBB.printCapacityReport();
    psramSafeSmokeTest();

  } else if (!strcmp(t0, "reboot")) {
    Console.printf("Rebooting..\n");
    delay(20);
    yield();
    rp2040.reboot();

  } else if (!strcmp(t0, "del")) {
    char* fn;
    if (!nextToken(p, fn)) {
      Console.println("usage: del <file>");
      return;
    }
    if (activeFs.deleteFile && activeFs.deleteFile(fn)) Console.println("deleted");
    else Console.println("delete failed");

  } else if (!strcmp(t0, "format")) {
    if (activeFs.format && activeFs.format()) Console.println("FS formatted");
    else Console.println("format failed");

  } else if (!strcmp(t0, "wipebootloader")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Console.println("Chip wiped, rebooting to bootloader now..");
      delay(20);
      yield();
      rp2040.rebootToBootloader();
    } else Console.println("wipe failed");

  } else if (!strcmp(t0, "wipereboot")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) {
      Console.println("Chip wiped, rebooting now..");
      delay(20);
      yield();
      rp2040.reboot();
    } else Console.println("wipe failed");

  } else if (!strcmp(t0, "wipe")) {
    Console.println("Erasing entire chip... this can take a while");
    if (activeFs.wipeChip && activeFs.wipeChip()) Console.println("Chip wiped");
    else Console.println("wipe failed");

  } else if (!strcmp(t0, "puthex")) {
    char* fn;
    char* hex;
    if (!nextToken(p, fn) || !nextToken(p, hex)) {
      Console.println("usage: puthex <file> <hex>");
      Console.println("tip: xxd -p -c 999999 your.bin | tr -d '\\n'");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeHexString(hex, bin, binLen)) {
      Console.println("puthex: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Console.print("puthex: wrote ");
      Console.print(binLen);
      Console.print(" bytes to ");
      Console.println(fn);
      if (binLen & 1u) Console.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
    } else Console.println("puthex: write failed");

  } else if (!strcmp(t0, "putb64")) {
    char* fn;
    char* b64;
    if (!nextToken(p, fn) || !nextToken(p, b64)) {
      Console.println("usage: putb64 <file> <base64>");
      Console.println("tip: base64 -w0 your.bin");
      return;
    }
    if (!checkNameLen(fn)) return;
    uint8_t* bin = nullptr;
    uint32_t binLen = 0;
    if (!decodeBase64String(b64, bin, binLen)) {
      Console.println("putb64: decode failed");
      return;
    }
    bool ok = writeBinaryToFS(fn, bin, binLen);
    free(bin);
    if (ok) {
      Console.print("putb64: wrote ");
      Console.print(binLen);
      Console.print(" bytes to ");
      Console.println(fn);
      if (binLen & 1u) Console.println("note: odd-sized file; if used as Thumb blob, exec will reject (needs even bytes).");
    } else Console.println("putb64: write failed");

  } else if (!strcmp(t0, "cc")) {
    char* src;
    char* dst;
    if (!nextToken(p, src) || !nextToken(p, dst)) {
      Console.println("usage: cc <src> <dst>");
      Console.println("example: cc myprog.c myprog.bin");
      return;
    }
    if (!compileTinyCFileToFile(src, dst)) {
      Console.println("cc: failed");
    }

  } else {
    Console.println("Unknown command. Type 'help'.");
  }
}

// ========== Setup and main loops ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(20); }
  delay(20);
  Console.println("System booting..");

  // Initialize Console
  Console.begin();

  // Initialize flash
  flashBB.begin();

  // PSRAM decoder + bus
  psramBB.configureDecoder138(PSRAM_NUM_CHIPS, PIN_138_EN, PIN_138_A0, PIN_138_A1, PIN_138_A2, /*enActiveLow=*/true);
  psramBB.begin();
  psramBB.setClockDelayUs(PSRAM_CLOCK_DELAY_US);

  // bind FS and mount
  bindActiveFs(g_storage);
  bool mounted = (g_storage == StorageBackend::Flash) ? activeFs.mount(true) : activeFs.mount(false);
  if (!mounted) { Console.println("FS mount failed on active storage"); }

  // Clear mailbox
  for (size_t i = 0; i < BLOB_MAILBOX_MAX; ++i) BLOB_MAILBOX[i] = 0;

  // ExecHost wiring
  Exec.attachConsole(&Console);
  updateExecFsTable();
  Exec.attachCoProc(&coprocLink, COPROC_BAUD);

  Console.printf("Controller serial link ready @ %u bps (RX=GP%u, TX=GP%u)\n", (unsigned)COPROC_BAUD, (unsigned)PIN_COPROC_RX, (unsigned)PIN_COPROC_TX);
  Console.printf("System ready. Type 'help'\n> ");
}
void setup1() {
  Exec.core1Setup();
}
void loop1() {
  Exec.core1Poll();
  tight_loop_contents();
}
void loop() {
  Exec.pollBackground();  // handle any background job completions/timeouts
  if (readLine()) {
    handleCommand(lineBuf);
    Console.print("> ");
  }
}