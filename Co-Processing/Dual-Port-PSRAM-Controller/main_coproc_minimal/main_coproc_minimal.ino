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
volatile uint8_t g_cancel_flag = 0;

// ------- ISP mode state -------
static volatile bool g_isp_active = false;
static inline void serviceISPIfActive() {
  if (g_isp_active) {
    ArbiterISP::serviceISPOnce();
  }
}

// ------- Executor and script handling (moved to header-only class) -------
#include "CoProcExec.h"
static CoProcExec g_exec;

// ------- Transport / protocol buffers -------
static CoProc::Frame g_reqHdr, g_respHdr;
static uint8_t* g_reqBuf = nullptr;
static uint8_t* g_respBuf = nullptr;
static const uint32_t RESP_MAX = 8192;
static const uint32_t REQ_MAX = 8192;

// ------- Serial byte helpers -------
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
    serviceISPIfActive();
    tight_loop_contents();
    yield();
  }
  return false;
}
static bool readExact(uint8_t* dst, size_t n, uint32_t timeoutPerByteMs) {
  for (size_t i = 0; i < n; ++i) {
    if (!readByte(dst[i], timeoutPerByteMs)) return false;
  }
  return true;
}
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
  link.flush();
  return true;
}

// ------- ISP handlers (unchanged) -------
static int32_t handleISP_ENTER(const uint8_t* in, size_t len, uint8_t* out, size_t cap, size_t& off) {
  (void)in;
  (void)len;
  if (g_isp_active) {
    int32_t st = CoProc::ST_STATE;
    CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
    DBG("[DBG] ISP_ENTER: already active\n");
#endif
    return st;
  }
  ArbiterISP::initTestPins();
  ArbiterISP::enterISPMode();
  g_isp_active = true;
  int32_t st = CoProc::ST_OK;
  CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
  DBG("[DBG] ISP_ENTER: entered ISP mode\n");
#endif
  return st;
}
static int32_t handleISP_EXIT(uint8_t* out, size_t cap, size_t& off) {
  if (!g_isp_active) {
    int32_t st = CoProc::ST_STATE;
    CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
    DBG("[DBG] ISP_EXIT: not active\n");
#endif
    return st;
  }
  ArbiterISP::exitISPMode();
  ArbiterISP::cleanupToResetState();
  g_isp_active = false;
  int32_t st = CoProc::ST_OK;
  CoProc::writePOD(out, cap, off, st);
#if COPROC_DEBUG
  DBG("[DBG] ISP_EXIT: exited ISP mode\n");
#endif
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
    case CoProc::CMD_FUNC: return "FUNC";
    case CoProc::CMD_ISP_ENTER: return "ISP_ENTER";
    case CoProc::CMD_ISP_EXIT: return "ISP_EXIT";
  }
  return "UNKNOWN";
}

// Build response from request (delegates exec/script commands to CoProcExec)
static void processRequest(const CoProc::Frame& hdr, const uint8_t* payload,
                           CoProc::Frame& respH, uint8_t* respBuf, uint32_t& respLen) {
  size_t off = 0;
  int32_t st = CoProc::ST_BAD_CMD;
  respLen = 0;
#if COPROC_DEBUG
  DBG("[DBG] CMD=%s (0x%02X) seq=%u len=%u\n",
      cmdName(hdr.cmd), (unsigned)hdr.cmd, (unsigned)hdr.seq, (unsigned)hdr.len);
#endif
  switch (hdr.cmd) {
    // Binary pipeline (delegated)
    case CoProc::CMD_HELLO:
      st = g_exec.cmdHELLO(g_isp_active, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_INFO:
      st = g_exec.cmdINFO(g_isp_active, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_LOAD_BEGIN:
      st = g_exec.cmdLOAD_BEGIN(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_LOAD_DATA:
      st = g_exec.cmdLOAD_DATA(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_LOAD_END:
      st = g_exec.cmdLOAD_END(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_EXEC:
      st = g_exec.cmdEXEC(payload, hdr.len, respBuf, RESP_MAX, off);
      break;

    // Housekeeping (delegated)
    case CoProc::CMD_STATUS:
      st = g_exec.cmdSTATUS(respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_MAILBOX_RD:
      st = g_exec.cmdMAILBOX_RD(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_CANCEL:
      st = g_exec.cmdCANCEL(respBuf, RESP_MAX, off);
      break;

    case CoProc::CMD_RESET:
#if COPROC_DEBUG
      DBG("[DBG] RESET\n");
#endif
      delay(10);
      watchdog_reboot(0, 0, 1500);
      st = CoProc::ST_OK;
      break;

    // Script pipeline (delegated)
    case CoProc::CMD_SCRIPT_BEGIN:
      st = g_exec.cmdSCRIPT_BEGIN(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_SCRIPT_DATA:
      st = g_exec.cmdSCRIPT_DATA(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_SCRIPT_END:
      st = g_exec.cmdSCRIPT_END(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_SCRIPT_EXEC:
      st = g_exec.cmdSCRIPT_EXEC(payload, hdr.len, respBuf, RESP_MAX, off);
      break;

    // Named function dispatch (new)
    case CoProc::CMD_FUNC:
      st = g_exec.cmdFUNC(payload, hdr.len, respBuf, RESP_MAX, off);
      break;

    // ISP control (kept local)
    case CoProc::CMD_ISP_ENTER:
      st = handleISP_ENTER(payload, hdr.len, respBuf, RESP_MAX, off);
      break;
    case CoProc::CMD_ISP_EXIT:
      st = handleISP_EXIT(respBuf, RESP_MAX, off);
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
  const uint8_t magicBytes[4] = { (uint8_t)('C'), (uint8_t)('P'), (uint8_t)('R'), (uint8_t)('0') };
  uint8_t w[4] = { 0, 0, 0, 0 };
  uint32_t lastActivity = millis();

  for (;;) {
    uint8_t b;
    if (readByte(b, 50)) {
      w[0] = w[1];
      w[1] = w[2];
      w[2] = w[3];
      w[3] = b;
      lastActivity = millis();
      if (w[0] == magicBytes[0] && w[1] == magicBytes[1] && w[2] == magicBytes[2] && w[3] == magicBytes[3]) {
        union {
          CoProc::Frame f;
          uint8_t bytes[sizeof(CoProc::Frame)];
        } u;
        memcpy(u.bytes, w, 4);
        if (!readExact(u.bytes + 4, sizeof(CoProc::Frame) - 4, 200)) {
          DBG("[COPROC] header tail timeout\n");
          return false;
        }
        hdr = u.f;
        if ((hdr.magic != CoProc::MAGIC) || (hdr.version != CoProc::VERSION)) {
          DBG("[COPROC] bad header magic=0x%08X ver=0x%04X\n", (unsigned)hdr.magic, (unsigned)hdr.version);
          return false;
        }
        if (hdr.len > REQ_MAX) {
          DBG("[COPROC] req too large (%u > %u), draining\n", (unsigned)hdr.len, (unsigned)REQ_MAX);
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
          size_t off = 0;
          int32_t st = CoProc::ST_SIZE;
          CoProc::writePOD(g_respBuf, RESP_MAX, off, st);
          uint32_t crc = CoProc::crc32_ieee(g_respBuf, off);
          CoProc::makeResponseHeader(g_respHdr, hdr.cmd, hdr.seq, (uint32_t)off, crc);
          writeAll(reinterpret_cast<const uint8_t*>(&g_respHdr), sizeof(CoProc::Frame), 2000);
          if (off) writeAll(g_respBuf, off, 5000);
          return false;
        }
        if (hdr.len) {
          if (!readExact(payloadBuf, hdr.len, 200)) {
            DBG("[COPROC] payload read timeout\n");
            return false;
          }
          uint32_t crc = CoProc::crc32_ieee(payloadBuf, hdr.len);
          if (crc != hdr.crc32) {
            DBG("[COPROC] CRC mismatch exp=0x%08X got=0x%08X\n", (unsigned)hdr.crc32, (unsigned)crc);
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
        return true;
      }
    } else {
      if ((millis() - lastActivity) > 1000) {
        lastActivity = millis();
      }
      serviceISPIfActive();
      tight_loop_contents();
      yield();
    }
  }
}

// ------- Core0: serial protocol loop -------
static void protocolLoop() {
  for (;;) {
    serviceISPIfActive();
    if (!readFramedRequest(g_reqHdr, g_reqBuf)) {
      continue;
    }
    uint32_t respLen = 0;
    processRequest(g_reqHdr, g_reqBuf, g_respHdr, g_respBuf, respLen);
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
    serviceISPIfActive();
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

// Example user function implementations
static int32_t fn_ping(const int32_t*, uint32_t) {
  return 1234;
}
static int32_t fn_add(const int32_t* a, uint32_t n) {
  int64_t sum = 0;
  for (uint32_t i = 0; i < n; ++i) sum += a[i];
  return (int32_t)sum;
}

// ========== Setup and main ==========
void setup() {
  Serial.begin();
  delay(5000);
  if (!Serial) delay(1000);

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
  bool ok = ArbiterISP::runTestSuiteOnce(true, true);
  ArbiterISP::cleanupToResetState();
  if (!ok) {
    Serial.println("P.O.S.T. failed!");
    /*Serial.println("Entering ISP mode, please exit serial console and reprogram MCU!");
    ArbiterISP::enterISPMode();
    while (!BOOTSEL) ArbiterISP::serviceISPOnce();  // inside loop
    delay(2000);
    ArbiterISP::exitISPMode();
    Serial.println("Exited ISP mode..");*/
    Serial.println("Are we possibly testing the 74HC32 emulator..?");
    bool ok2 = ArbiterISP::runHC32POST(true);  // verbose = true
    Serial.println(ok2 ? F("[74HC32] DUT PASS") : F("[74HC32] DUT FAIL"));
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
  g_exec.begin();

  // Register named functions for CMD_FUNC
  g_exec.registerFunc("ping", 0, fn_ping);  // expects no args
  g_exec.registerFunc("add", -1, fn_add);   // accepts any arg count

  Serial.printf("CoProc ready (soft-serial %u bps). RX=GP%u TX=GP%u\n", (unsigned)SOFT_BAUD, (unsigned)PIN_RX, (unsigned)PIN_TX);
}

void loop() {
  protocolLoop();  // never returns
}

// setup1/loop1 for core1 (Philhower)
void setup1() { /* nothing */
}
void loop1() {
  g_exec.workerPoll();
  tight_loop_contents();
}
