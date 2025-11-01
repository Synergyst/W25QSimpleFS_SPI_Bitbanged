#include <Arduino.h>
#include <SPI.h>
#include "UnifiedSPIMem.h"

// Pins: RP2040 Philhower core SPI1 mapping
const uint8_t PIN_FLASH_MISO = 12;  // GP12
const uint8_t PIN_PSRAM_MISO = 12;  // GP12
const uint8_t PIN_FLASH_MOSI = 11;  // GP11
const uint8_t PIN_PSRAM_MOSI = 11;  // GP11
const uint8_t PIN_FLASH_SCK = 10;   // GP10
const uint8_t PIN_PSRAM_SCK = 10;   // GP10
const uint8_t PIN_FLASH_CS = 9;     // GP9
const uint8_t PIN_PSRAM_CS0 = 14;   // GP14
const uint8_t PIN_PSRAM_CS1 = 15;   // GP15
const uint8_t PIN_PSRAM_CS2 = 26;   // GP26
const uint8_t PIN_PSRAM_CS3 = 27;   // GP27
const uint8_t PIN_NAND_CS = 28;     // GP28

UnifiedSpiMem::Manager mgr(PIN_PSRAM_SCK, PIN_PSRAM_MOSI, PIN_PSRAM_MISO);
UnifiedSpiMem::MemDevice* dev = nullptr;

static const uint32_t DUMP_COLS = 16;

void dumpHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i += DUMP_COLS) {
    Serial.printf("%04u: ", (unsigned)i);
    for (size_t j = 0; j < DUMP_COLS; ++j) {
      if (i + j < len) Serial.printf("%02X ", data[i + j]);
      else Serial.print("   ");
    }
    Serial.print(" |");
    for (size_t j = 0; j < DUMP_COLS && i + j < len; ++j) {
      char c = (char)data[i + j];
      Serial.print((c >= 32 && c <= 126) ? c : '.');
    }
    Serial.println("|");
  }
}

bool parseNumber(const String& s, uint32_t& out) {
  if (s.length() == 0) return false;
  char* endp = nullptr;
  char buf[32];
  s.substring(0, sizeof(buf) - 1).toCharArray(buf, sizeof(buf));
  if (strncasecmp(buf, "0x", 2) == 0) out = strtoul(buf + 2, &endp, 16);
  else out = strtoul(buf, &endp, 10);
  return true;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  I               - Ident info");
  Serial.println("  S               - Status/Features dump (MX35)");
  Serial.println("  E               - Erase block 0");
  Serial.println("  W <text...>     - Erase block 0 and write text at page 0, offset 0");
  Serial.println("  R <n>           - Read n bytes from 0x0000 and hex-dump");
  Serial.println("Example: W Hello NAND!");
  Serial.println("         R 64");
}

void cmdID() {
  size_t n = mgr.detectedCount();
  Serial.printf("Detected count: %u\r\n", (unsigned)n);
  for (size_t i = 0; i < n; ++i) {
    auto* di = mgr.detectedInfo(i);
    if (!di) continue;
    Serial.printf("  #%u: cs=%u type=%s vendor=%s cap=%llu bytes",
                  (unsigned)i, di->cs, UnifiedSpiMem::deviceTypeName(di->type),
                  di->vendorName, (unsigned long long)di->capacityBytes);
    if (di->type == UnifiedSpiMem::DeviceType::SpiNandMX35) {
      Serial.printf(" DID1=0x%02X DID2=0x%02X", di->did1, di->did2);
    }
    Serial.println();
  }
}

void cmdStatus() {
  if (!dev || dev->type() != UnifiedSpiMem::DeviceType::SpiNandMX35) {
    Serial.println("No NAND device open.");
    return;
  }
  auto* nd = static_cast<UnifiedSpiMem::MX35NandMemDevice*>(dev);
  uint8_t sr = nd->getFeature(0xC0);
  uint8_t prot = nd->getFeature(0xA0);
  uint8_t cfgb = nd->getFeature(0xB0);
  uint8_t cfg1 = nd->getFeature(0x10);
  uint8_t dsi0 = nd->getFeature(0xE0);
  Serial.printf("Status C0h = 0x%02X (OIP=%u WEL=%u PFAIL=%u EFAIL=%u ECC_S=%u%u CRBSY=%u)\r\n",
                sr,
                !!(sr & (1 << 0)), !!(sr & (1 << 1)), !!(sr & (1 << 3)), !!(sr & (1 << 2)),
                !!(sr & (1 << 5)), !!(sr & (1 << 4)), !!(sr & (1 << 7)));
  Serial.printf("Prot   A0h = 0x%02X\r\n", prot);
  Serial.printf("Cfg    B0h = 0x%02X\r\n", cfgb);
  Serial.printf("Cfg    10h = 0x%02X\r\n", cfg1);
  Serial.printf("DS_IO  E0h = 0x%02X\r\n", dsi0);
}

void cmdErase() {
  if (!dev) {
    Serial.println("No device open.");
    return;
  }
  uint32_t blkSize = dev->eraseSize();
  uint64_t addr = 0;
  Serial.printf("Erasing block 0... (addr=%llu size=%u)\r\n", (unsigned long long)addr, blkSize);
  if (!dev->eraseRange(addr, blkSize)) Serial.println("Erase FAILED");
  else Serial.println("Erase OK");
}

void cmdWrite(const String& payload) {
  if (!dev) {
    Serial.println("No device open.");
    return;
  }
  const uint32_t pageSize = dev->pageSize();
  static uint8_t pageBuf[8192];  // generous
  memset(pageBuf, 0xFF, pageSize);
  size_t n = payload.length();
  if (n > pageSize) n = pageSize;
  for (size_t i = 0; i < n; ++i) pageBuf[i] = (uint8_t)payload[i];

  Serial.println("Erasing block 0...");
  if (!dev->eraseRange(0, dev->eraseSize())) {
    Serial.println("Erase FAILED");
    return;
  }
  Serial.println("Programming page 0...");
  if (!dev->write(0, pageBuf, pageSize)) {
    Serial.println("Program FAILED");
    return;
  }
  // Verify
  memset(pageBuf, 0, pageSize);
  size_t got = dev->read(0, pageBuf, pageSize);
  if (got != pageSize) {
    Serial.println("Read-back FAILED");
    return;
  }
  Serial.println("First 64 bytes after write:");
  dumpHex(pageBuf, 64);
  Serial.println("Write DONE");
}

void cmdRead(const String& arg) {
  if (!dev) {
    Serial.println("No device open.");
    return;
  }
  uint32_t n = 0;
  if (!parseNumber(arg, n)) {
    Serial.println("R <n>   n can be decimal or 0xHEX");
    return;
  }
  const uint32_t pageSize = dev->pageSize();
  if (n > pageSize) n = pageSize;
  static uint8_t buf[8192];
  memset(buf, 0, sizeof(buf));
  size_t got = dev->read(0, buf, n);
  if (got == 0) {
    Serial.println("Read FAILED");
    return;
  }
  dumpHex(buf, got);
}

void processLine(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;
  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String arg = (sp < 0) ? "" : s.substring(sp + 1);
  cmd.toUpperCase();
  if (cmd == "I") cmdID();
  else if (cmd == "S") cmdStatus();
  else if (cmd == "E") cmdErase();
  else if (cmd == "W") cmdWrite(arg);
  else if (cmd == "R") cmdRead(arg);
  else if (cmd == "H" || cmd == "HELP" || cmd == "?") printHelp();
  else {
    Serial.println("Unknown command.");
    printHelp();
  }
}

void setup() {
  // Bind SPI1 pins
  SPI1.setSCK(PIN_FLASH_SCK);
  SPI1.setTX(PIN_FLASH_MOSI);
  SPI1.setRX(PIN_FLASH_MISO);
  SPI1.begin();

  Serial.begin(115200);
  while (!Serial) { /* wait for USB */
  }
  delay(5);

  mgr.begin();
  mgr.clearCsList();
  mgr.setCsList({ PIN_FLASH_CS, PIN_PSRAM_CS0, PIN_PSRAM_CS1, PIN_PSRAM_CS2, PIN_PSRAM_CS3, PIN_NAND_CS });
  mgr.scanSingle(PIN_NAND_CS, 1000000UL);  // identify at a safe low rate

  // Open NAND if present
  dev = mgr.openPreferred(UnifiedSpiMem::DeviceType::SpiNandMX35);
  if (!dev) {
    Serial.println("No MX35 SPI-NAND found on CS=28");
  } else {
    auto* nd = static_cast<UnifiedSpiMem::MX35NandMemDevice*>(dev);
    nd->setClock(20000000UL);
    Serial.printf("Opened NAND: capacity=%llu, pageSize=%u, eraseSize=%u, cs=%u\r\n",
                  (unsigned long long)nd->capacity(), nd->pageSize(), nd->eraseSize(), nd->cs());
  }

  cmdID();
  printHelp();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      processLine(line);
      line = "";
    } else {
      line += c;
      if (line.length() > 512) line = "";
    }
  }
}

/*#include <Arduino.h>
#include <SPI.h>

// RP2040/2350 Earle Philhower core: use SPI1 with your pins
// SCK=GP10, MOSI=GP11, MISO=GP12, CS=GP28
static const uint8_t FLASH_CS = 28;

// SPI settings: single I/O, Mode 0, start conservative
SPISettings flashSPI(20000000, MSBFIRST, SPI_MODE0);  // 20 MHz

// Opcodes
enum {
  CMD_RESET = 0xFF,
  CMD_READ_ID = 0x9F,
  CMD_WREN = 0x06,
  CMD_WRDI = 0x04,
  CMD_GET_FEATURE = 0x0F,
  CMD_SET_FEATURE = 0x1F,
  CMD_PAGE_READ = 0x13,
  CMD_READ_CACHE = 0x03,  // requires 1 dummy byte on MX35LFxGE4AD [1]
  CMD_PROG_LOAD = 0x02,
  CMD_PROG_EXEC = 0x10,
  CMD_BLOCK_ERASE = 0xD8,
  CMD_RDSR = 0x05
};

// Feature addresses
enum {
  FEAT_ADDR_CFG_10h = 0x10,     // CONT/BFT/ENPGM
  FEAT_ADDR_PROT_A0h = 0xA0,    // Block protection
  FEAT_ADDR_CFG_B0h = 0xB0,     // OTP/ECC/QE
  FEAT_ADDR_STATUS_C0h = 0xC0,  // Status
  FEAT_ADDR_DSIO_E0h = 0xE0     // Drive strength
};

// Status masks (C0h / RDSR)
static const uint8_t SR_OIP = 1 << 0;  // Operation in progress
static const uint8_t SR_WEL = 1 << 1;
static const uint8_t SR_EFAIL = 1 << 2;
static const uint8_t SR_PFAIL = 1 << 3;
static const uint8_t SR_ECC_S0 = 1 << 4;
static const uint8_t SR_ECC_S1 = 1 << 5;
static const uint8_t SR_BBMT_F = 1 << 6;
static const uint8_t SR_CRBSY = 1 << 7;

static inline void csLow() {
  digitalWrite(FLASH_CS, LOW);
}
static inline void csHigh() {
  digitalWrite(FLASH_CS, HIGH);
}

// SPI helpers (use SPI1 everywhere)
uint8_t getFeature(uint8_t addr) {
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_GET_FEATURE);
  SPI1.transfer(addr);
  uint8_t val = SPI1.transfer(0x00);
  csHigh();
  SPI1.endTransaction();
  return val;
}

void setFeature(uint8_t addr, uint8_t val) {
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_SET_FEATURE);
  SPI1.transfer(addr);
  SPI1.transfer(val);
  csHigh();
  SPI1.endTransaction();
}

uint8_t readStatus() {
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_RDSR);
  uint8_t v = SPI1.transfer(0x00);
  csHigh();
  SPI1.endTransaction();
  return v;
}

void writeEnable() {
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_WREN);
  csHigh();
  SPI1.endTransaction();
}

void resetChip() {
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_RESET);
  csHigh();
  SPI1.endTransaction();
  delayMicroseconds(10);  // tRST for idle/read ~6us, add margin [1]
}

bool waitReady(uint32_t timeout_us, uint8_t* outStatus = nullptr) {
  uint32_t start = micros();
  while (true) {
    uint8_t sr = readStatus();
    if ((sr & SR_OIP) == 0) {
      if (outStatus) *outStatus = sr;
      return true;
    }
    if ((micros() - start) > timeout_us) {
      if (outStatus) *outStatus = sr;
      return false;
    }
    delayMicroseconds(5);
  }
}

// Clear block protection so we can program/erase after power-up [1]
void unlockAll() {
  // A0h: write 0x00 => all unlocked; ensure WP# is high on your board
  setFeature(FEAT_ADDR_PROT_A0h, 0x00);
}

// 4Gb geometry: 64 pages/block, 4096+128 bytes per page [1]
static const uint32_t PAGES_PER_BLOCK = 64;
static const uint16_t PAGE_MAIN_SIZE = 4096;

// Row address for (block, page)
uint32_t makeRowAddr(uint32_t block, uint32_t pageInBlock) {
  return (block << 6) | (pageInBlock & 0x3F);
}

// Move array->cache for a page (13h), then poll ready [1]
bool pageReadToCache(uint32_t rowAddr) {
  uint8_t R2 = (uint8_t)((rowAddr >> 16) & 0x01);  // only RA[16] used
  uint8_t R1 = (uint8_t)((rowAddr >> 8) & 0xFF);
  uint8_t R0 = (uint8_t)(rowAddr & 0xFF);

  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_PAGE_READ);
  SPI1.transfer(R2);
  SPI1.transfer(R1);
  SPI1.transfer(R0);
  csHigh();
  SPI1.endTransaction();

  uint8_t sr = 0;
  if (!waitReady(2000, &sr)) return false;  // tRD typ 110us; 2ms timeout
  return true;
}

// Read from cache (x1) starting at column; requires 1 dummy byte even for 0x03 [1]
bool readCache(uint16_t column, uint8_t* buf, size_t len) {
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_READ_CACHE);
  SPI1.transfer((uint8_t)((column >> 8) & 0xFF));
  SPI1.transfer((uint8_t)(column & 0xFF));
  SPI1.transfer(0x00);  // dummy
  for (size_t i = 0; i < len; ++i) buf[i] = SPI1.transfer(0x00);
  csHigh();
  SPI1.endTransaction();
  return true;
}

bool readPageMain(uint32_t block, uint32_t pageInBlock, uint8_t* buf4096) {
  uint32_t row = makeRowAddr(block, pageInBlock);
  if (!pageReadToCache(row)) return false;
  return readCache(0x0000, buf4096, PAGE_MAIN_SIZE);
}

// Program main area of a page, starting at column 0
bool programPageMain(uint32_t block, uint32_t pageInBlock, const uint8_t* data4096) {
  writeEnable();
  // PROGRAM LOAD (02h) at column 0
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_PROG_LOAD);
  SPI1.transfer(0x00);  // column high
  SPI1.transfer(0x00);  // column low
  for (size_t i = 0; i < PAGE_MAIN_SIZE; ++i) {
    SPI1.transfer(data4096[i]);
  }
  csHigh();
  SPI1.endTransaction();

  // PROGRAM EXECUTE (10h)
  uint32_t row = makeRowAddr(block, pageInBlock);
  uint8_t R2 = (uint8_t)((row >> 16) & 0x01);
  uint8_t R1 = (uint8_t)((row >> 8) & 0xFF);
  uint8_t R0 = (uint8_t)(row & 0xFF);

  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_PROG_EXEC);
  SPI1.transfer(R2);
  SPI1.transfer(R1);
  SPI1.transfer(R0);
  csHigh();
  SPI1.endTransaction();

  uint8_t sr = 0;
  if (!waitReady(6000, &sr)) return false;  // tPROG typ 400us; 6ms timeout
  if (sr & SR_PFAIL) return false;
  return true;
}

// Erase any page-containing block (D8h)
bool eraseBlock(uint32_t block) {
  writeEnable();
  uint32_t row = makeRowAddr(block, 0);
  uint8_t R2 = (uint8_t)((row >> 16) & 0x01);
  uint8_t R1 = (uint8_t)((row >> 8) & 0xFF);
  uint8_t R0 = (uint8_t)(row & 0xFF);

  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_BLOCK_ERASE);
  SPI1.transfer(R2);
  SPI1.transfer(R1);
  SPI1.transfer(R0);
  csHigh();
  SPI1.endTransaction();

  uint8_t sr = 0;
  if (!waitReady(120000, &sr)) return false;  // tERS typ 4ms; 120ms timeout
  if (sr & SR_EFAIL) return false;
  return true;
}

bool readJedecID(uint8_t& mid, uint8_t& did1, uint8_t& did2) {
  SPI1.beginTransaction(flashSPI);
  csLow();
  SPI1.transfer(CMD_READ_ID);
  SPI1.transfer(0x00);  // dummy
  mid = SPI1.transfer(0x00);
  did1 = SPI1.transfer(0x00);
  did2 = SPI1.transfer(0x00);
  csHigh();
  SPI1.endTransaction();
  return true;
}

// ---------- Simple CLI ----------

static const uint32_t TEST_BLOCK = 0;  // "Beginning of flash" as requested
static const uint32_t TEST_PAGE = 0;

uint8_t pageBuf[PAGE_MAIN_SIZE];

void dumpHex(const uint8_t* data, size_t len) {
  const size_t cols = 16;
  for (size_t i = 0; i < len; i += cols) {
    Serial.printf("%04u: ", (unsigned)i);
    for (size_t j = 0; j < cols; ++j) {
      if (i + j < len) Serial.printf("%02X ", data[i + j]);
      else Serial.print("   ");
    }
    Serial.print(" |");
    for (size_t j = 0; j < cols && i + j < len; ++j) {
      char c = (char)data[i + j];
      Serial.print((c >= 32 && c <= 126) ? c : '.');
    }
    Serial.println("|");
  }
}

bool parseNumber(const String& s, uint32_t& out) {
  if (s.length() == 0) return false;
  char* endp = nullptr;
  // Copy to C-string
  char buf[32];
  s.substring(0, sizeof(buf) - 1).toCharArray(buf, sizeof(buf));
  if (strncasecmp(buf, "0x", 2) == 0) {
    out = strtoul(buf + 2, &endp, 16);
  } else {
    out = strtoul(buf, &endp, 10);
  }
  return true;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  I               - JEDEC ID");
  Serial.println("  S               - Status/Features dump");
  Serial.println("  E               - Erase block 0");
  Serial.println("  W <text...>     - Erase block 0 and write text at page 0, offset 0");
  Serial.println("  R <n>           - Read n bytes from 0x0000 and hex-dump");
  Serial.println("Example: W Hello NAND!");
  Serial.println("         R 64");
}

void cmdID() {
  uint8_t mid, d1, d2;
  readJedecID(mid, d1, d2);
  Serial.printf("JEDEC: MID=0x%02X DID1=0x%02X DID2=0x%02X\r\n", mid, d1, d2);
}

void cmdStatus() {
  uint8_t sr = getFeature(FEAT_ADDR_STATUS_C0h);
  uint8_t prot = getFeature(FEAT_ADDR_PROT_A0h);
  uint8_t cfgb0 = getFeature(FEAT_ADDR_CFG_B0h);
  uint8_t cfg10 = getFeature(FEAT_ADDR_CFG_10h);
  uint8_t dsi0 = getFeature(FEAT_ADDR_DSIO_E0h);

  Serial.printf("Status C0h = 0x%02X (OIP=%u WEL=%u PFAIL=%u EFAIL=%u ECC_S=%u%u CRBSY=%u)\r\n",
                sr, !!(sr & SR_OIP), !!(sr & SR_WEL), !!(sr & SR_PFAIL), !!(sr & SR_EFAIL),
                !!(sr & SR_ECC_S1), !!(sr & SR_ECC_S0), !!(sr & SR_CRBSY));
  Serial.printf("Prot   A0h = 0x%02X\r\n", prot);
  Serial.printf("Cfg    B0h = 0x%02X\r\n", cfgb0);
  Serial.printf("Cfg    10h = 0x%02X\r\n", cfg10);
  Serial.printf("DS_IO  E0h = 0x%02X\r\n", dsi0);
}

void cmdErase() {
  Serial.println("Erasing block 0...");
  if (!eraseBlock(TEST_BLOCK)) Serial.println("Erase FAILED");
  else Serial.println("Erase OK");
}

void cmdWrite(const String& payload) {
  // Fill page buffer with 0xFF, copy string at start
  memset(pageBuf, 0xFF, sizeof(pageBuf));
  size_t n = payload.length();
  if (n > PAGE_MAIN_SIZE) n = PAGE_MAIN_SIZE;
  for (size_t i = 0; i < n; ++i) pageBuf[i] = (uint8_t)payload[i];

  Serial.println("Erasing block 0...");
  if (!eraseBlock(TEST_BLOCK)) {
    Serial.println("Erase FAILED");
    return;
  }

  Serial.println("Programming page 0...");
  if (!programPageMain(TEST_BLOCK, TEST_PAGE, pageBuf)) {
    Serial.println("Program FAILED");
    return;
  }

  // Verify
  uint8_t rx[64];
  memset(rx, 0, sizeof(rx));
  if (!readPageMain(TEST_BLOCK, TEST_PAGE, pageBuf)) {
    Serial.println("Read-back FAILED");
    return;
  }
  Serial.println("First 64 bytes after write:");
  dumpHex(pageBuf, 64);
  Serial.println("Write DONE");
}

void cmdRead(const String& arg) {
  uint32_t n = 0;
  if (!parseNumber(arg, n)) {
    Serial.println("R <n>   n can be decimal or 0xHEX");
    return;
  }
  if (n > PAGE_MAIN_SIZE) n = PAGE_MAIN_SIZE;

  if (!readPageMain(TEST_BLOCK, TEST_PAGE, pageBuf)) {
    Serial.println("Read FAILED");
    return;
  }
  dumpHex(pageBuf, n);
}

void processLine(const String& line) {
  // Trim CR/LF/spaces
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  int sp = s.indexOf(' ');
  String cmd = (sp < 0) ? s : s.substring(0, sp);
  String arg = (sp < 0) ? "" : s.substring(sp + 1);

  cmd.toUpperCase();

  if (cmd == "I") cmdID();
  else if (cmd == "S") cmdStatus();
  else if (cmd == "E") cmdErase();
  else if (cmd == "W") cmdWrite(arg);
  else if (cmd == "R") cmdRead(arg);
  else if (cmd == "H" || cmd == "HELP" || cmd == "?") printHelp();
  else {
    Serial.println("Unknown command.");
    printHelp();
  }
}

void setup() {
  pinMode(FLASH_CS, OUTPUT);
  csHigh();

  // Map SPI1 pins (GP10 SCK, GP11 MOSI, GP12 MISO)
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  SPI1.begin();

  Serial.begin(115200);
  while (!Serial) { }
  delay(5);  // tVSL; VCC(min) to operation [1]

  resetChip();
  unlockAll();  // make sure it's writable after power-up [1]

  // Print ID
  cmdID();
  printHelp();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      processLine(line);
      line = "";
    } else {
      line += c;
      if (line.length() > 512) line = "";  // simple guard
    }
  }
}*/