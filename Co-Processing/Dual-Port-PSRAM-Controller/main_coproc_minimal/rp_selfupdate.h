#pragma once
/*
  rp_selfupdate.h - Minimal RP2040/RP2350 self-programmer from external SPI storage.
  - External storage backends:
      RPUPD_BACKEND_W25Q    (Winbond W25Q SPI NOR flash)
      RPUPD_BACKEND_PSRAM   (Basic/assumed SPI PSRAM using 0x02 write, 0x03 read)
  - SPI defaults: SPI1 on GP10..13 (Pico-friendly). Override with macros below.
  - No headers/CRC: you provide exact payload length.
  - Programs internal flash in 4KB sectors, 256B pages.
  - Program function runs critical flash ops from RAM with interrupts disabled.

  Tested with Arduino-Pico (Earle Philhower) core. Requires RP2040 or RP2350.
  You may need to enable/keep the Pico SDK headers exposed in your core version.

  Configuration macros (override before including this header):
    #define RPUPD_SPI_INSTANCE   1           // 0 -> SPI, 1 -> SPI1
    #define RPUPD_PIN_SCK        10
    #define RPUPD_PIN_MOSI       11
    #define RPUPD_PIN_MISO       12
    #define RPUPD_PIN_CS         13
    #define RPUPD_BACKEND        RPUPD_BACKEND_W25Q
    #define RPUPD_FLASH_TARGET_OFFSET  0x00000000u
    #define RPUPD_ONBOARD_FLASH_SIZE   (2u * 1024u * 1024u) // adjust for your board
*/

#ifndef ARDUINO_ARCH_RP2040
#if !defined(PICO_RP2040) && !defined(PICO_RP2350)
#error "This header targets RP2040/RP2350 (Arduino-Pico) only."
#endif
#endif

#include <Arduino.h>
#include <SPI.h>

extern "C" {
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
}

#ifndef RPUPD_SPI_INSTANCE
#define RPUPD_SPI_INSTANCE 1
#endif

#ifndef RPUPD_PIN_SCK
#define RPUPD_PIN_SCK 10
#endif
#ifndef RPUPD_PIN_MOSI
#define RPUPD_PIN_MOSI 11
#endif
#ifndef RPUPD_PIN_MISO
#define RPUPD_PIN_MISO 12
#endif
#ifndef RPUPD_PIN_CS
#define RPUPD_PIN_CS 13
#endif

// Choose backend: RPUPD_BACKEND_W25Q or RPUPD_BACKEND_PSRAM
#define RPUPD_BACKEND_W25Q 1
#define RPUPD_BACKEND_PSRAM 2
#ifndef RPUPD_BACKEND
#define RPUPD_BACKEND RPUPD_BACKEND_W25Q
#endif

#ifndef RPUPD_FLASH_TARGET_OFFSET
#define RPUPD_FLASH_TARGET_OFFSET 0x00000000u
#endif

#ifndef RPUPD_ONBOARD_FLASH_SIZE
#define RPUPD_ONBOARD_FLASH_SIZE (2u * 1024u * 1024u)
#endif

// Flash geometry
static constexpr size_t RPUPD_FLASH_SECTOR_SIZE = 4096;
static constexpr size_t RPUPD_FLASH_PAGE_SIZE = 256;

// One 4KB RAM buffer for staging
static uint8_t rpupd_sector_buf[RPUPD_FLASH_SECTOR_SIZE];

// Select SPI instance
#if RPUPD_SPI_INSTANCE == 0
#define RPUPD_SPI SPI
#else
#define RPUPD_SPI SPI1
#endif

// ---------- Backend: W25Q (Winbond) ----------
#if RPUPD_BACKEND == RPUPD_BACKEND_W25Q
namespace rpupd_w25q {
enum : uint8_t {
  CMD_WREN = 0x06,
  CMD_WRDI = 0x04,
  CMD_RDSR1 = 0x05,
  CMD_RDID = 0x9F,
  CMD_READ = 0x03,
  CMD_PP = 0x02,
  CMD_SECTOR_ERASE = 0x20,  // 4KB
  CMD_CHIP_ERASE = 0xC7
};

static inline void cs_low() {
  digitalWrite(RPUPD_PIN_CS, LOW);
}
static inline void cs_high() {
  digitalWrite(RPUPD_PIN_CS, HIGH);
}

static inline void cmd(uint8_t c) {
  RPUPD_SPI.transfer(c);
}

static void wren() {
  cs_low();
  cmd(CMD_WREN);
  cs_high();
}

static uint8_t rdsr1() {
  cs_low();
  cmd(CMD_RDSR1);
  uint8_t v = RPUPD_SPI.transfer(0x00);
  cs_high();
  return v;
}

static void wait_while_busy() {
  // WIP bit (bit0) set while busy
  while (rdsr1() & 0x01) { tight_loop_contents(); }
}

static uint32_t read_jedec() {
  cs_low();
  cmd(CMD_RDID);
  uint8_t mfr = RPUPD_SPI.transfer(0x00);
  uint8_t memt = RPUPD_SPI.transfer(0x00);
  uint8_t cap = RPUPD_SPI.transfer(0x00);
  cs_high();
  return (uint32_t(mfr) << 16) | (uint32_t(memt) << 8) | cap;
}

static void read(uint32_t addr, uint8_t* buf, size_t len) {
  cs_low();
  cmd(CMD_READ);
  RPUPD_SPI.transfer(uint8_t((addr >> 16) & 0xFF));
  RPUPD_SPI.transfer(uint8_t((addr >> 8) & 0xFF));
  RPUPD_SPI.transfer(uint8_t(addr & 0xFF));
  for (size_t i = 0; i < len; ++i) buf[i] = RPUPD_SPI.transfer(0x00);
  cs_high();
}

static void sector_erase(uint32_t addr_4k_aligned) {
  wren();
  cs_low();
  cmd(CMD_SECTOR_ERASE);
  RPUPD_SPI.transfer(uint8_t((addr_4k_aligned >> 16) & 0xFF));
  RPUPD_SPI.transfer(uint8_t((addr_4k_aligned >> 8) & 0xFF));
  RPUPD_SPI.transfer(uint8_t(addr_4k_aligned & 0xFF));
  cs_high();
  wait_while_busy();
}

static void page_program(uint32_t addr, const uint8_t* data, size_t len) {
  // len <= 256, must not cross 256-byte page boundary
  wren();
  cs_low();
  cmd(CMD_PP);
  RPUPD_SPI.transfer(uint8_t((addr >> 16) & 0xFF));
  RPUPD_SPI.transfer(uint8_t((addr >> 8) & 0xFF));
  RPUPD_SPI.transfer(uint8_t(addr & 0xFF));
  for (size_t i = 0; i < len; ++i) RPUPD_SPI.transfer(data[i]);
  cs_high();
  wait_while_busy();
}

static void write_stream(uint32_t addr, const uint8_t* data, size_t len) {
  // writes in 256-byte page chunks
  size_t i = 0;
  while (i < len) {
    uint32_t page_off = addr & 0xFF;
    size_t space = 256 - page_off;
    size_t n = (len - i < space) ? (len - i) : space;
    page_program(addr, data + i, n);
    addr += n;
    i += n;
  }
}
}
#endif  // W25Q

// ---------- Backend: SPI PSRAM (very basic, adjust for your chip) ----------
#if RPUPD_BACKEND == RPUPD_BACKEND_PSRAM
namespace rpupd_psram {
// Many SPI PSRAMs use 0x02 write / 0x03 read, 24-bit addressing in SPI mode.
// Adjust as needed for your part (some require mode register setup or QPI).
enum : uint8_t { CMD_WRITE = 0x02,
                 CMD_READ = 0x03 };

static inline void cs_low() {
  digitalWrite(RPUPD_PIN_CS, LOW);
}
static inline void cs_high() {
  digitalWrite(RPUPD_PIN_CS, HIGH);
}

static void read(uint32_t addr, uint8_t* buf, size_t len) {
  cs_low();
  RPUPD_SPI.transfer(CMD_READ);
  RPUPD_SPI.transfer(uint8_t((addr >> 16) & 0xFF));
  RPUPD_SPI.transfer(uint8_t((addr >> 8) & 0xFF));
  RPUPD_SPI.transfer(uint8_t(addr & 0xFF));
  for (size_t i = 0; i < len; ++i) buf[i] = RPUPD_SPI.transfer(0x00);
  cs_high();
}

static void write(uint32_t addr, const uint8_t* data, size_t len) {
  // No page constraints on many PSRAMs, but keep chunks reasonable
  cs_low();
  RPUPD_SPI.transfer(CMD_WRITE);
  RPUPD_SPI.transfer(uint8_t((addr >> 16) & 0xFF));
  RPUPD_SPI.transfer(uint8_t((addr >> 8) & 0xFF));
  RPUPD_SPI.transfer(uint8_t(addr & 0xFF));
  for (size_t i = 0; i < len; ++i) RPUPD_SPI.transfer(data[i]);
  cs_high();
}
}
#endif  // PSRAM

// ---------- Common init ----------
static inline void rpupd_begin(uint32_t spi_hz = 24'000'000) {
  pinMode(RPUPD_PIN_CS, OUTPUT);
  digitalWrite(RPUPD_PIN_CS, HIGH);
  RPUPD_SPI.setSCK(RPUPD_PIN_SCK);
  RPUPD_SPI.setTX(RPUPD_PIN_MOSI);
  RPUPD_SPI.setRX(RPUPD_PIN_MISO);
  RPUPD_SPI.begin();
  RPUPD_SPI.beginTransaction(SPISettings(spi_hz, MSBFIRST, SPI_MODE0));
#if RPUPD_BACKEND == RPUPD_BACKEND_W25Q
  // Optional: sanity print JEDEC (Winbond mfr=0xEF)
  uint32_t jedec = rpupd_w25q::read_jedec();
  (void)jedec;
#endif
}

// Optional helper for RP2350 PICO_NO_FLASH boards (Arduino rarely uses this)
static inline void rpupd_connect_internal_flash_if_needed() {
#if defined(PICO_RP2350) && defined(PICO_NO_FLASH) && PICO_NO_FLASH
  __compiler_memory_barrier();
  reinterpret_cast<void (*)(void)>(rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH))();
  reinterpret_cast<void (*)(void)>(rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE))();
  __compiler_memory_barrier();
#endif
}

// ---------- External store: read/write/wipe ----------
static inline void rpupd_ext_read(uint32_t addr, uint8_t* buf, size_t len) {
#if RPUPD_BACKEND == RPUPD_BACKEND_W25Q
  rpupd_w25q::read(addr, buf, len);
#elif RPUPD_BACKEND == RPUPD_BACKEND_PSRAM
  rpupd_psram::read(addr, buf, len);
#endif
}

static inline void rpupd_ext_write(uint32_t addr, const uint8_t* data, size_t len) {
#if RPUPD_BACKEND == RPUPD_BACKEND_W25Q
  rpupd_w25q::write_stream(addr, data, len);
#elif RPUPD_BACKEND == RPUPD_BACKEND_PSRAM
  rpupd_psram::write(addr, data, len);
#endif
}

// Wipe/prepare external store [0..bytes) to receive new payload starting at 0x00
// For W25Q: erases the necessary 4KB sectors. For PSRAM: fills with 0xFF.
static inline void rpupd_ext_wipe(uint32_t bytes) {
#if RPUPD_BACKEND == RPUPD_BACKEND_W25Q
  uint32_t end = (bytes + RPUPD_FLASH_SECTOR_SIZE - 1) & ~(RPUPD_FLASH_SECTOR_SIZE - 1);
  for (uint32_t a = 0; a < end; a += RPUPD_FLASH_SECTOR_SIZE) {
    rpupd_w25q::sector_erase(a);
  }
#elif RPUPD_BACKEND == RPUPD_BACKEND_PSRAM
  memset(rpupd_sector_buf, 0xFF, sizeof(rpupd_sector_buf));
  uint32_t remaining = bytes;
  uint32_t addr = 0;
  while (remaining) {
    size_t n = remaining > sizeof(rpupd_sector_buf) ? sizeof(rpupd_sector_buf) : remaining;
    rpupd_psram::write(addr, rpupd_sector_buf, n);
    addr += n;
    remaining -= n;
  }
#endif
}

// ---------- Internal flash programming (RAM-resident critical section) ----------

// Erase and program one 4KB sector at dst_offset using rpupd_sector_buf content.
// Must be called with interrupts disabled, runs from RAM.
static bool __not_in_flash_func(rpupd_erase_program_sector_ram)(uint32_t dst_offset) {
  // Erase full sector
  flash_range_erase(dst_offset, RPUPD_FLASH_SECTOR_SIZE);
  // Program 256B pages
  for (size_t off = 0; off < RPUPD_FLASH_SECTOR_SIZE; off += RPUPD_FLASH_PAGE_SIZE) {
    flash_range_program(dst_offset + off, rpupd_sector_buf + off, RPUPD_FLASH_PAGE_SIZE);
  }
  return true;
}

// Public: copy [ext_offset .. ext_offset+length) to internal flash starting at RPUPD_FLASH_TARGET_OFFSET.
// Reads up to 4KB from external storage, then erases/programs sector-by-sector.
// If reboot_after = true, watchdog_reboot() is called and this will not return.
static inline bool rpupd_program_from_external(uint32_t ext_offset, uint32_t length, bool reboot_after = true) {
  if (length == 0) return false;
  if (length > RPUPD_ONBOARD_FLASH_SIZE) return false;

  rpupd_connect_internal_flash_if_needed();

  uint32_t bytes_remaining = length;
  uint32_t src = ext_offset;
  uint32_t dst = RPUPD_FLASH_TARGET_OFFSET;

  while (bytes_remaining > 0) {
    // Read up to a sector from external storage into RAM buffer
    size_t this_len = bytes_remaining > RPUPD_FLASH_SECTOR_SIZE ? RPUPD_FLASH_SECTOR_SIZE : bytes_remaining;
    memset(rpupd_sector_buf, 0xFF, RPUPD_FLASH_SECTOR_SIZE);  // pad tail with 0xFF
    rpupd_ext_read(src, rpupd_sector_buf, this_len);

    // Critical section: erase+program one sector
    uint32_t irq = save_and_disable_interrupts();
    bool ok = rpupd_erase_program_sector_ram(dst);
    restore_interrupts(irq);
    if (!ok) return false;

    // Advance
    src += this_len;
    dst += RPUPD_FLASH_SECTOR_SIZE;
    bytes_remaining -= this_len;
  }

  if (reboot_after) {
    delay(20);
    watchdog_reboot(0, 0, 0);
    while (true) { /* wait for reboot */
    }
  }
  return true;
}