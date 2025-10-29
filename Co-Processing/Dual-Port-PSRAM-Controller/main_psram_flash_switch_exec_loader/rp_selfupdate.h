#pragma once
/*
  rp_selfupdate.h - Program RP2040/RP2350 internal flash from a file in PSRAMMulti FS.

  - No metadata/headers: copies exactly the bytes in the file to internal flash.
  - Programs in 4KB sectors (256B page writes), padding the tail of the last sector with 0xFF.
  - Critical flash ops run from RAM (__not_in_flash_func) with interrupts disabled.
  - Works on RP2040 and RP2350. If RP2350 PICO_NO_FLASH is set, connects internal flash.

  Expected FS API (from PSRAMSimpleFS_Generic):
    bool   getFileSize(const char* name, uint32_t& sizeOut);
    uint32_t readFileRange(const char* name, uint32_t offset, uint8_t* buf, uint32_t len);
    bool   format();
    bool   wipeChip();

  Configuration (override before including this header):
    #define RPUPD_FS_FLASH_TARGET_OFFSET   0x00000000u
    #define RPUPD_FS_ONBOARD_FLASH_SIZE    (2u * 1024u * 1024u)  // adjust to your board
*/

#ifndef ARDUINO_ARCH_RP2040
#if !defined(PICO_RP2040) && !defined(PICO_RP2350)
#error "This header targets RP2040/RP2350 (Arduino-Pico) only."
#endif
#endif

#include <Arduino.h>

extern "C" {
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <pico/bootrom.h>
}

#ifndef RPUPD_FS_FLASH_TARGET_OFFSET
#define RPUPD_FS_FLASH_TARGET_OFFSET 0x00000000u
#endif

#ifndef RPUPD_FS_ONBOARD_FLASH_SIZE
#define RPUPD_FS_ONBOARD_FLASH_SIZE (2u * 1024u * 1024u)
#endif

static constexpr size_t RPUPD_FS_SECTOR = 4096;
static constexpr size_t RPUPD_FS_PAGE = 256;

static uint8_t rpupdfs_sector_buf[RPUPD_FS_SECTOR];

// Connect internal flash where applicable (RP2350 PICO_NO_FLASH)
static inline void rpupdfs_connect_internal_flash_if_needed() {
#if defined(PICO_RP2350) && defined(PICO_NO_FLASH) && PICO_NO_FLASH
  __compiler_memory_barrier();
  reinterpret_cast<void (*)(void)>(rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH))();
  reinterpret_cast<void (*)(void)>(rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE))();
  __compiler_memory_barrier();
#endif
}

// RAM-resident critical: erase+program one 4KB sector from given RAM buffer
static bool __not_in_flash_func(rpupdfs_erase_program_sector_ram)(uint32_t dst_offset, const uint8_t* buf4k) {
  flash_range_erase(dst_offset, RPUPD_FS_SECTOR);
  for (size_t off = 0; off < RPUPD_FS_SECTOR; off += RPUPD_FS_PAGE) {
    flash_range_program(dst_offset + off, buf4k + off, RPUPD_FS_PAGE);
  }
  return true;
}

// Public: program internal flash from a file in your PSRAM FS.
// - fs: your PSRAMSimpleFS_Generic (or compatible) instance
// - filename: file containing the raw RP binary
// - reboot_after: reboot into new firmware when done
// - flash_offset: destination offset in internal flash (default 0)
//
// Returns true on success. No metadata: writes exactly the file bytes.
template<typename FS>
static inline bool rpupdfs_program_from_file(FS& fs,
                                             const char* filename,
                                             bool reboot_after = true,
                                             uint32_t flash_offset = RPUPD_FS_FLASH_TARGET_OFFSET) {
  if (!filename || !filename[0]) return false;

  // Query file size
  uint32_t total = 0;
  if (!fs.getFileSize(filename, total)) return false;
  if (total == 0) return false;

  // Capacity checks: make sure we don't exceed configured internal flash size
  if (flash_offset + total > RPUPD_FS_ONBOARD_FLASH_SIZE) return false;

  rpupdfs_connect_internal_flash_if_needed();

  uint32_t src_pos = 0;
  uint32_t dst_off = flash_offset;

  while (src_pos < total) {
    // Read up to a sector from FS
    const uint32_t to_read = (total - src_pos > RPUPD_FS_SECTOR) ? RPUPD_FS_SECTOR : (total - src_pos);
    memset(rpupdfs_sector_buf, 0xFF, RPUPD_FS_SECTOR);

    const uint32_t got = fs.readFileRange(filename, src_pos, rpupdfs_sector_buf, to_read);
    if (got != to_read) return false;  // short read -> fail safely

    // Critical section: erase/program 4KB sector
    const uint32_t irq = save_and_disable_interrupts();
    const bool ok = rpupdfs_erase_program_sector_ram(dst_off, rpupdfs_sector_buf);
    restore_interrupts(irq);
    if (!ok) return false;

    src_pos += to_read;
    dst_off += RPUPD_FS_SECTOR;
  }

  if (reboot_after) {
    delay(20);
    watchdog_reboot(0, 0, 0);
    while (true) {}
  }
  return true;
}

// Optional convenience: format directory region (preserves chip content except DIR area)
template<typename FS>
static inline bool rpupdfs_format(FS& fs) {
  return fs.format();
}

// Optional convenience: full-chip wipe to 0xFF (slow)
template<typename FS>
static inline bool rpupdfs_wipe_chip(FS& fs) {
  return fs.wipeChip();
}

// Toy brick function: erase first 4KB of internal flash (forces BOOTSEL recovery)
static inline void rpupdfs_corrupt_first_sector_and_reboot() {
  const uint32_t irq = save_and_disable_interrupts();
  flash_range_erase(0x00000000u, RPUPD_FS_SECTOR);
  restore_interrupts(irq);
  delay(20);
  watchdog_reboot(0, 0, 0);
  while (true) {}
}

// Optional: enter BootROM UF2 mode programmatically
static inline void rpupdfs_enter_bootsel_now() {
  reset_usb_boot(0, 0);
  while (true) {}
}