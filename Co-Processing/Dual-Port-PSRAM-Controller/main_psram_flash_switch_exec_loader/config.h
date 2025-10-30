#include "blob_mailbox_config.h" // Do not remove this line. This is the memory address where we handle the shared interprocess-mailbox-buffer

#include "blob_ret42.h"
#define FILE_RET42 "ret42"
#include "blob_add2.h"
#define FILE_ADD2 "add2"
#include "blob_pwmc.h"
#define FILE_PWMC "pwmc"
#include "blob_pwmc2350.h"
#define FILE_PWMC2350 "pwmc2350"
#include "blob_retmin.h"
#define FILE_RETMIN "retmin"
#include "scripts.h"
#define FILE_BLINKSCRIPT "blinkscript"
#define FILE_ONSCRIPT "onscript"

// ========== Static buffer-related compile-time constant ==========
#define FS_SECTOR_SIZE 4096

// ========== bitbang pin definitions (flash) ==========
const uint8_t PIN_FLASH_MISO = 12;  // GP12
const uint8_t PIN_FLASH_CS = 28;    // GP28
const uint8_t PIN_FLASH_SCK = 10;   // GP10
const uint8_t PIN_FLASH_MOSI = 11;  // GP11

// ========== bitbang pin definitions (psram) ==========
const bool PSRAM_ENABLE_QPI = false;     // Should we enable the PSRAM QPI-mode?
const uint8_t PSRAM_CLOCK_DELAY_US = 0;  // Small delay helps ensure decoder/address settle before EN
const uint8_t PIN_PSRAM_MISO = 12;       // GP12
const uint8_t PIN_PSRAM_MOSI = 11;       // GP11
const uint8_t PIN_PSRAM_SCK = 10;        // GP10
// 74HC138 pins (for 138-only default)
const uint8_t PIN_138_A0 = 8;     // 74HC138 pin 1 (A0)
const uint8_t PIN_138_A1 = 7;     // 74HC138 pin 2 (A1)
const uint8_t PIN_138_EN = 9;     // 74HC138 pins 4+5 (G2A/G2B tied), 10k pull-up to 3.3V
const uint8_t PIN_138_A2 = 255;   // 255 means "not used"; tie 74HC138 pin 3 (A2) to GND
const uint8_t PIN_595_LATCH = 4;  // 595.RCLK (unused in 138-only mode)

// ========== Flash/PSRAM instances (needed before bindActiveFs) ==========
const size_t PERSIST_LEN = 32;
enum class StorageBackend {
  Flash,
  PSRAM_BACKEND
};
static StorageBackend g_storage = StorageBackend::Flash;
constexpr uint8_t PSRAM_NUM_CHIPS = 4;
constexpr uint32_t PER_CHIP_CAP_BYTES = 8UL * 1024UL * 1024UL;
constexpr uint32_t AGG_CAPACITY_BYTES = PER_CHIP_CAP_BYTES * PSRAM_NUM_CHIPS;
W25QBitbang flashBB(PIN_FLASH_MISO, PIN_FLASH_CS, PIN_FLASH_SCK, PIN_FLASH_MOSI);
W25QSimpleFS fsFlash(flashBB);
PSRAMAggregateDevice psramBB(PIN_PSRAM_MISO, PIN_PSRAM_MOSI, PIN_PSRAM_SCK, PER_CHIP_CAP_BYTES);
PSRAMSimpleFS_Multi fsPSRAM(psramBB, AGG_CAPACITY_BYTES);
static bool g_dev_mode = true;