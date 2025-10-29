#pragma once
/*
  CoProcProto.h
  Single-header RPC protocol for Main<->Co-Processor over framed serial link.
  - Frame: 24B header + payload + CRC32(payload) (CRC=0 if len==0).
  - Requests: HELLO, INFO, LOAD_BEGIN, LOAD_DATA, LOAD_END, EXEC, STATUS, MAILBOX_RD, CANCEL, RESET.
  - Script Requests: SCRIPT_BEGIN, SCRIPT_DATA, SCRIPT_END, SCRIPT_EXEC (same framing as LOAD_EXEC).
  - ISP Requests: ISP_ENTER, ISP_EXIT.
  - Responses: cmd | 0x80, status-first payloads.
*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <type_traits>

namespace CoProc {

// ========== Constants ==========
static constexpr uint32_t MAGIC = 0x30525043;  // 'C''P''R''0'
static constexpr uint16_t VERSION = 0x0001;

// Commands (req: 0x00..0x7F, resp = req|0x80)
enum : uint16_t {
  CMD_HELLO = 0x01,
  CMD_INFO = 0x02,

  // Binary blob pipeline
  CMD_LOAD_BEGIN = 0x10,  // req: uint32 total_len
  CMD_LOAD_DATA = 0x11,   // req: raw bytes
  CMD_LOAD_END = 0x12,    // req: uint32 expected_crc32
  CMD_EXEC = 0x20,        // req: uint32 argc, int32 argv[argc], uint32 timeout_ms

  // Status/mailbox/cancel/reset
  CMD_STATUS = 0x21,      // req: -
  CMD_MAILBOX_RD = 0x22,  // req: uint32 max_bytes
  CMD_CANCEL = 0x23,      // req: -
  CMD_RESET = 0x24,       // req: -

  // Script pipeline
  CMD_SCRIPT_BEGIN = 0x30,  // req: uint32 total_len
  CMD_SCRIPT_DATA = 0x31,   // req: raw bytes (UTF-8 text)
  CMD_SCRIPT_END = 0x32,    // req: uint32 expected_crc32
  CMD_SCRIPT_EXEC = 0x33,   // req: uint32 argc, int32 argv[argc], uint32 timeout_ms
                            //       (alternate mode supported by impl: marker 0xFFFFFFFF + ASCII args)

  // ISP control
  CMD_ISP_ENTER = 0x60,  // req: -
  CMD_ISP_EXIT = 0x61    // req: -
};

// Status
enum : int32_t {
  ST_OK = 0,
  ST_BAD_MAGIC = -1,
  ST_BAD_VERSION = -2,
  ST_BAD_CMD = -3,
  ST_PARAM = -4,
  ST_STATE = -5,
  ST_NOMEM = -6,
  ST_SIZE = -7,
  ST_CRC = -8,
  ST_TIMEOUT = -9,
  ST_EXEC = -10
};

// ========== Frame header ==========
#pragma pack(push, 1)
struct Frame {
  uint32_t magic;
  uint16_t version;
  uint16_t cmd;
  uint32_t seq;
  uint32_t len;
  uint32_t crc32;  // CRC over payload (0 if len==0)
};
#pragma pack(pop)

inline void makeResponseHeader(Frame& h, uint16_t reqCmd, uint32_t seq, uint32_t len, uint32_t crc) {
  h.magic = MAGIC;
  h.version = VERSION;
  h.cmd = (uint16_t)(reqCmd | 0x80);
  h.seq = seq;
  h.len = len;
  h.crc32 = (len ? crc : 0);
}

// ========== CRC32 (IEEE 802.3) ==========
inline uint32_t crc32_ieee(const void* data, size_t len, uint32_t crc = 0xFFFFFFFFu) {
  const uint8_t* p = (const uint8_t*)data;
  while (len--) {
    uint32_t c = (crc ^ *p++) & 0xFFu;
    for (int i = 0; i < 8; ++i) c = (c >> 1) ^ (0xEDB88320u & (-(int)(c & 1u)));
    crc = (crc >> 8) ^ c;
  }
  return crc ^ 0xFFFFFFFFu;
}

// ========== Small PODs ==========
#pragma pack(push, 1)
struct Info {
  // impl_flags bits:
  //   bit0: blob_loaded
  //   bit1: exec_running
  //   bit2: mailbox_nonempty
  //   bit8: isp_active (if supported and currently in ISP mode)
  uint32_t impl_flags;
  uint32_t blob_len;     // bytes loaded
  uint32_t mailbox_max;  // bytes
};
#pragma pack(pop)

// ========== Payload helpers ==========
template<typename T>
inline bool writePOD(uint8_t* dst, size_t cap, size_t& off, const T& v) {
  using NV = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
  NV tmp = (NV)v;
  if (off + sizeof(NV) > cap) return false;
  memcpy(dst + off, &tmp, sizeof(NV));
  off += sizeof(NV);
  return true;
}

inline bool writeBytes(uint8_t* dst, size_t cap, size_t& off, const void* p, size_t n) {
  if (off + n > cap) return false;
  memcpy(dst + off, p, n);
  off += n;
  return true;
}

template<typename T>
inline bool readPOD(const uint8_t* src, size_t len, size_t& off, T& v) {
  if (off + sizeof(T) > len) return false;
  memcpy(&v, src + off, sizeof(T));
  off += sizeof(T);
  return true;
}

inline bool readBytes(const uint8_t* src, size_t len, size_t& off, void* p, size_t n) {
  if (off + n > len) return false;
  memcpy(p, src + off, n);
  off += n;
  return true;
}

// ========== Exec state flags ==========
enum : uint32_t {
  EXEC_IDLE = 0,
  EXEC_LOADED = 1,
  EXEC_RUNNING = 2,
  EXEC_DONE = 3
};

}  // namespace CoProc