// FastSPI_DMA.h
//
// RP2040 hardware SPI + optional DMA helper for high-throughput bitstream transfers.
// Intended as a drop-in fast backend for both PSRAM and W25Q drivers that use
// simple cmd/read/write semantics.
//
// Notes:
//  - RP2040-only. Requires the Arduino RP2040 core which exposes the Pico SDK headers.
//  - API intentionally small: begin(), setClockHz(), cmdRead(), transfer(), readData(), writeData()
//  - The class does NOT toggle CS for you beyond simple helper functions because your PSRAM
//    aggregator (74HC138 or decoder) needs specialized selection/settling semantics.
//  - For large transfers the class uses the SDK spi_write_read_blocking function which already uses
//    the hardware FIFOs; for the biggest transfers the code demonstrates a DMA path to keep CPU free.
//
// Usage summary:
//   FastSPI_DMA spi(SPI0, pinMISO, pinSCK, pinMOSI, pinCS);
//   spi.begin();
//   spi.setClockHz(20'000'000); // 20 MHz
//   spi.csLow();   // optionally manage CS externally if needed
//   spi.cmdRead(&cmd, cmdLen, resp, respLen);
//   spi.csHigh();
//
// If you want fully automatic CS handling (single-chip direct CS), use csLow()/csHigh() helpers.
// For 74HC138 or 595 decoder flows, keep CS control in the aggregator code.
#pragma once

#ifdef ARDUINO_ARCH_RP2040

#define FASTSPI_DEBUG 1
#ifdef FASTSPI_DEBUG
#define FSD_PRINT(...) \
  do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define FSD_PRINT(...) \
  do { \
  } while (0)
#endif

#include <Arduino.h>
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include <string.h>

class FastSPI_DMA {
public:
  // spi_inst: spi0 or spi1; csPin may be 255 to indicate "no auto CS"
  FastSPI_DMA(spi_inst_t* spi_inst = spi1, uint8_t pin_miso = 12, uint8_t pin_sck = 10, uint8_t pin_mosi = 11, uint8_t pin_cs = 9)
    : _spi(spi_inst), _miso(pin_miso), _sck(pin_sck), _mosi(pin_mosi), _cs(pin_cs),
      _clkHz(1000000), _dma_tx_chan(-1), _dma_rx_chan(-1) {}

  // Configure pins and SPI HW. Call once before transfers.
  void begin() {
    // configure pins for SPI HW
    gpio_set_function(_mosi, GPIO_FUNC_SPI);
    gpio_set_function(_miso, GPIO_FUNC_SPI);
    gpio_set_function(_sck, GPIO_FUNC_SPI);
    if (_cs != 255) {
      pinMode(_cs, OUTPUT);
      digitalWrite(_cs, HIGH);
    }

    // default SPI configuration (CPOL=0 CPHA=0)
    spi_init(_spi, _clkHz);
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Reserve DMA channels lazily
    _dma_tx_chan = -1;
    _dma_rx_chan = -1;
  }

  // Set SPI clock (Hz). Call before begin() or after (re-init applied).
  void setClockHz(uint32_t hz) {
    _clkHz = hz;
    if (_spi) spi_set_baudrate(_spi, _clkHz);
  }

  // Optional small helpers for CS (useful when using direct CS pins)
  inline void csLow() {
    if (_cs != 255) digitalWrite(_cs, LOW);
  }
  inline void csHigh() {
    if (_cs != 255) digitalWrite(_cs, HIGH);
  }

  // Generic command + read helper: send 'cmd' bytes then read 'respLen' bytes into resp
  // This is blocking and uses SPI hardware FIFO transfer (efficient).
  /*bool cmdRead(const uint8_t* cmd, size_t cmdLen, uint8_t* resp, size_t respLen) {
    if (cmd && cmdLen) {
      // write command (tx-only)
      spi_write_blocking(_spi, cmd, cmdLen);
    }
    if (resp && respLen) {
      // read respLen bytes (send 0x00s)
      spi_read_blocking(_spi, 0x00, resp, respLen);
    }
    return true;
  }*/
  bool cmdRead(const uint8_t* cmd, size_t cmdLen, uint8_t* resp, size_t respLen) {
    if (cmd && cmdLen) {
      FSD_PRINT("FastSPI: cmdWrite %u bytes: ", (unsigned)cmdLen);
      for (size_t i = 0; i < cmdLen; i++) FSD_PRINT("%02X ", cmd[i]);
      FSD_PRINT("\n");
      spi_write_blocking(_spi, cmd, cmdLen);
    }
    if (resp && respLen) {
      memset(resp, 0xEE, respLen);  // prefill to see if read changes it
      spi_read_blocking(_spi, 0x00, resp, respLen);
      FSD_PRINT("FastSPI: read %u bytes: ", (unsigned)respLen);
      for (size_t i = 0; i < respLen; i++) FSD_PRINT("%02X ", resp[i]);
      FSD_PRINT("\n");
    }
    return true;
  }

  // Full-duplex transfer: length bytes. If rx==nullptr, consume receive data.
  // Uses spi_write_read_blocking (fast, uses HW).
  bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (!len) return true;
    if (tx && rx) {
      spi_write_read_blocking(_spi, tx, rx, len);
      return true;
    } else if (tx && !rx) {
      spi_write_blocking(_spi, tx, len);
      // drain RX (if required) into small discard buffer
      // spi_write_blocking already shifts the FIFO and discards received bytes internally (no extra call needed)
      return true;
    } else if (!tx && rx) {
      spi_read_blocking(_spi, 0x00, rx, len);
      return true;
    }
    return true;
  }

  // Read a linear data region: sends 0x03 + 24-bit address (caller handles CS/DEcoder)
  // Equivalent to PSRAMBitbang::readData03 wrapper.
  bool readData03(uint32_t addr, uint8_t* buf, size_t len) {
    uint8_t header[4];
    header[0] = 0x03;
    header[1] = (uint8_t)(addr >> 16);
    header[2] = (uint8_t)(addr >> 8);
    header[3] = (uint8_t)(addr);
    // command write => then read len bytes
    // combine cmd send and read; spi_write_read_blocking handles only equal sized tx/rx buffers,
    // so we write the cmd (tx-only) then read separately (fast path).
    spi_write_blocking(_spi, header, 4);
    spi_read_blocking(_spi, 0x00, buf, len);
    return true;
  }

  // Write data with command 0x02 + 24-bit address
  bool writeData02(uint32_t addr, const uint8_t* buf, size_t len) {
    if (!buf || len == 0) return true;
    uint8_t header[4];
    header[0] = 0x02;
    header[1] = (uint8_t)(addr >> 16);
    header[2] = (uint8_t)(addr >> 8);
    header[3] = (uint8_t)(addr);
    spi_write_blocking(_spi, header, 4);
    spi_write_blocking(_spi, buf, len);
    return true;
  }

  // read JEDEC ID (0x9F). Caller provides out buffer >= len.
  bool readJEDEC(uint8_t* out, size_t len) {
    if (!out || len == 0) return false;
    uint8_t cmd = 0x9F;
    spi_write_blocking(_spi, &cmd, 1);
    spi_read_blocking(_spi, 0x00, out, len);
    return true;
  }

  // For very large transfers you can call these DMA-accelerated helpers.
  // These functions lazily allocate DMA channels and configure basic transfers.
  // They are blocking until transfer completes.
  bool dmaTransferTxRx(const uint8_t* txBuf, uint8_t* rxBuf, size_t len) {
    if (len == 0) return true;
    ensureDmaChannels();

    // Obtain the hardware SPI register block pointer (pico-sdk accessor).
    volatile uint32_t* spi_dr = (volatile uint32_t*)&(spi_get_hw(_spi)->dr);

    // Configure RX channel: read from SPI RX FIFO (8-bit), write to rxBuf
    dma_channel_config c_rx = dma_channel_get_default_config(_dma_rx_chan);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_rx, false);            // read from FIFO (fixed)
    channel_config_set_write_increment(&c_rx, true);            // write to rxBuf (increment)
    channel_config_set_dreq(&c_rx, spi_get_dreq(_spi, false));  // RX DREQ

    dma_channel_configure(
      _dma_rx_chan, &c_rx,
      rxBuf,          // dst
      (void*)spi_dr,  // src (SPI RX FIFO address)
      len, true);

    // Configure TX channel: read from txBuf, write to SPI TX FIFO
    dma_channel_config c_tx = dma_channel_get_default_config(_dma_tx_chan);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_tx, true);            // read from txBuf (increment)
    channel_config_set_write_increment(&c_tx, false);          // write to FIFO (fixed)
    channel_config_set_dreq(&c_tx, spi_get_dreq(_spi, true));  // TX DREQ

    dma_channel_configure(
      _dma_tx_chan, &c_tx,
      (void*)spi_dr,  // dst (SPI TX FIFO address)
      txBuf,          // src
      len, true);

    // Wait for DMA completion
    dma_channel_wait_for_finish_blocking(_dma_tx_chan);
    dma_channel_wait_for_finish_blocking(_dma_rx_chan);

    return true;
  }

  bool dmaTransferRx(const uint8_t fillByte, uint8_t* rxBuf, size_t len) {
    if (len == 0) return true;
    ensureDmaChannels();

    // For RX-only we create a tiny fill buffer (single byte) repeated via read-only single transfer,
    // or create a small statically allocated fill array; simpler: use spi_read_blocking for moderate sizes.
    // For extreme sizes, the caller can craft a txBuf repeating fillByte and call dmaTransferTxRx.
    spi_read_blocking(_spi, fillByte, rxBuf, len);
    return true;
  }

private:
  spi_inst_t* _spi;
  uint8_t _miso, _sck, _mosi, _cs;
  uint32_t _clkHz;
  int _dma_tx_chan;
  int _dma_rx_chan;

  void ensureDmaChannels() {
    if (_dma_tx_chan >= 0 && _dma_rx_chan >= 0) return;
    if (_dma_tx_chan < 0) {
      _dma_tx_chan = dma_claim_unused_channel(true);
    }
    if (_dma_rx_chan < 0) {
      _dma_rx_chan = dma_claim_unused_channel(true);
    }
  }
};

#else
#error "FastSPI_DMA.h: RP2040-specific implementation. Compile only for ARDUINO_ARCH_RP2040."
#endif