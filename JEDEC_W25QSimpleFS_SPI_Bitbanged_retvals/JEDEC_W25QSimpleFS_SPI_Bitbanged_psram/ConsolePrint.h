#pragma once
#ifndef CONSOLE_PRINT_H
#define CONSOLE_PRINT_H

#include <Arduino.h>
#include <stdarg.h>
#include <stdlib.h>

#if defined(CONSOLE_TFT_ENABLE)
#include <new>                  // placement new
#include "TFT_ST7789_Helper.h"  // New helper (SW-SPI capable)
#include <Adafruit_GFX.h>       // Legacy HW-SPI fallback
#include <Adafruit_ST7789.h>
#endif

class ConsolePrint : public Print {
public:
  ConsolePrint()
#if defined(CONSOLE_TFT_ENABLE)
    : _helper(nullptr), _helperConstructed(false),
      _tftLegacy(nullptr), _legacyConstructed(false)
#endif
  {
  }

  void begin() {
    // Serial should be started in the sketch
  }

  int printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int ret = vprintf_impl(fmt, ap);
    va_end(ap);
    return ret;
  }

  // Print-compatible writes
  virtual size_t write(uint8_t b) override {
    size_t s = Serial.write(b);
#if defined(CONSOLE_TFT_ENABLE)
    tftWriteChar((char)b);
#endif
    return s;
  }
  virtual size_t write(const uint8_t* buffer, size_t size) override {
    if (!buffer || !size) return 0;
    size_t s = Serial.write(buffer, size);
#if defined(CONSOLE_TFT_ENABLE)
    tftWriteBuffer((const char*)buffer, size);
#endif
    return s;
  }

#if defined(CONSOLE_TFT_ENABLE)
  // Backward-compatible attach (HW-SPI). If mosi/sck are provided (>=0),
  // use helper (SW-SPI). If left -1, use legacy HW-SPI path.
  bool tftAttach(SPIClass* spi, int8_t cs, int8_t dc, int8_t rst,
                 uint16_t w, uint16_t h,
                 uint8_t rotation = 0, bool invert = false,
                 int8_t mosi = -1, int8_t sck = -1,
                 uint32_t spiHz = 0) {
    tftDetach();
    if (!spi) return false;

    if (mosi >= 0 && sck >= 0) {
      // Helper (SW-SPI)
      _helper = new (_helperBuf) TFT_ST7789_Helper((uint8_t)cs, (uint8_t)dc, (uint8_t)mosi, (uint8_t)sck, (uint8_t)rst, w, h);
      _helperConstructed = true;
      _helper->setConsoleColors(_fg, _bg);
      _helper->setConsoleTextSize(_textSize);
      if (!_helper->begin(rotation & 3, invert, spiHz)) {
        _helper->~TFT_ST7789_Helper();
        _helper = nullptr;
        _helperConstructed = false;
        return false;
      }
      _mirrorTFT = true;
      _useHelper = true;
      _tftReady = true;
      _tftW = _helper->width();
      _tftH = _helper->height();
      return true;
    }

    // Legacy HW-SPI fallback (no dynamic allocation)
    _tftLegacy = new (_legacyBuf) Adafruit_ST7789(spi, cs, dc, rst);
    _legacyConstructed = true;
    _tftW = w;
    _tftH = h;
    _tftLegacy->init(w, h);
    _tftLegacy->setRotation(rotation & 3);
    if (invert) _tftLegacy->invertDisplay(true);
    _tftLegacy->setTextWrap(false);
    _tftLegacy->setTextColor(_fg, _bg);
    _tftLegacy->setTextSize(_textSize);
    _tftLegacy->fillScreen(_bg);
    _tftLegacy->setCursor(0, 0);
    _tftReady = true;
    _mirrorTFT = true;
    _useHelper = false;
    return true;
  }

  // Explicit helper-based (software SPI) attach
  bool tftAttachSW(int8_t cs, int8_t dc, int8_t rst, int8_t mosi, int8_t sck,
                   uint16_t w, uint16_t h,
                   uint8_t rotation = 0, bool invert = false,
                   uint32_t spiHz = 0) {
    tftDetach();
    if (cs < 0 || dc < 0 || rst < -1 || mosi < 0 || sck < 0) return false;
    _helper = new (_helperBuf) TFT_ST7789_Helper((uint8_t)cs, (uint8_t)dc, (uint8_t)mosi, (uint8_t)sck, (uint8_t)rst, w, h);
    _helperConstructed = true;
    _helper->setConsoleColors(_fg, _bg);
    _helper->setConsoleTextSize(_textSize);
    if (!_helper->begin(rotation & 3, invert, spiHz)) {
      _helper->~TFT_ST7789_Helper();
      _helper = nullptr;
      _helperConstructed = false;
      return false;
    }
    _mirrorTFT = true;
    _useHelper = true;
    _tftReady = true;
    _tftW = _helper->width();
    _tftH = _helper->height();
    return true;
  }

  // Disable and free any TFT instance
  void tftDetach() {
    if (_helperConstructed && _helper) {
      _helper->~TFT_ST7789_Helper();
      _helper = nullptr;
      _helperConstructed = false;
    }
    if (_legacyConstructed && _tftLegacy) {
      _tftLegacy->~Adafruit_ST7789();
      _tftLegacy = nullptr;
      _legacyConstructed = false;
    }
    _tftReady = false;
    _mirrorTFT = false;
    _useHelper = false;
  }

  // Enable/disable mirroring to TFT (without deallocating)
  void tftEnable(bool on) {
    _mirrorTFT = on && _tftReady;
  }

  // Set console colors (RGB565)
  void tftSetColors(uint16_t fg, uint16_t bg) {
    _fg = fg;
    _bg = bg;
    if (!_tftReady) return;
    if (_useHelper) {
      _helper->setConsoleColors(_fg, _bg);
    } else {
      _tftLegacy->setTextColor(_fg, _bg);
    }
  }

  // Set text size (1..N)
  void tftSetTextSize(uint8_t s) {
    if (s == 0) s = 1;
    _textSize = s;
    if (!_tftReady) return;
    if (_useHelper) {
      _helper->setConsoleTextSize(_textSize);
    } else {
      _tftLegacy->setTextSize(_textSize);
    }
  }

  // Clear screen and reset cursor
  void tftClear() {
    if (!_tftReady) return;
    if (_useHelper) {
      _helper->consoleClear();
    } else {
      _tftLegacy->fillScreen(_bg);
      _tftLegacy->setCursor(0, 0);
    }
  }

  // Rotation 0..3
  void tftSetRotation(uint8_t r) {
    if (!_tftReady) return;
    r &= 3;
    if (_useHelper) {
      _helper->setRotation(r);
      _tftW = _helper->width();
      _tftH = _helper->height();
    } else {
      _tftLegacy->setRotation(r);
      _tftW = _tftLegacy->width();
      _tftH = _tftLegacy->height();
    }
  }

  // Optional invert
  void tftInvert(bool inv) {
    if (!_tftReady) return;
    if (_useHelper) _helper->invertDisplay(inv);
    else _tftLegacy->invertDisplay(inv);
  }

  // Optional: Set SPI speed (helper path or legacy path)
  void tftSetSPISpeed(uint32_t hz) {
    if (!_tftReady) return;
    if (_useHelper) _helper->setSPISpeed(hz);
    else _tftLegacy->setSPISpeed(hz);
  }

  // Primitives exposed for commands
  void tftFillScreen(uint16_t c) {
    if (!_tftReady) return;
    if (_useHelper) _helper->fillScreen(c);
    else _tftLegacy->fillScreen(c);
  }
  void tftFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c) {
    if (!_tftReady) return;
    if (_useHelper) _helper->fillRect(x, y, w, h, c);
    else _tftLegacy->fillRect(x, y, w, h, c);
  }

  // File-backed RGB565 blit using active FS via callbacks
  bool tftDrawRGB565FromFS(const TFT_FSCallbacks& cb, const char* fname,
                           uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    if (!_tftReady || !fname || !cb.valid()) return false;
    if (_useHelper) {
      _helper->setFSCallbacks(cb);
      return _helper->drawRGB565FromFS(fname, x, y, w, h);
    }
    // Legacy Adafruit path
    uint16_t W = _tftLegacy->width(), H = _tftLegacy->height();
    if (x >= W || y >= H) return false;
    if (x + w > W || y + h > H) return false;
    uint32_t need = (uint32_t)w * (uint32_t)h * 2u;
    uint32_t fsz = 0;
    if (!cb.getFileSize(fname, fsz) || fsz < need) return false;

    _tftLegacy->startWrite();
    _tftLegacy->setAddrWindow(x, y, w, h);

    const uint32_t CHUNK = 1024;
    uint8_t buf[CHUNK];
    uint32_t off = 0, remain = need;
    while (remain) {
      uint32_t n = (remain > CHUNK) ? CHUNK : remain;
      uint32_t got = cb.readFileRange(fname, off, buf, n);
      if (got != n) {
        _tftLegacy->endWrite();
        return false;
      }
      for (uint32_t i = 0; i < got; i += 2) {
        uint8_t hi = buf[i];
        buf[i] = buf[i + 1];
        buf[i + 1] = hi;
      }
      _tftLegacy->writePixels((uint16_t*)buf, got / 2);
      off += n;
      remain -= n;
      yield();
    }
    _tftLegacy->endWrite();
    return true;
  }

  // Helper: RGB888 -> RGB565
  static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return TFT_ST7789_Helper::color565(r, g, b);
  }
#endif  // CONSOLE_TFT_ENABLE

private:
  int vprintf_impl(const char* fmt, va_list ap) {
    if (!fmt) return 0;
    char buf[256];
    va_list aq;
    va_copy(aq, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, aq);
    va_end(aq);
    if (n < 0) return n;
    if ((size_t)n < sizeof(buf)) {
      write((const uint8_t*)buf, (size_t)n);
      return n;
    } else {
      char* big = (char*)malloc(n + 1);
      if (!big) {
        write((const uint8_t*)buf, sizeof(buf) - 1);
        return (int)(sizeof(buf) - 1);
      }
      vsnprintf(big, n + 1, fmt, ap);
      write((const uint8_t*)big, (size_t)n);
      free(big);
      return n;
    }
  }

#if defined(CONSOLE_TFT_ENABLE)
  // Auto-paging/log write using helper if present, else legacy
  void tftWriteChar(char c) {
    if (!_tftReady || !_mirrorTFT) return;
    if (_useHelper) {
      _helper->consoleWriteChar(c);
      return;
    }
    // Legacy path (HW-SPI)
    if (c == '\r') return;
    if (c == '\n') {
      _tftLegacy->println();
      tftAutoPageLegacy();
    } else {
      _tftLegacy->write((uint8_t)c);
      tftAutoPageLegacy();
    }
  }

  void tftWriteBuffer(const char* s, size_t n) {
    if (!_tftReady || !_mirrorTFT || !s || n == 0) return;
    if (_useHelper) {
      _helper->consoleWrite(s, n);
      return;
    }
    // Legacy path (HW-SPI)
    _tftLegacy->startWrite();
    for (size_t i = 0; i < n; ++i) {
      char c = s[i];
      if (c == '\r') continue;
      if (c == '\n') {
        _tftLegacy->println();
        tftAutoPageLegacy();
      } else {
        _tftLegacy->write((uint8_t)c);
        tftAutoPageLegacy();
      }
    }
    _tftLegacy->endWrite();
  }

  void tftAutoPageLegacy() {
    int16_t cy = _tftLegacy->getCursorY();
    int16_t ch = 8 * (int16_t)_textSize;  // 7px font + 1px spacing heuristic
    int16_t h = (int16_t)_tftH;
    if (cy + ch > h) {
      _tftLegacy->fillScreen(_bg);
      _tftLegacy->setCursor(0, 0);
    }
  }

  // Inline storage + state (no heap, no delete)
  alignas(TFT_ST7789_Helper) uint8_t _helperBuf[sizeof(TFT_ST7789_Helper)];
  TFT_ST7789_Helper* _helper;
  bool _helperConstructed;

  alignas(Adafruit_ST7789) uint8_t _legacyBuf[sizeof(Adafruit_ST7789)];
  Adafruit_ST7789* _tftLegacy;
  bool _legacyConstructed;

  bool _tftReady = false;
  bool _mirrorTFT = false;
  bool _useHelper = false;
  uint16_t _tftW = 240;
  uint16_t _tftH = 320;
  uint16_t _fg = 0xFFFF;  // white
  uint16_t _bg = 0x0000;  // black
  uint8_t _textSize = 1;
#endif
};

#endif  // CONSOLE_PRINT_H