#pragma once
#ifndef CONSOLE_PRINT_H
#define CONSOLE_PRINT_H

#include <Arduino.h>
#include <stdarg.h>
#include <stdlib.h>

// Only pull in Adafruit libs if TFT mirroring is enabled by the build.
#if defined(CONSOLE_TFT_ENABLE)
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#endif

class ConsolePrint : public Print {
public:
  ConsolePrint()
#if defined(CONSOLE_TFT_ENABLE)
    : _tft(nullptr)
#endif
  {
  }

  void begin() {
    // Nothing special; Serial is expected to have been started in the sketch.
#if defined(CONSOLE_TFT_ENABLE)
    // If you want auto-attach via macros, you can add it here later.
#endif
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
  // Attach an ST7789 panel and enable mirroring
  // spi: SPI or SPI1, etc. Use &SPI by default.
  // cs, dc, rst: control pins (rst can be -1 if tied to board reset)
  // w, h: panel dimensions (e.g., 240x320)
  // rotation: 0..3
  // invert: true to invert colors on some panels
  bool tftAttach(SPIClass* spi, int8_t cs, int8_t dc, int8_t rst,
                 uint16_t w, uint16_t h, uint8_t rotation = 0, bool invert = false) {
    tftDetach();  // cleanup any previous instance

    if (!spi) return false;
    _tft = new Adafruit_ST7789(spi, cs, dc, rst);
    if (!_tft) return false;

    _tftW = w;
    _tftH = h;

    _tft->init(w, h);
    _tft->setRotation(rotation & 3);
    if (invert) _tft->invertDisplay(true);

    _tft->setTextWrap(false);
    _tft->setTextColor(_fg, _bg);
    _tft->setTextSize(_textSize);
    _tft->fillScreen(_bg);
    _tft->setCursor(0, 0);

    _tftReady = true;
    _mirrorTFT = true;
    return true;
  }

  // Disable and free the TFT instance
  void tftDetach() {
    if (_tft) {
      delete _tft;
      _tft = nullptr;
    }
    _tftReady = false;
    _mirrorTFT = false;
  }

  // Enable/disable mirroring to TFT (without deallocating)
  void tftEnable(bool on) {
    _mirrorTFT = on && _tftReady;
  }

  // Set console colors (RGB565)
  void tftSetColors(uint16_t fg, uint16_t bg) {
    _fg = fg;
    _bg = bg;
    if (_tftReady) _tft->setTextColor(_fg, _bg);
  }

  // Set text size (1..N)
  void tftSetTextSize(uint8_t s) {
    if (s == 0) s = 1;
    _textSize = s;
    if (_tftReady) _tft->setTextSize(_textSize);
  }

  // Clear screen and reset cursor
  void tftClear() {
    if (!_tftReady) return;
    _tft->fillScreen(_bg);
    _tft->setCursor(0, 0);
  }

  // Rotation 0..3
  void tftSetRotation(uint8_t r) {
    if (!_tftReady) return;
    _tft->setRotation(r & 3);
  }

  // Optional invert
  void tftInvert(bool inv) {
    if (!_tftReady) return;
    _tft->invertDisplay(inv);
  }

  // Helper: RGB888 -> RGB565
  static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
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
  // Auto-paging at the bottom
  void tftAutoPageIfNeeded() {
    if (!_tftReady || !_mirrorTFT) return;
    int16_t cy = _tft->getCursorY();
    int16_t ch = 8 * (int16_t)_textSize;  // 7px font + 1px spacing heuristic
    int16_t h = (int16_t)_tftH;
    if (cy + ch > h) {
      _tft->fillScreen(_bg);
      _tft->setCursor(0, 0);
    }
  }

  void tftWriteChar(char c) {
    if (!_tftReady || !_mirrorTFT) return;
    if (c == '\r') return;
    if (c == '\n') {
      _tft->println();
      tftAutoPageIfNeeded();
    } else {
      _tft->write((uint8_t)c);
      tftAutoPageIfNeeded();
    }
  }

  void tftWriteBuffer(const char* s, size_t n) {
    if (!_tftReady || !_mirrorTFT || !s || n == 0) return;
    _tft->startWrite();
    for (size_t i = 0; i < n; ++i) {
      char c = s[i];
      if (c == '\r') continue;
      if (c == '\n') {
        _tft->println();
        tftAutoPageIfNeeded();
      } else {
        _tft->write((uint8_t)c);
        tftAutoPageIfNeeded();
      }
    }
    _tft->endWrite();
  }

  Adafruit_ST7789* _tft = nullptr;
  bool _tftReady = false;
  bool _mirrorTFT = false;

  uint16_t _tftW = 240;
  uint16_t _tftH = 320;
  uint16_t _fg = 0xFFFF;  // white
  uint16_t _bg = 0x0000;  // black
  uint8_t _textSize = 1;
#endif
};

#endif  // CONSOLE_PRINT_H