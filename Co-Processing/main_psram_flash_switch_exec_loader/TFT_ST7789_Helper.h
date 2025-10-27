#ifndef TFT_ST7789_HELPER_H
#define TFT_ST7789_HELPER_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>



// ========== TFT console helpers to mirror Serial output ==========
/*static void tftConsoleBegin() {
  if (!tft_ready) return;
  tft.setTextWrap(false);
  tft.setTextColor(tft_console_fg, tft_console_bg);
  tft.setTextSize(tft_console_textsize);
  tft.fillScreen(tft_console_bg);
  tft.setCursor(0, 0);
}
static void tftConsoleClear() {
  if (!tft_ready) return;
  tft.fillScreen(tft_console_bg);
  tft.setCursor(0, 0);
}*/



// Simple callback bundle so we can read from your active FS without depending on its symbol.
struct TFT_FSCallbacks {
  // Must match your activeFs: bool getFileSize(const char*, uint32_t&)
  bool (*getFileSize)(const char* name, uint32_t& size) = nullptr;

  // Must match your activeFs: uint32_t readFileRange(const char*, uint32_t, uint8_t*, uint32_t)
  uint32_t (*readFileRange)(const char* name, uint32_t offset, uint8_t* buf, uint32_t len) = nullptr;

  bool valid() const {
    return getFileSize && readFileRange;
  }
};

class TFT_ST7789_Helper {
public:
  // Software-SPI constructor (matches your current usage)
  TFT_ST7789_Helper(uint8_t pinCS, uint8_t pinDC, uint8_t pinMOSI, uint8_t pinSCK, uint8_t pinRST,
                    uint16_t w = 240, uint16_t h = 320)
    : _tft(pinCS, pinDC, pinMOSI, pinSCK, pinRST),
      _w(w), _h(h), _rot(0),
      _xstart(0), _ystart(0),
      _ready(false),
      _fg(0xFFFF), _bg(0x0000), _textSize(1) {}

  // Initialize panel and console defaults
  bool begin(uint8_t rotation = 1, bool invert = true, uint32_t spiHz = 0) {
    _tft.init(_w, _h);
    _tft.setRotation(rotation & 3);
    _rot = rotation & 3;
    if (spiHz) _tft.setSPISpeed(spiHz);
    if (invert) _tft.invertDisplay(true);
    _w = _tft.width();
    _h = _tft.height();
    _tft.setTextWrap(false);
    _tft.setTextSize(_textSize);
    _tft.setTextColor(_fg, _bg);
    _tft.fillScreen(_bg);
    _tft.setCursor(0, 0);
    _ready = true;
    return true;
  }

  // Set optional start offsets if your panel needs them
  void setPanelOffsets(uint16_t xstart, uint16_t ystart) {
    _xstart = xstart;
    _ystart = ystart;
  }

  void setRotation(uint8_t rotation) {
    _tft.setRotation(rotation & 3);
    _rot = rotation & 3;
    _w = _tft.width();
    _h = _tft.height();
  }

  void setSPISpeed(uint32_t hz) {
    _tft.setSPISpeed(hz);
  }
  void invertDisplay(bool inv) {
    _tft.invertDisplay(inv);
  }

  // Console-style defaults
  void setConsoleColors(uint16_t fg, uint16_t bg) {
    _fg = fg;
    _bg = bg;
    _tft.setTextColor(_fg, _bg);
  }
  void setConsoleTextSize(uint8_t s) {
    if (s < 1) s = 1;
    _textSize = s;
    _tft.setTextSize(_textSize);
  }

  // Clear and reset cursor
  void consoleClear() {
    if (!_ready) return;
    _tft.fillScreen(_bg);
    _tft.setCursor(0, 0);
  }

  // Console write helpers with simple auto-paging like your code
  void consoleWriteChar(char c) {
    if (!_ready) return;
    if (c == '\r') return;
    if (c == '\n') _tft.println();
    else _tft.print(c);
    autoPageIfNeeded();
  }

  void consoleWrite(const char* s, size_t n) {
    if (!_ready || !s || !n) return;
    _tft.startWrite();
    while (n--) {
      char c = *s++;
      if (c == '\r') continue;
      if (c == '\n') _tft.println();
      else _tft.write((uint8_t)c);
    }
    _tft.endWrite();
    autoPageIfNeeded();
  }

  void consoleWriteStr(const char* s) {
    if (!_ready || !s) return;
    while (*s) consoleWriteChar(*s++);
  }

  // Primitives you use
  void fillScreen(uint16_t c) {
    if (!_ready) return;
    _tft.fillScreen(c);
  }

  void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c) {
    if (!_ready) return;
    if (x >= _w || y >= _h) return;
    if (x + w > _w) w = _w - x;
    if (y + h > _h) h = _h - y;
    _tft.fillRect(x, y, w, h, c);
  }

  // File-backed RGB565 blit using your active FS via callbacks.
  // File layout: tightly packed, big-endian RGB565 (same as your code).
  bool drawRGB565FromFS(const char* fname, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    if (!_ready || !fname) return false;
    if (!_fs.valid()) return false;
    if (x >= _w || y >= _h) return false;
    if (x + w > _w || y + h > _h) return false;

    uint32_t need = (uint32_t)w * (uint32_t)h * 2u;
    uint32_t fsz = 0;
    if (!_fs.getFileSize(fname, fsz) || fsz < need) return false;

    _tft.startWrite();
    setAddrWindow(x, y, w, h);

    const uint32_t CHUNK = 1024;  // bytes; must be even
    uint8_t buf[CHUNK];
    uint32_t off = 0, remain = need;
    while (remain) {
      uint32_t n = (remain > CHUNK) ? CHUNK : remain;
      uint32_t got = _fs.readFileRange(fname, off, buf, n);
      if (got != n) {
        _tft.endWrite();
        return false;
      }
      // Byte-swap BE RGB565 -> native for writePixels()
      for (uint32_t i = 0; i < got; i += 2) {
        uint8_t hi = buf[i];
        buf[i] = buf[i + 1];
        buf[i + 1] = hi;
      }
      _tft.writePixels((uint16_t*)buf, got / 2);
      off += n;
      remain -= n;
      yield();
    }
    _tft.endWrite();
    return true;
  }

  // Wire FS callbacks from your active FS
  void setFSCallbacks(const TFT_FSCallbacks& cb) {
    _fs = cb;
  }

  // Accessors
  uint16_t width() const {
    return _w;
  }
  uint16_t height() const {
    return _h;
  }
  uint8_t rotation() const {
    return _rot;
  }

  Adafruit_ST7789& tft() {
    return _tft;
  }

  // Utility: RGB helper (same signature you had)
  static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }

private:
  void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    _tft.setAddrWindow(x + _xstart, y + _ystart, w, h);
  }

  void autoPageIfNeeded() {
    int16_t cy = _tft.getCursorY();
    int16_t ch = 8 * _textSize;
    if (cy + ch > (int16_t)_h) {
      _tft.fillScreen(_bg);
      _tft.setCursor(0, 0);
    }
  }

  Adafruit_ST7789 _tft;
  uint16_t _w, _h;
  uint8_t _rot;
  uint16_t _xstart, _ystart;
  bool _ready;

  // Console settings
  uint16_t _fg, _bg;
  uint8_t _textSize;

  // FS callbacks
  TFT_FSCallbacks _fs;
};

#endif  // TFT_ST7789_HELPER_H