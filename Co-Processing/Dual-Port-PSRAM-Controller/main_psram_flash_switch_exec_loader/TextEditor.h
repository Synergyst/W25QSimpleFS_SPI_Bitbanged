#ifndef TEXTEDITOR_H_
#define TEXTEDITOR_H_
// ----------------- Nano-like editor integration -----------------
namespace Term {
static inline void write(const char* s) {
  if (s) Serial.print(s);
}
static inline void writeChar(char c) {
  Serial.write((uint8_t)c);
}
static inline void clear() {
  write("\x1b[2J\x1b[H");
}
static inline void hideCursor() {
  write("\x1b[?25l");
}
static inline void showCursor() {
  write("\x1b[?25h");
}
static inline void move(int row1, int col1) {
  if (row1 < 1) row1 = 1;
  if (col1 < 1) col1 = 1;
  char buf[24];
  snprintf(buf, sizeof(buf), "\x1b[%d;%dH", row1, col1);
  write(buf);
}
static inline void clrEol() {
  write("\x1b[K");
}
static inline void invertOn() {
  write("\x1b[7m");
}
static inline void invertOff() {
  write("\x1b[0m");
}
static inline void saveCursor() {
  write("\x1b[s");
}
static inline void restoreCursor() {
  write("\x1b[u");
}
static inline void requestCursorPosition() {
  write("\x1b[6n");
}
}  // namespace Term

// Configurable defaults (can be overridden before including this header)
#ifndef TE_DEFAULT_ROWS
#define TE_DEFAULT_ROWS 24
#endif
#ifndef TE_DEFAULT_COLS
#define TE_DEFAULT_COLS 80
#endif
#ifndef TE_MIN_ROWS
#define TE_MIN_ROWS 8
#endif
#ifndef TE_MIN_COLS
#define TE_MIN_COLS 20
#endif
#ifndef TE_GUTTER_WIDTH
#define TE_GUTTER_WIDTH 5
#endif

// Key codes for special handling
enum {
  KEY_UP = 1000,
  KEY_DOWN,
  KEY_RIGHT,
  KEY_LEFT,
  KEY_DELETE,
  KEY_HOME,
  KEY_END,
  KEY_PGUP,
  KEY_PGDN,
  KEY_WORD_LEFT,
  KEY_WORD_RIGHT
};

struct NanoEditor {
  static const int MAX_LINES = 768;
  static const int MAX_COLS = 256;  // allow longer lines than console view

  // Dynamic screen geometry (detected or defaults)
  int screenRows = TE_DEFAULT_ROWS;
  int screenCols = TE_DEFAULT_COLS;

  // Buffer
  char lines[MAX_LINES][MAX_COLS];
  int lineCount = 0;

  // Cursor and view
  int cx = 0;       // cursor x in current line (column, 0-based)
  int cy = 0;       // cursor y (line index, 0-based)
  int rowOff = 0;   // topmost visible line
  int colOff = 0;   // leftmost visible column
  int prefCol = 0;  // preferred column for up/down

  // UI state
  bool modified = false;
  bool showLineNumbers = true;

  char filename[ActiveFS::MAX_NAME + 1];

  // ---- Terminal geometry detection (best-effort) ----
  bool readCursorReport(int& outRow, int& outCol, uint32_t timeoutMs = 150) {
    // Response format: ESC [ rows ; cols R
    char buf[32];
    size_t p = 0;
    uint32_t start = millis();
    // Read until 'R' or timeout
    while ((millis() - start) < timeoutMs) {
      if (Serial.available()) {
        int ch = Serial.read();
        if (p < sizeof(buf) - 1) buf[p++] = (char)ch;
        if (ch == 'R') break;
      } else {
        Exec.pollBackground();
        tight_loop_contents();
        yield();
      }
    }
    buf[p] = 0;
    // Find ESC[
    const char* s = buf;
    while (*s && *s != 0x1B) ++s;
    if (*s != 0x1B) return false;
    ++s;
    if (*s != '[') return false;
    ++s;
    // Parse "rows;cols"
    int r = 0, c = 0;
    while (*s >= '0' && *s <= '9') {
      r = r * 10 + (*s - '0');
      ++s;
    }
    if (*s != ';') return false;
    ++s;
    while (*s >= '0' && *s <= '9') {
      c = c * 10 + (*s - '0');
      ++s;
    }
    if (*s != 'R') return false;
    outRow = r;
    outCol = c;
    return true;
  }

  void detectTerminalSize() {
    // Flush any pending bytes
    while (Serial.available()) Serial.read();
    // Save cursor, move far bottom-right, request position, restore
    Term::saveCursor();
    Term::move(9999, 9999);
    Term::requestCursorPosition();
    int r = 0, c = 0;
    bool ok = readCursorReport(r, c, 200);
    Term::restoreCursor();
    if (ok) {
      screenRows = r;
      screenCols = c;
    } else {
      screenRows = TE_DEFAULT_ROWS;
      screenCols = TE_DEFAULT_COLS;
    }
    if (screenRows < TE_MIN_ROWS) screenRows = TE_MIN_ROWS;
    if (screenCols < TE_MIN_COLS) screenCols = TE_MIN_COLS;
  }

  // Effective view columns accounting for gutter
  // FIX: gutterWidth equals exactly the printed gutter width (digits + trailing space).
  int gutterWidth() const {
    return showLineNumbers ? TE_GUTTER_WIDTH : 0;
  }
  int usableRows() const {
    return screenRows - 2;
  }  // status + help
  int viewCols() const {
    int vc = screenCols - gutterWidth();
    return (vc < 1) ? 1 : vc;
  }

  // Load file (UTF-8 treated as bytes)
  bool load(const char* path) {
    filename[0] = 0;
    if (path) {
      strncpy(filename, path, sizeof(filename) - 1);
      filename[sizeof(filename) - 1] = 0;
    }
    lineCount = 0;
    if (!path || !activeFs.exists(path)) {
      lineCount = 1;
      lines[0][0] = 0;
      cx = cy = rowOff = colOff = prefCol = 0;
      modified = false;
      return true;
    }
    uint32_t sz = 0;
    if (!activeFs.getFileSize(path, sz)) return false;
    char* tmp = (char*)malloc(sz + 1);
    if (!tmp) return false;
    uint32_t got = activeFs.readFile(path, (uint8_t*)tmp, sz);
    if (got != sz) {
      free(tmp);
      return false;
    }
    tmp[sz] = 0;
    char* p = tmp;
    while (*p && lineCount < MAX_LINES) {
      char* e = p;
      while (*e && *e != '\n' && *e != '\r') ++e;
      int L = (int)(e - p);
      if (L >= MAX_COLS) L = MAX_COLS - 1;
      memcpy(lines[lineCount], p, L);
      lines[lineCount][L] = 0;
      lineCount++;
      while (*e == '\n' || *e == '\r') ++e;
      p = e;
    }
    if (lineCount == 0) {
      lineCount = 1;
      lines[0][0] = 0;
    }
    free(tmp);
    cx = cy = rowOff = colOff = prefCol = 0;
    modified = false;
    return true;
  }

  // Save current buffer either to existing filename or prompt for one
  bool save() {
    if (filename[0]) {
      return saveAs(filename);
    }
    char outname[ActiveFS::MAX_NAME + 1];
    if (!promptFilename(outname, sizeof(outname))) return false;
    return saveAs(outname);
  }

  // Slot-safe writer
  bool saveAs(const char* path) {
    if (!path || !checkNameLen(path)) return false;
    // Join buffer into a single blob with '\n'
    uint32_t total = 0;
    for (int i = 0; i < lineCount; ++i) total += (uint32_t)strlen(lines[i]) + 1;
    uint8_t* out = (uint8_t*)malloc(total ? total : 1);
    if (!out) return false;
    uint32_t off = 0;
    for (int i = 0; i < lineCount; ++i) {
      size_t L = strlen(lines[i]);
      memcpy(out + off, lines[i], L);
      off += (uint32_t)L;
      out[off++] = '\n';
    }
    auto alignUp = [](uint32_t v, uint32_t a) -> uint32_t {
      return (v + (a - 1)) & ~(a - 1);
    };
    uint32_t reserve = alignUp(off, ActiveFS::SECTOR_SIZE);
    if (reserve == 0) reserve = ActiveFS::SECTOR_SIZE;
    if (activeFs.exists(path)) {
      (void)activeFs.deleteFile(path);
    }
    bool ok = activeFs.createFileSlot(path, reserve, out, off);
    free(out);
    if (ok) {
      strncpy(filename, path, sizeof(filename) - 1);
      filename[sizeof(filename) - 1] = 0;
      modified = false;
    }
    return ok;
  }

  // ---- UI ----
  void drawStatus(const char* msg) {
    // Status bar (second to last row)
    Term::move(screenRows - 1, 1);
    Term::invertOn();
    // Compute percent
    int percent = 0;
    if (lineCount > 1) {
      percent = (int)((((long long)cy) * 100 + (lineCount - 2)) / (lineCount - 1));
      if (percent < 0) percent = 0;
      if (percent > 100) percent = 100;
    }
    char status[160];
    snprintf(status, sizeof(status), " %s%s  %s  Ln %d, Col %d  %d/%d  %3d%%",
             (filename[0] ? filename : "[No Name]"),
             (modified ? " *" : ""),
             (showLineNumbers ? "[#]" : "[ ]"),
             cy + 1, cx + 1, cy + 1, lineCount, percent);
    Serial.print(status);
    Term::clrEol();
    Term::invertOff();

    // Help bar (last row)
    Term::move(screenRows, 1);
    Serial.print("^X Exit  ^S Save  ^C Pos  ^G Goto  ^F Find  ^N #Toggle  Home/End  PgUp/PgDn  Arrows Move");
    Term::clrEol();

    // Message (row above status if provided)
    if (msg && *msg) {
      Term::move(screenRows - 2, 1);
      Serial.print(msg);
      Term::clrEol();
    } else {
      Term::move(screenRows - 2, 1);
      Term::clrEol();
    }
  }

  void drawRows() {
    // Draw from row 1 to screenRows-2 (reserve bottom 2 rows for bars)
    int usable = usableRows();
    for (int i = 0; i < usable; ++i) {
      int fileRow = rowOff + i;
      Term::move(1 + i, 1);
      Term::clrEol();

      // Gutter with line numbers (optional)
      if (showLineNumbers) {
        if (fileRow < lineCount) {
          char gut[TE_GUTTER_WIDTH + 2];
          int ln = fileRow + 1;
          // right-justify in TE_GUTTER_WIDTH-1 and add a space => total TE_GUTTER_WIDTH
          snprintf(gut, sizeof(gut), "%*d ", TE_GUTTER_WIDTH - 1, ln);
          Serial.print(gut);
        } else {
          // tilde in gutter to mimic classic editors
          char gut[TE_GUTTER_WIDTH + 2];
          snprintf(gut, sizeof(gut), "%*s ", TE_GUTTER_WIDTH - 1, "~");
          Serial.print(gut);
        }
      }

      if (fileRow >= lineCount) {
        if (!showLineNumbers) Serial.print("~");
      } else {
        const char* s = lines[fileRow];
        int len = (int)strlen(s);
        int vcols = viewCols();
        if (len > colOff) {
          const char* p = s + colOff;
          int toShow = len - colOff;
          if (toShow < 0) toShow = 0;
          if (toShow > vcols) toShow = vcols;
          Serial.write((const uint8_t*)p, toShow);
        }
      }
    }
  }

  void refresh(const char* msg = nullptr) {
    Term::hideCursor();
    drawRows();
    drawStatus(msg);
    // place cursor (1-based columns; include gutter width)
    int crow = cy - rowOff + 1;
    int ccol = (showLineNumbers ? gutterWidth() : 0) + (cx - colOff) + 1;
    if (crow < 1) crow = 1;
    if (ccol < 1) ccol = 1;
    int maxRow = screenRows - 2;
    int maxCol = screenCols;
    if (crow > maxRow) crow = maxRow;
    if (ccol > maxCol) ccol = maxCol;
    Term::move(crow, ccol);
    Term::showCursor();
  }

  // ---- Editing actions ----
  void moveLeft() {
    if (cx > 0) {
      cx--;
      prefCol = cx;
      ensureCursorVisible();
      return;
    }
    if (cy > 0) {
      cy--;
      cx = (int)strlen(lines[cy]);
      prefCol = cx;
      if (rowOff > cy) rowOff = cy;
      ensureCursorVisible();
    }
  }
  void moveRight() {
    int L = (int)strlen(lines[cy]);
    if (cx < L) {
      cx++;
      prefCol = cx;
      ensureCursorVisible();
      return;
    }
    if (cy + 1 < lineCount) {
      cy++;
      cx = 0;
      prefCol = cx;
      ensureCursorVisible();
    }
  }
  void moveUp() {
    if (cy > 0) cy--;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void moveDown() {
    if (cy + 1 < lineCount) cy++;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void pageUp() {
    int step = usableRows() > 1 ? (usableRows() - 1) : 1;
    cy -= step;
    if (cy < 0) cy = 0;
    if (rowOff > cy) rowOff = cy;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void pageDown() {
    int step = usableRows() > 1 ? (usableRows() - 1) : 1;
    cy += step;
    if (cy >= lineCount) cy = lineCount - 1;
    int bottom = rowOff + usableRows() - 1;
    if (cy > bottom) rowOff = cy - usableRows() + 1;
    int L = (int)strlen(lines[cy]);
    if (prefCol > L) cx = L;
    else cx = prefCol;
    ensureCursorVisible();
  }
  void homeLine() {
    cx = 0;
    prefCol = 0;
    ensureCursorVisible();
  }
  void endLine() {
    cx = (int)strlen(lines[cy]);
    prefCol = cx;
    ensureCursorVisible();
  }
  void wordLeft() {
    if (cx == 0 && cy > 0) {
      cy--;
      cx = (int)strlen(lines[cy]);
    } else {
      const char* s = lines[cy];
      int i = cx;
      // skip spaces before
      while (i > 0 && (s[i - 1] == ' ' || s[i - 1] == '\t')) --i;
      // skip word
      while (i > 0 && s[i - 1] != ' ' && s[i - 1] != '\t') --i;
      cx = i;
    }
    prefCol = cx;
    ensureCursorVisible();
  }
  void wordRight() {
    const char* s = lines[cy];
    int L = (int)strlen(s);
    int i = cx;
    // skip non-space
    while (i < L && s[i] != ' ' && s[i] != '\t') ++i;
    // skip spaces
    while (i < L && (s[i] == ' ' || s[i] == '\t')) ++i;
    if (i >= L && cy + 1 < lineCount) {
      cy++;
      cx = 0;
    } else {
      cx = i;
    }
    prefCol = cx;
    ensureCursorVisible();
  }
  void ensureCursorVisible() {
    // Horizontal
    if (cx < colOff) colOff = cx;
    int vc = viewCols();
    if (cx >= colOff + vc) colOff = cx - vc + 1;
    if (colOff < 0) colOff = 0;
    // Vertical
    if (cy < rowOff) rowOff = cy;
    int vr = usableRows();
    if (cy >= rowOff + vr) rowOff = cy - vr + 1;
    if (rowOff < 0) rowOff = 0;
  }
  void insertChar(char c) {
    char* line = lines[cy];
    int L = (int)strlen(line);
    if (L >= MAX_COLS - 1) return;  // full, ignore
    if (cx > L) cx = L;
    // shift right
    for (int i = L; i >= cx; --i) line[i + 1] = line[i];
    line[cx] = c;
    cx++;
    prefCol = cx;
    modified = true;
    ensureCursorVisible();
  }
  void backspace() {
    if (cx > 0) {
      char* line = lines[cy];
      int L = (int)strlen(line);
      for (int i = cx - 1; i < L; ++i) line[i] = line[i + 1];
      cx--;
      prefCol = cx;
      modified = true;
      return;
    }
    // at start of line: join with previous if any
    if (cy > 0) {
      int prevL = (int)strlen(lines[cy - 1]);
      int curL = (int)strlen(lines[cy]);
      int canCopy = min(MAX_COLS - 1 - prevL, curL);
      // append as much as fits
      memcpy(lines[cy - 1] + prevL, lines[cy], canCopy);
      lines[cy - 1][prevL + canCopy] = 0;
      // remove this line
      for (int i = cy; i < lineCount - 1; ++i) {
        strcpy(lines[i], lines[i + 1]);
      }
      lineCount--;
      cy--;
      cx = prevL;
      prefCol = cx;
      modified = true;
      ensureCursorVisible();
    }
  }
  void delChar() {
    char* line = lines[cy];
    int L = (int)strlen(line);
    if (cx < L) {
      for (int i = cx; i < L; ++i) line[i] = line[i + 1];
      modified = true;
      return;
    }
    // at end of line: join with next line
    if (cy + 1 < lineCount) {
      int curL = (int)strlen(lines[cy]);
      int nextL = (int)strlen(lines[cy + 1]);
      int canCopy = min(MAX_COLS - 1 - curL, nextL);
      memcpy(lines[cy] + curL, lines[cy + 1], canCopy);
      lines[cy][curL + canCopy] = 0;
      // shift lines up
      for (int i = cy + 1; i < lineCount - 1; ++i) {
        strcpy(lines[i], lines[i + 1]);
      }
      lineCount--;
      modified = true;
    }
  }
  void newline() {
    if (lineCount >= MAX_LINES) return;
    char* line = lines[cy];
    int L = (int)strlen(line);
    // compute leading indent from current line
    int indent = 0;
    while (indent < L && (line[indent] == ' ' || line[indent] == '\t')) indent++;
    // split at cx
    char tail[MAX_COLS];
    int tailLen = (cx < L) ? (L - cx) : 0;
    if (tailLen > 0) {
      memcpy(tail, line + cx, tailLen);
    }
    tail[tailLen] = 0;
    line[cx] = 0;  // truncate current
    // shift lines down to insert
    for (int i = lineCount; i > cy + 1; --i) {
      strcpy(lines[i], lines[i - 1]);
    }
    // insert tail
    strcpy(lines[cy + 1], tail);
    // apply indent to new line (as much as fits)
    int newL = (int)strlen(lines[cy + 1]);
    int add = indent;
    if (add > MAX_COLS - 1 - newL) add = MAX_COLS - 1 - newL;
    if (add > 0) {
      // shift right to make room
      for (int i = newL; i >= 0; --i) lines[cy + 1][i + add] = lines[cy + 1][i];
      // copy indent chars from original line start (spaces/tabs)
      for (int i = 0; i < add; ++i) lines[cy + 1][i] = line[i];
    }
    lineCount++;
    cy++;
    cx = add;
    prefCol = cx;
    modified = true;
    ensureCursorVisible();
  }

  // ---- Reading keys with ESC-sequence parsing ----
  int readKey() {
    for (;;) {
      if (Serial.available() > 0) {
        int ch = Serial.read();
        if (ch == '\r') {
          // swallow optional '\n'
          if (Serial.available() && Serial.peek() == '\n') Serial.read();
          return '\n';
        }
        if (ch == 0x1B) {  // ESC
          // Try to parse CSI or SS3 or simple Alt combos
          uint32_t tstart = millis();
          while (!Serial.available() && (millis() - tstart) < 200) {
            Exec.pollBackground();
            tight_loop_contents();
            yield();
          }
          if (!Serial.available()) return -1;
          int ch1 = Serial.read();
          if (ch1 == '[') {
            // Read a parameterized sequence until we get a final byte
            char seq[16];
            int sp = 0;
            // Collect digits and semicolons, or final char
            int final = 0;
            for (;;) {
              uint32_t ts = millis();
              while (!Serial.available() && (millis() - ts) < 200) {
                Exec.pollBackground();
                tight_loop_contents();
                yield();
              }
              if (!Serial.available()) break;
              int ch2 = Serial.read();
              if ((ch2 >= '0' && ch2 <= '9') || ch2 == ';') {
                if (sp < (int)sizeof(seq) - 1) seq[sp++] = (char)ch2;
              } else {
                final = ch2;
                break;
              }
            }
            seq[sp] = 0;
            // Interpret
            if (final == 'A') return KEY_UP;    // up
            if (final == 'B') return KEY_DOWN;  // down
            if (final == 'C') {
              // check ctrl-right: "1;5C" (or "*;5C")
              if (strstr(seq, ";5")) return KEY_WORD_RIGHT;
              return KEY_RIGHT;
            }
            if (final == 'D') {
              if (strstr(seq, ";5")) return KEY_WORD_LEFT;
              return KEY_LEFT;
            }
            if (final == 'H') return KEY_HOME;  // Home
            if (final == 'F') return KEY_END;   // End
            if (final == '~') {
              int val = atoi(seq);
              if (val == 1 || val == 7) return KEY_HOME;
              if (val == 4 || val == 8) return KEY_END;
              if (val == 3) return KEY_DELETE;  // Delete
              if (val == 5) return KEY_PGUP;
              if (val == 6) return KEY_PGDN;
            }
            // ignore other CSI
            return -1;
          } else if (ch1 == 'O') {
            // SS3 sequences: 'H' Home, 'F' End on some terms
            uint32_t ts = millis();
            while (!Serial.available() && (millis() - ts) < 200) {
              Exec.pollBackground();
              tight_loop_contents();
              yield();
            }
            if (!Serial.available()) return -1;
            int ch2 = Serial.read();
            if (ch2 == 'H') return KEY_HOME;
            if (ch2 == 'F') return KEY_END;
            return -1;
          } else {
            // Possible Alt-key combos: ESC + key
            if (ch1 == 'n' || ch1 == 'N') return 0x0E;  // Alt-N -> map to Ctrl-N
            return -1;
          }
        }
        return ch;
      }
      Exec.pollBackground();
      tight_loop_contents();
      yield();
    }
  }

  // ---- Prompts ----
  // prompt Yes/No, returns 1 for yes, 0 for no, -1 for cancel (Esc)
  int promptYesNo(const char* q) {
    refresh(q);
    for (;;) {
      int k = readKey();
      if (k == 'y' || k == 'Y') return 1;
      if (k == 'n' || k == 'N') return 0;
      if (k == 27) return -1;
    }
  }
  // prompt for small text input (single-line), returns true if Enter
  bool promptLine(const char* title, char* out, size_t outCap, bool digitsOnly = false) {
    char buf[128];
    buf[0] = 0;
    size_t len = 0;
    char msg[160];
    for (;;) {
      snprintf(msg, sizeof(msg), "%s: %s", title, buf);
      refresh(msg);
      int k = readKey();
      if (k == '\n') {
        if (len == 0) return false;
        strncpy(out, buf, outCap - 1);
        out[outCap - 1] = 0;
        return true;
      } else if (k == 27) {
        return false;
      } else if (k == 0x08 || k == 0x7F) {
        if (len > 0) {
          buf[--len] = 0;
        }
      } else if (k >= 32 && k <= 126) {
        if (digitsOnly && !(k >= '0' && k <= '9')) continue;
        if (len + 1 < sizeof(buf)) {
          buf[len++] = (char)k;
          buf[len] = 0;
        }
      }
    }
  }
  // prompt for filename (default shown), return true if got a name in out
  bool promptFilename(char* out, size_t outCap) {
    char buf[ActiveFS::MAX_NAME + 1];
    buf[0] = 0;
    if (filename[0]) strncpy(buf, filename, sizeof(buf) - 1);
    size_t len = strlen(buf);
    char msg[96];
    snprintf(msg, sizeof(msg), "File Name to Write: %s", buf);
    refresh(msg);
    for (;;) {
      int k = readKey();
      if (k == '\n') {
        if (len == 0) return false;
        strncpy(out, buf, outCap - 1);
        out[outCap - 1] = 0;
        return true;
      } else if (k == 27) {
        return false;
      } else if (k == 0x08 || k == 0x7F) {
        if (len > 0) {
          buf[--len] = 0;
        }
      } else if (k >= 32 && k <= 126) {
        if (len + 1 < sizeof(buf)) {
          buf[len++] = (char)k;
          buf[len] = 0;
        }
      }
      snprintf(msg, sizeof(msg), "File Name to Write: %s", buf);
      refresh(msg);
    }
  }

  // ---- Search and navigation helpers ----
  bool gotoLinePrompt() {
    char tmp[16];
    if (!promptLine("Goto Line", tmp, sizeof(tmp), true)) return false;
    long n = strtol(tmp, nullptr, 10);
    if (n <= 0) return false;
    if (n > lineCount) n = lineCount;
    cy = (int)n - 1;
    int L = (int)strlen(lines[cy]);
    cx = (prefCol > L) ? L : prefCol;
    ensureCursorVisible();
    return true;
  }
  bool findPrompt() {
    char pat[64];
    if (!promptLine("Find", pat, sizeof(pat), false)) return false;
    if (!*pat) return false;
    // Search from current position forward, then wrap once
    int startCy = cy;
    int startCx = cx + 1;
    for (int pass = 0; pass < 2; ++pass) {
      for (int y = (pass == 0 ? startCy : 0); y < lineCount; ++y) {
        const char* s = lines[y];
        const char* pos = nullptr;
        if (y == startCy && pass == 0) {
          if ((int)strlen(s) > startCx) pos = strstr(s + startCx, pat);
        } else {
          pos = strstr(s, pat);
        }
        if (pos) {
          cy = y;
          cx = (int)(pos - s);
          prefCol = cx;
          ensureCursorVisible();
          return true;
        }
      }
    }
    refresh("Not found.");
    (void)readKey();
    return false;
  }

  // ---- Main loop ----
  bool run() {
    detectTerminalSize();
    Term::clear();
    refresh("Welcome to Nano-like editor. ^X exit, ^S save, ^N toggle line numbers");

    for (;;) {
      int k = readKey();

      // Control shortcuts
      if (k == 3) {  // Ctrl-C -> show pos
        char m[64];
        snprintf(m, sizeof(m), "Cursor position: Ln %d, Col %d", cy + 1, cx + 1);
        refresh(m);
        continue;
      }
      if (k == 19) {  // Ctrl-S -> Save
        if (save()) refresh("Wrote file.");
        else refresh("Save canceled or failed.");
        continue;
      }
      if (k == 14) {  // Ctrl-N -> toggle line numbers
        showLineNumbers = !showLineNumbers;
        ensureCursorVisible();
        refresh(nullptr);
        continue;
      }
      if (k == 6) {  // Ctrl-F -> Find
        (void)findPrompt();
        refresh(nullptr);
        continue;
      }
      if (k == 7) {  // Ctrl-G -> Goto Line
        (void)gotoLinePrompt();
        refresh(nullptr);
        continue;
      }
      if (k == 12) {  // Ctrl-L -> full redraw
        Term::clear();
        refresh(nullptr);
        continue;
      }
      if (k == 24) {  // Ctrl-X -> prompt save then exit
        int ans = modified ? promptYesNo("Save modified buffer? (Y/N)") : 0;
        if (ans == 1) {
          char outname[ActiveFS::MAX_NAME + 1];
          if (!promptFilename(outname, sizeof(outname))) {
            refresh("Save canceled.");
            continue;  // cancel save, stay
          }
          if (!checkNameLen(outname)) {
            refresh("Error: filename too long.");
            continue;
          }
          if (saveAs(outname)) {
            refresh("Wrote file. Exiting.");
          } else {
            refresh("Write failed! Press any key to continue.");
            (void)readKey();
          }
        }
        // if ans == 0 (No) or not modified, exit without saving
        Term::clear();
        Term::showCursor();
        return true;
      }

      // Navigation and edit keys
      if (k == '\n') {
        newline();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_UP) {
        moveUp();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_DOWN) {
        moveDown();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_RIGHT) {
        moveRight();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_LEFT) {
        moveLeft();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_DELETE) {
        delChar();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_HOME) {
        homeLine();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_END) {
        endLine();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_PGUP) {
        pageUp();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_PGDN) {
        pageDown();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_WORD_LEFT) {
        wordLeft();
        refresh(nullptr);
        continue;
      }
      if (k == KEY_WORD_RIGHT) {
        wordRight();
        refresh(nullptr);
        continue;
      }
      if (k == 0x08 || k == 0x7F) {
        backspace();
        refresh(nullptr);
        continue;
      }
      if (k >= 32 && k <= 126) {
        insertChar((char)k);
        refresh(nullptr);
        continue;
      }
      // ignore other keys
    }
  }
};

// Entry point
static bool runNanoEditor(const char* path) {
  NanoEditor ed;
  if (!ed.load(path)) {
    Console.println("nano: load failed");
    return false;
  }
  bool ok = ed.run();
  return ok;
}
// ----------------- End Nano-like editor -----------------
#endif  // TEXTEDITOR_H_