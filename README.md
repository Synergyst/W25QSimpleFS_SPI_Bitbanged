# 🤖 toyOS + Fun Libraries

## 🌟 Overview

This repository is a practical embedded "lab" for RP2040/RP2350-based systems that combines:

- Lightweight file systems over SPI NOR Flash (W25Q) and multi-chip PSRAM
- A framed, CRC-protected RPC link to a co-processor over software serial
- A dual-path execution model: raw binary "blob" executor + a small script language (CoProcLang)
- Background job control with timeouts, cancelation, and a shared mailbox
- Hardware glue for multi-bank PSRAM using a 74HC138 decoder or 74HC138 decoder + 74HC595 shift register, plus in-system programming (ISP) support

It's equal parts "toy OS" exploration and a collection of reusable embedded utilities that solve real constraints in memory, transport, and execution on small MCUs.

---

## ✨ Core Components

### 1) Storage and File Systems

- W25QSimpleFS (SPI NOR Flash)
  - Purpose-built FS for W25Q-class SPI NOR parts (non-volatile program/data storage)
  - Features: fixed-page writes, sector alignment, replace-in-place when possible

- PSRAMAggregateDevice + PSRAMSimpleFS_Multi
  - Aggregates multiple PSRAM chips into one logical space
  - File system optimized for speed and simplicity on volatile media
  - Includes helpers for auto-creating files, verifying integrity, and a non-destructive "smoke test" writer/reader

Hardware glue for PSRAM banking:
- Direct chip select (what you're used to probably)
- 74HC138-based chip select with configurable pins (A0/A1/EN; A2 optional)
- 74HC138 + 74HC595-based chip select with configurable pins (A0/A1/EN; A2 optional)
- Optional clock settle delays for reliable bus transactions

### 2) Execution and Control

- ExecHost (Main CPU)
  - Manages local FS, stages binaries/scripts, and drives the co-processor via an RPC
  - Foreground and background execution flows with a configurable timeout override
  - Shared mailbox buffer for small result/status passing

- CoProcExec + CoProcLang (Co-Processor)
  - Binary blob executor (Thumb) on core1 with argument passing
  - Script engine (CoProcLang) with I/O primitives (e.g., delays, pin ops)
  - Cancelation via mailbox flag; execution state reporting (idle, loaded, running, done)
  - New: named function dispatch (CMD_FUNC) with strict argc checking

### 3) Transport: Framed RPC over Software Serial

- CoProcProto
  - 24-byte header + payload + CRC32(payload)
  - Versioned, CRC-protected, command-based request/response
  - Commands include:
    - HELLO, INFO
    - LOAD_BEGIN/DATA/END, EXEC
    - STATUS, MAILBOX_RD, CANCEL, RESET
    - SCRIPT_BEGIN/DATA/END, SCRIPT_EXEC (also supports ASCII-encoded arg mode)
    - ISP_ENTER, ISP_EXIT
    - FUNC (named function dispatch on co-processor)

Defaults:
- UART over two GPIOs using SoftwareSerial
- 8N1 framing, default 230400 baud

### 4) ISP and Hardware Glue

- BusArbiterWithISP
  - Safe handoff and test harness for entering/exiting an ISP mode on the co-processor side
  - Optional device tests (e.g., discrete logic like 74HC32 POST)
- Clean recovery and "service once" helpers to keep ISP responsive while the RPC loop runs

---

## 🧠 Architecture (at a glance)

- Main CPU
  - Manages Flash/PSRAM FS
  - Presents a rich serial console (help text, file ops, blob/script staging, diagnostics)
  - Drives the co-processor RPC (frame building, CRC, responses)
  - Background job manager (submit, query, cancel)

- Co-Processor (RP2040/2350)
  - Core0: framed serial RPC loop + handlers
  - Core1: execution worker (blob/script)
  - Shared mailbox and cancel flag
  - Optional ISP service loop while idle in transport

---

## 🔌 Hardware Notes (defaults)

- Flash/PSRAM SPI: shared SPI pins with 74HC138 decoder for PSRAM chip select
- PSRAM pins: SCK=GP10, MOSI=GP11, MISO=GP12 (defaults shown in the code)
- 74HC138: EN=GP9, A0=GP8, A1=GP7, A2 optional (GND if unused)
- 74HC138 decoder + 74HC595 shift register wiring: **TODO**
- Co-Processor soft-serial: RX=GP0, TX=GP1 (both sides)

Baud: 230400 by default (configurable)

---

## 🛠️ Getting Started

### Requirements

- RP2040 or RP2350 boards (Earle Philhower Arduino core recommended)
- Arduino IDE (tested using 2.3.2) with the Philhower core/toolchain
- Hardware:
  - SPI NOR Flash (W25Q series or compatible) for non-volatile FS
  - One or more PSRAM chips with a 74HC138 decoder or 74HC138 decoder + 74HC595 shift register for bank selection (optional but supported)

### Build and Flash

- Co-Processor firmware
  - Sketch: main_coproc_softserial.ino
  - Board: RP2040/RP2350 via Earle Philhower core
  - Upload to the co-processor board

- Main CPU firmware
  - Sketch: main_psram_flash_switch_exec_loader_serial.ino
  - Configure storage backend (Flash vs PSRAM)
  - Upload to the main board

- Console
  - Open a 115200 baud serial terminal to the main CPU
  - Type `help` to see supported commands

---

## 🧪 Typical Workflows

### 1) Put a blob into storage and execute it on the co-processor

- Upload a program image into the active FS:
  - putb64 <file> <base64> or puthex <file> <hex>
- Execute via:
  - `exec <file> [args] [&]`  (local background/foreground on main)
  - or `coproc exec <file> [args]` to stage into the co-processor over RPC and run

### 2) Send and run a script on the co-processor

- Place a script file in FS and run:
  - `coproc sexec <file> [args]`
- Script engine supports numeric args in int32 or alternate ASCII mode (for safer token transport)

### 3) Call a named function on the co-processor (new)

- Register functions on the co-processor (in firmware) with fixed or flexible argc
- From the main CPU console:
  - `coproc func <name> [a0..aN]`
- The handler validates existence and argument count and returns status + int32 result

### 4) Inspect status and mailbox

- `coproc status`
- `coproc mbox [n]`
- `coproc cancel` (sets the cancel flag so cooperative jobs can exit)

### 5) ISP operations

- `coproc isp enter`
- `coproc isp exit`

---

## 📦 Protocol Essentials (CoProcProto)

- Header: 24 bytes (MAGIC ‘CPR0', VERSION, CMD, SEQ, LEN, CRC32)
- Payload CRC: IEEE 802.3 polynomial
- Response CMD = request CMD | 0x80
- Status-first payloads (int32 status followed by any result fields)
- Notable commands:
  - EXEC/SCRIPT_EXEC: argc + args, plus timeout; alternate ASCII arg encoding supported
  - FUNC: uint32 name_len + bytes[name] + uint32 argc + argv (classic or ASCII variant)
    - Response: int32 status, int32 result

---

## 🧰 Console Highlights (Main CPU)

Examples (run at 115200 baud):

- File operations:
  - `files`, `info <file>`, `dump <file> <n>`, `mkSlot <file> <reserve>`, `del <file>`
  - `writeblob <file> <blobId>, autogen`
- Execution:
  - `exec <file> [a0..aN] [&]`
  - `bg status|query|kill|cancel`
- Co-Processor:
  - `coproc ping | info | status | cancel | reset`
  - `coproc exec <file> [a0..aN]`
  - `coproc sexec <file> [a0..aN]`
  - `coproc func <name> [a0..aN]`
  - `coproc mbox [n]`
  - `coproc isp enter|exit`
- System:
  - `storage [flash|psram]`, `format`, `wipe`, `wipereboot`, `wipebootloader`, `timeout [ms]`, `meminfo`, `psramsmoketest`, `reboot`

Prebuilt blobs/scripts are included and can be auto-created in storage with autogen to simplify initial testing.

---

## ⚠️ Notes and Caveats

- Blob execution expects even byte length for Thumb entry alignment
- Only one background job is supported at a time
- Cancelation is cooperative (jobs should poll the cancel flag)
- PSRAM smoke test writes and verifies patterns in free regions; designed to be non-destructive to the FS area

---

## 🤝 Contribution

Issues and PRs are welcome. If you extend storage backends, add script opcodes, or improve protocol robustness, please share the changes. Clear repro steps and hardware notes are appreciated to keep the tooling reproducible.

---
