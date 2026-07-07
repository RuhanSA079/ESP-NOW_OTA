# ESP-NOW_OTA

An ESP-NOW OTA updater that can remotely update ESP8266 devices in the field over ESP-NOW — no WiFi router required.    
   
This code can be adapted to work on the ESP32 variants, but I don't have the use-case for it yet, unless you can test and make a PR, it would be appreciated, or I can update the code to work on a ESP32, but compatibility will be limited, as I cannot test all variants and flash sizes of the ESP32, nor the ESP8266.  

   
This code has been confirmed to be working on ESP12-F and generic ESP8266 boards, with 1MB of flash.


> **Disclaimer:** I am not responsible for any damage or data loss. Use this code at your own risk.
> This is a proof of concept, vibe-coded with [Claude Code](https://claude.ai/code).

---

## What it does

This project lets you push firmware updates to ESP8266 devices wirelessly using **ESP-NOW** — Espressif's connectionless peer-to-peer protocol that works without any WiFi infrastructure. Think of it as OTA over a direct radio link.

It consists of three parts:

| Component | Hardware | Role |
|---|---|---|
| **Sender** | ESP8266 (USB to PC) | Bridges PC ↔ target(s) over ESP-NOW |
| **Receiver** | ESP8266 (target device) | Beacons on boot, accepts/flashes firmware |
| **PC Tool** | Python script | Reads `.bin`, streams it to sender over serial |

The sender can update **multiple targets in a single session** (up to 5), each streamed independently at its own pace — a slow or retry-heavy device never holds back the others.   

You are limited to the serial link data-rates, so the data transfer will slow down for more than one-device, as the blocks are streamed individually over the serial link, but in my tests, an ESP-NOW OTA update seems to be much faster than the normal serial port upload.

---

## How it works

```
PC  ──serial──►  Sender ESP8266  ──ESP-NOW──►  Target ESP8266(s)
    ◄──────────              ◄──────────────
```

1. You run the Python tool on your PC, pointing it at a firmware `.bin` file.
2. The PC tool sends the firmware metadata (size, CRC32, version, device type, and how long to wait) to the **sender** over serial at 921600 baud.
3. You reset the **target** device(s). On every boot, each broadcasts ESP-NOW beacons for 3 seconds advertising its current firmware version and device type.
4. The sender hears a beacon, and — once it's heard the first one — waits briefly (well under the target's 3-second listening window) to catch any other devices booting at nearly the same time, then broadcasts an OTA offer to everyone it found.
5. Each target checks the offer's device type against its own; a mismatch is rejected immediately (see [Firmware type checking](#firmware-type-checking)). If the type matches and the version differs, it accepts.
6. The PC streams the firmware to each accepting device independently, in 200-byte chunks. Each chunk is forwarded over ESP-NOW and must be ACK'd by that device before its next chunk is sent (**stop-and-wait flow control per device**).
7. The sender never stores the firmware — only one chunk per active device lives in RAM at a time (no SPIFFS needed on the sender).
8. Once a target has received all chunks, it calls `Update.end()`, verifies the flash, and reboots into the new firmware. The PC tool reports each device's result as it finishes.

The sender also resets itself to idle at the start of every PC tool session (via `CMD_CANCEL`), so a previous run that was interrupted or timed out won't leave it stuck — you can just re-run the tool.

---

## Firmware version checking

The receiver sketch embeds a version tag directly in the compiled binary:

```cpp
#define FIRMWARE_VERSION  1u  // bump this for each release

volatile const uint8_t OTA_VERSION_TAG[] = {
  'O', 'T', 'A', 'V',
  (FIRMWARE_VERSION >> 0) & 0xFF, ...
};
```

The Python tool automatically scans the `.bin` file for the `OTAV` magic tag and extracts the version — no `--version` argument needed. If a target is already running the same version, it rejects the offer and the sender keeps streaming to any other accepting devices.

Use `--force` to override this and flash regardless of version match.

## Firmware type checking

Because a single sender can serve multiple *kinds* of devices (not just multiple units of the same firmware), each receiver sketch also embeds a device-type tag:

```cpp
volatile const uint8_t OTA_TYPE_TAG[] = {
  'O', 'T', 'A', 'T',
  'E', 'S', 'P', '-', 'N', 'O', 'D', 'E', 0, 0, 0, 0   // up to 12 chars, null-padded
};
```

The Python tool scans for the `OTAT` tag the same way it scans for `OTAV`, and includes the extracted type in every OTA offer. A target whose own type tag doesn't match the offer's type rejects it outright, even if the version differs — so pointing the tool at the wrong `.bin` for a given device is a safe no-op rather than a bad flash.

Both tags are read back at runtime by the firmware itself (`getVersion()` / `getDeviceType()`) — this isn't just documentation, it's required, because the linker's `--gc-sections` will silently strip either array if nothing in the sketch actually reads it back.

---

## Requirements

### Hardware
- 2+ ESP8266 boards (NodeMCU, ESP-12E/F, or similar) — one sender, one or more targets (up to 5 in a single session)
- USB cable for the sender board

### Software
- [Arduino IDE](https://www.arduino.cc/en/software) with ESP8266 core installed
- Python 3.8+
- `pyserial` (`pip install pyserial`)

### Arduino flash layout (receiver only)
The receiver uses the ESP8266 `Updater` library, which requires a dual-slot flash layout. In the Arduino IDE:

> **Tools → Flash Size → "4M (1M SPIFFS)"** or **"4M (2M SPIFFS)"**

---

## Setup

### 1. Flash the sender
Open `esp8266_espnow_ota_sender/esp8266_espnow_ota_sender.ino` in the Arduino IDE and flash it to the USB-connected ESP8266. It will sit idle and wait for commands from the PC tool. Its onboard LED also gives you a live diagnostic: a short blip on any received ESP-NOW packet, and a longer flash the instant a beacon is successfully recognized and registered — handy for confirming the radio link is working before you even look at the PC tool's output.

### 2. Build and export the target binary

The PC tool needs a compiled `.bin` file of your receiver/node firmware — **you do not flash the target via USB every time**. After your initial flash, future updates go over ESP-NOW.

To export the binary from the Arduino IDE:

**Arduino IDE 2.x:**
> **Sketch → Export Compiled Binary**

**Arduino IDE 1.x:**
> **Sketch → Export compiled Binary** (or press `Ctrl+Alt+S`)

The `.bin` file will appear in the sketch folder alongside the `.ino` file. Each time you change your application code or bump `FIRMWARE_VERSION`, re-export the binary before running the PC tool.

> **Important:** Always increment `FIRMWARE_VERSION` before exporting a new build, otherwise the target will reject the update (same version check). Also make sure `OTA_TYPE_TAG` matches the actual device you're targeting — a mismatch is rejected, not flashed.

### 3. Flash the target (first time only)
Open your target sketch (e.g. `esp8266_espnow_ota_receiver/esp8266_espnow_ota_receiver.ino`), set your initial `FIRMWARE_VERSION` and `OTA_TYPE_TAG`, and flash it to the device via USB. Add your application code to `startApplication()` and `loop()`. From this point forward, use the PC tool for all future updates.

### 4. Run the PC tool

```bash
pip install pyserial

# Basic usage — waits up to 120s (default) for target(s) to beacon in
python pc_ota_tool.py COM3 firmware.bin

# Custom wait timeout, in seconds
python pc_ota_tool.py COM3 firmware.bin --wait 60

# Force update even if version matches
python pc_ota_tool.py COM3 firmware.bin --force
```

Then reset the target device(s) — any time within the wait window. Each one that beacons in with a matching device type will flash automatically and reboot; the tool prints per-device progress and a final result for each.

---

## Protocol overview

**PC ↔ Sender** (serial, `[0xAA][cmd][len_h][len_l][payload]` / `[0xBB][rsp][len_h][len_l][payload]`):

| Direction | Message | Description |
|---|---|---|
| PC → Sender | `CMD_INIT` | `[fw_size][crc32][version][flags][device_type][discovery_ms]` — starts a session |
| PC → Sender | `CMD_CHUNK_FOR` | `[device_idx][chunk_bytes]` — chunk for one specific device |
| PC → Sender | `CMD_CANCEL` | Force the sender back to idle (sent automatically at the start of every session) |
| Sender → PC | `RSP_OK` | Command accepted |
| Sender → PC | `RSP_ERROR` | `[error_code]` — fatal error (timeout, busy, ESP-NOW init failure, ...) |
| Sender → PC | `RSP_DEVICE_FOUND` | `[device_idx][mac]` — a device accepted the offer |
| Sender → PC | `RSP_VERSION_MATCH` | `[mac]` — device already up to date, skipped |
| Sender → PC | `RSP_TYPE_MISMATCH` | `[mac]` — device rejected, wrong firmware type |
| Sender → PC | `RSP_NEED_CHUNK` | `[device_idx][offset]` — send this device its next chunk |
| Sender → PC | `RSP_DEVICE_DONE` | `[device_idx][status]` — one device finished (0 = ok) |
| Sender → PC | `RSP_COMPLETE` | Session finished, all devices reported |

**Sender ↔ Target(s)** (ESP-NOW):

| Direction | Message | Description |
|---|---|---|
| Target → Sender (broadcast) | `MSG_BEACON` | Sent on boot for 3s: current version + device type |
| Sender → Targets (broadcast) | `MSG_OTA_OFFER` | Version, size, CRC32, device type |
| Target → Sender | `MSG_OTA_ACCEPT` / `MSG_OTA_REJECT` | Accept, or reject (wrong type / same version / flash error) |
| Sender → Target | `MSG_OTA_DATA` | One firmware chunk (unicast) |
| Target → Sender | `MSG_OTA_ACK` / `MSG_OTA_NAK` | Per-chunk acknowledgement |
| Sender → Target | `MSG_OTA_END` | All chunks sent |
| Target → Sender | `MSG_OTA_DONE` | Flash result (success/error) |

---

## Limitations / known issues

- **Range**: ESP-NOW operates on standard 2.4 GHz 802.11 channels. Range is similar to WiFi (~30–100 m line of sight).
- **Up to 5 targets per session**: The sender can stream to up to 5 devices independently in one run; reset more than that and only the first 5 to beacon in are picked up.
- **Target beacon window is short**: each target only advertises itself for 3 seconds after boot, then stops listening for offers until the next reset. The PC tool's `--wait` controls how long the *sender* waits for that beacon to arrive (default 120s, so you have time to physically power-cycle the board) — but once it hears one, it offers back within well under a second, so it doesn't matter that the target's own window is so much shorter.
- **CH340 adapters on Windows**: Some CH340 USB-serial chips do not reliably support 921600 baud. If you see corrupt transfers, try dropping `--baud` to `460800` and update `Serial.begin()` in the sender sketch to match.
- **Flash layout**: If `Update.begin()` fails on the target, check that you compiled with a dual-slot flash layout (see setup above).
- **No encryption**: ESP-NOW traffic is unencrypted in this implementation. Do not use in security-sensitive applications without adding encryption.

---

## License

This project is licensed under the **GNU General Public License v3.0**.
See [LICENSE](LICENSE) for the full text, or visit https://www.gnu.org/licenses/gpl-3.0.html.

You are free to use, modify, and distribute this code under the terms of the GPL v3. Any derivative work must also be distributed under the same license.
