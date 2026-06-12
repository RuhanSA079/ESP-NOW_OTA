# ESP-NOW_OTA

An ESP-NOW OTA updater that can remotely update ESP8266 devices in the field over ESP-NOW — no WiFi router required.

> **Disclaimer:** I am not responsible for any damage or data loss. Use this code at your own risk.
> This is a proof of concept, vibe-coded with [Claude Code](https://claude.ai/code).

---

## What it does

This project lets you push firmware updates to ESP8266 devices wirelessly using **ESP-NOW** — Espressif's connectionless peer-to-peer protocol that works without any WiFi infrastructure. Think of it as OTA over a direct radio link.

It consists of three parts:

| Component | Hardware | Role |
|---|---|---|
| **Sender** | ESP8266 (USB to PC) | Bridges PC ↔ target over ESP-NOW |
| **Receiver** | ESP8266 (target device) | Beacons on boot, accepts/flashes firmware |
| **PC Tool** | Python script | Reads `.bin`, streams it to sender over serial |

---

## How it works

```
PC  ──serial──►  Sender ESP8266  ──ESP-NOW──►  Target ESP8266
    ◄──────────              ◄──────────────
```

1. You run the Python tool on your PC, pointing it at a firmware `.bin` file.
2. The PC tool sends the firmware metadata (size, CRC32, version) to the **sender** over serial at 921600 baud.
3. You reset the **target** device. On every boot, it broadcasts ESP-NOW beacons for 3 seconds advertising its current firmware version.
4. The sender hears the beacon, compares versions, and sends an OTA offer.
5. If the target accepts (version differs), the PC streams the firmware in 200-byte chunks. Each chunk is forwarded over ESP-NOW and must be ACK'd by the target before the next one is sent (**stop-and-wait flow control**).
6. The sender never stores the firmware — only one chunk lives in RAM at a time (no SPIFFS needed on the sender).
7. Once all chunks are received, the target calls `Update.end()`, verifies the flash, and reboots into the new firmware.

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

The Python tool automatically scans the `.bin` file for the `OTAV` magic tag and extracts the version — no `--version` argument needed. If the target is already running the same version, it rejects the offer and the sender keeps listening for other devices.

Use `--force` to override this and flash regardless of version match.

---

## Requirements

### Hardware
- 2× ESP8266 boards (NodeMCU, ESP-12E/F, or similar)
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
Open `esp8266_espnow_ota_sender/esp8266_espnow_ota_sender.ino` in the Arduino IDE and flash it to the USB-connected ESP8266. It will sit idle and wait for commands from the PC tool.

### 2. Build and export the receiver binary

The PC tool needs a compiled `.bin` file of your receiver/node firmware — **you do not flash the receiver via USB every time**. After your initial flash, future updates go over ESP-NOW.

To export the binary from the Arduino IDE:

**Arduino IDE 2.x:**
> **Sketch → Export Compiled Binary**

**Arduino IDE 1.x:**
> **Sketch → Export compiled Binary** (or press `Ctrl+Alt+S`)

The `.bin` file will appear in the sketch folder alongside the `.ino` file. Each time you change your application code or bump `FIRMWARE_VERSION`, re-export the binary before running the PC tool.

> **Important:** Always increment `FIRMWARE_VERSION` in `esp8266_espnow_ota_receiver.ino` before exporting a new build, otherwise the target will reject the update (same version check).

### 3. Flash the receiver (first time only)
Open `esp8266_espnow_ota_receiver/esp8266_espnow_ota_receiver.ino`, set your initial `FIRMWARE_VERSION`, and flash it to your target device via USB. Add your application code to `startApplication()` and `loop()`. From this point forward, use the PC tool for all future updates.

### 4. Run the PC tool

```bash
pip install pyserial

# Basic usage
python pc_ota_tool.py COM3 firmware.bin

# Custom wait timeout
python pc_ota_tool.py COM3 firmware.bin --wait 60

# Force update even if version matches
python pc_ota_tool.py COM3 firmware.bin --force
```

Then reset the target device. If an update is available it will flash automatically and reboot.

---

## Protocol overview

| Direction | Message | Description |
|---|---|---|
| PC → Sender | `CMD_INIT` | Firmware size, CRC32, version, flags |
| PC → Sender | `CMD_DATA` | One 200-byte firmware chunk |
| Sender → Target | `MSG_OTA_OFFER` | Offer with version and size |
| Target → Sender | `MSG_OTA_ACCEPT` / `MSG_OTA_REJECT` | Accept or reject offer |
| Sender → Target | `MSG_OTA_DATA` | Chunk forwarded over ESP-NOW |
| Target → Sender | `MSG_OTA_ACK` / `MSG_OTA_NAK` | Per-chunk acknowledgement |
| Target → Sender | `MSG_OTA_DONE` | Flash result (success/error) |
| Sender → PC | `RSP_COMPLETE` | OTA finished, target rebooting |

Serial framing: `[0xAA][cmd][len_h][len_l][payload...]`  
Response framing: `[0xBB][rsp][len_h][len_l][payload...]`

---

## Limitations / known issues

- **Range**: ESP-NOW operates on standard 2.4 GHz 802.11 channels. Range is similar to WiFi (~30–100 m line of sight).
- **One target at a time**: The sender updates one device per session. Reset the target to trigger a new update cycle.
- **CH340 adapters on Windows**: Some CH340 USB-serial chips do not reliably support 921600 baud. If you see corrupt transfers, try dropping `--baud` to `460800` and update `Serial.begin()` in the sender sketch to match.
- **Flash layout**: If `Update.begin()` fails on the target, check that you compiled with a dual-slot flash layout (see setup above).
- **No encryption**: ESP-NOW traffic is unencrypted in this implementation. Do not use in security-sensitive applications without adding encryption.

---

## License

This project is licensed under the **GNU General Public License v3.0**.  
See [LICENSE](LICENSE) for the full text, or visit https://www.gnu.org/licenses/gpl-3.0.html.

You are free to use, modify, and distribute this code under the terms of the GPL v3. Any derivative work must also be distributed under the same license.
