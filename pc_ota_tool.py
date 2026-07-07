"""
pc_ota_tool.py – ESP-NOW OTA PC Tool  (per-device independent streaming)

Each target device gets its own unicast chunk stream so devices advance
at their own pace.  The sender drives the protocol: it emits
RSP_NEED_CHUNK [device_idx][offset] whenever a device needs its next
chunk; the PC responds with CMD_CHUNK_FOR [device_idx][data].
Multiple devices stream in parallel — no device waits for another.

Usage:
    python pc_ota_tool.py /dev/ttyUSB0 firmware.bin
    python pc_ota_tool.py /dev/ttyUSB0 firmware.bin --baud 921600 --wait 60

Requirements:
    pip install pyserial
"""

import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

import serial

# ── Protocol constants (must match Arduino sketch) ──────────────────────────
SERIAL_START = 0xAA
SERIAL_RESP  = 0xBB

CMD_INIT      = 0x01  # [fw_size:4LE][crc32:4LE][version:4LE][flags:1][device_type:12][discovery_ms:4LE]
CMD_CHUNK_FOR = 0x05  # [device_idx:1][chunk_bytes:<=200]
CMD_STATUS    = 0x03
CMD_CANCEL    = 0x04

RSP_OK            = 0x01
RSP_ERROR         = 0x02  # [error_code:1]
RSP_DEVICE_FOUND  = 0x03  # [device_idx:1][mac:6]
RSP_COMPLETE      = 0x05
RSP_VERSION_MATCH = 0x06  # [mac:6]
RSP_NEED_CHUNK    = 0x07  # [device_idx:1][offset:4LE] — send chunk to this device
RSP_DEVICE_DONE   = 0x08  # [device_idx:1][status:1]   — 0=ok, else flash error
RSP_TYPE_MISMATCH = 0x09  # [mac:6]                    — wrong firmware type rejected

ERROR_NAMES = {
    0x01: "ESP-NOW init failed",
    0x02: "Timeout (no ACK from target)",
    0x03: "Target rejected OTA offer",
    0x04: "CRC mismatch",
    0x05: "Busy / unexpected state",
    0x06: "Target flash error",
}

# Chunk size must be ≤ CHUNK_DATA_SIZE in the Arduino sketch (200 bytes).
# Keeping it equal maximises throughput (one ESP-NOW packet per chunk).
CHUNK_SIZE = 200


class ProtocolError(RuntimeError):
    pass


VERSION_MAGIC = b'OTAV'  # must match OTA_VERSION_TAG in the receiver sketch
TYPE_MAGIC    = b'OTAT'  # must match OTA_TYPE_TAG    in the receiver sketch


def extract_version(firmware: bytes) -> int:
    """Search the binary for the OTAV magic tag and return the embedded version."""
    idx = firmware.find(VERSION_MAGIC)
    if idx == -1:
        raise ValueError(
            "No OTAV version tag found in firmware binary.\n"
            "  Make sure OTA_VERSION_TAG[] is defined in the receiver sketch."
        )
    if idx + 8 > len(firmware):
        raise ValueError("OTAV tag is truncated at end of firmware binary.")
    return struct.unpack_from("<I", firmware, idx + 4)[0]


def extract_device_type(firmware: bytes) -> str:
    """Search the binary for the OTAT magic tag and return the 12-byte type string."""
    idx = firmware.find(TYPE_MAGIC)
    if idx == -1:
        raise ValueError(
            "No OTAT device type tag found in firmware binary.\n"
            "  Make sure OTA_TYPE_TAG[] is defined in the receiver sketch."
        )
    if idx + 16 > len(firmware):
        raise ValueError("OTAT tag is truncated at end of firmware binary.")
    return firmware[idx + 4 : idx + 16].rstrip(b'\x00').decode('ascii')


class OtaTool:
    def __init__(self, port: str, baud: int = 921600, force: bool = False) -> None:
        self.port  = port
        self.baud  = baud
        self.force = force
        self.ser: serial.Serial | None = None

    # ── Connection ────────────────────────────────────────────────────────

    def connect(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=2.0)
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        print(f"  Connected: {self.port} @ {self.baud} baud")

    def disconnect(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ── Low-level framing ─────────────────────────────────────────────────

    def _send(self, cmd: int, data: bytes = b"") -> None:
        pkt = (
            bytes([SERIAL_START, cmd,
                   (len(data) >> 8) & 0xFF,
                   len(data) & 0xFF])
            + data
        )
        self.ser.write(pkt)

    def _recv(self, timeout: float = 5.0) -> tuple[int, bytes]:
        """Read one framed response packet.  Returns (rsp_code, payload)."""
        deadline = time.monotonic() + timeout
        phase = "idle"
        rsp = 0
        length = 0
        buf = bytearray()

        while time.monotonic() < deadline:
            raw = self.ser.read(1)
            if not raw:
                continue
            b = raw[0]

            if phase == "idle":
                if b == SERIAL_RESP:
                    phase = "rsp"
            elif phase == "rsp":
                rsp   = b
                phase = "len_h"
            elif phase == "len_h":
                length = b << 8
                phase  = "len_l"
            elif phase == "len_l":
                length |= b
                if length == 0:
                    return rsp, b""
                buf.clear()
                phase = "data"
            elif phase == "data":
                buf.append(b)
                if len(buf) >= length:
                    return rsp, bytes(buf)

        raise TimeoutError("No response from device")

    def _expect(self, expected: int, timeout: float = 5.0) -> bytes:
        """Wait for a specific response code; raise on error or wrong code."""
        rsp, data = self._recv(timeout)
        if rsp == RSP_ERROR:
            code = data[0] if data else 0xFF
            raise ProtocolError(f"Device error: {ERROR_NAMES.get(code, f'0x{code:02X}')}")
        if rsp != expected:
            raise ProtocolError(
                f"Unexpected response 0x{rsp:02X} (expected 0x{expected:02X})"
            )
        return data

    # ── High-level transfer ───────────────────────────────────────────────

    def cancel(self) -> None:
        try:
            self._send(CMD_CANCEL)
        except Exception:
            pass

    def run(self, firmware_path: str, device_wait: float = 120.0) -> bool:
        """
        Full OTA session.  Returns True if all updating devices succeeded.

        The protocol is event-driven: the sender emits RSP_NEED_CHUNK when
        a device needs its next chunk; this tool responds immediately with
        CMD_CHUNK_FOR.  Multiple devices stream in parallel at their own pace.
        """
        path = Path(firmware_path)
        if not path.exists():
            raise FileNotFoundError(f"Not found: {firmware_path}")

        firmware     = path.read_bytes()
        fw_size      = len(firmware)
        fw_crc       = zlib.crc32(firmware) & 0xFFFFFFFF
        fw_version   = extract_version(firmware)
        fw_type      = extract_device_type(firmware)
        total_chunks = (fw_size + CHUNK_SIZE - 1) // CHUNK_SIZE

        print(f"\n  File    : {path.name}")
        print(f"  Size    : {fw_size:,} bytes")
        print(f"  CRC32   : 0x{fw_crc:08X}")
        print(f"  Version : {fw_version}" + (" (force)" if self.force else ""))
        print(f"  Type    : {fw_type}")
        print(f"  Chunks  : {total_chunks} × {CHUNK_SIZE} B")

        # Drain startup RSP_OK if the sender just powered on
        try:
            rsp, _ = self._recv(timeout=2.0)
            if rsp == RSP_OK:
                print("  Sender ready (startup OK).")
        except TimeoutError:
            pass

        # Force the sender back to idle in case a previous run left it mid-session
        # (e.g. killed before its own timeout could fire) — CMD_CANCEL always
        # resets state regardless of what the firmware currently thinks it's doing.
        self._send(CMD_CANCEL)
        try:
            self._expect(RSP_OK, timeout=2.0)
        except (TimeoutError, ProtocolError):
            pass

        # ── 1. Init ───────────────────────────────────────────────────────
        flags         = 0x01 if self.force else 0x00
        type_bytes    = fw_type.encode('ascii')[:12].ljust(12, b'\x00')
        discovery_ms  = int(device_wait * 1000)
        print("\n  Sending CMD_INIT...")
        self._send(CMD_INIT, struct.pack("<IIIB12sI", fw_size, fw_crc, fw_version,
                                         flags, type_bytes, discovery_ms))
        self._expect(RSP_OK, timeout=5.0)
        print("  >>> Reset ALL target devices now to start beaconing. <<<")
        print(f"  Waiting for targets (up to {device_wait:.0f} s)...")

        # ── 2 + 3: Event loop — handles discovery, streaming, and completion ──
        #
        # RSP_DEVICE_FOUND  → log device (discovery phase)
        # RSP_VERSION_MATCH → log skip  (discovery phase)
        # RSP_NEED_CHUNK    → respond immediately with CMD_CHUNK_FOR (streaming)
        # RSP_DEVICE_DONE   → log result per device
        # RSP_COMPLETE      → session finished, exit loop
        # RSP_ERROR         → fatal or per-device error

        found_macs: dict[int, str] = {}   # device_idx → mac string
        dev_bytes:  dict[int, int] = {}   # device_idx → bytes delivered so far
        dev_done:   dict[int, int] = {}   # device_idx → done_status (0=ok)
        errors:     list[str]      = []
        streaming   = False
        t_start     = time.monotonic()
        # The sender's own discovery window is set to device_wait (see CMD_INIT
        # above), but it only broadcasts the offer and starts collecting accepts
        # *after* that window closes — RSP_DEVICE_FOUND can't arrive until a bit
        # later still. Give the local deadline extra room so we don't bail out
        # right as the sender is finishing that handshake.
        DISCOVERY_GRACE_S = 5.0
        deadline    = time.monotonic() + device_wait + DISCOVERY_GRACE_S

        while True:
            # Discovery polls in <=10s slices so it stays responsive, but only
            # gives up once the overall device_wait deadline has passed.
            # Once streaming, a single 30s silence is fatal outright.
            if not streaming and time.monotonic() >= deadline:
                print("\n  Timed out – no target devices responded.")
                return False
            timeout = (
                min(10.0, max(0.5, deadline - time.monotonic()))
                if not streaming else 30.0
            )
            try:
                rsp, data = self._recv(timeout=timeout)
            except TimeoutError:
                if not streaming:
                    continue
                print("\n  Timeout waiting for device completion.")
                return False

            # ── Device discovered (accepted offer) ────────────────────────
            if rsp == RSP_DEVICE_FOUND:
                if len(data) < 7:
                    continue
                idx = data[0]
                mac = ":".join(f"{b:02X}" for b in data[1:7])
                found_macs[idx] = mac
                dev_bytes[idx]  = 0
                print(f"  Device {idx}: {mac} — accepted")

            # ── Device already current ────────────────────────────────────
            elif rsp == RSP_VERSION_MATCH:
                mac = ":".join(f"{b:02X}" for b in data[:6])
                print(f"  Already v{fw_version}: {mac} (skipped)")

            # ── Device rejected — wrong firmware type ─────────────────────
            elif rsp == RSP_TYPE_MISMATCH:
                if len(data) >= 6:
                    mac = ":".join(f"{b:02X}" for b in data[:6])
                    print(f"  Type mismatch: {mac} rejected "
                          f"(not a '{fw_type}' device — skipped)")

            # ── Sender needs next chunk for a device ──────────────────────
            elif rsp == RSP_NEED_CHUNK:
                if len(data) < 5:
                    continue
                if not streaming:
                    streaming = True
                    if not found_macs:
                        print("\n  No devices accepted the update.")
                        return False
                    print(f"  Streaming to {len(found_macs)} device(s) independently...")

                idx    = data[0]
                offset = struct.unpack_from("<I", data, 1)[0]
                chunk  = firmware[offset : offset + CHUNK_SIZE]
                self._send(CMD_CHUNK_FOR, bytes([idx]) + chunk)

                # Update per-device progress line
                dev_bytes[idx] = offset + len(chunk)
                elapsed = time.monotonic() - t_start or 0.001
                parts = []
                for i, mac in sorted(found_macs.items()):
                    b   = dev_bytes.get(i, 0)
                    pct = int(b * 100 / fw_size)
                    rate = b / elapsed
                    parts.append(f"[{i}] {pct:3d}%  {rate/1024:.1f}KB/s")
                print(f"\r  {' | '.join(parts)}   ", end="", flush=True)

            # ── A device finished flashing ────────────────────────────────
            elif rsp == RSP_DEVICE_DONE:
                if len(data) < 2:
                    continue
                idx    = data[0]
                status = data[1]
                mac    = found_macs.get(idx, "?")
                dev_done[idx] = status
                if status == 0:
                    print(f"\n  Device {idx} ({mac}): flashed OK")
                else:
                    print(f"\n  Device {idx} ({mac}): FAILED (error 0x{status:02X})")
                    errors.append(f"device {idx} ({mac}): flash error 0x{status:02X}")

            # ── All devices finished ───────────────────────────────────────
            elif rsp == RSP_COMPLETE:
                print()
                if errors:
                    for e in errors:
                        print(f"  ERROR: {e}")
                    return False
                elapsed = time.monotonic() - t_start
                print(f"\n  OTA complete — {len(found_macs)} device(s) rebooting "
                      f"({elapsed:.1f} s)")
                return True

            # ── Fatal or per-device error from sender ────────────────────
            elif rsp == RSP_ERROR:
                code = data[0] if data else 0xFF
                msg  = ERROR_NAMES.get(code, f"0x{code:02X}")
                if not streaming:
                    raise ProtocolError(f"Error: {msg}")
                errors.append(msg)
                print(f"\n  Sender error: {msg}")


# ── CLI ────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="ESP-NOW OTA PC Tool – stream firmware to target via ESP-NOW"
    )
    parser.add_argument("port",     help="Serial port (e.g. COM3)")
    parser.add_argument("firmware", help="Firmware .bin file")
    parser.add_argument("--baud",  type=int,   default=921600,
                        help="Baud rate (default: 921600)")
    parser.add_argument("--wait",  type=float, default=120.0,
                        help="Seconds to wait for target beacon (default: 120)")
    parser.add_argument("--force", action="store_true",
                        help="Force update even if target already runs the same version")
    args = parser.parse_args()

    tool = OtaTool(args.port, args.baud, force=args.force)
    try:
        print("=== ESP-NOW OTA Tool (streaming) ===")
        tool.connect()
        success = tool.run(args.firmware, device_wait=args.wait)
    except KeyboardInterrupt:
        print("\n  Cancelled.")
        tool.cancel()
        success = False
    except (FileNotFoundError, ProtocolError, serial.SerialException) as exc:
        print(f"\n  Error: {exc}")
        success = False
    finally:
        tool.disconnect()

    print()
    print("=== DONE ===" if success else "=== FAILED ===")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
