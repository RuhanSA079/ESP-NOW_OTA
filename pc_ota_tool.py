"""
pc_ota_tool.py – ESP-NOW OTA PC Tool  (streaming mode)

The transmitter holds only one chunk in RAM at a time.  The PC drives
the transfer: send a chunk, wait for RSP_READY_DATA (target ACK'd it),
then send the next chunk.  No SPIFFS, no pre-loading.

Usage:
    python pc_ota_tool.py COM3 firmware.bin
    python pc_ota_tool.py COM3 firmware.bin --baud 921600 --wait 60

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

CMD_INIT   = 0x01  # [fw_size:4LE][crc32:4LE]
CMD_DATA   = 0x02  # [chunk bytes]
CMD_STATUS = 0x03
CMD_CANCEL = 0x04

RSP_OK            = 0x01
RSP_ERROR         = 0x02  # [error_code:1]
RSP_DEVICE_FOUND  = 0x03  # [mac:6]
RSP_READY_DATA    = 0x04
RSP_COMPLETE      = 0x05
RSP_VERSION_MATCH = 0x06  # [mac:6] — target already runs offered version

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


MAGIC = b'OTAV'  # must match OTA_VERSION_TAG in the receiver sketch


def extract_version(firmware: bytes) -> int:
    """Search the binary for the OTAV magic tag and return the embedded version."""
    idx = firmware.find(MAGIC)
    if idx == -1:
        raise ValueError(
            "No OTAV version tag found in firmware binary.\n"
            "  Make sure OTA_VERSION_TAG[] is defined in the receiver sketch."
        )
    if idx + 8 > len(firmware):
        raise ValueError("OTAV tag is truncated at end of firmware binary.")
    return struct.unpack_from("<I", firmware, idx + 4)[0]


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
        Full OTA session.  Returns True on success.

        Steps:
          1.  Send CMD_INIT with file size and CRC32.
          2.  Wait for RSP_DEVICE_FOUND (user must reset target device).
          3.  Stream CMD_DATA chunks, one at a time, gated by RSP_READY_DATA.
          4.  Last chunk response is RSP_COMPLETE (after target has flashed).
        """
        path = Path(firmware_path)
        if not path.exists():
            raise FileNotFoundError(f"Not found: {firmware_path}")

        firmware     = path.read_bytes()
        fw_size      = len(firmware)
        fw_crc       = zlib.crc32(firmware) & 0xFFFFFFFF
        fw_version   = extract_version(firmware)
        total_chunks = (fw_size + CHUNK_SIZE - 1) // CHUNK_SIZE

        print(f"\n  File    : {path.name}")
        print(f"  Size    : {fw_size:,} bytes")
        print(f"  CRC32   : 0x{fw_crc:08X}")
        print(f"  Version : {fw_version}" + (" (force)" if self.force else ""))
        print(f"  Chunks  : {total_chunks} × {CHUNK_SIZE} B")

        # Drain startup RSP_OK if the device just powered on
        try:
            rsp, _ = self._recv(timeout=2.0)
            if rsp == RSP_OK:
                print("  Sender ready (startup OK).")
        except TimeoutError:
            pass  # Already running; fine

        # ── 1. Init ───────────────────────────────────────────────────────
        flags = 0x01 if self.force else 0x00
        print("\n  Sending CMD_INIT...")
        self._send(CMD_INIT, struct.pack("<IIIB", fw_size, fw_crc, fw_version, flags))
        self._expect(RSP_OK, timeout=5.0)
        print("  >>> Reset the target device now to start beaconing. <<<")

        # ── 2. Wait for target ────────────────────────────────────────────
        print(f"  Waiting for target (up to {device_wait:.0f} s)...")
        deadline = time.monotonic() + device_wait
        mac      = None

        while time.monotonic() < deadline:
            try:
                rsp, data = self._recv(timeout=min(10.0, deadline - time.monotonic()))
            except TimeoutError:
                print("\n  Timed out – no target device responded.")
                return False

            if rsp == RSP_DEVICE_FOUND:
                mac = ":".join(f"{b:02X}" for b in data[:6])
                print(f"\n  Target: {mac}")
                break
            elif rsp == RSP_VERSION_MATCH:
                skip_mac = ":".join(f"{b:02X}" for b in data[:6])
                if self.force:
                    print(f"\n  WARNING: {skip_mac} rejected force-update "
                          f"(target firmware does not support OTA_FLAG_FORCE)")
                else:
                    print(f"\n  {skip_mac} already runs v{fw_version}, skipping")
                # Sender goes back to listening — wait for a different device
            elif rsp == RSP_ERROR:
                code = data[0] if data else 0xFF
                raise ProtocolError(f"Error: {ERROR_NAMES.get(code, f'0x{code:02X}')}")

        if mac is None:
            print("\n  No target responded within timeout.")
            return False

        # RSP_READY_DATA follows RSP_DEVICE_FOUND immediately
        self._expect(RSP_READY_DATA, timeout=5.0)

        # ── 3. Stream chunks ──────────────────────────────────────────────
        print("  Streaming...")
        offset    = 0
        chunk_idx = 0
        t_start   = time.monotonic()

        while offset < fw_size:
            chunk   = firmware[offset : offset + CHUNK_SIZE]
            is_last = (offset + len(chunk)) >= fw_size

            self._send(CMD_DATA, chunk)

            # Progress display (before blocking on response)
            elapsed = time.monotonic() - t_start or 0.001
            rate    = (offset + len(chunk)) / elapsed
            pct     = int((offset + len(chunk)) * 100 / fw_size)
            print(f"\r  {pct:3d}%  chunk {chunk_idx + 1}/{total_chunks}"
                  f"  {rate / 1024:.1f} KB/s   ", end="", flush=True)

            if is_last:
                # Wait for target to flash (Update.end() can take a few seconds)
                rsp, data = self._recv(timeout=30.0)
                print()  # end progress line
                if rsp == RSP_COMPLETE:
                    print("\n  OTA complete – target is rebooting.")
                    return True
                elif rsp == RSP_ERROR:
                    code = data[0] if data else 0xFF
                    print(f"\n  Flash error: {ERROR_NAMES.get(code, f'0x{code:02X}')}")
                    return False
                else:
                    raise ProtocolError(f"Unexpected final response 0x{rsp:02X}")
            else:
                self._expect(RSP_READY_DATA, timeout=10.0)

            offset    += len(chunk)
            chunk_idx += 1

        return False  # unreachable, but satisfies linter


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
