#!/usr/bin/env python3
"""Finish a TLSR825x firmware image the way Telink's check_fw does, then verify it.

A Telink image carries "KNLT" at offset 8 and its total length at offset 24, and ends in a CRC32 of
every byte before it (standard CRC-32 polynomial and init, no final inversion). Updaters check the
CRC before installing: ATC_BLE_OEPL downloads an image, recomputes it, and silently keeps its own
firmware on a mismatch. The SDK's check_fw only ships as Windows/Linux binaries, so this does the
same job anywhere Python runs.

    finish_image.py IMAGE.bin            append the CRC, fix the length field, verify
    finish_image.py --verify IMAGE.bin   verify only (works on any Telink image, e.g. ATC's)
"""

from __future__ import annotations

import argparse
import struct
import sys
import zlib

MAGIC_OFFSET, MAGIC = 8, b"KNLT"
SIZE_OFFSET = 24


def telink_crc(data: bytes) -> int:
    return zlib.crc32(data) ^ 0xFFFFFFFF


def problems(image: bytes) -> list[str]:
    found = []
    if image[MAGIC_OFFSET:MAGIC_OFFSET + 4] != MAGIC:
        found.append("no KNLT magic at offset 8")
    declared = struct.unpack_from("<I", image, SIZE_OFFSET)[0]
    if declared != len(image):
        found.append(f"length field {declared} != file size {len(image)}")
    if len(image) % 4:
        found.append(f"size {len(image)} is not a multiple of 4")
    stored = struct.unpack_from("<I", image, len(image) - 4)[0]
    if stored != telink_crc(image[:-4]):
        found.append(f"trailing CRC 0x{stored:08x} != computed 0x{telink_crc(image[:-4]):08x}")
    return found


def finish(image: bytes) -> bytes:
    if image[MAGIC_OFFSET:MAGIC_OFFSET + 4] != MAGIC:
        raise SystemExit("not a Telink image: no KNLT magic at offset 8")
    body = bytearray(image + b"\x00" * (-len(image) % 4))
    struct.pack_into("<I", body, SIZE_OFFSET, len(body) + 4)
    return bytes(body) + struct.pack("<I", telink_crc(bytes(body)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--verify", action="store_true", help="check only, do not modify")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        image = f.read()
    if not args.verify:
        image = finish(image)
        with open(args.image, "wb") as f:
            f.write(image)
    bad = problems(image)
    if bad:
        print(f"{args.image}: " + "; ".join(bad), file=sys.stderr)
        return 1
    crc = struct.unpack_from("<I", image, len(image) - 4)[0]
    print(f"image ok: {len(image)} bytes, KNLT header, length field and CRC 0x{crc:08x} consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
