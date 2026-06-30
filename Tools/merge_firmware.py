#!/usr/bin/env python3
"""
Merge Firmware Tool for STM32WL55
Combines application and bootloader into a single firmware image

Memory Layout:
  App: 0x08000000 - 0x08032FFF (204KB)
  BL:  0x08033000 - 0x0803AFFF (32KB)
  User:0x0803B000 - 0x0803FFFF (20KB) - Not included in merged image

Usage:
    python merge_firmware.py --app app.bin --bootloader bootloader.bin --output-bin full.bin --output-hex full.hex
"""

import argparse
import struct
import sys
from pathlib import Path

# Memory layout constants
FLASH_BASE = 0x08000000
APP_BASE = 0x08000000
APP_SIZE = 0x33000       # 204KB
BL_BASE = 0x08033000
BL_SIZE = 0x8000         # 32KB
FLASH_END = 0x0803B000   # End before user data


def read_binary(filepath):
    """Read binary file"""
    with open(filepath, 'rb') as f:
        return f.read()


def create_intel_hex(data, base_addr):
    """Create Intel HEX format from binary data"""
    lines = []
    offset = 0

    while offset < len(data):
        # Extended linear address record every 64KB
        if offset % 0x10000 == 0:
            addr_high = (base_addr + offset) >> 16
            record = bytes([2, 0, 0, 4, (addr_high >> 8) & 0xFF, addr_high & 0xFF])
            checksum = (~sum(record) + 1) & 0xFF
            lines.append(f":{record.hex().upper()}{checksum:02X}")

        # Data record (max 16 bytes per line)
        chunk_size = min(16, len(data) - offset)
        addr_low = (base_addr + offset) & 0xFFFF
        chunk = data[offset:offset + chunk_size]

        record = bytes([chunk_size, (addr_low >> 8) & 0xFF, addr_low & 0xFF, 0]) + chunk
        checksum = (~sum(record) + 1) & 0xFF
        lines.append(f":{record.hex().upper()}{checksum:02X}")

        offset += chunk_size

    # End of file record
    lines.append(":00000001FF")

    return '\n'.join(lines) + '\n'


def merge_firmware(app_path, bl_path, output_bin, output_hex, app_addr, bl_addr):
    """Merge application and bootloader binaries"""

    # Read input files
    print(f"Reading application: {app_path}")
    app_data = read_binary(app_path)
    print(f"  Size: {len(app_data)} bytes (0x{len(app_data):X})")

    print(f"Reading bootloader: {bl_path}")
    bl_data = read_binary(bl_path)
    print(f"  Size: {len(bl_data)} bytes (0x{len(bl_data):X})")

    # Validate sizes
    if len(app_data) > APP_SIZE:
        print(f"ERROR: Application too large ({len(app_data)} > {APP_SIZE})")
        return False

    if len(bl_data) > BL_SIZE:
        print(f"ERROR: Bootloader too large ({len(bl_data)} > {BL_SIZE})")
        return False

    # Calculate offsets
    app_offset = app_addr - FLASH_BASE
    bl_offset = bl_addr - FLASH_BASE
    total_size = bl_offset + len(bl_data)

    print(f"\nMerging:")
    print(f"  App at offset 0x{app_offset:X} (address 0x{app_addr:08X})")
    print(f"  BL at offset 0x{bl_offset:X} (address 0x{bl_addr:08X})")
    print(f"  Total size: {total_size} bytes (0x{total_size:X})")

    # Create merged binary
    merged = bytearray(total_size)

    # Fill with 0xFF (erased flash)
    for i in range(len(merged)):
        merged[i] = 0xFF

    # Copy application
    merged[app_offset:app_offset + len(app_data)] = app_data

    # Copy bootloader
    merged[bl_offset:bl_offset + len(bl_data)] = bl_data

    # Write binary output
    if output_bin:
        print(f"\nWriting binary: {output_bin}")
        with open(output_bin, 'wb') as f:
            f.write(merged)
        print(f"  Size: {len(merged)} bytes")

    # Write HEX output
    if output_hex:
        print(f"Writing HEX: {output_hex}")
        hex_content = create_intel_hex(bytes(merged), FLASH_BASE)
        with open(output_hex, 'w') as f:
            f.write(hex_content)
        print(f"  Lines: {hex_content.count(chr(10))}")

    return True


def main():
    parser = argparse.ArgumentParser(
        description='Merge application and bootloader into single firmware image',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Memory Layout:
  App: 0x08000000 - 0x08032FFF (204KB)
  BL:  0x08033000 - 0x0803AFFF (32KB)

Example:
  python merge_firmware.py --app build/app.bin --bootloader Bootloader/build/bootloader.bin \\
      --output-bin build/full.bin --output-hex build/full.hex
        """
    )

    parser.add_argument('--app', required=True, help='Application binary file')
    parser.add_argument('--bootloader', required=True, help='Bootloader binary file')
    parser.add_argument('--output-bin', help='Output binary file')
    parser.add_argument('--output-hex', help='Output Intel HEX file')
    parser.add_argument('--app-addr', type=lambda x: int(x, 0), default=APP_BASE,
                        help=f'Application base address (default: 0x{APP_BASE:08X})')
    parser.add_argument('--bl-addr', type=lambda x: int(x, 0), default=BL_BASE,
                        help=f'Bootloader base address (default: 0x{BL_BASE:08X})')

    args = parser.parse_args()

    # Validate inputs
    if not Path(args.app).exists():
        print(f"ERROR: Application file not found: {args.app}")
        return 1

    if not Path(args.bootloader).exists():
        print(f"ERROR: Bootloader file not found: {args.bootloader}")
        return 1

    if not args.output_bin and not args.output_hex:
        print("ERROR: At least one output file (--output-bin or --output-hex) required")
        return 1

    # Create output directories if needed
    for output in [args.output_bin, args.output_hex]:
        if output:
            Path(output).parent.mkdir(parents=True, exist_ok=True)

    # Merge
    print("=" * 50)
    print("STM32WL55 Firmware Merge Tool")
    print("=" * 50)

    if merge_firmware(args.app, args.bootloader, args.output_bin, args.output_hex,
                      args.app_addr, args.bl_addr):
        print("\n" + "=" * 50)
        print("Merge complete!")
        print("=" * 50)
        return 0
    else:
        print("\nMerge FAILED!")
        return 1


if __name__ == '__main__':
    sys.exit(main())
