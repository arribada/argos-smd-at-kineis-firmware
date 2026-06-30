#!/usr/bin/env python3
"""
Create DFU Binary Tool for STM32WL55
Generates firmware binary for DFU update (with optional header)

The DFU binary is sent to the bootloader for application update.
It can optionally include an application header for validation.

Usage:
    python create_dfu.py input.elf output_dfu.bin --info
    python create_dfu.py input.elf output_dfu.bin --no-header
"""

import argparse
import struct
import subprocess
import sys
import os
from pathlib import Path

# Memory layout constants
APP_FLASH_BASE = 0x08000000
APP_FLASH_SIZE = 0x33000  # 204KB


def crc32_stm32(data):
    """Calculate CRC32 using STM32 default polynomial (0x04C11DB7)"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc


def run_objcopy(elf_path, bin_path, sections=None):
    """Extract binary from ELF file using arm-none-eabi-objcopy"""
    cmd = ['arm-none-eabi-objcopy', '-O', 'binary']

    if sections:
        for section in sections:
            cmd.extend(['-j', section])

    cmd.extend([str(elf_path), str(bin_path)])

    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"objcopy error: {result.stderr}")
            return False
        return True
    except FileNotFoundError:
        print("ERROR: arm-none-eabi-objcopy not found in PATH")
        return False


def get_elf_info(elf_path):
    """Get ELF file information using arm-none-eabi-size"""
    try:
        result = subprocess.run(
            ['arm-none-eabi-size', str(elf_path)],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            return result.stdout
        return None
    except FileNotFoundError:
        return None


def create_dfu_binary(elf_path, output_path, add_header=True, build_date=None,
                      git_commit=None, show_info=False):
    """Create DFU binary from ELF file"""

    # Extract sections to binary
    sections = [
        '.app_header', '.isr_vector', '.text', '.rodata',
        '.ARM.extab', '.ARM', '.preinit_array', '.init_array',
        '.fini_array', '.data2', '.data'
    ]

    temp_bin = output_path.with_suffix('.tmp')

    print(f"Extracting binary from: {elf_path}")
    if not run_objcopy(elf_path, temp_bin, sections):
        return False

    # Read extracted binary
    with open(temp_bin, 'rb') as f:
        firmware = f.read()

    # Clean up temp file
    temp_bin.unlink()

    if len(firmware) == 0:
        print("ERROR: Extracted binary is empty")
        return False

    print(f"  Firmware size: {len(firmware)} bytes (0x{len(firmware):X})")

    # Validate size
    if len(firmware) > APP_FLASH_SIZE:
        print(f"ERROR: Firmware too large ({len(firmware)} > {APP_FLASH_SIZE})")
        return False

    # Calculate CRC
    firmware_crc = crc32_stm32(firmware)
    print(f"  CRC32: 0x{firmware_crc:08X}")

    # Write output
    with open(output_path, 'wb') as f:
        f.write(firmware)

    print(f"\nDFU binary created: {output_path}")
    print(f"  Size: {len(firmware)} bytes")
    print(f"  CRC32: 0x{firmware_crc:08X}")

    if show_info:
        elf_info = get_elf_info(elf_path)
        if elf_info:
            print(f"\nELF info:\n{elf_info}")

    return True


def main():
    parser = argparse.ArgumentParser(
        description='Create DFU binary for STM32WL55 bootloader update',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python create_dfu.py build/app.elf build/app_dfu.bin --info
    python create_dfu.py build/app.elf build/app_dfu.bin --build-date "2025-01-01" --git-commit "abc1234"
        """
    )

    parser.add_argument('input', help='Input ELF file')
    parser.add_argument('output', help='Output DFU binary file')
    parser.add_argument('--no-header', action='store_true',
                        help='Create binary without app header (legacy mode)')
    parser.add_argument('--build-date', help='Build date string')
    parser.add_argument('--git-commit', help='Git commit hash')
    parser.add_argument('--info', action='store_true',
                        help='Show detailed information')

    args = parser.parse_args()

    # Validate input
    input_path = Path(args.input)
    if not input_path.exists():
        print(f"ERROR: Input file not found: {args.input}")
        return 1

    output_path = Path(args.output)

    # Create output directory if needed
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Create DFU binary
    print("=" * 50)
    print("STM32WL55 DFU Binary Creator")
    print("=" * 50)

    add_header = not args.no_header

    if create_dfu_binary(input_path, output_path, add_header,
                         args.build_date, args.git_commit, args.info):
        print("\n" + "=" * 50)
        print("DFU binary ready for update!")
        print("=" * 50)
        return 0
    else:
        print("\nDFU binary creation FAILED!")
        return 1


if __name__ == '__main__':
    sys.exit(main())
