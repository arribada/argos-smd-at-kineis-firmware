#!/usr/bin/env python3
"""
DFU Tool for STM32WL55 Bootloader
Supports firmware update via UART and SPI

Usage:
    python dfu_tool.py --port COM3 --uart --erase --write app.bin --verify
    python dfu_tool.py --port COM3 --uart --info
    python dfu_tool.py --port COM3 --uart --reset
"""

import argparse
import serial
import time
import struct
import binascii
import sys
from pathlib import Path

# CRC32 calculation (same polynomial as STM32 hardware CRC)
def crc32_stm32(data):
    """Calculate CRC32 using STM32 default polynomial"""
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

class DFUTool:
    """DFU Tool for UART communication with bootloader"""

    CHUNK_SIZE = 256
    TIMEOUT = 5.0

    def __init__(self, port, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None

    def connect(self):
        """Open serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.TIMEOUT
            )
            time.sleep(0.1)  # Wait for connection to stabilize
            return True
        except serial.SerialException as e:
            print(f"Error opening port {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()

    def send_command(self, cmd):
        """Send AT command and get response"""
        if not self.serial:
            return None

        # Clear any pending data
        self.serial.reset_input_buffer()

        # Send command
        cmd_bytes = f"{cmd}\r\n".encode('ascii')
        self.serial.write(cmd_bytes)

        # Wait for response
        response = ""
        start_time = time.time()

        while (time.time() - start_time) < self.TIMEOUT:
            if self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                response += data.decode('ascii', errors='ignore')

                # Check for complete response
                if '\r\n' in response or '+DFU=' in response:
                    # Find the response line
                    lines = response.split('\r\n')
                    for line in lines:
                        if line.startswith('+DFU='):
                            return line
            time.sleep(0.01)

        return None

    def ping(self):
        """Ping bootloader"""
        print("Pinging bootloader...")
        response = self.send_command("AT+DFU=PING")

        if response and "+DFU=OK" in response:
            version = response.split(',')[1] if ',' in response else "Unknown"
            print(f"  Bootloader responded: {version}")
            return True
        else:
            print("  No response from bootloader")
            return False

    def get_info(self):
        """Get bootloader info"""
        print("Getting bootloader info...")
        response = self.send_command("AT+DFU=INFO")

        if response and "+DFU=OK" in response:
            # Parse hex data
            data = response.split(',')[1] if ',' in response else ""
            print(f"  Info: {data}")
            return True
        else:
            print(f"  Failed: {response}")
            return False

    def erase(self):
        """Erase application flash"""
        print("Erasing application flash...")
        response = self.send_command("AT+DFU=ERASE")

        if response and "+DFU=OK" in response:
            print("  Erase complete")
            return True
        else:
            print(f"  Erase failed: {response}")
            return False

    def write_firmware(self, filename):
        """Write firmware binary to flash"""
        filepath = Path(filename)

        if not filepath.exists():
            print(f"Error: File not found: {filename}")
            return False

        # Read firmware
        with open(filepath, 'rb') as f:
            firmware = f.read()

        print(f"Writing firmware: {filename}")
        print(f"  Size: {len(firmware)} bytes")

        # Write in chunks
        base_addr = 0x08008100  # Application start address
        offset = 0
        total_chunks = (len(firmware) + self.CHUNK_SIZE - 1) // self.CHUNK_SIZE

        while offset < len(firmware):
            chunk = firmware[offset:offset + self.CHUNK_SIZE]
            addr = base_addr + offset

            # Convert chunk to hex string
            hex_data = binascii.hexlify(chunk).decode('ascii').upper()

            # Send write command
            cmd = f"AT+DFU=WRITE,{addr:08X},{hex_data}"
            response = self.send_command(cmd)

            if not response or "+DFU=OK" not in response:
                print(f"\n  Write failed at offset 0x{offset:X}: {response}")
                return False

            offset += len(chunk)
            chunk_num = offset // self.CHUNK_SIZE

            # Progress bar
            progress = int((chunk_num / total_chunks) * 50)
            bar = '=' * progress + ' ' * (50 - progress)
            print(f"\r  [{bar}] {chunk_num}/{total_chunks}", end='', flush=True)

        print("\n  Write complete")
        return True

    def verify(self, filename):
        """Verify firmware CRC"""
        filepath = Path(filename)

        if not filepath.exists():
            print(f"Error: File not found: {filename}")
            return False

        # Read firmware and calculate CRC
        with open(filepath, 'rb') as f:
            firmware = f.read()

        crc = crc32_stm32(firmware)
        print(f"Verifying firmware (CRC32: 0x{crc:08X})...")

        # Send verify command
        response = self.send_command(f"AT+DFU=VERIFY,{crc:08X}")

        if response and "+DFU=OK" in response:
            print("  Verification passed")
            return True
        else:
            print(f"  Verification failed: {response}")
            return False

    def reset(self):
        """Reset MCU"""
        print("Resetting MCU...")
        self.send_command("AT+DFU=RESET")
        print("  Reset command sent")
        return True

    def jump(self):
        """Jump to application"""
        print("Jumping to application...")
        response = self.send_command("AT+DFU=JUMP")

        if response and "+DFU=OK" in response:
            print("  Jump command sent")
            return True
        else:
            print(f"  Jump failed: {response}")
            return False

    def get_status(self):
        """Get DFU status"""
        print("Getting DFU status...")
        response = self.send_command("AT+DFU=STATUS")

        if response and "+DFU=OK" in response:
            data = response.split(',')[1] if ',' in response else ""
            print(f"  Status: {data}")
            return True
        else:
            print(f"  Failed: {response}")
            return False


def create_app_header(firmware_data, version=(1, 0, 0)):
    """Create application header for firmware binary"""
    import struct
    import time

    MAGIC = 0x4B494E45  # "KINE"
    HEADER_VERSION = 0x0001
    HEADER_SIZE = 256
    APP_START_ADDR = 0x08008100

    # Calculate CRC of firmware
    app_crc = crc32_stm32(firmware_data)

    # Pack version
    app_version = (version[0] << 16) | (version[1] << 8) | version[2]

    # Create header
    header = bytearray(HEADER_SIZE)

    # Identification block (16 bytes)
    struct.pack_into('<I', header, 0, MAGIC)
    struct.pack_into('<H', header, 4, HEADER_VERSION)
    struct.pack_into('<H', header, 6, HEADER_SIZE)
    struct.pack_into('<I', header, 8, app_version)

    # Flash layout block (16 bytes)
    struct.pack_into('<I', header, 16, APP_START_ADDR)
    struct.pack_into('<I', header, 20, len(firmware_data))
    struct.pack_into('<I', header, 24, APP_START_ADDR)  # Entry point
    struct.pack_into('<I', header, 28, APP_START_ADDR)  # Vector table

    # Integrity block (16 bytes)
    struct.pack_into('<I', header, 32, app_crc)
    # header_crc will be calculated after

    # Build info (32 bytes)
    build_date = time.strftime("%Y-%m-%d %H:%M").encode('ascii')[:16]
    header[48:48+len(build_date)] = build_date

    # Compatibility (16 bytes)
    struct.pack_into('<I', header, 80, 0x00010000)  # Min BL version 1.0.0
    struct.pack_into('<I', header, 84, 0x00000001)  # HW compatibility

    # Calculate header CRC (excluding app_crc and header_crc fields)
    header_for_crc = bytes(header[:32]) + bytes(header[40:])
    header_crc = crc32_stm32(header_for_crc)
    struct.pack_into('<I', header, 36, header_crc)

    return bytes(header)


def main():
    parser = argparse.ArgumentParser(description='DFU Tool for STM32WL55 Bootloader')
    parser.add_argument('--port', '-p', required=True, help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=9600, help='Baud rate (default: 9600)')
    parser.add_argument('--uart', action='store_true', help='Use UART protocol')
    parser.add_argument('--spi', action='store_true', help='Use SPI protocol (not yet implemented)')
    parser.add_argument('--ping', action='store_true', help='Ping bootloader')
    parser.add_argument('--info', action='store_true', help='Get bootloader info')
    parser.add_argument('--erase', action='store_true', help='Erase application flash')
    parser.add_argument('--write', '-w', metavar='FILE', help='Write firmware binary')
    parser.add_argument('--verify', '-v', action='store_true', help='Verify firmware CRC')
    parser.add_argument('--reset', '-r', action='store_true', help='Reset MCU')
    parser.add_argument('--jump', '-j', action='store_true', help='Jump to application')
    parser.add_argument('--status', '-s', action='store_true', help='Get DFU status')
    parser.add_argument('--create-header', metavar='FILE', help='Create app header for firmware')
    parser.add_argument('--version', nargs=3, type=int, default=[1, 0, 0],
                        metavar=('MAJOR', 'MINOR', 'PATCH'), help='Version for header creation')

    args = parser.parse_args()

    # Handle header creation (offline operation)
    if args.create_header:
        filepath = Path(args.create_header)
        if not filepath.exists():
            print(f"Error: File not found: {args.create_header}")
            return 1

        with open(filepath, 'rb') as f:
            firmware = f.read()

        header = create_app_header(firmware, tuple(args.version))

        # Write header + firmware
        output = filepath.with_suffix('.dfu')
        with open(output, 'wb') as f:
            f.write(header)
            f.write(firmware)

        print(f"Created: {output}")
        print(f"  Header size: {len(header)} bytes")
        print(f"  Firmware size: {len(firmware)} bytes")
        print(f"  Total size: {len(header) + len(firmware)} bytes")
        return 0

    # Check protocol selection
    if args.spi:
        print("SPI protocol not yet implemented")
        return 1

    if not args.uart and not args.spi:
        args.uart = True  # Default to UART

    # Create DFU tool
    dfu = DFUTool(args.port, args.baudrate)

    # Connect
    print(f"Connecting to {args.port} at {args.baudrate} baud...")
    if not dfu.connect():
        return 1

    print("Connected")

    try:
        success = True

        # Execute commands in order
        if args.ping:
            success = success and dfu.ping()

        if args.info:
            success = success and dfu.get_info()

        if args.status:
            success = success and dfu.get_status()

        if args.erase:
            success = success and dfu.erase()

        if args.write:
            success = success and dfu.write_firmware(args.write)

        if args.verify and args.write:
            success = success and dfu.verify(args.write)

        if args.jump:
            success = success and dfu.jump()

        if args.reset:
            success = success and dfu.reset()

        return 0 if success else 1

    finally:
        dfu.disconnect()


if __name__ == '__main__':
    sys.exit(main())
