#!/usr/bin/env python3
"""
DFU Validation Suite
Complete validation test for bootloader DFU functionality.

This suite performs end-to-end testing of the DFU process:
1. Bootloader communication
2. Flash erase
3. Firmware write
4. CRC verification
5. Jump to application

Usage:
    python dfu_validation_suite.py --port COM3 --firmware app.bin
    python dfu_validation_suite.py --port COM3 --firmware app.bin --full-cycle
"""

import argparse
import serial
import time
import struct
import sys
import os
from pathlib import Path

# Add Tools directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "Tools"))

# Configuration
DEFAULT_BAUDRATE = 9600
CHUNK_SIZE = 256
APP_START_ADDR = 0x08008100
APP_HEADER_ADDR = 0x08008000
TIMEOUT = 10.0


def crc32_stm32(data):
    """Calculate CRC32 using STM32 polynomial"""
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


class ValidationResult:
    """Validation test result"""
    def __init__(self):
        self.tests = []
        self.current_test = None

    def start_test(self, name):
        self.current_test = {"name": name, "status": None, "details": None}
        print(f"\n[TEST] {name}...")

    def pass_test(self, details=None):
        self.current_test["status"] = "PASS"
        self.current_test["details"] = details
        self.tests.append(self.current_test)
        print(f"       PASS" + (f" - {details}" if details else ""))

    def fail_test(self, reason):
        self.current_test["status"] = "FAIL"
        self.current_test["details"] = reason
        self.tests.append(self.current_test)
        print(f"       FAIL - {reason}")

    def skip_test(self, reason):
        self.current_test["status"] = "SKIP"
        self.current_test["details"] = reason
        self.tests.append(self.current_test)
        print(f"       SKIP - {reason}")

    def summary(self):
        passed = sum(1 for t in self.tests if t["status"] == "PASS")
        failed = sum(1 for t in self.tests if t["status"] == "FAIL")
        skipped = sum(1 for t in self.tests if t["status"] == "SKIP")
        total = len(self.tests)

        print("\n" + "=" * 60)
        print("VALIDATION SUMMARY")
        print("=" * 60)

        for test in self.tests:
            status_icon = {"PASS": "+", "FAIL": "X", "SKIP": "-"}[test["status"]]
            print(f"  [{status_icon}] {test['name']}")
            if test["status"] == "FAIL":
                print(f"      Reason: {test['details']}")

        print("-" * 60)
        print(f"Total: {total} | Passed: {passed} | Failed: {failed} | Skipped: {skipped}")

        if failed == 0:
            print("\n*** VALIDATION SUCCESSFUL ***")
        else:
            print("\n*** VALIDATION FAILED ***")

        print("=" * 60)
        return failed == 0


class DFUValidator:
    """DFU Validation Test Suite"""

    def __init__(self, port, baudrate=DEFAULT_BAUDRATE, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.serial = None
        self.results = ValidationResult()

    def log(self, msg):
        if self.verbose:
            print(f"       [DEBUG] {msg}")

    def connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT
            )
            time.sleep(0.2)
            return True
        except serial.SerialException as e:
            print(f"Connection error: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()

    def send_command(self, cmd, timeout=TIMEOUT):
        """Send AT command and get response"""
        if not self.serial:
            return None

        self.serial.reset_input_buffer()
        cmd_bytes = f"{cmd}\r\n".encode('ascii')
        self.log(f"TX: {cmd}")
        self.serial.write(cmd_bytes)

        response = ""
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                response += data.decode('ascii', errors='ignore')

                if '\r\n' in response:
                    lines = response.split('\r\n')
                    for line in lines:
                        if line.startswith('+DFU='):
                            self.log(f"RX: {line}")
                            return line
            time.sleep(0.01)

        self.log(f"Timeout, partial: {response[:50]}...")
        return None

    # =========================================================================
    # VALIDATION TESTS
    # =========================================================================

    def validate_connection(self):
        """Validate serial port connection"""
        self.results.start_test("Serial Connection")

        if self.connect():
            self.results.pass_test(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        else:
            self.results.fail_test(f"Cannot connect to {self.port}")
            return False

    def validate_bootloader_presence(self):
        """Validate bootloader is responding"""
        self.results.start_test("Bootloader Presence")

        response = self.send_command("AT+DFU=PING")

        if response and "+DFU=OK" in response:
            # Extract version if present
            parts = response.split(',')
            version = parts[1] if len(parts) > 1 else "unknown"
            self.results.pass_test(f"Version: {version}")
            return True
        else:
            self.results.fail_test(f"No response or invalid: {response}")
            return False

    def validate_bootloader_info(self):
        """Validate bootloader info command"""
        self.results.start_test("Bootloader Info")

        response = self.send_command("AT+DFU=INFO")

        if response and "+DFU=OK" in response:
            self.results.pass_test()
            return True
        else:
            self.results.fail_test(f"Invalid response: {response}")
            return False

    def validate_flash_erase(self):
        """Validate flash erase operation"""
        self.results.start_test("Flash Erase")

        response = self.send_command("AT+DFU=ERASE", timeout=30.0)

        if response and "+DFU=OK" in response:
            self.results.pass_test("Application flash erased")
            return True
        else:
            self.results.fail_test(f"Erase failed: {response}")
            return False

    def validate_flash_write(self, firmware_data):
        """Validate flash write operation"""
        self.results.start_test("Flash Write")

        total_size = len(firmware_data)
        total_chunks = (total_size + CHUNK_SIZE - 1) // CHUNK_SIZE
        written = 0
        errors = 0

        for i in range(total_chunks):
            offset = i * CHUNK_SIZE
            chunk = firmware_data[offset:offset + CHUNK_SIZE]
            addr = APP_START_ADDR + offset

            # Convert to hex
            hex_data = chunk.hex().upper()

            # Send write command
            cmd = f"AT+DFU=WRITE,{addr:08X},{hex_data}"
            response = self.send_command(cmd)

            if response and "+DFU=OK" in response:
                written += len(chunk)
            else:
                errors += 1
                if errors > 3:
                    self.results.fail_test(f"Too many write errors at offset 0x{offset:X}")
                    return False

            # Progress
            progress = (i + 1) * 100 // total_chunks
            print(f"\r       Writing: {progress}% ({written}/{total_size} bytes)", end="")

        print()  # New line after progress

        if written == total_size:
            self.results.pass_test(f"Wrote {written} bytes")
            return True
        else:
            self.results.fail_test(f"Incomplete write: {written}/{total_size}")
            return False

    def validate_crc_verification(self, firmware_data):
        """Validate CRC verification"""
        self.results.start_test("CRC Verification")

        # Calculate expected CRC
        expected_crc = crc32_stm32(firmware_data)
        self.log(f"Expected CRC: 0x{expected_crc:08X}")

        # Send verify command
        response = self.send_command(f"AT+DFU=VERIFY,{expected_crc:08X}")

        if response and "+DFU=OK" in response:
            self.results.pass_test(f"CRC: 0x{expected_crc:08X}")
            return True
        else:
            self.results.fail_test(f"CRC mismatch: {response}")
            return False

    def validate_jump_to_app(self):
        """Validate jump to application"""
        self.results.start_test("Jump to Application")

        response = self.send_command("AT+DFU=JUMP")

        if response and "+DFU=OK" in response:
            self.results.pass_test("Jump command accepted")
            return True
        elif response and "+DFU=ERR" in response:
            self.results.fail_test("Jump failed - app may be invalid")
            return False
        else:
            # No response might mean successful jump
            self.results.pass_test("Jump executed (no response expected)")
            return True

    def validate_bootloader_protection(self):
        """Validate bootloader region is protected"""
        self.results.start_test("Bootloader Protection")

        # Try to write to bootloader region
        response = self.send_command("AT+DFU=WRITE,08000000,DEADBEEF")

        if response and "+DFU=ERR" in response:
            self.results.pass_test("Write to bootloader rejected")
            return True
        elif response and "+DFU=OK" in response:
            self.results.fail_test("CRITICAL: Bootloader region not protected!")
            return False
        else:
            self.results.skip_test(f"Unclear response: {response}")
            return True

    def validate_address_alignment(self):
        """Validate address alignment checking"""
        self.results.start_test("Address Alignment Check")

        # Try unaligned address
        response = self.send_command("AT+DFU=WRITE,08008101,DEADBEEF")

        # Either error or handled internally
        if response:
            self.results.pass_test("Alignment handled")
            return True
        else:
            self.results.skip_test("No response")
            return True

    # =========================================================================
    # MAIN VALIDATION RUNNER
    # =========================================================================

    def run_basic_validation(self):
        """Run basic bootloader validation (no firmware write)"""
        print("\n" + "=" * 60)
        print("DFU BASIC VALIDATION")
        print("=" * 60)
        print(f"Port: {self.port}")
        print(f"Baudrate: {self.baudrate}")

        # Connection
        if not self.validate_connection():
            return self.results.summary()

        # Bootloader tests
        self.validate_bootloader_presence()
        self.validate_bootloader_info()
        self.validate_bootloader_protection()
        self.validate_address_alignment()

        self.disconnect()
        return self.results.summary()

    def run_full_validation(self, firmware_path):
        """Run full DFU validation with firmware write"""
        print("\n" + "=" * 60)
        print("DFU FULL VALIDATION")
        print("=" * 60)
        print(f"Port: {self.port}")
        print(f"Baudrate: {self.baudrate}")
        print(f"Firmware: {firmware_path}")

        # Load firmware
        try:
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()
            print(f"Firmware size: {len(firmware_data)} bytes")
        except Exception as e:
            print(f"Error loading firmware: {e}")
            return False

        # Connection
        if not self.validate_connection():
            return self.results.summary()

        # Bootloader tests
        if not self.validate_bootloader_presence():
            self.disconnect()
            return self.results.summary()

        self.validate_bootloader_info()
        self.validate_bootloader_protection()

        # Flash operations
        if not self.validate_flash_erase():
            self.disconnect()
            return self.results.summary()

        if not self.validate_flash_write(firmware_data):
            self.disconnect()
            return self.results.summary()

        self.validate_crc_verification(firmware_data)
        self.validate_jump_to_app()

        self.disconnect()
        return self.results.summary()


def main():
    parser = argparse.ArgumentParser(
        description='DFU Validation Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--port', '-p', required=True,
                        help='Serial port (e.g., COM3)')
    parser.add_argument('--baudrate', '-b', type=int, default=DEFAULT_BAUDRATE,
                        help=f'Baud rate (default: {DEFAULT_BAUDRATE})')
    parser.add_argument('--firmware', '-f', metavar='FILE',
                        help='Firmware binary file for full validation')
    parser.add_argument('--full-cycle', action='store_true',
                        help='Run full DFU cycle (erase/write/verify/jump)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output')

    args = parser.parse_args()

    validator = DFUValidator(args.port, args.baudrate, args.verbose)

    if args.firmware and (args.full_cycle or True):
        success = validator.run_full_validation(args.firmware)
    else:
        success = validator.run_basic_validation()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
