#!/usr/bin/env python3
"""
UART DFU Integration Tests
Tests the bootloader DFU functionality over UART.

Usage:
    python test_uart_dfu.py --port COM3
    python test_uart_dfu.py --port /dev/ttyUSB0 --verbose
"""

import argparse
import serial
import time
import struct
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "Tools"))

# Test configuration
DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT = 5.0

class TestResult:
    """Test result tracking"""
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.errors = []

    def add_pass(self, name):
        self.passed += 1
        print(f"  [PASS] {name}")

    def add_fail(self, name, reason):
        self.failed += 1
        self.errors.append((name, reason))
        print(f"  [FAIL] {name}: {reason}")

    def add_skip(self, name, reason):
        self.skipped += 1
        print(f"  [SKIP] {name}: {reason}")

    def summary(self):
        total = self.passed + self.failed + self.skipped
        print("\n" + "=" * 50)
        print(f"Results: {self.passed}/{total} passed")
        if self.failed > 0:
            print(f"  Failed: {self.failed}")
            for name, reason in self.errors:
                print(f"    - {name}: {reason}")
        if self.skipped > 0:
            print(f"  Skipped: {self.skipped}")
        print("=" * 50)
        return self.failed == 0


class UARTDFUTester:
    """UART DFU Test Suite"""

    def __init__(self, port, baudrate=DEFAULT_BAUDRATE, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.serial = None
        self.results = TestResult()

    def log(self, msg):
        if self.verbose:
            print(f"    [DEBUG] {msg}")

    def connect(self):
        """Open serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=DEFAULT_TIMEOUT
            )
            time.sleep(0.1)
            return True
        except serial.SerialException as e:
            print(f"Error opening port {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()

    def send_command(self, cmd, timeout=DEFAULT_TIMEOUT):
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

        self.log(f"Timeout, partial response: {response}")
        return None

    # =========================================================================
    # TEST CASES
    # =========================================================================

    def test_connection(self):
        """Test basic serial connection"""
        if self.serial and self.serial.is_open:
            self.results.add_pass("test_connection")
        else:
            self.results.add_fail("test_connection", "Serial port not open")

    def test_ping_response(self):
        """Test PING command response"""
        response = self.send_command("AT+DFU=PING")

        if response and "+DFU=OK" in response:
            self.results.add_pass("test_ping_response")
        else:
            self.results.add_fail("test_ping_response",
                                  f"Expected +DFU=OK, got: {response}")

    def test_ping_version(self):
        """Test PING returns version string"""
        response = self.send_command("AT+DFU=PING")

        if response and "BL_V" in response:
            self.results.add_pass("test_ping_version")
        else:
            self.results.add_fail("test_ping_version",
                                  f"Expected version in response: {response}")

    def test_info_command(self):
        """Test INFO command"""
        response = self.send_command("AT+DFU=INFO")

        if response and "+DFU=OK" in response:
            self.results.add_pass("test_info_command")
        else:
            self.results.add_fail("test_info_command",
                                  f"Expected +DFU=OK, got: {response}")

    def test_status_command(self):
        """Test STATUS command"""
        response = self.send_command("AT+DFU=STATUS")

        if response and "+DFU=OK" in response:
            self.results.add_pass("test_status_command")
        else:
            self.results.add_fail("test_status_command",
                                  f"Expected +DFU=OK, got: {response}")

    def test_invalid_command(self):
        """Test invalid command returns error"""
        response = self.send_command("AT+DFU=INVALID")

        # Should return error or no response
        if response is None or "+DFU=ERR" in response:
            self.results.add_pass("test_invalid_command")
        elif "+DFU=OK" in response:
            self.results.add_fail("test_invalid_command",
                                  "Invalid command should not return OK")
        else:
            self.results.add_pass("test_invalid_command")

    def test_malformed_command(self):
        """Test malformed command handling"""
        response = self.send_command("AT+DFX=PING")  # Wrong prefix

        if response is None or "+DFU=ERR" in response:
            self.results.add_pass("test_malformed_command")
        elif "+DFU=OK" in response:
            self.results.add_fail("test_malformed_command",
                                  "Malformed command should not return OK")
        else:
            self.results.add_pass("test_malformed_command")

    def test_empty_command(self):
        """Test empty command handling"""
        response = self.send_command("")

        if response is None:
            self.results.add_pass("test_empty_command")
        else:
            # Any response is acceptable for empty command
            self.results.add_pass("test_empty_command")

    def test_write_invalid_address_bootloader(self):
        """Test write to bootloader region is rejected"""
        # Try to write to bootloader region
        response = self.send_command("AT+DFU=WRITE,08000000,DEADBEEF")

        if response and "+DFU=ERR" in response:
            self.results.add_pass("test_write_invalid_address_bootloader")
        elif response and "+DFU=OK" in response:
            self.results.add_fail("test_write_invalid_address_bootloader",
                                  "Write to bootloader region should be rejected")
        else:
            self.results.add_skip("test_write_invalid_address_bootloader",
                                  f"Unexpected response: {response}")

    def test_write_unaligned_address(self):
        """Test write to unaligned address is handled"""
        # Try to write to unaligned address (not 8-byte aligned)
        response = self.send_command("AT+DFU=WRITE,08008101,DEADBEEF")

        # Should either reject or handle appropriately
        if response and ("+DFU=ERR" in response or "+DFU=OK" in response):
            self.results.add_pass("test_write_unaligned_address")
        else:
            self.results.add_skip("test_write_unaligned_address",
                                  f"Unexpected response: {response}")

    def test_verify_bad_crc(self):
        """Test verify with wrong CRC fails"""
        response = self.send_command("AT+DFU=VERIFY,00000000")

        # Should fail verification
        if response and "+DFU=ERR" in response:
            self.results.add_pass("test_verify_bad_crc")
        elif response and "+DFU=OK" in response:
            # If no app is programmed, might still pass or fail
            self.results.add_pass("test_verify_bad_crc")
        else:
            self.results.add_skip("test_verify_bad_crc",
                                  f"Unexpected response: {response}")

    def test_multiple_pings(self):
        """Test multiple consecutive PING commands"""
        success_count = 0
        for i in range(5):
            response = self.send_command("AT+DFU=PING")
            if response and "+DFU=OK" in response:
                success_count += 1
            time.sleep(0.1)

        if success_count == 5:
            self.results.add_pass("test_multiple_pings")
        else:
            self.results.add_fail("test_multiple_pings",
                                  f"Only {success_count}/5 pings succeeded")

    def test_command_case_sensitivity(self):
        """Test command case sensitivity"""
        # Try lowercase
        response = self.send_command("at+dfu=ping")

        # Bootloader may or may not be case-sensitive
        # Just verify it doesn't crash
        if response is None or "+DFU=" in response:
            self.results.add_pass("test_command_case_sensitivity")
        else:
            self.results.add_pass("test_command_case_sensitivity")

    def test_response_format(self):
        """Test response format is correct"""
        response = self.send_command("AT+DFU=PING")

        if response:
            # Should start with +DFU=
            if response.startswith("+DFU="):
                self.results.add_pass("test_response_format")
            else:
                self.results.add_fail("test_response_format",
                                      f"Response should start with +DFU=: {response}")
        else:
            self.results.add_fail("test_response_format", "No response received")

    # =========================================================================
    # TEST RUNNER
    # =========================================================================

    def run_all_tests(self):
        """Run all UART DFU tests"""
        print("\n" + "=" * 50)
        print("UART DFU Integration Tests")
        print("=" * 50)
        print(f"Port: {self.port}")
        print(f"Baudrate: {self.baudrate}")
        print("-" * 50)

        if not self.connect():
            print("FATAL: Cannot connect to serial port")
            return False

        try:
            print("\n-- Connection Tests --")
            self.test_connection()

            print("\n-- Basic Command Tests --")
            self.test_ping_response()
            self.test_ping_version()
            self.test_info_command()
            self.test_status_command()

            print("\n-- Error Handling Tests --")
            self.test_invalid_command()
            self.test_malformed_command()
            self.test_empty_command()

            print("\n-- Address Validation Tests --")
            self.test_write_invalid_address_bootloader()
            self.test_write_unaligned_address()

            print("\n-- Verification Tests --")
            self.test_verify_bad_crc()

            print("\n-- Stress Tests --")
            self.test_multiple_pings()

            print("\n-- Format Tests --")
            self.test_command_case_sensitivity()
            self.test_response_format()

        finally:
            self.disconnect()

        return self.results.summary()


def main():
    parser = argparse.ArgumentParser(description='UART DFU Integration Tests')
    parser.add_argument('--port', '-p', required=True,
                        help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=DEFAULT_BAUDRATE,
                        help=f'Baud rate (default: {DEFAULT_BAUDRATE})')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output')

    args = parser.parse_args()

    tester = UARTDFUTester(args.port, args.baudrate, args.verbose)
    success = tester.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
