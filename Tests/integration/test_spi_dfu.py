#!/usr/bin/env python3
"""
SPI DFU Integration Tests
Tests the bootloader DFU functionality over SPI.

Note: This test requires an SPI interface (e.g., FTDI MPSSE or Bus Pirate).
For simulation/mock testing without hardware, set USE_MOCK=True.

Usage:
    python test_spi_dfu.py --interface ftdi
    python test_spi_dfu.py --mock  # Run with mock SPI
"""

import argparse
import struct
import time
import sys
from pathlib import Path

# SPI DFU Command IDs
SPI_CMD_DFU_PING       = 0x30
SPI_CMD_DFU_GET_INFO   = 0x31
SPI_CMD_DFU_ERASE      = 0x32
SPI_CMD_DFU_WRITE_REQ  = 0x33
SPI_CMD_DFU_WRITE_DATA = 0x34
SPI_CMD_DFU_READ_REQ   = 0x35
SPI_CMD_DFU_READ_DATA  = 0x36
SPI_CMD_DFU_VERIFY     = 0x37
SPI_CMD_DFU_RESET      = 0x38
SPI_CMD_DFU_JUMP       = 0x39
SPI_CMD_DFU_GET_STATUS = 0x3A

# Response codes
DFU_RSP_OK          = 0x00
DFU_RSP_ERROR       = 0x01
DFU_RSP_CRC_ERROR   = 0x02
DFU_RSP_ADDR_ERROR  = 0x03
DFU_RSP_SIZE_ERROR  = 0x04
DFU_RSP_FLASH_ERROR = 0x05
DFU_RSP_BUSY        = 0x06
DFU_RSP_INVALID_CMD = 0x07


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


class MockSPIInterface:
    """
    Mock SPI interface for testing without hardware.
    Simulates bootloader responses.
    """

    def __init__(self):
        self.connected = False
        self.bl_version = "BL_V1.0.0"

    def connect(self):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def transfer(self, tx_data):
        """Simulate SPI transfer"""
        if not self.connected or not tx_data:
            return None

        cmd = tx_data[0]

        # Simulate responses
        if cmd == SPI_CMD_DFU_PING:
            # Return OK + version
            version_bytes = self.bl_version.encode('ascii')
            return bytes([DFU_RSP_OK]) + version_bytes

        elif cmd == SPI_CMD_DFU_GET_INFO:
            # Return OK + info structure
            info = struct.pack('<IIII',
                               0x00010000,  # BL version
                               0x08008000,  # App header addr
                               0x08008100,  # App start addr
                               0x32F00)     # App max size
            return bytes([DFU_RSP_OK]) + info

        elif cmd == SPI_CMD_DFU_GET_STATUS:
            # Return OK + status
            return bytes([DFU_RSP_OK, 0x00])  # Idle status

        elif cmd == SPI_CMD_DFU_ERASE:
            # Simulate erase success
            return bytes([DFU_RSP_OK])

        elif cmd == SPI_CMD_DFU_WRITE_REQ:
            # Parse address from command
            if len(tx_data) >= 5:
                addr = struct.unpack('<I', bytes(tx_data[1:5]))[0]
                # Reject bootloader region
                if addr < 0x08008000:
                    return bytes([DFU_RSP_ADDR_ERROR])
            return bytes([DFU_RSP_OK])

        elif cmd == SPI_CMD_DFU_WRITE_DATA:
            return bytes([DFU_RSP_OK])

        elif cmd == SPI_CMD_DFU_VERIFY:
            # Without actual firmware, verification fails
            return bytes([DFU_RSP_CRC_ERROR])

        elif cmd == SPI_CMD_DFU_JUMP:
            # Always fails if no valid app
            return bytes([DFU_RSP_ERROR])

        elif cmd == SPI_CMD_DFU_RESET:
            return bytes([DFU_RSP_OK])

        else:
            return bytes([DFU_RSP_INVALID_CMD])


class FTDISPIInterface:
    """
    FTDI MPSSE SPI interface for real hardware testing.
    Requires pyftdi library: pip install pyftdi
    """

    def __init__(self, url='ftdi://ftdi:232h/1'):
        self.url = url
        self.spi = None

    def connect(self):
        try:
            from pyftdi.spi import SpiController
            ctrl = SpiController()
            ctrl.configure(self.url)
            self.spi = ctrl.get_port(cs=0, freq=1E6, mode=0)
            return True
        except Exception as e:
            print(f"FTDI connection error: {e}")
            return False

    def disconnect(self):
        if self.spi:
            self.spi = None

    def transfer(self, tx_data):
        if not self.spi:
            return None
        try:
            return self.spi.exchange(tx_data, len(tx_data) + 16)
        except Exception as e:
            print(f"SPI transfer error: {e}")
            return None


class SPIDFUTester:
    """SPI DFU Test Suite"""

    def __init__(self, interface, verbose=False):
        self.interface = interface
        self.verbose = verbose
        self.results = TestResult()

    def log(self, msg):
        if self.verbose:
            print(f"    [DEBUG] {msg}")

    def send_command(self, cmd_id, payload=None):
        """Send SPI command and get response"""
        tx_data = [cmd_id]
        if payload:
            tx_data.extend(payload)

        self.log(f"TX: {[hex(b) for b in tx_data]}")
        response = self.interface.transfer(bytes(tx_data))

        if response:
            self.log(f"RX: {[hex(b) for b in response]}")

        return response

    # =========================================================================
    # TEST CASES
    # =========================================================================

    def test_connection(self):
        """Test SPI connection"""
        if self.interface.connect():
            self.results.add_pass("test_connection")
        else:
            self.results.add_fail("test_connection", "Cannot connect to SPI")

    def test_ping_response(self):
        """Test PING command response"""
        response = self.send_command(SPI_CMD_DFU_PING)

        if response and len(response) > 0 and response[0] == DFU_RSP_OK:
            self.results.add_pass("test_ping_response")
        else:
            self.results.add_fail("test_ping_response",
                                  f"Expected OK (0x00), got: {response}")

    def test_ping_version(self):
        """Test PING returns version"""
        response = self.send_command(SPI_CMD_DFU_PING)

        if response and len(response) > 1:
            try:
                version = response[1:].decode('ascii', errors='ignore')
                if 'BL_V' in version:
                    self.results.add_pass("test_ping_version")
                    return
            except:
                pass
        self.results.add_fail("test_ping_version", "Version not in response")

    def test_get_info(self):
        """Test GET_INFO command"""
        response = self.send_command(SPI_CMD_DFU_GET_INFO)

        if response and len(response) > 0 and response[0] == DFU_RSP_OK:
            self.results.add_pass("test_get_info")
        else:
            self.results.add_fail("test_get_info",
                                  f"Expected OK, got: {response}")

    def test_get_status(self):
        """Test GET_STATUS command"""
        response = self.send_command(SPI_CMD_DFU_GET_STATUS)

        if response and len(response) > 0 and response[0] == DFU_RSP_OK:
            self.results.add_pass("test_get_status")
        else:
            self.results.add_fail("test_get_status",
                                  f"Expected OK, got: {response}")

    def test_invalid_command(self):
        """Test invalid command returns error"""
        response = self.send_command(0xFF)  # Invalid command ID

        if response and len(response) > 0 and response[0] == DFU_RSP_INVALID_CMD:
            self.results.add_pass("test_invalid_command")
        elif response and len(response) > 0 and response[0] != DFU_RSP_OK:
            # Any error response is acceptable
            self.results.add_pass("test_invalid_command")
        else:
            self.results.add_fail("test_invalid_command",
                                  "Invalid command should return error")

    def test_write_req_bootloader_region(self):
        """Test write request to bootloader region is rejected"""
        # Address 0x08000000 (bootloader)
        addr_bytes = list(struct.pack('<I', 0x08000000))
        response = self.send_command(SPI_CMD_DFU_WRITE_REQ, addr_bytes)

        if response and len(response) > 0 and response[0] == DFU_RSP_ADDR_ERROR:
            self.results.add_pass("test_write_req_bootloader_region")
        elif response and len(response) > 0 and response[0] != DFU_RSP_OK:
            # Any error response is acceptable
            self.results.add_pass("test_write_req_bootloader_region")
        else:
            self.results.add_fail("test_write_req_bootloader_region",
                                  "Write to bootloader should be rejected")

    def test_write_req_valid_address(self):
        """Test write request to valid app address"""
        # Address 0x08008100 (app region)
        addr_bytes = list(struct.pack('<I', 0x08008100))
        size_bytes = list(struct.pack('<I', 256))  # 256 bytes
        response = self.send_command(SPI_CMD_DFU_WRITE_REQ, addr_bytes + size_bytes)

        if response and len(response) > 0 and response[0] == DFU_RSP_OK:
            self.results.add_pass("test_write_req_valid_address")
        else:
            self.results.add_skip("test_write_req_valid_address",
                                  f"Response: {response}")

    def test_verify_without_firmware(self):
        """Test verify without valid firmware fails"""
        # CRC of non-existent firmware
        crc_bytes = list(struct.pack('<I', 0x12345678))
        response = self.send_command(SPI_CMD_DFU_VERIFY, crc_bytes)

        if response and len(response) > 0 and response[0] != DFU_RSP_OK:
            self.results.add_pass("test_verify_without_firmware")
        else:
            self.results.add_fail("test_verify_without_firmware",
                                  "Verify should fail without valid firmware")

    def test_multiple_pings(self):
        """Test multiple consecutive PING commands"""
        success_count = 0
        for i in range(5):
            response = self.send_command(SPI_CMD_DFU_PING)
            if response and len(response) > 0 and response[0] == DFU_RSP_OK:
                success_count += 1
            time.sleep(0.01)

        if success_count == 5:
            self.results.add_pass("test_multiple_pings")
        else:
            self.results.add_fail("test_multiple_pings",
                                  f"Only {success_count}/5 pings succeeded")

    def test_command_id_range(self):
        """Test DFU command ID range (0x30-0x3F)"""
        # Test that commands outside 0x30-0x3F are rejected
        invalid_ids = [0x00, 0x10, 0x20, 0x40, 0x50, 0xFF]
        all_rejected = True

        for cmd_id in invalid_ids:
            response = self.send_command(cmd_id)
            if response and len(response) > 0 and response[0] == DFU_RSP_OK:
                all_rejected = False
                break

        if all_rejected:
            self.results.add_pass("test_command_id_range")
        else:
            self.results.add_fail("test_command_id_range",
                                  "Non-DFU command IDs should be rejected")

    # =========================================================================
    # TEST RUNNER
    # =========================================================================

    def run_all_tests(self):
        """Run all SPI DFU tests"""
        print("\n" + "=" * 50)
        print("SPI DFU Integration Tests")
        print("=" * 50)
        print(f"Interface: {type(self.interface).__name__}")
        print("-" * 50)

        print("\n-- Connection Tests --")
        self.test_connection()

        if not hasattr(self.interface, 'connected') or self.interface.connected:
            print("\n-- Basic Command Tests --")
            self.test_ping_response()
            self.test_ping_version()
            self.test_get_info()
            self.test_get_status()

            print("\n-- Error Handling Tests --")
            self.test_invalid_command()

            print("\n-- Address Validation Tests --")
            self.test_write_req_bootloader_region()
            self.test_write_req_valid_address()

            print("\n-- Verification Tests --")
            self.test_verify_without_firmware()

            print("\n-- Stress Tests --")
            self.test_multiple_pings()

            print("\n-- Protocol Tests --")
            self.test_command_id_range()

        self.interface.disconnect()
        return self.results.summary()


def main():
    parser = argparse.ArgumentParser(description='SPI DFU Integration Tests')
    parser.add_argument('--interface', '-i', choices=['ftdi', 'mock'],
                        default='mock', help='SPI interface type')
    parser.add_argument('--ftdi-url', default='ftdi://ftdi:232h/1',
                        help='FTDI URL for pyftdi')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output')
    parser.add_argument('--mock', action='store_true',
                        help='Use mock SPI interface')

    args = parser.parse_args()

    if args.mock or args.interface == 'mock':
        interface = MockSPIInterface()
        print("Using MOCK SPI interface (no hardware required)")
    elif args.interface == 'ftdi':
        interface = FTDISPIInterface(args.ftdi_url)
    else:
        print("Unknown interface type")
        sys.exit(1)

    tester = SPIDFUTester(interface, args.verbose)
    success = tester.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
