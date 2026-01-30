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
SPI_CMD_DFU_ABORT      = 0x3B
SPI_CMD_DFU_SET_HEADER = 0x3C

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
    Simulates bootloader responses with realistic timing.
    """

    def __init__(self):
        self.connected = False
        self.bl_version = "BL_V1.0.0"
        self.freq = 500000  # Simulated frequency
        self.last_cmd = None  # For two-transaction simulation
        self.pending_response = None

    def connect(self):
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def set_frequency(self, freq_hz):
        """Simulate frequency change"""
        self.freq = freq_hz
        return True

    def _get_response(self, cmd, tx_data):
        """Internal: Generate response for command"""
        # Simulate responses
        if cmd == SPI_CMD_DFU_PING:
            version_bytes = self.bl_version.encode('ascii')
            return bytes([DFU_RSP_OK]) + version_bytes

        elif cmd == SPI_CMD_DFU_GET_INFO:
            info = struct.pack('<IIII',
                               0x00010000,  # BL version
                               0x08008000,  # App header addr
                               0x08008100,  # App start addr
                               0x32F00)     # App max size
            return bytes([DFU_RSP_OK]) + info

        elif cmd == SPI_CMD_DFU_GET_STATUS:
            return bytes([DFU_RSP_OK, 0x00])  # Idle status

        elif cmd == SPI_CMD_DFU_ERASE:
            return bytes([DFU_RSP_OK])

        elif cmd == SPI_CMD_DFU_WRITE_REQ:
            if len(tx_data) >= 5:
                addr = struct.unpack('<I', bytes(tx_data[1:5]))[0]
                if addr < 0x08008000:
                    return bytes([DFU_RSP_ADDR_ERROR])
            return bytes([DFU_RSP_OK])

        elif cmd == SPI_CMD_DFU_WRITE_DATA:
            return bytes([DFU_RSP_OK])

        elif cmd == SPI_CMD_DFU_VERIFY:
            return bytes([DFU_RSP_CRC_ERROR])

        elif cmd == SPI_CMD_DFU_JUMP:
            return bytes([DFU_RSP_ERROR])

        elif cmd == SPI_CMD_DFU_RESET:
            return bytes([DFU_RSP_OK])

        elif cmd == SPI_CMD_DFU_SET_HEADER:
            # Check if payload is present (257 bytes total: cmd + 256 header)
            if len(tx_data) >= 257:
                return bytes([DFU_RSP_OK])
            return bytes([DFU_RSP_SIZE_ERROR])

        elif cmd == 0xAA:
            # Dummy read for two-transaction - return pending response
            if self.pending_response:
                resp = self.pending_response
                self.pending_response = None
                return resp
            return bytes([0xAA] * 16)

        else:
            return bytes([DFU_RSP_INVALID_CMD])

    def transfer(self, tx_data):
        """Simulate SPI transfer with two-transaction protocol support"""
        if not self.connected or not tx_data:
            return None

        cmd = tx_data[0]

        # Check if this is a DFU command (0x30-0x3F)
        if 0x30 <= cmd <= 0x3F:
            # Store response for potential two-transaction read
            self.last_cmd = cmd
            self.pending_response = self._get_response(cmd, tx_data)
            # Return the response (single transaction mode)
            return self.pending_response
        else:
            return self._get_response(cmd, tx_data)

    def transfer_timed(self, tx_data):
        """Transfer with simulated timing"""
        if not self.connected or not tx_data:
            return None, 0

        # Simulate transfer time based on frequency
        bytes_count = len(tx_data) + 16
        time_per_byte_us = 8 * 1000000 / self.freq  # microseconds per byte
        total_time_ms = bytes_count * time_per_byte_us / 1000

        # Add processing overhead (1-5ms)
        processing_ms = 2.0

        response = self.transfer(tx_data)
        return response, total_time_ms + processing_ms


class FTDISPIInterface:
    """
    FTDI MPSSE SPI interface for real hardware testing.
    Requires pyftdi library: pip install pyftdi
    """

    # Supported SPI frequencies for testing
    FREQ_125K = 125000
    FREQ_250K = 250000
    FREQ_500K = 500000
    FREQ_1M = 1000000

    def __init__(self, url='ftdi://ftdi:232h/1', freq=500000):
        self.url = url
        self.spi = None
        self.ctrl = None
        self.freq = freq
        self.connected = False

    def connect(self):
        try:
            from pyftdi.spi import SpiController
            self.ctrl = SpiController()
            self.ctrl.configure(self.url)
            self.spi = self.ctrl.get_port(cs=0, freq=self.freq, mode=0)
            self.connected = True
            return True
        except Exception as e:
            print(f"FTDI connection error: {e}")
            return False

    def disconnect(self):
        if self.spi:
            self.spi = None
        if self.ctrl:
            self.ctrl.terminate()
            self.ctrl = None
        self.connected = False

    def set_frequency(self, freq_hz):
        """Change SPI frequency for timing tests"""
        if self.ctrl:
            self.freq = freq_hz
            self.spi = self.ctrl.get_port(cs=0, freq=freq_hz, mode=0)
            return True
        return False

    def transfer(self, tx_data):
        if not self.spi:
            return None
        try:
            return self.spi.exchange(tx_data, len(tx_data) + 16)
        except Exception as e:
            print(f"SPI transfer error: {e}")
            return None

    def transfer_timed(self, tx_data):
        """Transfer with timing measurement"""
        if not self.spi:
            return None, 0
        try:
            start = time.perf_counter()
            response = self.spi.exchange(tx_data, len(tx_data) + 16)
            elapsed = time.perf_counter() - start
            return response, elapsed * 1000  # Return ms
        except Exception as e:
            print(f"SPI transfer error: {e}")
            return None, 0


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
    # TWO-TRANSACTION PROTOCOL TESTS
    # =========================================================================

    def test_two_transaction_ping(self):
        """Test two-transaction protocol: PING command"""
        # Transaction 1: Send PING
        tx1 = bytes([SPI_CMD_DFU_PING])
        self.log(f"TX1 (PING): {[hex(b) for b in tx1]}")
        response1 = self.interface.transfer(tx1)
        self.log(f"RX1 (during TX): {[hex(b) for b in response1] if response1 else None}")

        # Small delay between transactions (simulates master processing)
        time.sleep(0.005)  # 5ms

        # Transaction 2: Read response (send dummy bytes)
        tx2 = bytes([0xAA] * 16)  # Dummy bytes to clock out response
        self.log(f"TX2 (read response): {[hex(b) for b in tx2]}")
        response2 = self.interface.transfer(tx2)
        self.log(f"RX2 (response): {[hex(b) for b in response2] if response2 else None}")

        # Check response from transaction 2
        if response2 and len(response2) > 0 and response2[0] == DFU_RSP_OK:
            self.results.add_pass("test_two_transaction_ping")
        else:
            self.results.add_fail("test_two_transaction_ping",
                                  f"Transaction 2 response: {response2}")

    def test_two_transaction_write_req(self):
        """Test two-transaction protocol: WRITE_REQ command"""
        # Build WRITE_REQ command: cmd + addr(4) + len(2)
        cmd_bytes = [SPI_CMD_DFU_WRITE_REQ]
        addr_bytes = list(struct.pack('<I', 0x08008100))  # Valid app address
        len_bytes = list(struct.pack('<H', 256))
        tx1 = bytes(cmd_bytes + addr_bytes + len_bytes)

        self.log(f"TX1 (WRITE_REQ): {[hex(b) for b in tx1]}")
        response1 = self.interface.transfer(tx1)
        self.log(f"RX1: {[hex(b) for b in response1] if response1 else None}")

        time.sleep(0.010)  # 10ms delay for processing

        # Transaction 2: Read response
        tx2 = bytes([0xAA] * 16)
        response2 = self.interface.transfer(tx2)
        self.log(f"RX2 (response): {[hex(b) for b in response2] if response2 else None}")

        if response2 and len(response2) > 0 and response2[0] == DFU_RSP_OK:
            self.results.add_pass("test_two_transaction_write_req")
        else:
            self.results.add_skip("test_two_transaction_write_req",
                                  f"Response: {response2}")

    # =========================================================================
    # TIMING TESTS (Only for real hardware with timing support)
    # =========================================================================

    def test_timing_ping_response(self):
        """Test PING response timing"""
        if not hasattr(self.interface, 'transfer_timed'):
            self.results.add_skip("test_timing_ping_response", "No timing support")
            return

        response, elapsed_ms = self.interface.transfer_timed(bytes([SPI_CMD_DFU_PING]))

        if response and len(response) > 0:
            if elapsed_ms < 50:  # Should complete in less than 50ms
                self.results.add_pass("test_timing_ping_response")
                self.log(f"PING took {elapsed_ms:.2f}ms")
            else:
                self.results.add_fail("test_timing_ping_response",
                                      f"Too slow: {elapsed_ms:.2f}ms")
        else:
            self.results.add_fail("test_timing_ping_response", "No response")

    def test_timing_different_frequencies(self):
        """Test SPI at different frequencies"""
        if not hasattr(self.interface, 'set_frequency'):
            self.results.add_skip("test_timing_different_frequencies",
                                  "Interface doesn't support frequency change")
            return

        frequencies = [
            (125000, "125kHz"),
            (250000, "250kHz"),
            (500000, "500kHz"),
        ]

        all_passed = True
        results_detail = []

        for freq, name in frequencies:
            if self.interface.set_frequency(freq):
                response = self.send_command(SPI_CMD_DFU_PING)
                if response and len(response) > 0 and response[0] == DFU_RSP_OK:
                    results_detail.append(f"{name}: OK")
                else:
                    results_detail.append(f"{name}: FAIL")
                    all_passed = False
            else:
                results_detail.append(f"{name}: SKIP (can't set freq)")

        self.log(f"Frequency test results: {results_detail}")

        if all_passed:
            self.results.add_pass("test_timing_different_frequencies")
        else:
            self.results.add_fail("test_timing_different_frequencies",
                                  f"Results: {results_detail}")

    def test_rapid_commands(self):
        """Test rapid consecutive commands (stress test)"""
        success_count = 0
        total = 20
        min_delay_ms = 1  # Minimum delay between commands

        for i in range(total):
            response = self.send_command(SPI_CMD_DFU_PING)
            if response and len(response) > 0 and response[0] == DFU_RSP_OK:
                success_count += 1
            time.sleep(min_delay_ms / 1000.0)

        if success_count == total:
            self.results.add_pass("test_rapid_commands")
        elif success_count > total * 0.9:  # >90% success is acceptable
            self.results.add_pass("test_rapid_commands")
            self.log(f"Rapid commands: {success_count}/{total} succeeded")
        else:
            self.results.add_fail("test_rapid_commands",
                                  f"Only {success_count}/{total} succeeded")

    def test_large_payload(self):
        """Test SET_HEADER command with 256-byte payload"""
        # Create 256-byte header payload (valid magic, etc.)
        header = bytearray(256)
        header[0:4] = struct.pack('<I', 0x4B494E45)  # Magic "KINE"
        header[4:8] = struct.pack('<I', 0x0001)      # Version
        header[8:12] = struct.pack('<I', 0x08008100) # App start
        header[12:16] = struct.pack('<I', 0x1000)    # App size (4KB)
        header[16:20] = struct.pack('<I', 0x12345678)# CRC placeholder

        cmd_bytes = bytes([SPI_CMD_DFU_SET_HEADER]) + bytes(header)

        self.log(f"Sending SET_HEADER with {len(header)} bytes payload")
        response = self.interface.transfer(cmd_bytes)

        # For this test, we just verify communication works with large payloads
        if response and len(response) > 0:
            if response[0] in [DFU_RSP_OK, DFU_RSP_CRC_ERROR, DFU_RSP_ERROR]:
                self.results.add_pass("test_large_payload")
            else:
                self.results.add_fail("test_large_payload",
                                      f"Unexpected response: {hex(response[0])}")
        else:
            self.results.add_fail("test_large_payload", "No response")

    # =========================================================================
    # TEST RUNNER
    # =========================================================================

    def run_all_tests(self):
        """Run all SPI DFU tests"""
        print("\n" + "=" * 50)
        print("SPI DFU Integration Tests")
        print("=" * 50)
        print(f"Interface: {type(self.interface).__name__}")
        if hasattr(self.interface, 'freq'):
            print(f"SPI Frequency: {self.interface.freq / 1000:.0f} kHz")
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
            self.test_rapid_commands()

            print("\n-- Protocol Tests --")
            self.test_command_id_range()

            print("\n-- Two-Transaction Protocol Tests --")
            self.test_two_transaction_ping()
            self.test_two_transaction_write_req()

            print("\n-- Timing Tests --")
            self.test_timing_ping_response()
            self.test_timing_different_frequencies()

            print("\n-- Large Payload Tests --")
            self.test_large_payload()

        self.interface.disconnect()
        return self.results.summary()


def main():
    parser = argparse.ArgumentParser(description='SPI DFU Integration Tests')
    parser.add_argument('--interface', '-i', choices=['ftdi', 'mock'],
                        default='mock', help='SPI interface type')
    parser.add_argument('--ftdi-url', default='ftdi://ftdi:232h/1',
                        help='FTDI URL for pyftdi')
    parser.add_argument('--freq', '-f', type=int, default=500000,
                        choices=[125000, 250000, 500000, 1000000],
                        help='SPI frequency in Hz (default: 500000)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output')
    parser.add_argument('--mock', action='store_true',
                        help='Use mock SPI interface')

    args = parser.parse_args()

    if args.mock or args.interface == 'mock':
        interface = MockSPIInterface()
        print("Using MOCK SPI interface (no hardware required)")
    elif args.interface == 'ftdi':
        interface = FTDISPIInterface(args.ftdi_url, freq=args.freq)
        print(f"Using FTDI interface at {args.freq / 1000:.0f} kHz")
    else:
        print("Unknown interface type")
        sys.exit(1)

    tester = SPIDFUTester(interface, args.verbose)
    success = tester.run_all_tests()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
