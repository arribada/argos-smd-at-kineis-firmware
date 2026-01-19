#!/usr/bin/env python3
"""
Comprehensive Test Runner for Bootloader DFU
Runs unit tests and integration tests.

Usage:
    python run_all_tests.py                     # Run all unit tests
    python run_all_tests.py --uart COM3         # Run UART integration tests
    python run_all_tests.py --spi-mock          # Run SPI mock tests
    python run_all_tests.py --all --uart COM3   # Run everything
"""

import argparse
import subprocess
import sys
import os
from pathlib import Path

# Test directory
TEST_DIR = Path(__file__).parent
UNIT_DIR = TEST_DIR / "unit"
INTEGRATION_DIR = TEST_DIR / "integration"
BUILD_DIR = TEST_DIR / "build"


def run_command(cmd, description):
    """Run a command and return success status"""
    print(f"\n{'=' * 60}")
    print(f"  {description}")
    print(f"{'=' * 60}\n")

    try:
        result = subprocess.run(cmd, shell=True, cwd=str(TEST_DIR))
        return result.returncode == 0
    except Exception as e:
        print(f"Error running command: {e}")
        return False


def build_unit_tests():
    """Build C unit tests"""
    print("\n" + "=" * 60)
    print("  Building Unit Tests")
    print("=" * 60)

    # Create build directory
    BUILD_DIR.mkdir(exist_ok=True)

    tests = [
        ("test_crc32", "unit/test_crc32.c"),
        ("test_dfu_protocol", "unit/test_dfu_protocol.c"),
        ("test_app_header", "unit/test_app_header.c"),
    ]

    success = True
    for name, source in tests:
        output = BUILD_DIR / name
        if sys.platform == "win32":
            output = BUILD_DIR / f"{name}.exe"

        cmd = f"gcc -Wall -Wextra -g -O0 -std=c99 -I{UNIT_DIR} {source} -o {output}"
        print(f"  Building {name}...")

        result = subprocess.run(cmd, shell=True, cwd=str(TEST_DIR),
                                capture_output=True, text=True)
        if result.returncode != 0:
            print(f"    FAILED: {result.stderr}")
            success = False
        else:
            print(f"    OK")

    return success


def run_unit_tests():
    """Run C unit tests"""
    print("\n" + "=" * 60)
    print("  Running Unit Tests")
    print("=" * 60)

    tests = ["test_crc32", "test_dfu_protocol", "test_app_header"]
    all_passed = True

    for name in tests:
        exe = BUILD_DIR / name
        if sys.platform == "win32":
            exe = BUILD_DIR / f"{name}.exe"

        if not exe.exists():
            print(f"\n  [SKIP] {name} - not built")
            continue

        print(f"\n  Running {name}...")
        result = subprocess.run(str(exe), shell=True, cwd=str(TEST_DIR))

        if result.returncode != 0:
            all_passed = False

    return all_passed


def run_uart_tests(port, baudrate=9600, verbose=False):
    """Run UART integration tests"""
    script = INTEGRATION_DIR / "test_uart_dfu.py"

    cmd = f"python {script} --port {port} --baudrate {baudrate}"
    if verbose:
        cmd += " --verbose"

    return run_command(cmd, f"UART Integration Tests (port: {port})")


def run_spi_mock_tests(verbose=False):
    """Run SPI integration tests with mock interface"""
    script = INTEGRATION_DIR / "test_spi_dfu.py"

    cmd = f"python {script} --mock"
    if verbose:
        cmd += " --verbose"

    return run_command(cmd, "SPI Integration Tests (Mock)")


def run_spi_hardware_tests(interface="ftdi", verbose=False):
    """Run SPI integration tests with hardware"""
    script = INTEGRATION_DIR / "test_spi_dfu.py"

    cmd = f"python {script} --interface {interface}"
    if verbose:
        cmd += " --verbose"

    return run_command(cmd, f"SPI Integration Tests ({interface})")


def print_summary(results):
    """Print test summary"""
    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)

    total = len(results)
    passed = sum(1 for r in results.values() if r)
    failed = total - passed

    for name, result in results.items():
        status = "PASS" if result else "FAIL"
        print(f"  [{status}] {name}")

    print("-" * 60)
    print(f"  Total: {total}, Passed: {passed}, Failed: {failed}")
    print("=" * 60)

    return failed == 0


def main():
    parser = argparse.ArgumentParser(
        description='Bootloader DFU Test Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_all_tests.py                     Run unit tests only
  python run_all_tests.py --uart COM3         Run unit + UART tests
  python run_all_tests.py --spi-mock          Run unit + SPI mock tests
  python run_all_tests.py --all --uart COM3   Run all tests
        """
    )

    parser.add_argument('--unit-only', action='store_true',
                        help='Run only unit tests')
    parser.add_argument('--uart', metavar='PORT',
                        help='Run UART tests on specified port')
    parser.add_argument('--baudrate', type=int, default=9600,
                        help='UART baud rate (default: 9600)')
    parser.add_argument('--spi-mock', action='store_true',
                        help='Run SPI tests with mock interface')
    parser.add_argument('--spi', metavar='INTERFACE',
                        choices=['ftdi', 'buspirate'],
                        help='Run SPI tests with hardware interface')
    parser.add_argument('--all', action='store_true',
                        help='Run all available tests')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose output')
    parser.add_argument('--skip-build', action='store_true',
                        help='Skip building unit tests')

    args = parser.parse_args()

    results = {}

    # Build unit tests
    if not args.skip_build:
        if not build_unit_tests():
            print("\nFailed to build unit tests!")
            sys.exit(1)

    # Run unit tests
    print("\n")
    results['Unit Tests'] = run_unit_tests()

    # Run integration tests based on arguments
    if args.all or args.spi_mock:
        results['SPI Mock Tests'] = run_spi_mock_tests(args.verbose)

    if args.all or args.uart:
        if args.uart:
            results['UART Tests'] = run_uart_tests(
                args.uart, args.baudrate, args.verbose)
        elif args.all:
            print("\n  [SKIP] UART tests - no port specified (use --uart PORT)")

    if args.spi:
        results['SPI Hardware Tests'] = run_spi_hardware_tests(
            args.spi, args.verbose)

    # Print summary
    success = print_summary(results)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
