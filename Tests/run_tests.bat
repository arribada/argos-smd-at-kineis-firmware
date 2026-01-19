@echo off
REM ============================================
REM Bootloader Unit Tests Runner (Windows)
REM ============================================

echo ============================================
echo Bootloader DFU Tests
echo ============================================

REM Check for Python
where python >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: Python not found in PATH
    exit /b 1
)

REM Run Python SPI Mock Tests
echo.
echo [1/2] Running SPI Mock Integration Tests...
echo --------------------------------------------
python integration\test_spi_dfu.py --mock
if %ERRORLEVEL% neq 0 (
    echo SPI Mock tests FAILED!
    set TESTS_FAILED=1
) else (
    echo SPI Mock tests PASSED!
)

REM Check if GCC is available for C unit tests
where gcc >nul 2>&1
if %ERRORLEVEL% equ 0 (
    echo.
    echo [2/2] Building and Running C Unit Tests...
    echo --------------------------------------------

    if not exist build mkdir build

    echo Building test_crc32...
    gcc -Wall -Wextra -g -O0 -std=c99 -Iunit unit\test_crc32.c -o build\test_crc32.exe
    if %ERRORLEVEL% equ 0 (
        build\test_crc32.exe
    ) else (
        echo Build failed for test_crc32
    )

    echo Building test_dfu_protocol...
    gcc -Wall -Wextra -g -O0 -std=c99 -Iunit unit\test_dfu_protocol.c -o build\test_dfu_protocol.exe
    if %ERRORLEVEL% equ 0 (
        build\test_dfu_protocol.exe
    ) else (
        echo Build failed for test_dfu_protocol
    )

    echo Building test_app_header...
    gcc -Wall -Wextra -g -O0 -std=c99 -Iunit unit\test_app_header.c -o build\test_app_header.exe
    if %ERRORLEVEL% equ 0 (
        build\test_app_header.exe
    ) else (
        echo Build failed for test_app_header
    )
) else (
    echo.
    echo [2/2] C Unit Tests SKIPPED (GCC not found)
    echo     Install MinGW or add GCC to PATH to run C unit tests
)

echo.
echo ============================================
echo Tests Complete
echo ============================================

REM To run UART tests, use:
echo.
echo To run UART integration tests with hardware:
echo   python integration\test_uart_dfu.py --port COMx
echo.

if defined TESTS_FAILED (
    exit /b 1
)
exit /b 0
