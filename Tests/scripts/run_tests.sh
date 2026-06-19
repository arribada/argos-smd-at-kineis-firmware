#!/bin/bash
# Build + run all unit tests. Bypasses Tests/Makefile which fails on MSYS2
# because the make subshell inherits a corrupted TMP env (gcc tries to write
# temp files to C:\WINDOWS\). Direct gcc invocation works.
#
# Usage: ./Tests/scripts/run_tests.sh
# Exit code: 0 if all tests pass, 1 otherwise.

set -u

cd "$(dirname "$0")/.."

CFLAGS="-Wall -Wextra -g -O0 -std=c99 -Iunit"

mkdir -p build

build_ok=0
build_fail=0
for src in unit/test_*.c; do
    name=$(basename "$src" .c)
    if gcc $CFLAGS "$src" -o "build/$name" 2>/dev/null; then
        build_ok=$((build_ok + 1))
    else
        build_fail=$((build_fail + 1))
        printf "BUILD FAIL: %s\n" "$name"
        gcc $CFLAGS "$src" -o "build/$name" 2>&1 | grep -E "error|warning:" | head -5
    fi
done

if [ $build_fail -gt 0 ]; then
    echo ""
    echo "=== $build_fail build failures, aborting ==="
    exit 1
fi

run_ok=0
run_fail=0
tests_total=0
for exe in build/test_*.exe; do
    name=$(basename "$exe" .exe)
    if "$exe" > /tmp/test_out.log 2>&1; then
        # Format A: "Results: N/M passed" (test_framework.h TEST_SUITE_END)
        # Format B: "N tests, M passed, K failed" (some legacy tests)
        # Format C: "N/M tests passed (K failed)" (other legacy tests)
        cnt=$(tail -5 /tmp/test_out.log | grep -oE "Results: [0-9]+/[0-9]+" | head -1 | sed 's:.*/::' | grep -oE "[0-9]+")
        if [ -z "$cnt" ]; then cnt=$(tail -5 /tmp/test_out.log | grep -oE "[0-9]+/[0-9]+ tests" | head -1 | sed 's:.*/::' | grep -oE "[0-9]+"); fi
        if [ -z "$cnt" ]; then cnt=$(tail -5 /tmp/test_out.log | grep -oE "[0-9]+ tests" | head -1 | grep -oE "[0-9]+"); fi
        if [ -z "$cnt" ]; then cnt=0; fi
        tests_total=$((tests_total + cnt))
        run_ok=$((run_ok + 1))
        printf "  OK  %-30s %s tests\n" "$name" "$cnt"
    else
        run_fail=$((run_fail + 1))
        printf "FAIL  %s\n" "$name"
        tail -5 /tmp/test_out.log | sed 's/^/    /'
    fi
done

echo ""
echo "=== SUITES: $run_ok/$((run_ok + run_fail)) OK"
echo "=== TESTS:  $tests_total individual checks"

if [ $run_fail -gt 0 ]; then
    exit 1
fi
exit 0
