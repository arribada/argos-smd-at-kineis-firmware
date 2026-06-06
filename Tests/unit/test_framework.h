/**
 * @file    test_framework.h
 * @brief   Simple unit test framework for bootloader tests
 * @date    2025
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Test counters */
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

/* Current test name */
static const char* current_test = NULL;

/* Test macros */
#define TEST_START(name) \
    do { \
        current_test = name; \
        tests_run++; \
        printf("  [TEST] %s... ", name); \
    } while(0)

#define TEST_PASS() \
    do { \
        tests_passed++; \
        printf("PASS\n"); \
    } while(0)

#define TEST_FAIL(msg) \
    do { \
        tests_failed++; \
        printf("FAIL: %s\n", msg); \
    } while(0)

#define ASSERT_TRUE(cond) \
    do { \
        if (!(cond)) { \
            TEST_FAIL("Assertion failed: " #cond); \
            return; \
        } \
    } while(0)

#define ASSERT_FALSE(cond) ASSERT_TRUE(!(cond))

#define ASSERT_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            char _msg[128]; \
            snprintf(_msg, sizeof(_msg), "Expected %ld, got %ld", \
                    (long)(expected), (long)(actual)); \
            TEST_FAIL(_msg); \
            return; \
        } \
    } while(0)

#define ASSERT_NE(unexpected, actual) \
    do { \
        if ((unexpected) == (actual)) { \
            char _msg[128]; \
            snprintf(_msg, sizeof(_msg), "Did not expect %ld", \
                    (long)(actual)); \
            TEST_FAIL(_msg); \
            return; \
        } \
    } while(0)

#define ASSERT_EQ_HEX(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            char _msg[128]; \
            snprintf(_msg, sizeof(_msg), "Expected 0x%08lX, got 0x%08lX", \
                    (unsigned long)(expected), (unsigned long)(actual)); \
            TEST_FAIL(_msg); \
            return; \
        } \
    } while(0)

#define ASSERT_STR_EQ(expected, actual) \
    do { \
        if (strcmp((expected), (actual)) != 0) { \
            char _msg[256]; \
            snprintf(_msg, sizeof(_msg), "Expected \"%s\", got \"%s\"", \
                    (expected), (actual)); \
            TEST_FAIL(_msg); \
            return; \
        } \
    } while(0)

#define ASSERT_MEM_EQ(expected, actual, len) \
    do { \
        if (memcmp((expected), (actual), (len)) != 0) { \
            TEST_FAIL("Memory comparison failed"); \
            return; \
        } \
    } while(0)

#define ASSERT_NOT_NULL(ptr) \
    do { \
        if ((ptr) == NULL) { \
            TEST_FAIL("Pointer is NULL"); \
            return; \
        } \
    } while(0)

#define RUN_TEST(test_func) \
    do { \
        TEST_START(#test_func); \
        test_func(); \
        if (tests_passed == tests_run) { \
            /* Already marked as pass in test */ \
        } \
    } while(0)

#define TEST_SUITE_START(name) \
    do { \
        printf("\n======================================\n"); \
        printf("Test Suite: %s\n", name); \
        printf("======================================\n"); \
    } while(0)

#define TEST_SUITE_END() \
    do { \
        printf("--------------------------------------\n"); \
        printf("Results: %d/%d passed", tests_passed, tests_run); \
        if (tests_failed > 0) { \
            printf(" (%d FAILED)", tests_failed); \
        } \
        printf("\n======================================\n\n"); \
    } while(0)

#define TEST_SUMMARY() \
    do { \
        printf("\n****************************************\n"); \
        printf("TOTAL: %d tests, %d passed, %d failed\n", \
               tests_run, tests_passed, tests_failed); \
        printf("****************************************\n"); \
        return (tests_failed == 0) ? 0 : 1; \
    } while(0)

#endif /* TEST_FRAMEWORK_H */
