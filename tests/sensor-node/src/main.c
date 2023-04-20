/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "sample.h"


ZTEST_SUITE(framework_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(framework_tests, test_fail)
{
  /** This will fail the test */
  zassert_true(false, NULL);
}

/**
 * @brief Test Asserts
 *
 * This test verifies various assert macros provided by ztest.
 *
 */
ZTEST(framework_tests, test_assert)
{
	zassert_true(1, "1 was false");
	zassert_false(0, "0 was true");
	zassert_is_null(NULL, "NULL was not NULL");
	zassert_not_null("foo", "\"foo\" was NULL");
	zassert_equal(1, 1, "1 was not equal to 1");
	zassert_equal_ptr(NULL, NULL, "NULL was not equal to NULL");
	zassert_equal(TEXT, "HELL0", "string are not equal");
	zassert_equal(4, add(2, 2), "NOT GOOD");
	zassert_equal(3, add(2, 2), "NOT GOOD");
}