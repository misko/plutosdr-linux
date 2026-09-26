// SPDX-License-Identifier: GPL-2.0
#include <kunit/test.h>
#include <linux/module.h>

#include "ad9361-counter.h"

static void ad9361_counter_interval_layout_test(struct kunit *test)
{
	u32 control = 0;

	KUNIT_EXPECT_EQ(test, 0,
		ad9361_counter_timestamp_control(1000000,
			ADI_RX_COUNTER_SCAN_MASK_RX1, &control));
	KUNIT_EXPECT_EQ(test, 1000000U, control);
	KUNIT_EXPECT_EQ(test, 0,
		ad9361_counter_timestamp_control(1000000,
			ADI_RX_COUNTER_SCAN_MASK_RX1_RX2, &control));
	KUNIT_EXPECT_EQ(test, 2000000U, control);
}

static void ad9361_counter_interval_bounds_test(struct kunit *test)
{
	u32 control = 0;

	KUNIT_EXPECT_EQ(test, -EINVAL,
		ad9361_counter_timestamp_control(0,
			ADI_RX_COUNTER_SCAN_MASK_RX1, &control));
	KUNIT_EXPECT_EQ(test, -EINVAL,
		ad9361_counter_timestamp_control(999999,
			ADI_RX_COUNTER_SCAN_MASK_RX1_RX2, &control));
	KUNIT_EXPECT_EQ(test, -EINVAL,
		ad9361_counter_timestamp_control(0x80000000U,
			ADI_RX_COUNTER_SCAN_MASK_RX1_RX2, &control));
	KUNIT_EXPECT_EQ(test, -EINVAL,
		ad9361_counter_timestamp_control(1000000, 0x33U, &control));
	KUNIT_EXPECT_EQ(test, 0,
		ad9361_counter_timestamp_control(0x7ffffffeU,
			ADI_RX_COUNTER_SCAN_MASK_RX1_RX2, &control));
	KUNIT_EXPECT_EQ(test, 0xfffffffcU, control);
}

static struct kunit_case ad9361_counter_interval_cases[] = {
	KUNIT_CASE(ad9361_counter_interval_layout_test),
	KUNIT_CASE(ad9361_counter_interval_bounds_test),
	{}
};

static struct kunit_suite ad9361_counter_interval_suite = {
	.name = "ad9361-counter-interval",
	.test_cases = ad9361_counter_interval_cases,
};

kunit_test_suite(ad9361_counter_interval_suite);

MODULE_LICENSE("GPL");
