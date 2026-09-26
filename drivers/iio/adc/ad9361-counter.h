/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __AD9361_COUNTER_H__
#define __AD9361_COUNTER_H__

#include <linux/errno.h>
#include <linux/types.h>

#define ADI_RX_COUNTER_SCAN_MASK_RX1	0x03U
#define ADI_RX_COUNTER_SCAN_MASK_RX1_RX2	0x0fU

static inline int ad9361_counter_timestamp_control(u32 samples_per_channel,
						    u32 scan_mask, u32 *control)
{
	if (!control || !samples_per_channel || (samples_per_channel & 1) ||
	    samples_per_channel > 0x7ffffffeU)
		return -EINVAL;
	if (scan_mask == ADI_RX_COUNTER_SCAN_MASK_RX1)
		*control = samples_per_channel;
	else if (scan_mask == ADI_RX_COUNTER_SCAN_MASK_RX1_RX2)
		*control = samples_per_channel * 2U;
	else
		return -EINVAL;
	return 0;
}

#endif
