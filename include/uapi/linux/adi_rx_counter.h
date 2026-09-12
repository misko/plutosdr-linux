/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_ADI_RX_COUNTER_H
#define _UAPI_ADI_RX_COUNTER_H
#include <linux/types.h>
#include <linux/ioctl.h>
/* A separate request namespace; never a tandem HOLD/AUTO request. */
#define ADI_RX_COUNTER_MAGIC 0x43465053U /* SPFC */
#define ADI_RX_COUNTER_VERSION 1U
#define ADI_RX_COUNTER_FEATURES 7U /* counter, canonical CI16, exact gaps */
struct adi_rx_counter_request {
	__u32 magic;
	__u16 version;
	__u16 size;
	__u32 required_features;
	__u32 scan_mask;
	__u32 sample_rate_hz;
	__u32 samples_per_channel;
	__u32 reserved[2];
};
#define ADI_RX_COUNTER_IOC_ACQUIRE _IOW('T', 0x20, struct adi_rx_counter_request)
#endif
