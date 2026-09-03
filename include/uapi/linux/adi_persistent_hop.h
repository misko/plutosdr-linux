/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ADI_PERSISTENT_HOP_H
#define _UAPI_LINUX_ADI_PERSISTENT_HOP_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ADI_PERSISTENT_HOP_ABI_VERSION	1U
#define ADI_PERSISTENT_HOP_PROFILE_NONE	0xffffffffU

#define ADI_PERSISTENT_HOP_FEATURE_COUNTER64	(1U << 0)
#define ADI_PERSISTENT_HOP_FEATURE_OWNER_RECALL	(1U << 1)
#define ADI_PERSISTENT_HOP_FEATURE_LO_READBACK	(1U << 2)
#define ADI_PERSISTENT_HOP_FEATURE_EVENT_ID	(1U << 3)
#define ADI_PERSISTENT_HOP_FEATURE_AUTO_RESTORE	(1U << 4)
#define ADI_PERSISTENT_HOP_REQUIRED_FEATURES	( \
	ADI_PERSISTENT_HOP_FEATURE_COUNTER64 | \
	ADI_PERSISTENT_HOP_FEATURE_OWNER_RECALL | \
	ADI_PERSISTENT_HOP_FEATURE_LO_READBACK | \
	ADI_PERSISTENT_HOP_FEATURE_EVENT_ID | \
	ADI_PERSISTENT_HOP_FEATURE_AUTO_RESTORE)

struct adi_persistent_hop_caps_v1 {
	__u16 version;
	__u16 size;
	__u32 features;
	__u32 maximum_profiles;
	__u32 fpga_identity;
	__u32 fpga_abi;
	__u32 reserved[7];
};

struct adi_persistent_hop_counter_v1 {
	__u16 version;
	__u16 size;
	__u32 reserved0;
	__u64 sample_counter;
	__u32 reserved[4];
};

struct adi_persistent_hop_start_v1 {
	__u16 version;
	__u16 size;
	__u32 required_features;
	__u64 expected_original_lo_hz;
	__u64 actual_original_lo_hz;
	__u32 active_profile;
	__u32 reserved[5];
};

struct adi_persistent_hop_transition_v1 {
	__u16 version;
	__u16 size;
	__u32 required_features;
	__u32 profile;
	__u32 active_profile;
	__u64 expected_lo_hz;
	__u64 actual_lo_hz;
	__u64 transition_before;
	__u64 transition_after;
	__u64 device_event_id;
	__u32 reserved[2];
};

struct adi_persistent_hop_restore_v1 {
	__u16 version;
	__u16 size;
	__u32 required_features;
	__u64 expected_original_lo_hz;
	__u64 actual_lo_hz;
	__u64 transition_before;
	__u64 transition_after;
	__u32 active_profile;
	__u32 reserved[5];
};

/* These commands intentionally run on the existing exclusively opened
 * /dev/tandem-agc-events descriptor.  That descriptor is the ownership token;
 * no sysfs mutation path is granted an exception while tandem owns AD9361.
 */
#define ADI_PERSISTENT_HOP_IOC_MAGIC	'H'
#define ADI_PERSISTENT_HOP_IOC_GET_CAPS \
	_IOR(ADI_PERSISTENT_HOP_IOC_MAGIC, 0x00, \
		struct adi_persistent_hop_caps_v1)
#define ADI_PERSISTENT_HOP_IOC_GET_COUNTER \
	_IOR(ADI_PERSISTENT_HOP_IOC_MAGIC, 0x01, \
		struct adi_persistent_hop_counter_v1)
#define ADI_PERSISTENT_HOP_IOC_START \
	_IOWR(ADI_PERSISTENT_HOP_IOC_MAGIC, 0x02, \
		struct adi_persistent_hop_start_v1)
#define ADI_PERSISTENT_HOP_IOC_RECALL \
	_IOWR(ADI_PERSISTENT_HOP_IOC_MAGIC, 0x03, \
		struct adi_persistent_hop_transition_v1)
#define ADI_PERSISTENT_HOP_IOC_RESTORE \
	_IOWR(ADI_PERSISTENT_HOP_IOC_MAGIC, 0x04, \
		struct adi_persistent_hop_restore_v1)

#endif /* _UAPI_LINUX_ADI_PERSISTENT_HOP_H */
