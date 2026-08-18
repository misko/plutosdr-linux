/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_ADI_TANDEM_AGC_H
#define _UAPI_LINUX_ADI_TANDEM_AGC_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ADI_TANDEM_AGC_REQUEST_MAGIC	0x54465053U /* "SPFT" */
#define ADI_TANDEM_AGC_ABI_VERSION	1U

#define ADI_TANDEM_AGC_FEATURE_EVENTS		(1U << 0)
#define ADI_TANDEM_AGC_FEATURE_FAIL_CLOSED	(1U << 1)
#define ADI_TANDEM_AGC_FEATURE_PAIRED_GAIN	(1U << 2)

#define ADI_TANDEM_AGC_POLICY_FAIL_SESSION	0U

#define ADI_TANDEM_AGC_FAULT_RESTORE		(1U << 31)

#define ADI_TANDEM_AGC_GAIN_TABLE_200_1300_MHZ	1U
#define ADI_TANDEM_AGC_GAIN_TABLE_1300_4000_MHZ	2U
#define ADI_TANDEM_AGC_GAIN_TABLE_4000_6000_MHZ	3U

enum adi_tandem_agc_mode {
	ADI_TANDEM_AGC_MODE_HOLD = 0,
	ADI_TANDEM_AGC_MODE_AUTO = 1,
};

enum adi_tandem_agc_state {
	ADI_TANDEM_AGC_STATE_IDLE = 0,
	ADI_TANDEM_AGC_STATE_VALIDATING = 1,
	ADI_TANDEM_AGC_STATE_ARMED_HOLD = 2,
	ADI_TANDEM_AGC_STATE_ARMED_AUTO = 3,
	ADI_TANDEM_AGC_STATE_FAULTED = 4,
	ADI_TANDEM_AGC_STATE_RESTORING = 5,
};

/*
 * The metadata provider decodes the little-endian SPF wire request into this
 * native-endian ioctl request before ACQUIRE. Gain values are integer dB.
 * Detector thresholds are the native unsigned
 * AD9361 threshold codes and must be reported back in session provenance.
 */
struct adi_tandem_agc_request_v1 {
	__u32 magic;
	__u16 version;
	__u16 size;
	__u32 required_features;
	__u32 mode;
	__u32 observation_capacity;
	__u32 event_capacity;
	__s32 minimum_gain_db;
	__s32 maximum_gain_db;
	__s32 initial_gain_db;
	__u32 power_measurement_samples;
	__u32 low_power_dwell_periods;
	__u32 cooldown_periods;
	__u32 pulse_high_cycles;
	__u32 pulse_low_cycles;
	__u32 detector_blanking_cycles;
	__u8 low_power_threshold;
	__u8 large_lmt_overload_threshold;
	__u8 large_adc_overload_threshold;
	__u8 small_adc_overload_threshold;
	__u32 overflow_policy;
	__u32 sync_fault_policy;
	__u32 reserved[8];
};

struct adi_tandem_agc_caps {
	__u16 version;
	__u16 size;
	__u32 features;
	__u32 fpga_identity;
	__u32 fpga_abi;
	__u32 event_size;
	__u32 fifo_depth;
	__u32 reserved[8];
};

struct adi_tandem_agc_status {
	__u16 version;
	__u16 size;
	__u32 state;
	__u32 ownership_epoch;
	__u32 fault_flags;
	__u32 fifo_level;
	__u32 overflow_count;
	__u32 transition_count;
	__s32 minimum_gain_db;
	__s32 maximum_gain_db;
	__s32 initial_gain_db;
	__u8 minimum_gain_index;
	__u8 maximum_gain_index;
	__u8 rx1_gain_index;
	__u8 rx2_gain_index;
	__u32 gain_table_id;
	__u32 threshold_provenance;
	__u32 reserved[6];
};

struct adi_tandem_agc_acquire {
	struct adi_tandem_agc_request_v1 request;
	struct adi_tandem_agc_status status;
};

struct adi_tandem_agc_event {
	__u64 sample_sequence;
	__u32 event_sequence;
	__u16 flags;
	__u8 rx1_gain_index;
	__u8 rx2_gain_index;
};

#define ADI_TANDEM_AGC_IOC_MAGIC	'T'
#define ADI_TANDEM_AGC_IOC_GET_CAPS \
	_IOR(ADI_TANDEM_AGC_IOC_MAGIC, 0x00, struct adi_tandem_agc_caps)
#define ADI_TANDEM_AGC_IOC_GET_STATUS \
	_IOR(ADI_TANDEM_AGC_IOC_MAGIC, 0x01, struct adi_tandem_agc_status)
#define ADI_TANDEM_AGC_IOC_ACQUIRE \
	_IOWR(ADI_TANDEM_AGC_IOC_MAGIC, 0x02, struct adi_tandem_agc_acquire)
#define ADI_TANDEM_AGC_IOC_RELEASE \
	_IO(ADI_TANDEM_AGC_IOC_MAGIC, 0x03)

#endif /* _UAPI_LINUX_ADI_TANDEM_AGC_H */
