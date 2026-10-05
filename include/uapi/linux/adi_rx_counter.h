/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_ADI_RX_COUNTER_H
#define _UAPI_ADI_RX_COUNTER_H
#include <linux/types.h>
#include <linux/ioctl.h>
/* A separate request namespace; never a tandem HOLD/AUTO request. */
#define ADI_RX_COUNTER_MAGIC 0x43465053U /* SPFC */
#define ADI_RX_COUNTER_VERSION 1U
#define ADI_RX_COUNTER_FEATURES 7U /* counter, canonical CI16, exact gaps */
#define ADI_RX_COUNTER_SCAN_VERSION 1U
#define ADI_RX_COUNTER_SCAN_MAX_PROFILES 8U
#define ADI_RX_COUNTER_SCAN_FEATURES 0x1fU
/* IIO scan-mask values.  A counter tick is one sample instant, irrespective
 * of whether that instant contains RX1 alone or the paired RX1/RX2 frame. */
#define ADI_RX_COUNTER_SCAN_MASK_RX1 0x03U
#define ADI_RX_COUNTER_SCAN_MASK_RX1_RX2 0x0fU
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

/*
 * Fastlock profiles are loaded through the normal IIO interface before the
 * exclusive counter lease is acquired.  The lease owner then freezes the
 * allowed RX profile set with this request.  crc32 is the standard reflected
 * CRC-32 of the 16-byte AD9361 fastlock profile (initial value and final xor
 * both ~0U).  frequency_hz is the requested/read-back RX LO frequency; the
 * driver permits only the AD9361 clock representation's two-Hz rounding.
 */
struct adi_rx_counter_scan_profile {
	__aligned_u64 frequency_hz;
	__u32 crc32;
	__u32 reserved;
};

struct adi_rx_counter_scan_config {
	__u32 magic;
	__u16 version;
	__u16 size;
	__u32 profile_mask;
	__u32 reserved;
	struct adi_rx_counter_scan_profile
		profiles[ADI_RX_COUNTER_SCAN_MAX_PROFILES];
};

/*
 * A successful recall returns a source-counter bracket.  The values are the
 * coherent low 32 bits of the same 64-bit ADC sample counter embedded in IQ
 * blocks.  Userspace extends them against the surrounding IQ timestamps.
 * counter_after is sampled after checked hardware profile selection, profile
 * frequency/integrity reads and bounded RX PLL lock attestation. frequency_hz
 * is decoded from the selected profile; AD9361 ordinary RFPLL SPI registers
 * retain the last conventional tune during Fast Lock and cannot independently
 * measure the active RF LO. RF tone qualification remains a separate oracle.
 * profile_crc32 is the post-recall value because the AD9361 ALC workaround
 * may legitimately rewrite the profile's ALC word.
 */
struct adi_rx_counter_scan_recall {
	__u32 magic;
	__u16 version;
	__u16 size;
	__u32 profile;
	__u32 flags;
	__aligned_u64 frequency_hz;
	__u32 profile_crc32;
	__u32 counter_before;
	__u32 counter_after;
	__u32 reserved[3];
};

struct adi_rx_counter_scan_caps {
	__u32 magic;
	__u16 version;
	__u16 size;
	__u32 features;
	__u32 maximum_profiles;
	__u32 source_counter_bits;
	__u32 frequency_resolution_hz;
	__u32 reserved[2];
};

struct adi_rx_counter_scan_release {
	__u32 magic;
	__u16 version;
	__u16 size;
	__u32 flags;
	__u32 reserved0;
	__aligned_u64 frequency_hz;
	__u32 counter_before;
	__u32 counter_after;
	__u32 reserved[4];
};

/*
 * Owner-only coherent source-time sample used to pace dwell boundaries
 * independently of DMA block completion.
 */
struct adi_rx_counter_scan_snapshot {
	__u32 magic;
	__u16 version;
	__u16 size;
	__u32 flags;
	__u32 counter;
	__u32 reserved[4];
};

#define ADI_RX_COUNTER_IOC_ACQUIRE _IOW('T', 0x20, struct adi_rx_counter_request)
#define ADI_RX_COUNTER_IOC_CONFIGURE_SCAN \
	_IOW('T', 0x21, struct adi_rx_counter_scan_config)
#define ADI_RX_COUNTER_IOC_RECALL \
	_IOWR('T', 0x22, struct adi_rx_counter_scan_recall)
#define ADI_RX_COUNTER_IOC_GET_SCAN_CAPS \
	_IOR('T', 0x23, struct adi_rx_counter_scan_caps)
#define ADI_RX_COUNTER_IOC_RELEASE_SCAN \
	_IOWR('T', 0x24, struct adi_rx_counter_scan_release)
#define ADI_RX_COUNTER_IOC_SCAN_SNAPSHOT \
	_IOWR('T', 0x25, struct adi_rx_counter_scan_snapshot)

/* Independent, additive diagnostic ABI. Existing scan structures stay frozen.
 * Time values use CLOCK_MONOTONIC nanoseconds; counters are raw low 32 bits.
 * The ring is bounded and survives RELEASE until the next successful ACQUIRE.
 * sequence orders the circular records; first errors survive ring rollover.
 */
#define ADI_RX_COUNTER_DIAG_VERSION 1U
#define ADI_RX_COUNTER_DIAG_CAPACITY 32U
#define ADI_RX_COUNTER_DIAG_RECALL 1U
#define ADI_RX_COUNTER_DIAG_CONFIGURE 2U
#define ADI_RX_COUNTER_DIAG_RESTORE 3U
#define ADI_RX_COUNTER_DIAG_OPERATION 0U
#define ADI_RX_COUNTER_DIAG_LOCK 1U
struct adi_rx_counter_diag_context {
	__u32 magic;
	__u16 version, size;
	__aligned_u64 session, visit;
};
struct adi_rx_counter_diag_event {
	__aligned_u64 sequence, session, visit;
	__aligned_u64 start_ns, end_ns, spi_last_ns, spi_max_ns;
	__u32 stage, step, profile, counter_before, counter_after, polls;
	__s32 last_status, error;
};
struct adi_rx_counter_diagnostics {
	__u32 magic;
	__u16 version, size;
	__u32 count, reserved;
	__aligned_u64 total;
	struct adi_rx_counter_diag_event first_failure, restoration_failure;
	struct adi_rx_counter_diag_event events[ADI_RX_COUNTER_DIAG_CAPACITY];
};
#define ADI_RX_COUNTER_IOC_DIAG_CONTEXT \
	_IOW('T', 0x26, struct adi_rx_counter_diag_context)
#define ADI_RX_COUNTER_IOC_GET_DIAGNOSTICS \
	_IOR('T', 0x27, struct adi_rx_counter_diagnostics)
#endif
