// SPDX-License-Identifier: GPL-2.0-only
/*
 * Starlink PSS coarse phase-map IIO driver
 *
 * One hardware map is 20,000 u16 bins.  IIO scan_type.repeat is u8, so maps
 * are transported as 200 self-describing scans.  Each scan is one indivisible
 * array whose first 59 u32 words are nine metadata words followed by 100 packed
 * u16 bins.  The transport scan is zero-padded to 64 words (256 bytes): Linux
 * IIO aligns a repeated channel by its complete width using ALIGN(), so that
 * width must be a power of two for kernel and libiio strides to agree.  A
 * single padded channel also prevents partial scan masks.  Generation, start
 * index, chunk ordinal, and first-bin metadata make reassembly and incomplete-
 * map rejection deterministic.  A hardware bank is released only after every
 * chunk has entered the IIO kfifo.
 */
#include <linux/bitfield.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define MAP_REG_ID                         0x00
#define MAP_REG_VERSION                    0x04
#define MAP_REG_PHASE_BINS                 0x08
#define MAP_REG_TILE_GEOMETRY              0x0c
#define MAP_REG_CAPABILITIES               0x10
#define MAP_REG_CONTROL                    0x14
#define MAP_REG_STATUS                     0x18
#define MAP_REG_SELECT                     0x1c
#define MAP_REG_INDEX                      0x20
#define MAP_REG_DATA                       0x24
#define MAP_REG_RELEASE                    0x28
#define MAP_REG_COMMAND_STATUS             0x2c
#define MAP_REG_SNAPSHOT_CONTROL           0x30
#define MAP_REG_SNAPSHOT_STATUS            0x34
#define MAP_REG_SNAPSHOT_GENERATION        0x38
#define MAP_REG_SNAPSHOT_READY             0x3c
#define MAP_REG_SNAPSHOT_MAP_GENERATION_0  0x40
#define MAP_REG_SNAPSHOT_MAP_GENERATION_1  0x44
#define MAP_REG_SNAPSHOT_START_0_LO        0x48
#define MAP_REG_SNAPSHOT_START_0_HI        0x4c
#define MAP_REG_SNAPSHOT_START_1_LO        0x50
#define MAP_REG_SNAPSHOT_START_1_HI        0x54
#define MAP_REG_SNAPSHOT_ACCEPTED          0x58
#define MAP_REG_SNAPSHOT_DISCARDED         0x5c
#define MAP_REG_SNAPSHOT_DISCONTINUITY     0x60
#define MAP_REG_SNAPSHOT_PUBLISHED         0x64
#define MAP_REG_SNAPSHOT_OVERRUN           0x68
#define MAP_REG_SNAPSHOT_PROTOCOL_ERROR    0x6c
#define MAP_REG_SNAPSHOT_ARITH_OVERFLOW    0x70
#define MAP_REG_SNAPSHOT_READ_ERROR        0x74
#define MAP_REG_SNAPSHOT_RELEASE_ERROR     0x78
#define MAP_REG_BRIDGE_READ_ERROR          0x7c
#define MAP_REG_BRIDGE_RELEASE_ERROR       0x80
#define MAP_REG_SNAPSHOT_REQUEST_OVERRUN   0x84
#define MAP_REG_SNAPSHOT_HEALTH_FLAGS      0x88
#define MAP_REG_SNAPSHOT_INGRESS_DROPPED   0x8c
#define MAP_REG_SNAPSHOT_INGRESS_FIFO      0x90
#define MAP_REG_SNAPSHOT_SCHEDULER_GAP     0x94
#define MAP_REG_SNAPSHOT_SCHEDULER_INDEX   0x98
#define MAP_REG_SNAPSHOT_SCHEDULER_OVERFLOW 0x9c
#define MAP_REG_SNAPSHOT_DETECTOR_FAULT    0xa0
#define MAP_REG_SNAPSHOT_PHASE_DISCONTINUITY 0xa4
#define MAP_REG_SNAPSHOT_DENOMINATOR_ZERO  0xa8
#define MAP_REG_SNAPSHOT_CANDIDATE_FIFO    0xac
#define MAP_REG_INPUT_RATE_MSPS            0xb0
#define MAP_REG_DDC_CONFIG                 0xb4
#define MAP_REG_DDC_GROUP_DELAY            0xb8
#define MAP_REG_COEFFICIENT_ENERGY         0xbc
#define MAP_REG_DDC_CONTRACT_0             0xc0
#define MAP_REG_DDC_ACCEPTED_LO             0xe0
#define MAP_REG_DDC_EMITTED_LO              0xe4
#define MAP_REG_DDC_DISCONTINUITY           0xe8
#define MAP_REG_DDC_SATURATION              0xec
#define MAP_REG_DDC_ACCEPTED_HI             0xf0
#define MAP_REG_DDC_EMITTED_HI              0xf4
#define MAP_REG_STOP_TICKET                 0xf8
#define MAP_REG_STOP_WORD                   0xfc

#define MAP_IDENTIFICATION                 0x50534d41U /* "PSMA" */
#define MAP_MIN_VERSION                    0x00010001U
#define MAP_MAX_VERSION                    0x00010006U
#define MAP_STOP_VERSION                   0x00010006U
#define MAP_STOP_CAPABILITIES              0x0000033fU
#define MAP_STOP_MAGIC                     0x50535354U
#define MAP_STOP_VERSION_WORDS             0x0001000cU
#define MAP_STOP_WORDS                     12U
#define MAP_STOP_FLAGS_MASK                0x0000003fU
#define MAP_STOP_REASONS_MASK              0x0000003fU
#define MAP_STOP_ENABLED                   BIT(5)
#define MAP_PHASE_BINS                     20000U
#define MAP_TILE_GEOMETRY                  0x00401002U
#define MAP_CHUNK_MAGIC                    0x4b4e4843U /* "CHNK" */
#define MAP_CHUNK_BINS                     100U
#define MAP_CHUNKS                         (MAP_PHASE_BINS / MAP_CHUNK_BINS)
#define MAP_META_WORDS                     9U
#define MAP_CHUNK_PAYLOAD_WORDS            (MAP_META_WORDS + MAP_CHUNK_BINS / 2U)
#define MAP_SCAN_WORDS                     64U
#define MAP_HEALTH_WORDS                   46U

#define MAP_STATUS_EPOCH_LIVE              BIT(0)
#define MAP_STATUS_ENABLED                 BIT(1)
#define MAP_STATUS_READY_MASK              GENMASK(3, 2)
#define MAP_COMMAND_READ_PENDING           BIT(0)
#define MAP_COMMAND_RELEASE_PENDING        BIT(1)
#define MAP_COMMAND_READ_ERROR             BIT(2)
#define MAP_COMMAND_RELEASE_ERROR          BIT(3)

#define MAP_FAULT_CONTRACT                 BIT(0)
#define MAP_FAULT_SNAPSHOT                 BIT(1)
#define MAP_FAULT_DATA                     BIT(2)
#define MAP_FAULT_COHERENCE                BIT(3)
#define MAP_FAULT_BUFFER_FULL              BIT(4)
#define MAP_FAULT_RELEASE                  BIT(5)
#define MAP_FAULT_HARDWARE_HEALTH          BIT(6)

enum map_attr {
	MAP_ATTR_ID,
	MAP_ATTR_VERSION,
	MAP_ATTR_PHASE_BINS,
	MAP_ATTR_TILE_GEOMETRY,
	MAP_ATTR_CAPABILITIES,
	MAP_ATTR_STATUS,
	MAP_ATTR_INPUT_RATE,
	MAP_ATTR_DDC_CONFIG,
	MAP_ATTR_DDC_GROUP_DELAY,
	MAP_ATTR_ACQUISITION_ENABLE,
	MAP_ATTR_ACQUISITION_FLUSH,
	MAP_ATTR_MAPS_DELIVERED,
	MAP_ATTR_CHUNKS_DELIVERED,
	MAP_ATTR_BUFFER_PUSH_FAILURES,
	MAP_ATTR_REASSEMBLY_CHUNKS,
	MAP_ATTR_FAULT_FLAGS,
	MAP_ATTR_FAULT_CLEAR,
};

struct map_snapshot {
	u32 generation;
	u32 ready_mask;
	u32 map_generation[2];
	u64 start_index[2];
	u32 fault_signature[14];
	u32 accepted;
	u32 published;
	u32 denominator_zero;
	u32 ingress_fifo;
	u32 candidate_fifo;
};

struct map_scan {
	u32 words[MAP_SCAN_WORDS];
};

struct adi_starlink_pss_map {
	struct device *dev;
	struct iio_dev *indio_dev;
	void __iomem *regs;
	struct mutex lock;
	int irq;
	bool irq_live;
	bool streaming;
	bool acquisition_enabled;
	u32 version;
	u32 input_rate_msps;
	u16 *map;
	u32 fault_flags;
	u32 maps_delivered;
	u32 chunks_delivered;
	u32 buffer_push_failures;
};

static const struct iio_chan_spec map_channels[] = {
	{
		.type = IIO_COUNT,
		.indexed = 1,
		.channel = 0,
		.extend_name = "chunk_words",
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
			.repeat = MAP_SCAN_WORDS,
			.endianness = IIO_LE,
		},
	},
};

static u32 map_read(struct adi_starlink_pss_map *st, unsigned int reg)
{
	return ioread32(st->regs + reg);
}

static void map_write(struct adi_starlink_pss_map *st, unsigned int reg,
		      u32 value)
{
	iowrite32(value, st->regs + reg);
}

static void map_stop_on_fault(struct adi_starlink_pss_map *st, u32 fault)
{
	st->fault_flags |= fault;
	st->acquisition_enabled = false;
	map_write(st, MAP_REG_CONTROL, 0);
	disable_irq_nosync(st->irq);
	st->irq_live = false;
}

static int map_take_snapshot(struct adi_starlink_pss_map *st,
			     struct map_snapshot *snapshot)
{
	static const u16 fault_registers[] = {
		MAP_REG_SNAPSHOT_DISCARDED,
		MAP_REG_SNAPSHOT_DISCONTINUITY,
		MAP_REG_SNAPSHOT_OVERRUN,
		MAP_REG_SNAPSHOT_PROTOCOL_ERROR,
		MAP_REG_SNAPSHOT_ARITH_OVERFLOW,
		MAP_REG_SNAPSHOT_READ_ERROR,
		MAP_REG_SNAPSHOT_RELEASE_ERROR,
		MAP_REG_SNAPSHOT_HEALTH_FLAGS,
		MAP_REG_SNAPSHOT_INGRESS_DROPPED,
		MAP_REG_SNAPSHOT_SCHEDULER_GAP,
		MAP_REG_SNAPSHOT_SCHEDULER_INDEX,
		MAP_REG_SNAPSHOT_SCHEDULER_OVERFLOW,
		MAP_REG_SNAPSHOT_DETECTOR_FAULT,
		MAP_REG_SNAPSHOT_PHASE_DISCONTINUITY,
	};
	u32 before, status, low, high;
	u32 overrun_before, overrun_after;
	unsigned int index;
	int ret;

	status = map_read(st, MAP_REG_SNAPSHOT_STATUS);
	if (status & BIT(1))
		return -EBUSY;
	before = map_read(st, MAP_REG_SNAPSHOT_GENERATION);
	overrun_before = map_read(st, MAP_REG_SNAPSHOT_REQUEST_OVERRUN);
	if (before == U32_MAX || overrun_before == U32_MAX)
		return -EOVERFLOW;
	map_write(st, MAP_REG_SNAPSHOT_CONTROL, 1U);
	ret = readl_poll_timeout(st->regs + MAP_REG_SNAPSHOT_STATUS, status,
				 (status & 0x3U) == 1U &&
				 map_read(st, MAP_REG_SNAPSHOT_GENERATION) ==
				 before + 1U, 1, 10000);
	if (ret)
		return ret;

	snapshot->generation = map_read(st, MAP_REG_SNAPSHOT_GENERATION);
	/* Do not accept another request completed between poll and payload. */
	if (snapshot->generation != before + 1U)
		return -EIO;
	snapshot->ready_mask = map_read(st, MAP_REG_SNAPSHOT_READY) & 0x3U;
	snapshot->map_generation[0] =
		map_read(st, MAP_REG_SNAPSHOT_MAP_GENERATION_0);
	snapshot->map_generation[1] =
		map_read(st, MAP_REG_SNAPSHOT_MAP_GENERATION_1);
	low = map_read(st, MAP_REG_SNAPSHOT_START_0_LO);
	high = map_read(st, MAP_REG_SNAPSHOT_START_0_HI);
	snapshot->start_index[0] = ((u64)high << 32) | low;
	low = map_read(st, MAP_REG_SNAPSHOT_START_1_LO);
	high = map_read(st, MAP_REG_SNAPSHOT_START_1_HI);
	snapshot->start_index[1] = ((u64)high << 32) | low;
	for (index = 0; index < ARRAY_SIZE(fault_registers); index++)
		snapshot->fault_signature[index] =
			map_read(st, fault_registers[index]);
	snapshot->accepted = map_read(st, MAP_REG_SNAPSHOT_ACCEPTED);
	snapshot->published = map_read(st, MAP_REG_SNAPSHOT_PUBLISHED);
	snapshot->denominator_zero =
		map_read(st, MAP_REG_SNAPSHOT_DENOMINATOR_ZERO);
	snapshot->ingress_fifo = map_read(st, MAP_REG_SNAPSHOT_INGRESS_FIFO);
	snapshot->candidate_fifo = map_read(st, MAP_REG_SNAPSHOT_CANDIDATE_FIFO);
	overrun_after = map_read(st, MAP_REG_SNAPSHOT_REQUEST_OVERRUN);
	if (overrun_after != overrun_before ||
	    (map_read(st, MAP_REG_SNAPSHOT_STATUS) & 0x3U) != 1U ||
	    map_read(st, MAP_REG_SNAPSHOT_GENERATION) != snapshot->generation)
		return -EIO;
	return 0;
}

static int map_read_live_ddc64(struct adi_starlink_pss_map *st,
			       unsigned int low_reg, unsigned int high_reg,
			       u32 *low, u32 *high)
{
	unsigned int attempt;
	u32 before;

	/* These registers are live, NOT in the coherent map snapshot payload.
	 * Avoid a torn individual counter at rollover. This does not make the
	 * accepted/emitted pair, or either counter and the snapshot, atomic.
	 */
	for (attempt = 0; attempt < 3; attempt++) {
		before = map_read(st, high_reg);
		*low = map_read(st, low_reg);
		*high = map_read(st, high_reg);
		if (before == *high)
			return 0;
	}
	return -EAGAIN;
}

/* acquisition_health v1: "PSMH 1 46" and 46 fixed-width hex u32 words.
 *
 *  0 ABI; 1 declared (not measured PHY) input MSPS; 2 snapshot generation;
 *  3/4 live status before/after; 5 driver streaming/enabled/IRQ bits 0/1/2;
 *  6 driver faults; 7/8/9 maps/chunks/push failures (existing u32 counters);
 * 10 snapshot ready; 11/12 map generations; 13..16 start0/start1 LO,HI;
 * 17..30 complete coherent fault_signature in map_take_snapshot() order;
 * 31/32 live bridge read/release errors; 33 live snapshot-request overruns;
 * 34 DDC mode: 0 absent, 1 live low32, 2 live individually sampled64;
 * 35..38 live DDC accepted/emitted LO,HI; 39/40 discontinuity/saturation;
 * 41..45 coherent accepted/published/denominator-zero/ingress/candidate FIFO.
 * FIFO words contain current level in low16, maximum level in high16.
 *
 * Only words 2, 10..30 and 41..45 are one hardware snapshot. The st->lock
 * mutex serializes driver lifecycle and delivery fields with map_irq_thread.
 * No fault, bank, acquisition or IRQ state is changed by taking this receipt.
 * Even with no ready map it requests fresh health, exposing a late fault.
 * A successful read reports faults; it is not itself a fault-free verdict.
 */
static int map_take_health(struct adi_starlink_pss_map *st,
			   u32 words[MAP_HEALTH_WORDS])
{
	struct map_snapshot snapshot;
	unsigned int index;
	u32 overrun_before;
	int ret;

	if (map_read(st, MAP_REG_ID) != MAP_IDENTIFICATION ||
	    map_read(st, MAP_REG_VERSION) != st->version)
		return -ENODEV;
	/* ABI 1.1 predates the input-rate register; its admitted rate is 15. */
	if (st->version != MAP_MIN_VERSION &&
	    map_read(st, MAP_REG_INPUT_RATE_MSPS) != st->input_rate_msps)
		return -ENODEV;
	memset(words, 0, sizeof(u32) * MAP_HEALTH_WORDS);
	words[0] = st->version;
	words[1] = st->input_rate_msps;
	words[3] = map_read(st, MAP_REG_STATUS);
	overrun_before = map_read(st, MAP_REG_SNAPSHOT_REQUEST_OVERRUN);
	ret = map_take_snapshot(st, &snapshot);
	if (ret)
		return ret;
	words[2] = snapshot.generation;
	words[5] = (st->streaming ? BIT(0) : 0) |
		(st->acquisition_enabled ? BIT(1) : 0) |
		(st->irq_live ? BIT(2) : 0);
	words[6] = st->fault_flags;
	words[7] = st->maps_delivered;
	words[8] = st->chunks_delivered;
	words[9] = st->buffer_push_failures;
	words[10] = snapshot.ready_mask;
	words[11] = snapshot.map_generation[0];
	words[12] = snapshot.map_generation[1];
	words[13] = lower_32_bits(snapshot.start_index[0]);
	words[14] = upper_32_bits(snapshot.start_index[0]);
	words[15] = lower_32_bits(snapshot.start_index[1]);
	words[16] = upper_32_bits(snapshot.start_index[1]);
	for (index = 0; index < ARRAY_SIZE(snapshot.fault_signature); index++)
		words[17 + index] = snapshot.fault_signature[index];
	words[31] = map_read(st, MAP_REG_BRIDGE_READ_ERROR);
	words[32] = map_read(st, MAP_REG_BRIDGE_RELEASE_ERROR);
	if (st->input_rate_msps != 15) {
		/* Only ABI 1.4 exposes high words. Paired 30 MS/s may count
		 * internally in 64 bits, but its frozen ABI remains low32.
		 */
		words[34] = st->version == 0x00010004U ? 2U : 1U;
		if (words[34] == 2U) {
			ret = map_read_live_ddc64(st, MAP_REG_DDC_ACCEPTED_LO,
					MAP_REG_DDC_ACCEPTED_HI,
					&words[35], &words[36]);
			if (ret)
				return ret;
			ret = map_read_live_ddc64(st, MAP_REG_DDC_EMITTED_LO,
					MAP_REG_DDC_EMITTED_HI,
					&words[37], &words[38]);
			if (ret)
				return ret;
		} else {
			words[35] = map_read(st, MAP_REG_DDC_ACCEPTED_LO);
			words[37] = map_read(st, MAP_REG_DDC_EMITTED_LO);
		}
		words[39] = map_read(st, MAP_REG_DDC_DISCONTINUITY);
		words[40] = map_read(st, MAP_REG_DDC_SATURATION);
	}
	words[41] = snapshot.accepted;
	words[42] = snapshot.published;
	words[43] = snapshot.denominator_zero;
	words[44] = snapshot.ingress_fifo;
	words[45] = snapshot.candidate_fifo;
	words[4] = map_read(st, MAP_REG_STATUS);
	words[33] = map_read(st, MAP_REG_SNAPSHOT_REQUEST_OVERRUN);
	if (words[33] != overrun_before ||
	    (map_read(st, MAP_REG_SNAPSHOT_STATUS) & 3U) != 1U ||
	    map_read(st, MAP_REG_SNAPSHOT_GENERATION) != snapshot.generation)
		return -EIO;
	return 0;
}

static ssize_t map_acquisition_health_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adi_starlink_pss_map *st = iio_priv(indio_dev);
	u32 words[MAP_HEALTH_WORDS];
	unsigned int index;
	int ret, length;

	mutex_lock(&st->lock);
	ret = map_take_health(st, words);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	length = sysfs_emit(buf, "PSMH 1 %u", MAP_HEALTH_WORDS);
	for (index = 0; index < MAP_HEALTH_WORDS; index++)
		length += sysfs_emit_at(buf, length, " %08x", words[index]);
	return length + sysfs_emit_at(buf, length, "\n");
}

static bool map_snapshot_fault_free(struct adi_starlink_pss_map *st,
				    const struct map_snapshot *snapshot)
{
	u32 health_mask = st->version == MAP_MIN_VERSION ? 0x17ffU : 0x37ffU;
	unsigned int index;

	/* ABI 1.5/1.6 are canonical 15 MS/s with one shared transform service.
	 * Preserve all legacy fatal bits and additionally reject service bit 14;
	 * do not reinterpret the dedicated forward/inverse diagnostics.
	 */
	if (st->version == 0x00010005U || st->version == MAP_STOP_VERSION)
		health_mask = 0x57ffU;

	for (index = 0; index < ARRAY_SIZE(snapshot->fault_signature); index++) {
		if (index == 7) {
			if (snapshot->fault_signature[index] & health_mask)
				return false;
		} else if (snapshot->fault_signature[index]) {
			return false;
		}
	}
	return true;
}

static int map_require_stop(struct adi_starlink_pss_map *st)
{
	if (st->version != MAP_STOP_VERSION)
		return -EOPNOTSUPP;
	if (st->input_rate_msps != 15U ||
	    map_read(st, MAP_REG_ID) != MAP_IDENTIFICATION ||
	    map_read(st, MAP_REG_VERSION) != MAP_STOP_VERSION ||
	    map_read(st, MAP_REG_CAPABILITIES) != MAP_STOP_CAPABILITIES ||
	    map_read(st, MAP_REG_PHASE_BINS) != MAP_PHASE_BINS ||
	    map_read(st, MAP_REG_TILE_GEOMETRY) != MAP_TILE_GEOMETRY ||
	    map_read(st, MAP_REG_INPUT_RATE_MSPS) != 15U ||
	    map_read(st, MAP_REG_DDC_CONFIG) != 0x000f0202U ||
	    map_read(st, MAP_REG_DDC_GROUP_DELAY) != 0U)
		return -ENODEV;
	return 0;
}

/* Caller holds st->lock, shared with IRQ bank copying. Selector writes are
 * side-effect-free except for the selected word; never use auto-increment.
 */
static u32 map_read_stop_word(struct adi_starlink_pss_map *st, u32 selector)
{
	map_write(st, MAP_REG_STOP_WORD, selector);
	return map_read(st, MAP_REG_STOP_WORD);
}

static int map_take_stop(struct adi_starlink_pss_map *st,
			 u32 words[MAP_STOP_WORDS])
{
	static const u32 bracket[] = { 2U, 3U, 4U, 10U, 11U };
	u32 before[ARRAY_SIZE(bracket)], after[ARRAY_SIZE(bracket)];
	u32 status_before, status_after, ticket_before, ticket_after;
	unsigned int attempt, index;
	int ret;

	ret = map_require_stop(st);
	if (ret)
		return ret;
	for (attempt = 0; attempt < 3; attempt++) {
		status_before = map_read(st, MAP_REG_STATUS);
		ticket_before = map_read(st, MAP_REG_STOP_TICKET);
		for (index = 0; index < ARRAY_SIZE(bracket); index++)
			before[index] = map_read_stop_word(st, bracket[index]);
		for (index = 0; index < MAP_STOP_WORDS; index++)
			words[index] = map_read_stop_word(st, index);
		for (index = 0; index < ARRAY_SIZE(bracket); index++)
			after[index] = map_read_stop_word(st, bracket[index]);
		ticket_after = map_read(st, MAP_REG_STOP_TICKET);
		status_after = map_read(st, MAP_REG_STATUS);
		if (memcmp(before, after, sizeof(before)) ||
		    ticket_before != ticket_after || words[3] != ticket_after ||
		    ((status_before ^ status_after) &
		     (MAP_STATUS_EPOCH_LIVE | MAP_STATUS_ENABLED)))
			continue;
		for (index = 0; index < ARRAY_SIZE(bracket); index++)
			if (words[bracket[index]] != after[index])
				break;
		if (index != ARRAY_SIZE(bracket))
			continue;
		ret = map_require_stop(st);
		if (ret)
			return ret;
		if (!(status_after & MAP_STATUS_EPOCH_LIVE))
			return -ENODEV;
		if (words[0] != MAP_STOP_MAGIC ||
		    words[1] != MAP_STOP_VERSION_WORDS ||
		    (words[2] & ~MAP_STOP_FLAGS_MASK) ||
		    (words[10] & ~MAP_STOP_REASONS_MASK) || words[11] > 5U)
			return -EPROTO;
		if (!!(words[2] & MAP_STOP_ENABLED) !=
		    !!(status_after & MAP_STATUS_ENABLED))
			continue;
		/* Refresh only the cache. Success never disables IRQ, streaming,
		 * the pilot path or buffers. Failed/historical tuples are returned
		 * verbatim, not upgraded to structural/health/delivery success.
		 */
		st->acquisition_enabled = !!(status_after & MAP_STATUS_ENABLED);
		return 0;
	}
	return -EAGAIN;
}

static ssize_t map_acquisition_stop_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct adi_starlink_pss_map *st = iio_priv(dev_to_iio_dev(dev));
	u32 words[MAP_STOP_WORDS];
	unsigned int index;
	int ret, length;

	mutex_lock(&st->lock);
	ret = map_take_stop(st, words);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	length = sysfs_emit(buf, "PSST 1 %u", MAP_STOP_WORDS);
	for (index = 0; index < MAP_STOP_WORDS; index++)
		length += sysfs_emit_at(buf, length, " %08x", words[index]);
	return length + sysfs_emit_at(buf, length, "\n");
}

static int map_stop_command_error(u32 command)
{
	switch (command) {
	case 0: return 0;
	case 1: return -EINVAL;
	case 2: return -ERANGE;
	case 3: return -EBUSY;
	case 4: return -EPIPE;
	case 5: return -EIO;
	default: return -EPROTO;
	}
}

static int map_request_stop(struct adi_starlink_pss_map *st, u32 ticket)
{
	u32 words[MAP_STOP_WORDS], accepted;
	int ret, poll_ret;

	if (!ticket)
		return -EINVAL;
	ret = map_take_stop(st, words);
	if (ret)
		return ret;
	if (!st->streaming || !st->irq_live)
		return -EBUSY;
	/* A repeated accepted ticket remains queryable after stop, including
	 * a lost response. It cannot arm another stop after a restart.
	 */
	if (ticket != words[3]) {
		if (st->fault_flags)
			return -EIO;
		if (!st->acquisition_enabled)
			return -EPIPE;
		if (words[3] == U32_MAX)
			return -EOVERFLOW;
		if (ticket != words[3] + 1U)
			return -ERANGE;
	}
	map_write(st, MAP_REG_STOP_TICKET, ticket);
	/* This is only the bounded engine-acceptance phase, never a tile wait.
	 * Hardware performs the same-edge admission/health fence. A staged
	 * request may fail before acceptance; rejected status ends the poll.
	 */
	poll_ret = readl_poll_timeout(st->regs + MAP_REG_STOP_TICKET, accepted,
		accepted == ticket || map_read_stop_word(st, 11U) != 0U,
		1, 10000);
	ret = map_take_stop(st, words);
	if (ret)
		return ret;
	ret = map_stop_command_error(words[11]);
	if (ret)
		return ret;
	if (poll_ret)
		return poll_ret;
	if (words[3] != ticket)
		return -EIO;
	return 0;
}

static ssize_t map_acquisition_stop_request_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct adi_starlink_pss_map *st = iio_priv(dev_to_iio_dev(dev));
	u32 ticket;
	int ret;

	ret = kstrtou32(buf, 0, &ticket);
	if (ret)
		return ret;
	mutex_lock(&st->lock);
	ret = map_request_stop(st, ticket);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;
	return len;
}

static unsigned int map_choose_bank(const struct map_snapshot *snapshot)
{
	if (snapshot->ready_mask == 1U)
		return 0;
	if (snapshot->ready_mask == 2U)
		return 1;
	return snapshot->map_generation[0] < snapshot->map_generation[1] ? 0 : 1;
}

static int map_copy_locked(struct adi_starlink_pss_map *st,
			   const struct map_snapshot *before,
			   unsigned int bank)
{
	struct map_snapshot after;
	u32 bridge_before, bridge_after, command, value;
	unsigned int index;
	int ret;

	bridge_before = map_read(st, MAP_REG_BRIDGE_READ_ERROR);
	if (bridge_before == U32_MAX)
		return -EOVERFLOW;
	map_write(st, MAP_REG_SELECT, bank);
	map_write(st, MAP_REG_INDEX, 0);
	for (index = 0; index < MAP_PHASE_BINS; index++) {
		value = map_read(st, MAP_REG_DATA);
		if (value > U16_MAX)
			return -EIO;
		st->map[index] = value;
	}
	command = map_read(st, MAP_REG_COMMAND_STATUS);
	bridge_after = map_read(st, MAP_REG_BRIDGE_READ_ERROR);
	if (command & (MAP_COMMAND_READ_PENDING | MAP_COMMAND_READ_ERROR) ||
	    bridge_after != bridge_before)
		return -EIO;
	ret = map_take_snapshot(st, &after);
	if (ret)
		return ret;
	if (!(after.ready_mask & BIT(bank)) ||
	    after.map_generation[bank] != before->map_generation[bank] ||
	    after.start_index[bank] != before->start_index[bank] ||
	    memcmp(after.fault_signature, before->fault_signature,
		   sizeof(after.fault_signature)))
		return -ESTALE;
	return 0;
}

static int map_push_chunks_locked(struct adi_starlink_pss_map *st,
				  const struct map_snapshot *snapshot,
				  unsigned int bank)
{
	struct map_scan scan;
	unsigned int chunk;
	int ret;

	for (chunk = 0; chunk < MAP_CHUNKS; chunk++) {
		memset(&scan, 0, sizeof(scan));
		scan.words[0] = MAP_CHUNK_MAGIC;
		scan.words[1] = st->version;
		scan.words[2] = snapshot->map_generation[bank];
		scan.words[3] = chunk;
		scan.words[4] = MAP_CHUNKS;
		scan.words[5] = lower_32_bits(snapshot->start_index[bank]);
		scan.words[6] = upper_32_bits(snapshot->start_index[bank]);
		scan.words[7] = chunk * MAP_CHUNK_BINS;
		scan.words[8] = MAP_CHUNK_BINS;
		memcpy(&scan.words[MAP_META_WORDS],
		       &st->map[chunk * MAP_CHUNK_BINS],
		       MAP_CHUNK_BINS * sizeof(*st->map));
		ret = iio_push_to_buffers(st->indio_dev, &scan);
		if (ret)
			return ret;
		st->chunks_delivered++;
	}
	return 0;
}

static int map_release_locked(struct adi_starlink_pss_map *st,
			      unsigned int bank)
{
	u32 before, after, command;
	int ret;

	before = map_read(st, MAP_REG_BRIDGE_RELEASE_ERROR);
	if (before == U32_MAX)
		return -EOVERFLOW;
	map_write(st, MAP_REG_SELECT, bank);
	map_write(st, MAP_REG_RELEASE, 1U);
	ret = readl_poll_timeout(st->regs + MAP_REG_COMMAND_STATUS, command,
				 !(command & MAP_COMMAND_RELEASE_PENDING),
				 1, 10000);
	if (ret)
		return ret;
	after = map_read(st, MAP_REG_BRIDGE_RELEASE_ERROR);
	if ((command & MAP_COMMAND_RELEASE_ERROR) || after != before)
		return -EIO;
	return 0;
}

static irqreturn_t map_irq_thread(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct adi_starlink_pss_map *st = iio_priv(indio_dev);
	struct map_snapshot snapshot;
	unsigned int bank;
	int ret;

	mutex_lock(&st->lock);
	if (!st->streaming) {
		mutex_unlock(&st->lock);
		return IRQ_HANDLED;
	}
	if (!(map_read(st, MAP_REG_STATUS) & MAP_STATUS_READY_MASK)) {
		mutex_unlock(&st->lock);
		return IRQ_NONE;
	}
	ret = map_take_snapshot(st, &snapshot);
	if (ret || !snapshot.ready_mask) {
		map_stop_on_fault(st, MAP_FAULT_SNAPSHOT);
		goto out;
	}
	if (!map_snapshot_fault_free(st, &snapshot)) {
		map_stop_on_fault(st, MAP_FAULT_HARDWARE_HEALTH);
		goto out;
	}
	if (snapshot.ready_mask == 3U &&
	    snapshot.map_generation[0] == snapshot.map_generation[1]) {
		map_stop_on_fault(st, MAP_FAULT_COHERENCE);
		goto out;
	}
	bank = map_choose_bank(&snapshot);
	ret = map_copy_locked(st, &snapshot, bank);
	if (ret) {
		map_stop_on_fault(st, ret == -ESTALE ?
				  MAP_FAULT_COHERENCE : MAP_FAULT_DATA);
		goto out;
	}
	ret = map_push_chunks_locked(st, &snapshot, bank);
	if (ret) {
		st->buffer_push_failures++;
		map_stop_on_fault(st, MAP_FAULT_BUFFER_FULL);
		goto out;
	}
	ret = map_release_locked(st, bank);
	if (ret) {
		map_stop_on_fault(st, MAP_FAULT_RELEASE);
		goto out;
	}
	st->maps_delivered++;
out:
	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}

static int map_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adi_starlink_pss_map *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	if (st->streaming) {
		mutex_unlock(&st->lock);
		return -EBUSY;
	}
	st->streaming = true;
	st->irq_live = true;
	mutex_unlock(&st->lock);
	enable_irq(st->irq);
	return 0;
}

static int map_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adi_starlink_pss_map *st = iio_priv(indio_dev);
	bool irq_live;

	mutex_lock(&st->lock);
	st->streaming = false;
	st->acquisition_enabled = false;
	map_write(st, MAP_REG_CONTROL, 0);
	irq_live = st->irq_live;
	st->irq_live = false;
	mutex_unlock(&st->lock);
	if (irq_live)
		disable_irq(st->irq);
	return 0;
}

static const struct iio_buffer_setup_ops map_buffer_ops = {
	.postenable = map_buffer_postenable,
	.predisable = map_buffer_predisable,
};

static ssize_t map_attr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adi_starlink_pss_map *st = iio_priv(indio_dev);
	u32 value;

	mutex_lock(&st->lock);
	switch (this_attr->address) {
	case MAP_ATTR_ID:
		value = map_read(st, MAP_REG_ID);
		break;
	case MAP_ATTR_VERSION:
		value = st->version;
		break;
	case MAP_ATTR_PHASE_BINS:
		value = MAP_PHASE_BINS;
		break;
	case MAP_ATTR_TILE_GEOMETRY:
		value = map_read(st, MAP_REG_TILE_GEOMETRY);
		break;
	case MAP_ATTR_CAPABILITIES:
		value = map_read(st, MAP_REG_CAPABILITIES);
		break;
	case MAP_ATTR_STATUS:
		value = map_read(st, MAP_REG_STATUS);
		break;
	case MAP_ATTR_INPUT_RATE:
		value = st->input_rate_msps;
		break;
	case MAP_ATTR_DDC_CONFIG:
		value = map_read(st, MAP_REG_DDC_CONFIG);
		break;
	case MAP_ATTR_DDC_GROUP_DELAY:
		value = map_read(st, MAP_REG_DDC_GROUP_DELAY);
		break;
	case MAP_ATTR_ACQUISITION_ENABLE:
		value = st->acquisition_enabled;
		break;
	case MAP_ATTR_MAPS_DELIVERED:
		value = st->maps_delivered;
		break;
	case MAP_ATTR_CHUNKS_DELIVERED:
		value = st->chunks_delivered;
		break;
	case MAP_ATTR_BUFFER_PUSH_FAILURES:
		value = st->buffer_push_failures;
		break;
	case MAP_ATTR_REASSEMBLY_CHUNKS:
		value = MAP_CHUNKS;
		break;
	case MAP_ATTR_FAULT_FLAGS:
		value = st->fault_flags;
		break;
	default:
		mutex_unlock(&st->lock);
		return -EINVAL;
	}
	mutex_unlock(&st->lock);
	return sysfs_emit(buf, "%u\n", value);
}

static ssize_t map_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adi_starlink_pss_map *st = iio_priv(indio_dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;
	mutex_lock(&st->lock);
	switch (this_attr->address) {
	case MAP_ATTR_ACQUISITION_ENABLE:
		if (enable && (!st->streaming || st->fault_flags)) {
			ret = -EINVAL;
			break;
		}
		st->acquisition_enabled = enable;
		map_write(st, MAP_REG_CONTROL, enable ? 1U : 0U);
		break;
	case MAP_ATTR_ACQUISITION_FLUSH:
		if (!enable || st->streaming || st->acquisition_enabled) {
			ret = -EBUSY;
			break;
		}
		map_write(st, MAP_REG_CONTROL, 2U);
		break;
	case MAP_ATTR_FAULT_CLEAR:
		if (!enable || st->streaming ||
		    (map_read(st, MAP_REG_STATUS) & MAP_STATUS_READY_MASK)) {
			ret = -EBUSY;
			break;
		}
		st->fault_flags = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);
	return ret ? ret : len;
}

#define MAP_ATTR_RO(_name, _address) \
	IIO_DEVICE_ATTR(_name, 0444, map_attr_show, NULL, _address)
#define MAP_ATTR_RW(_name, _address) \
	IIO_DEVICE_ATTR(_name, 0644, map_attr_show, map_attr_store, _address)
#define MAP_ATTR_WO(_name, _address) \
	IIO_DEVICE_ATTR(_name, 0200, NULL, map_attr_store, _address)

static MAP_ATTR_RO(fpga_identity, MAP_ATTR_ID);
static MAP_ATTR_RO(abi_version, MAP_ATTR_VERSION);
static MAP_ATTR_RO(phase_bins, MAP_ATTR_PHASE_BINS);
static MAP_ATTR_RO(tile_geometry, MAP_ATTR_TILE_GEOMETRY);
static MAP_ATTR_RO(capabilities, MAP_ATTR_CAPABILITIES);
static MAP_ATTR_RO(status, MAP_ATTR_STATUS);
static MAP_ATTR_RO(input_rate_msps, MAP_ATTR_INPUT_RATE);
static MAP_ATTR_RO(ddc_config, MAP_ATTR_DDC_CONFIG);
static MAP_ATTR_RO(ddc_group_delay, MAP_ATTR_DDC_GROUP_DELAY);
static MAP_ATTR_RW(acquisition_enable, MAP_ATTR_ACQUISITION_ENABLE);
static MAP_ATTR_WO(acquisition_flush, MAP_ATTR_ACQUISITION_FLUSH);
static MAP_ATTR_RO(maps_delivered, MAP_ATTR_MAPS_DELIVERED);
static MAP_ATTR_RO(chunks_delivered, MAP_ATTR_CHUNKS_DELIVERED);
static MAP_ATTR_RO(buffer_push_failures, MAP_ATTR_BUFFER_PUSH_FAILURES);
static MAP_ATTR_RO(reassembly_chunks, MAP_ATTR_REASSEMBLY_CHUNKS);
static MAP_ATTR_RO(fault_flags, MAP_ATTR_FAULT_FLAGS);
static MAP_ATTR_WO(fault_clear, MAP_ATTR_FAULT_CLEAR);
static IIO_DEVICE_ATTR(acquisition_health, 0444, map_acquisition_health_show,
		       NULL, 0);
static IIO_DEVICE_ATTR(acquisition_stop, 0444, map_acquisition_stop_show,
		       NULL, 0);
static IIO_DEVICE_ATTR(acquisition_stop_request, 0200, NULL,
		       map_acquisition_stop_request_store, 0);

static struct attribute *map_attrs[] = {
	&iio_dev_attr_fpga_identity.dev_attr.attr,
	&iio_dev_attr_abi_version.dev_attr.attr,
	&iio_dev_attr_phase_bins.dev_attr.attr,
	&iio_dev_attr_tile_geometry.dev_attr.attr,
	&iio_dev_attr_capabilities.dev_attr.attr,
	&iio_dev_attr_status.dev_attr.attr,
	&iio_dev_attr_input_rate_msps.dev_attr.attr,
	&iio_dev_attr_ddc_config.dev_attr.attr,
	&iio_dev_attr_ddc_group_delay.dev_attr.attr,
	&iio_dev_attr_acquisition_enable.dev_attr.attr,
	&iio_dev_attr_acquisition_flush.dev_attr.attr,
	&iio_dev_attr_maps_delivered.dev_attr.attr,
	&iio_dev_attr_chunks_delivered.dev_attr.attr,
	&iio_dev_attr_buffer_push_failures.dev_attr.attr,
	&iio_dev_attr_reassembly_chunks.dev_attr.attr,
	&iio_dev_attr_fault_flags.dev_attr.attr,
	&iio_dev_attr_fault_clear.dev_attr.attr,
	&iio_dev_attr_acquisition_health.dev_attr.attr,
	&iio_dev_attr_acquisition_stop.dev_attr.attr,
	&iio_dev_attr_acquisition_stop_request.dev_attr.attr,
	NULL,
};

static const struct attribute_group map_attr_group = {
	.attrs = map_attrs,
};

static const struct iio_info map_iio_info = {
	.attrs = &map_attr_group,
};

static int map_require_contract(struct adi_starlink_pss_map *st)
{
	static const u32 contract_30[8] = {
		0x73142604U, 0x7077b036U, 0xf9213db3U, 0x574e4a55U,
		0x6fd424b9U, 0x7a293843U, 0xbd6ee085U, 0xc2bf33afU,
	};
	static const u32 contract_60[8] = {
		0x8e807d15U, 0xd5372b0aU, 0x9669d119U, 0x0d899697U,
		0xe7c2911aU, 0x73ddfb23U, 0x095806c2U, 0xa31de5b2U,
	};
	const u32 *contract = NULL;
	u32 capabilities, expected_capabilities;
	u32 config = 0, delay = 0, energy = 0;
	unsigned int index;

	switch (st->version) {
	case 0x00010001U:
		st->input_rate_msps = 15;
		expected_capabilities = 0x0000003fU;
		break;
	case 0x00010002U:
		st->input_rate_msps = 30;
		expected_capabilities = 0x0000007fU;
		config = 0x000f0203U;
		delay = 7;
		energy = 1073744004U;
		contract = contract_30;
		break;
	case 0x00010003U:
		st->input_rate_msps = 60;
		expected_capabilities = 0x0000007fU;
		config = 0x020f0403U;
		delay = 21;
		energy = 1073765335U;
		contract = contract_60;
		break;
	case 0x00010004U:
		st->input_rate_msps = 60;
		expected_capabilities = 0x000000ffU;
		config = 0x020f0403U;
		delay = 21;
		energy = 1073765335U;
		contract = contract_60;
		break;
	case 0x00010005U:
	case MAP_STOP_VERSION:
		/* Explicit opt-in image identity; never a 30/60 MS/s contract. */
		st->input_rate_msps = 15;
		expected_capabilities = st->version == MAP_STOP_VERSION ?
			MAP_STOP_CAPABILITIES : 0x0000013fU;
		if (map_read(st, MAP_REG_INPUT_RATE_MSPS) != 15U ||
		    map_read(st, MAP_REG_DDC_CONFIG) != 0x000f0202U ||
		    map_read(st, MAP_REG_DDC_GROUP_DELAY) != 0U)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	capabilities = map_read(st, MAP_REG_CAPABILITIES);
	if (capabilities != expected_capabilities)
		return -EINVAL;
	if (!contract)
		return 0;
	if (map_read(st, MAP_REG_INPUT_RATE_MSPS) != st->input_rate_msps ||
	    map_read(st, MAP_REG_DDC_CONFIG) != config ||
	    map_read(st, MAP_REG_DDC_GROUP_DELAY) != delay ||
	    map_read(st, MAP_REG_COEFFICIENT_ENERGY) != energy)
		return -EINVAL;
	for (index = 0; index < ARRAY_SIZE(contract_30); index++)
		if (map_read(st, MAP_REG_DDC_CONTRACT_0 + 4 * index) !=
		    contract[index])
			return -EINVAL;
	return 0;
}

static int adi_starlink_pss_map_probe(struct platform_device *pdev)
{
	struct adi_starlink_pss_map *st;
	struct iio_dev *indio_dev;
	u32 status;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;
	st->indio_dev = indio_dev;
	mutex_init(&st->lock);
	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);
	if (map_read(st, MAP_REG_ID) != MAP_IDENTIFICATION)
		return -ENODEV;
	st->version = map_read(st, MAP_REG_VERSION);
	if (st->version < MAP_MIN_VERSION || st->version > MAP_MAX_VERSION ||
	    map_read(st, MAP_REG_PHASE_BINS) != MAP_PHASE_BINS ||
	    map_read(st, MAP_REG_TILE_GEOMETRY) != MAP_TILE_GEOMETRY)
		return -ENODEV;
	status = map_read(st, MAP_REG_STATUS);
	if (!(status & MAP_STATUS_EPOCH_LIVE))
		return -EPROBE_DEFER;
	ret = map_require_contract(st);
	if (ret)
		return -EINVAL;
	st->map = devm_kmalloc_array(&pdev->dev, MAP_PHASE_BINS,
				     sizeof(*st->map), GFP_KERNEL);
	if (!st->map)
		return -ENOMEM;
	/* Probe never starts acquisition inherited from a previous userspace. */
	map_write(st, MAP_REG_CONTROL, 0);

	st->irq = platform_get_irq(pdev, 0);
	if (st->irq < 0)
		return st->irq;
	ret = devm_request_threaded_irq(&pdev->dev, st->irq, NULL,
					map_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(&pdev->dev), indio_dev);
	if (ret)
		return ret;

	indio_dev->name = "starlink-pss-map";
	indio_dev->info = &map_iio_info;
	indio_dev->channels = map_channels;
	indio_dev->num_channels = ARRAY_SIZE(map_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  INDIO_BUFFER_SOFTWARE,
					  &map_buffer_ops);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, indio_dev);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int adi_starlink_pss_map_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct adi_starlink_pss_map *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	st->acquisition_enabled = false;
	map_write(st, MAP_REG_CONTROL, 0);
	mutex_unlock(&st->lock);
	return 0;
}

static const struct of_device_id adi_starlink_pss_map_of_match[] = {
	{ .compatible = "adi,starlink-pss-map-1.00.a" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_starlink_pss_map_of_match);

static struct platform_driver adi_starlink_pss_map_driver = {
	.driver = {
		.name = "adi-starlink-pss-map",
		.of_match_table = adi_starlink_pss_map_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = adi_starlink_pss_map_probe,
	.remove = adi_starlink_pss_map_remove,
};
module_platform_driver(adi_starlink_pss_map_driver);

MODULE_AUTHOR("SPF contributors");
MODULE_DESCRIPTION("Starlink PSS coarse phase-map IIO driver");
MODULE_LICENSE("GPL");
