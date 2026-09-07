// SPDX-License-Identifier: GPL-2.0-only
/*
 * Starlink PSS sparse fine-timing tracker IIO driver
 *
 * The FPGA publishes one immutable 26-word result and holds its bank until
 * software releases it.  This driver copies that complete packet into one
 * IIO scan from the threaded level interrupt, validates the self-describing
 * envelope, and releases the FPGA bank only after the IIO kfifo accepted it.
 * Raw receive samples never cross the PS boundary.
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
#include <linux/workqueue.h>

#define PSS_REG_ID                         0x00
#define PSS_REG_VERSION                    0x04
#define PSS_REG_RATE_MSPS                  0x08
#define PSS_REG_GEOMETRY                   0x0c
#define PSS_REG_CAPABILITIES               0x10
#define PSS_REG_STATUS                     0x14
#define PSS_REG_CURRENT_INDEX_LO           0x18
#define PSS_REG_CURRENT_INDEX_HI           0x1c
#define PSS_REG_CANDIDATE_REQUEST          0x20
#define PSS_REG_CANDIDATE_CENTER_LO        0x24
#define PSS_REG_CANDIDATE_CENTER_HI        0x28
#define PSS_REG_CANDIDATE_TIMESTAMP_LO     0x2c
#define PSS_REG_CANDIDATE_TIMESTAMP_HI     0x30
#define PSS_REG_CANDIDATE_CONTROL          0x34
#define PSS_REG_COEFFICIENT_DATA           0x40
#define PSS_REG_COEFFICIENT_CONTROL        0x44
#define PSS_REG_COEFFICIENT_GENERATION     0x48
#define PSS_REG_ACTIVE_COEFFICIENT_GEN     0x4c
#define PSS_REG_RESULT_WORD_INDEX          0x50
#define PSS_REG_RESULT_WORD_DATA           0x54
#define PSS_REG_RESULT_CONTROL             0x58
#define PSS_REG_RESULT_STATUS              0x5c

#define PSS_IDENTIFICATION                 0x50535354U /* "PSST" */
#define PSS_VERSION_1_2                    0x00010002U
#define PSS_VERSION_1_3                    0x00010003U
#define PSS_PACKET_MAGIC                   0x31535350U /* "PSS1" */
#define PSS_PACKET_HEADER                  0x1a010001U
#define PSS_RESULT_WORDS                   26U
#define PSS_MAX_COEFFICIENTS               264U
#define PSS_COMMAND_FIFO_USABLE            7U

#define PSS_STATUS_RESET_RELEASED          BIT(0)
#define PSS_STATUS_CANDIDATE_READY         BIT(1)
#define PSS_STATUS_COMMAND_BUFFERED        BIT(2)
#define PSS_STATUS_COEFFICIENT_VALID       BIT(3)
#define PSS_STATUS_COEFFICIENT_READY       BIT(4)
#define PSS_STATUS_COEFFICIENT_COMMIT_READY BIT(5)
#define PSS_STATUS_RESULT_AVAILABLE        BIT(6)

#define PSS_RESULT_STATUS_AVAILABLE        BIT(0)
#define PSS_RESULT_STATUS_WORDS            GENMASK(28, 24)

#define PSS_FAULT_BAD_PACKET               BIT(0)
#define PSS_FAULT_BUFFER_FULL              BIT(1)
#define PSS_FAULT_BAD_RESULT_STATUS        BIT(2)
#define PSS_FAULT_SCHEDULE                  BIT(3)

enum pss_attr {
	PSS_ATTR_ID,
	PSS_ATTR_VERSION,
	PSS_ATTR_RATE,
	PSS_ATTR_GEOMETRY,
	PSS_ATTR_CAPABILITIES,
	PSS_ATTR_STATUS,
	PSS_ATTR_CURRENT_INDEX,
	PSS_ATTR_ACTIVE_COEFFICIENT_GENERATION,
	PSS_ATTR_COEFFICIENT_GENERATION,
	PSS_ATTR_COEFFICIENT_WORDS,
	PSS_ATTR_COEFFICIENT_COMMIT,
	PSS_ATTR_SCHEDULE_FIRST_CENTER,
	PSS_ATTR_SCHEDULE_PERIOD_Q32_32,
	PSS_ATTR_SCHEDULE_REQUEST_BASE,
	PSS_ATTR_SCHEDULE_COUNT,
	PSS_ATTR_SCHEDULE_QUEUE_TARGET,
	PSS_ATTR_SCHEDULE_ENABLE,
	PSS_ATTR_SCHEDULE_SUBMITTED,
	PSS_ATTR_PACKETS_DELIVERED,
	PSS_ATTR_BUFFER_PUSH_FAILURES,
	PSS_ATTR_PACKET_VALIDATION_FAILURES,
	PSS_ATTR_FAULT_FLAGS,
	PSS_ATTR_FAULT_CLEAR,
};

struct pss_scan {
	u32 words[PSS_RESULT_WORDS];
	s64 timestamp __aligned(8);
};

struct adi_starlink_pss_tracker {
	struct device *dev;
	struct iio_dev *indio_dev;
	void __iomem *regs;
	struct mutex lock;
	struct work_struct fill_work;
	int irq;
	bool irq_live;
	bool streaming;
	bool scheduling;
	bool coefficients_staged;
	u32 rate_msps;
	u32 coefficient_count;
	u32 coefficient_count_mask;
	u32 queue_room_shift;
	u32 coefficient_generation;
	u64 first_center;
	u64 next_center;
	u64 period_q32_32;
	u32 next_fraction;
	u32 request_base;
	u32 schedule_count;
	u32 queue_target;
	u32 submitted;
	u32 fault_flags;
	u32 packets_delivered;
	u32 buffer_push_failures;
	u32 packet_validation_failures;
};

static const struct iio_chan_spec pss_channels[] = {
	{
		.type = IIO_COUNT,
		.indexed = 1,
		.channel = 0,
		.extend_name = "packet_words",
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
			.repeat = PSS_RESULT_WORDS,
			.endianness = IIO_LE,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static u32 pss_read(struct adi_starlink_pss_tracker *st, unsigned int reg)
{
	return ioread32(st->regs + reg);
}

static void pss_write(struct adi_starlink_pss_tracker *st, unsigned int reg,
		      u32 value)
{
	iowrite32(value, st->regs + reg);
}

static u64 pss_current_index(struct adi_starlink_pss_tracker *st)
{
	u32 low = pss_read(st, PSS_REG_CURRENT_INDEX_LO);
	u32 high = pss_read(st, PSS_REG_CURRENT_INDEX_HI);

	return ((u64)high << 32) | low;
}

static s64 pss_s48(u32 low, u32 high)
{
	u64 value = ((u64)(high & 0xffffU) << 32) | low;

	return sign_extend64(value, 47);
}

static bool pss_packet_valid(struct adi_starlink_pss_tracker *st,
			     const u32 *words)
{
	s32 lag = (s32)words[7];
	s32 maximum_lag = 30 * (s32)(st->rate_msps / 15);
	u64 center_index = ((u64)words[4] << 32) | words[3];
	u64 center = ((u64)words[6] << 32) | words[5];
	u64 winner = ((u64)words[9] << 32) | words[8];

	/* The exact header and generation make packet interpretation immutable. */
	if (words[0] != PSS_PACKET_MAGIC || words[1] != PSS_PACKET_HEADER ||
	    !words[2] || !words[10] ||
	    words[10] != pss_read(st, PSS_REG_ACTIVE_COEFFICIENT_GEN))
		return false;
	if (center_index != center || lag < -maximum_lag || lag > maximum_lag)
		return false;
	if (winner != (u64)((s64)center + lag))
		return false;
	if (pss_s48(words[15], words[16]) <= 0 ||
	    pss_s48(words[17], words[18]) <= 0 || words[19])
		return false;
	return true;
}

static void pss_advance_schedule(struct adi_starlink_pss_tracker *st)
{
	u32 old_fraction = st->next_fraction;

	st->next_center += st->period_q32_32 >> 32;
	st->next_fraction += (u32)st->period_q32_32;
	if (st->next_fraction < old_fraction)
		st->next_center++;
}

static bool pss_more_to_submit(struct adi_starlink_pss_tracker *st)
{
	return !st->schedule_count || st->submitted < st->schedule_count;
}

static int pss_submit_one_locked(struct adi_starlink_pss_tracker *st)
{
	u32 status = pss_read(st, PSS_REG_STATUS);
	u32 room = (status >> st->queue_room_shift) & 0x7U;
	u32 request_id;
	u64 center;

	if (!(status & PSS_STATUS_CANDIDATE_READY) ||
	    (status & PSS_STATUS_COMMAND_BUFFERED) || !room)
		return -EAGAIN;
	if (!pss_more_to_submit(st)) {
		st->scheduling = false;
		return -ENODATA;
	}

	request_id = st->request_base + st->submitted;
	if (!request_id)
		return -ERANGE;
	center = st->next_center;
	pss_write(st, PSS_REG_CANDIDATE_REQUEST, request_id);
	pss_write(st, PSS_REG_CANDIDATE_CENTER_LO, lower_32_bits(center));
	pss_write(st, PSS_REG_CANDIDATE_CENTER_HI, upper_32_bits(center));
	pss_write(st, PSS_REG_CANDIDATE_TIMESTAMP_LO, lower_32_bits(center));
	pss_write(st, PSS_REG_CANDIDATE_TIMESTAMP_HI, upper_32_bits(center));
	pss_write(st, PSS_REG_CANDIDATE_CONTROL, 1U);
	if (readl_poll_timeout(st->regs + PSS_REG_STATUS, status,
			       !(status & PSS_STATUS_COMMAND_BUFFERED), 1, 1000))
		return -ETIMEDOUT;
	st->submitted++;
	pss_advance_schedule(st);
	return 0;
}

static void pss_fill_work(struct work_struct *work)
{
	struct adi_starlink_pss_tracker *st = container_of(work,
		struct adi_starlink_pss_tracker, fill_work);
	u32 status, room, wanted;
	int ret;

	mutex_lock(&st->lock);
	while (st->streaming && st->scheduling && !st->fault_flags) {
		status = pss_read(st, PSS_REG_STATUS);
		room = (status >> st->queue_room_shift) & 0x7U;
		wanted = min(st->queue_target, PSS_COMMAND_FIFO_USABLE);
		if (!room || room < PSS_COMMAND_FIFO_USABLE - wanted + 1)
			break;
		ret = pss_submit_one_locked(st);
		if (ret == -EAGAIN || ret == -ENODATA)
			break;
		if (ret) {
			st->fault_flags |= PSS_FAULT_SCHEDULE;
			st->scheduling = false;
			break;
		}
	}
	mutex_unlock(&st->lock);
}

static irqreturn_t pss_irq_thread(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct adi_starlink_pss_tracker *st = iio_priv(indio_dev);
	struct pss_scan scan = { };
	u32 result_status;
	unsigned int index;
	int ret;

	mutex_lock(&st->lock);
	if (!st->streaming) {
		mutex_unlock(&st->lock);
		return IRQ_HANDLED;
	}
	result_status = pss_read(st, PSS_REG_RESULT_STATUS);
	if (!(result_status & PSS_RESULT_STATUS_AVAILABLE)) {
		mutex_unlock(&st->lock);
		return IRQ_NONE;
	}
	if (FIELD_GET(PSS_RESULT_STATUS_WORDS, result_status) !=
	    PSS_RESULT_WORDS) {
		st->fault_flags |= PSS_FAULT_BAD_RESULT_STATUS;
		st->packet_validation_failures++;
		st->scheduling = false;
		disable_irq_nosync(st->irq);
		st->irq_live = false;
		mutex_unlock(&st->lock);
		return IRQ_HANDLED;
	}

	for (index = 0; index < PSS_RESULT_WORDS; index++) {
		pss_write(st, PSS_REG_RESULT_WORD_INDEX, index);
		scan.words[index] = pss_read(st, PSS_REG_RESULT_WORD_DATA);
	}
	if (!pss_packet_valid(st, scan.words)) {
		st->fault_flags |= PSS_FAULT_BAD_PACKET;
		st->packet_validation_failures++;
		st->scheduling = false;
		disable_irq_nosync(st->irq);
		st->irq_live = false;
		mutex_unlock(&st->lock);
		return IRQ_HANDLED;
	}

	ret = iio_push_to_buffers_with_timestamp(indio_dev, &scan,
					 iio_get_time_ns(indio_dev));
	if (ret) {
		st->fault_flags |= PSS_FAULT_BUFFER_FULL;
		st->buffer_push_failures++;
		st->scheduling = false;
		disable_irq_nosync(st->irq);
		st->irq_live = false;
		mutex_unlock(&st->lock);
		return IRQ_HANDLED;
	}

	/* The hardware bank is released only after a complete accepted scan. */
	pss_write(st, PSS_REG_RESULT_CONTROL, 1U);
	st->packets_delivered++;
	mutex_unlock(&st->lock);
	schedule_work(&st->fill_work);
	return IRQ_HANDLED;
}

static int pss_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adi_starlink_pss_tracker *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	if (st->streaming) {
		mutex_unlock(&st->lock);
		return -EBUSY;
	}
	st->streaming = true;
	st->irq_live = true;
	mutex_unlock(&st->lock);
	enable_irq(st->irq);
	schedule_work(&st->fill_work);
	return 0;
}

static int pss_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adi_starlink_pss_tracker *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	st->streaming = false;
	st->scheduling = false;
	mutex_unlock(&st->lock);
	cancel_work_sync(&st->fill_work);
	if (st->irq_live) {
		disable_irq(st->irq);
		st->irq_live = false;
	}
	return 0;
}

static const struct iio_buffer_setup_ops pss_buffer_ops = {
	.postenable = pss_buffer_postenable,
	.predisable = pss_buffer_predisable,
};

static ssize_t pss_attr_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adi_starlink_pss_tracker *st = iio_priv(indio_dev);
	u64 value;

	mutex_lock(&st->lock);
	switch (this_attr->address) {
	case PSS_ATTR_ID:
		value = pss_read(st, PSS_REG_ID);
		break;
	case PSS_ATTR_VERSION:
		value = pss_read(st, PSS_REG_VERSION);
		break;
	case PSS_ATTR_RATE:
		value = st->rate_msps;
		break;
	case PSS_ATTR_GEOMETRY:
		value = pss_read(st, PSS_REG_GEOMETRY);
		break;
	case PSS_ATTR_CAPABILITIES:
		value = pss_read(st, PSS_REG_CAPABILITIES);
		break;
	case PSS_ATTR_STATUS:
		value = pss_read(st, PSS_REG_STATUS);
		break;
	case PSS_ATTR_CURRENT_INDEX:
		value = pss_current_index(st);
		break;
	case PSS_ATTR_ACTIVE_COEFFICIENT_GENERATION:
		value = pss_read(st, PSS_REG_ACTIVE_COEFFICIENT_GEN);
		break;
	case PSS_ATTR_COEFFICIENT_GENERATION:
		value = st->coefficient_generation;
		break;
	case PSS_ATTR_SCHEDULE_FIRST_CENTER:
		value = st->first_center;
		break;
	case PSS_ATTR_SCHEDULE_PERIOD_Q32_32:
		value = st->period_q32_32;
		break;
	case PSS_ATTR_SCHEDULE_REQUEST_BASE:
		value = st->request_base;
		break;
	case PSS_ATTR_SCHEDULE_COUNT:
		value = st->schedule_count;
		break;
	case PSS_ATTR_SCHEDULE_QUEUE_TARGET:
		value = st->queue_target;
		break;
	case PSS_ATTR_SCHEDULE_ENABLE:
		value = st->scheduling;
		break;
	case PSS_ATTR_SCHEDULE_SUBMITTED:
		value = st->submitted;
		break;
	case PSS_ATTR_PACKETS_DELIVERED:
		value = st->packets_delivered;
		break;
	case PSS_ATTR_BUFFER_PUSH_FAILURES:
		value = st->buffer_push_failures;
		break;
	case PSS_ATTR_PACKET_VALIDATION_FAILURES:
		value = st->packet_validation_failures;
		break;
	case PSS_ATTR_FAULT_FLAGS:
		value = st->fault_flags;
		break;
	default:
		mutex_unlock(&st->lock);
		return -EINVAL;
	}
	mutex_unlock(&st->lock);
	return sysfs_emit(buf, "%llu\n", (unsigned long long)value);
}

static int pss_parse_u64(const char *buf, u64 *value)
{
	return kstrtou64(buf, 0, value);
}

static ssize_t pss_coefficients_store(struct adi_starlink_pss_tracker *st,
				      const char *buf, size_t len)
{
	char *copy, *cursor, *token;
	u32 *words;
	u32 status;
	unsigned int count = 0;
	int ret = 0;

	copy = kstrndup(buf, len, GFP_KERNEL);
	words = kmalloc_array(st->coefficient_count, sizeof(*words), GFP_KERNEL);
	if (!copy || !words) {
		ret = -ENOMEM;
		goto out;
	}
	cursor = copy;
	while ((token = strsep(&cursor, ", \t\r\n")) != NULL) {
		if (!*token)
			continue;
		if (count >= st->coefficient_count ||
		    kstrtou32(token, 0, &words[count])) {
			ret = -EINVAL;
			goto out;
		}
		count++;
	}
	if (count != st->coefficient_count) {
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&st->lock);
	if (st->streaming || st->scheduling) {
		ret = -EBUSY;
		goto unlock;
	}
	pss_write(st, PSS_REG_COEFFICIENT_CONTROL, 1U);
	ret = readl_poll_timeout(st->regs + PSS_REG_STATUS, status,
				 !(status & (st->coefficient_count_mask << 8)) &&
				 (status & PSS_STATUS_COEFFICIENT_READY),
				 1, 10000);
	if (ret)
		goto unlock;
	for (count = 0; count < st->coefficient_count; count++) {
		pss_write(st, PSS_REG_COEFFICIENT_DATA, words[count]);
		ret = readl_poll_timeout(st->regs + PSS_REG_STATUS, status,
					 ((status >> 8) &
					  st->coefficient_count_mask) == count + 1,
					 1, 10000);
		if (ret)
			goto unlock;
	}
	st->coefficients_staged = true;
unlock:
	mutex_unlock(&st->lock);
out:
	kfree(words);
	kfree(copy);
	return ret ? ret : len;
}

static ssize_t pss_attr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adi_starlink_pss_tracker *st = iio_priv(indio_dev);
	u64 value;
	u64 current_index;
	u32 status;
	int ret;

	if (this_attr->address == PSS_ATTR_COEFFICIENT_WORDS)
		return pss_coefficients_store(st, buf, len);
	ret = pss_parse_u64(buf, &value);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	switch (this_attr->address) {
	case PSS_ATTR_COEFFICIENT_GENERATION:
		if (!value || value > U32_MAX || st->streaming)
			ret = -EINVAL;
		else
			st->coefficient_generation = value;
		break;
	case PSS_ATTR_COEFFICIENT_COMMIT:
		if (value != 1 || st->streaming || !st->coefficients_staged ||
		    !st->coefficient_generation) {
			ret = -EINVAL;
			break;
		}
		status = pss_read(st, PSS_REG_STATUS);
		if (!(status & PSS_STATUS_COEFFICIENT_COMMIT_READY)) {
			ret = -EBUSY;
			break;
		}
		pss_write(st, PSS_REG_COEFFICIENT_GENERATION,
			  st->coefficient_generation);
		pss_write(st, PSS_REG_COEFFICIENT_CONTROL, 2U);
		ret = readl_poll_timeout(st->regs +
					 PSS_REG_ACTIVE_COEFFICIENT_GEN, status,
					 status == st->coefficient_generation,
					 1, 10000);
		if (!ret)
			ret = readl_poll_timeout(st->regs + PSS_REG_STATUS,
						 status,
						 (status &
						  PSS_STATUS_COEFFICIENT_VALID) &&
						 !((status >> 8) &
						   st->coefficient_count_mask),
						 1, 10000);
		if (!ret)
			st->coefficients_staged = false;
		break;
	case PSS_ATTR_SCHEDULE_FIRST_CENTER:
		if (st->scheduling)
			ret = -EBUSY;
		else
			st->first_center = value;
		break;
	case PSS_ATTR_SCHEDULE_PERIOD_Q32_32:
		if (st->scheduling || !(value >> 32))
			ret = -EINVAL;
		else
			st->period_q32_32 = value;
		break;
	case PSS_ATTR_SCHEDULE_REQUEST_BASE:
		if (st->scheduling || !value || value > U32_MAX)
			ret = -EINVAL;
		else
			st->request_base = value;
		break;
	case PSS_ATTR_SCHEDULE_COUNT:
		if (st->scheduling || value > U32_MAX)
			ret = -EINVAL;
		else
			st->schedule_count = value;
		break;
	case PSS_ATTR_SCHEDULE_QUEUE_TARGET:
		if (st->scheduling || !value || value > PSS_COMMAND_FIFO_USABLE)
			ret = -EINVAL;
		else
			st->queue_target = value;
		break;
	case PSS_ATTR_SCHEDULE_ENABLE:
		if (value == 0) {
			st->scheduling = false;
			break;
		}
		status = pss_read(st, PSS_REG_STATUS);
		current_index = pss_current_index(st);
		if (value != 1 || !st->streaming || st->fault_flags ||
		    !st->first_center || !st->period_q32_32 || !st->request_base ||
		    st->first_center < current_index ||
		    st->first_center - current_index <
			65536U * (st->rate_msps / 15) ||
		    !(status & PSS_STATUS_COEFFICIENT_VALID) ||
		    !pss_read(st, PSS_REG_ACTIVE_COEFFICIENT_GEN)) {
			ret = -EINVAL;
			break;
		}
		st->next_center = st->first_center;
		st->next_fraction = 0;
		st->submitted = 0;
		st->scheduling = true;
		break;
	case PSS_ATTR_FAULT_CLEAR:
		if (value != 1 || st->streaming ||
		    (pss_read(st, PSS_REG_RESULT_STATUS) &
		     PSS_RESULT_STATUS_AVAILABLE))
			ret = -EBUSY;
		else
			st->fault_flags = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);
	if (!ret && this_attr->address == PSS_ATTR_SCHEDULE_ENABLE && value)
		schedule_work(&st->fill_work);
	return ret ? ret : len;
}

#define PSS_ATTR_RO(_name, _address) \
	IIO_DEVICE_ATTR(_name, 0444, pss_attr_show, NULL, _address)
#define PSS_ATTR_RW(_name, _address) \
	IIO_DEVICE_ATTR(_name, 0644, pss_attr_show, pss_attr_store, _address)
#define PSS_ATTR_WO(_name, _address) \
	IIO_DEVICE_ATTR(_name, 0200, NULL, pss_attr_store, _address)

static PSS_ATTR_RO(fpga_identity, PSS_ATTR_ID);
static PSS_ATTR_RO(abi_version, PSS_ATTR_VERSION);
static PSS_ATTR_RO(rate_msps, PSS_ATTR_RATE);
static PSS_ATTR_RO(geometry, PSS_ATTR_GEOMETRY);
static PSS_ATTR_RO(capabilities, PSS_ATTR_CAPABILITIES);
static PSS_ATTR_RO(status, PSS_ATTR_STATUS);
static PSS_ATTR_RO(current_index, PSS_ATTR_CURRENT_INDEX);
static PSS_ATTR_RO(active_coefficient_generation,
		   PSS_ATTR_ACTIVE_COEFFICIENT_GENERATION);
static PSS_ATTR_RW(coefficient_generation, PSS_ATTR_COEFFICIENT_GENERATION);
static PSS_ATTR_WO(coefficient_words, PSS_ATTR_COEFFICIENT_WORDS);
static PSS_ATTR_WO(coefficient_commit, PSS_ATTR_COEFFICIENT_COMMIT);
static PSS_ATTR_RW(schedule_first_center, PSS_ATTR_SCHEDULE_FIRST_CENTER);
static PSS_ATTR_RW(schedule_period_q32_32, PSS_ATTR_SCHEDULE_PERIOD_Q32_32);
static PSS_ATTR_RW(schedule_request_base, PSS_ATTR_SCHEDULE_REQUEST_BASE);
static PSS_ATTR_RW(schedule_count, PSS_ATTR_SCHEDULE_COUNT);
static PSS_ATTR_RW(schedule_queue_target, PSS_ATTR_SCHEDULE_QUEUE_TARGET);
static PSS_ATTR_RW(schedule_enable, PSS_ATTR_SCHEDULE_ENABLE);
static PSS_ATTR_RO(schedule_submitted, PSS_ATTR_SCHEDULE_SUBMITTED);
static PSS_ATTR_RO(packets_delivered, PSS_ATTR_PACKETS_DELIVERED);
static PSS_ATTR_RO(buffer_push_failures, PSS_ATTR_BUFFER_PUSH_FAILURES);
static PSS_ATTR_RO(packet_validation_failures,
		   PSS_ATTR_PACKET_VALIDATION_FAILURES);
static PSS_ATTR_RO(fault_flags, PSS_ATTR_FAULT_FLAGS);
static PSS_ATTR_WO(fault_clear, PSS_ATTR_FAULT_CLEAR);

static struct attribute *pss_attrs[] = {
	&iio_dev_attr_fpga_identity.dev_attr.attr,
	&iio_dev_attr_abi_version.dev_attr.attr,
	&iio_dev_attr_rate_msps.dev_attr.attr,
	&iio_dev_attr_geometry.dev_attr.attr,
	&iio_dev_attr_capabilities.dev_attr.attr,
	&iio_dev_attr_status.dev_attr.attr,
	&iio_dev_attr_current_index.dev_attr.attr,
	&iio_dev_attr_active_coefficient_generation.dev_attr.attr,
	&iio_dev_attr_coefficient_generation.dev_attr.attr,
	&iio_dev_attr_coefficient_words.dev_attr.attr,
	&iio_dev_attr_coefficient_commit.dev_attr.attr,
	&iio_dev_attr_schedule_first_center.dev_attr.attr,
	&iio_dev_attr_schedule_period_q32_32.dev_attr.attr,
	&iio_dev_attr_schedule_request_base.dev_attr.attr,
	&iio_dev_attr_schedule_count.dev_attr.attr,
	&iio_dev_attr_schedule_queue_target.dev_attr.attr,
	&iio_dev_attr_schedule_enable.dev_attr.attr,
	&iio_dev_attr_schedule_submitted.dev_attr.attr,
	&iio_dev_attr_packets_delivered.dev_attr.attr,
	&iio_dev_attr_buffer_push_failures.dev_attr.attr,
	&iio_dev_attr_packet_validation_failures.dev_attr.attr,
	&iio_dev_attr_fault_flags.dev_attr.attr,
	&iio_dev_attr_fault_clear.dev_attr.attr,
	NULL,
};

static const struct attribute_group pss_attr_group = {
	.attrs = pss_attrs,
};

static const struct iio_info pss_iio_info = {
	.attrs = &pss_attr_group,
};

static int adi_starlink_pss_tracker_probe(struct platform_device *pdev)
{
	struct adi_starlink_pss_tracker *st;
	struct iio_dev *indio_dev;
	u32 version, status, expected_geometry, expected_capabilities;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;
	st->indio_dev = indio_dev;
	mutex_init(&st->lock);
	INIT_WORK(&st->fill_work, pss_fill_work);
	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);
	if (pss_read(st, PSS_REG_ID) != PSS_IDENTIFICATION)
		return -ENODEV;
	version = pss_read(st, PSS_REG_VERSION);
	st->rate_msps = pss_read(st, PSS_REG_RATE_MSPS);
	if ((st->rate_msps == 15 && version != PSS_VERSION_1_2) ||
	    ((st->rate_msps == 30 || st->rate_msps == 60) &&
	     version != PSS_VERSION_1_3))
		return -ENODEV;
	switch (st->rate_msps) {
	case 15:
		expected_geometry = 0x003d8242U;
		expected_capabilities = 0x0000003dU;
		break;
	case 30:
		expected_geometry = 0x0bca0884U;
		expected_capabilities = 0x0000001dU;
		break;
	case 60:
		expected_geometry = 0x0f8c1108U;
		expected_capabilities = 0x0000001dU;
		break;
	default:
		return -ENODEV;
	}
	if (pss_read(st, PSS_REG_GEOMETRY) != expected_geometry ||
	    pss_read(st, PSS_REG_CAPABILITIES) != expected_capabilities)
		return -ENODEV;
	status = pss_read(st, PSS_REG_STATUS);
	if (!(status & PSS_STATUS_RESET_RELEASED))
		return -EPROBE_DEFER;
	st->coefficient_count = 66 * (st->rate_msps / 15);
	if (!st->coefficient_count ||
	    st->coefficient_count > PSS_MAX_COEFFICIENTS)
		return -EINVAL;
	st->coefficient_count_mask = (1U << fls(st->coefficient_count)) - 1U;
	st->queue_room_shift = 10 + fls(st->coefficient_count);
	st->queue_target = PSS_COMMAND_FIFO_USABLE;
	st->request_base = 1;
	st->period_q32_32 = (u64)(20000 * (st->rate_msps / 15)) << 32;

	st->irq = platform_get_irq(pdev, 0);
	if (st->irq < 0)
		return st->irq;
	ret = devm_request_threaded_irq(&pdev->dev, st->irq, NULL,
					pss_irq_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(&pdev->dev), indio_dev);
	if (ret)
		return ret;

	indio_dev->name = "starlink-pss-track";
	indio_dev->info = &pss_iio_info;
	indio_dev->channels = pss_channels;
	indio_dev->num_channels = ARRAY_SIZE(pss_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  INDIO_BUFFER_SOFTWARE,
					  &pss_buffer_ops);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, indio_dev);
	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int adi_starlink_pss_tracker_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct adi_starlink_pss_tracker *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	st->scheduling = false;
	st->streaming = false;
	mutex_unlock(&st->lock);
	cancel_work_sync(&st->fill_work);
	if (st->irq_live) {
		disable_irq(st->irq);
		st->irq_live = false;
	}
	return 0;
}

static const struct of_device_id adi_starlink_pss_tracker_of_match[] = {
	{ .compatible = "adi,starlink-pss-tracker-1.00.a" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_starlink_pss_tracker_of_match);

static struct platform_driver adi_starlink_pss_tracker_driver = {
	.driver = {
		.name = "adi-starlink-pss-tracker",
		.of_match_table = adi_starlink_pss_tracker_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = adi_starlink_pss_tracker_probe,
	.remove = adi_starlink_pss_tracker_remove,
};
module_platform_driver(adi_starlink_pss_tracker_driver);

MODULE_AUTHOR("SPF contributors");
MODULE_DESCRIPTION("Starlink PSS sparse fine-timing IIO driver");
MODULE_LICENSE("GPL");
