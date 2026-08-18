// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADI tandem AGC controller
 *
 * The IIO device is read-only discovery/status.  All mutation is confined to
 * one exclusive local misc-device lease so closing the descriptor can always
 * disarm the FPGA and restore the AD9361 transaction.
 */
#include <linux/fs.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/adi_tandem_agc.h>

#include "ad9361.h"

#define TANDEM_REG_ID			0x00
#define TANDEM_REG_ABI			0x04
#define TANDEM_REG_CAPS			0x08
#define TANDEM_REG_CONTROL		0x0c
#define TANDEM_REG_STATUS		0x10
#define TANDEM_REG_EPOCH		0x14
#define TANDEM_REG_GAIN_LIMITS		0x18
#define TANDEM_REG_GAIN_CURRENT		0x1c
#define TANDEM_REG_POWER_PERIOD		0x20
#define TANDEM_REG_DWELL_COOLDOWN	0x24
#define TANDEM_REG_PULSE		0x28
#define TANDEM_REG_BLANKING		0x2c
#define TANDEM_REG_THRESHOLDS		0x30
#define TANDEM_REG_FAULT		0x34
#define TANDEM_REG_FIFO_LEVEL		0x38
#define TANDEM_REG_OVERFLOW_COUNT	0x3c
#define TANDEM_REG_TRANSITION_COUNT	0x40
#define TANDEM_REG_EVENT_WORD0		0x44
#define TANDEM_REG_EVENT_WORD1		0x48
#define TANDEM_REG_EVENT_WORD2		0x4c
#define TANDEM_REG_EVENT_WORD3_POP	0x50

#define TANDEM_FPGA_ID			0x54414732U /* "TAG2" */
#define TANDEM_FPGA_ABI			1U
#define TANDEM_WATCHDOG_TIMEOUT		(5 * HZ)

#define TANDEM_CONTROL_OWN		BIT(0)
#define TANDEM_CONTROL_AUTO		BIT(1)
#define TANDEM_CONTROL_CLEAR		BIT(8)

#define TANDEM_STATUS_STATE_MASK	GENMASK(2, 0)
#define TANDEM_STATUS_FAULT		BIT(8)

#define TANDEM_CAP_FIFO_DEPTH_MASK	GENMASK(15, 0)

#define TANDEM_REQUIRED_FEATURES \
	(ADI_TANDEM_AGC_FEATURE_EVENTS | \
	 ADI_TANDEM_AGC_FEATURE_FAIL_CLOSED | \
	 ADI_TANDEM_AGC_FEATURE_PAIRED_GAIN)

enum tandem_attr {
	TANDEM_ATTR_ABI_VERSION,
	TANDEM_ATTR_FPGA_IDENTITY,
	TANDEM_ATTR_FPGA_ABI,
	TANDEM_ATTR_FEATURES,
	TANDEM_ATTR_STATE,
	TANDEM_ATTR_EPOCH,
	TANDEM_ATTR_FAULTS,
	TANDEM_ATTR_FIFO_DEPTH,
	TANDEM_ATTR_FIFO_LEVEL,
	TANDEM_ATTR_RX1_GAIN_INDEX,
	TANDEM_ATTR_RX2_GAIN_INDEX,
	TANDEM_ATTR_TRANSITIONS,
	TANDEM_ATTR_OVERFLOWS,
};

struct adi_tandem_agc {
	struct device *dev;
	void __iomem *regs;
	struct ad9361_rf_phy *phy;
	struct spi_device *spi;
	struct miscdevice miscdev;
	/* Serializes lease lifetime, MMIO access, and teardown. */
	struct mutex lock;
	bool session_open;
	bool acquired;
	bool permanent_fault;
	u32 software_fault;
	u32 epoch;
	u32 fifo_depth;
	struct ad9361_tandem_result gain;
	struct delayed_work watchdog_work;
};

static u32 tandem_read(struct adi_tandem_agc *st, unsigned int reg)
{
	return ioread32(st->regs + reg);
}

static void tandem_write(struct adi_tandem_agc *st, unsigned int reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static int tandem_wait_state(struct adi_tandem_agc *st, u32 wanted)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(st->regs + TANDEM_REG_STATUS, status,
				 (status & TANDEM_STATUS_FAULT) ||
				 (status & TANDEM_STATUS_STATE_MASK) == wanted,
				 1, 10000);
	if (ret)
		return ret;
	return status & TANDEM_STATUS_FAULT ? -EIO : 0;
}

static void tandem_fill_status(struct adi_tandem_agc *st,
			       struct adi_tandem_agc_status *status)
{
	u32 gain_current = tandem_read(st, TANDEM_REG_GAIN_CURRENT);

	memset(status, 0, sizeof(*status));
	status->version = ADI_TANDEM_AGC_ABI_VERSION;
	status->size = sizeof(*status);
	status->state = (st->permanent_fault || st->software_fault) ?
		ADI_TANDEM_AGC_STATE_FAULTED :
		st->acquired ?
		(tandem_read(st, TANDEM_REG_STATUS) & TANDEM_STATUS_STATE_MASK) :
		ADI_TANDEM_AGC_STATE_IDLE;
	status->ownership_epoch = st->acquired ? st->epoch : 0;
	status->fault_flags = tandem_read(st, TANDEM_REG_FAULT) |
		st->software_fault |
		(st->permanent_fault ? ADI_TANDEM_AGC_FAULT_RESTORE : 0);
	status->fifo_level = tandem_read(st, TANDEM_REG_FIFO_LEVEL);
	status->overflow_count = tandem_read(st, TANDEM_REG_OVERFLOW_COUNT);
	status->transition_count = tandem_read(st, TANDEM_REG_TRANSITION_COUNT);
	status->minimum_gain_db = st->gain.minimum_gain_db;
	status->maximum_gain_db = st->gain.maximum_gain_db;
	status->initial_gain_db = st->gain.initial_gain_db;
	status->minimum_gain_index = st->gain.minimum_gain_index;
	status->maximum_gain_index = st->gain.maximum_gain_index;
	status->rx1_gain_index = gain_current;
	status->rx2_gain_index = gain_current >> 8;
	status->gain_table_id = st->gain.gain_table_id;
	status->threshold_provenance = tandem_read(st, TANDEM_REG_THRESHOLDS);
}

static void tandem_verify_radio_locked(struct adi_tandem_agc *st)
{
	u32 control;
	u32 gain_current;
	bool resume_auto;
	int ret;

	if (!st->acquired || st->software_fault)
		return;

	/*
	 * GAIN_CURRENT and the two AD9361 SPI reads are not one atomic bus
	 * transaction.  Quiesce AUTO while ownership remains asserted so a paired
	 * gain pulse cannot land inside the verification window and manufacture a
	 * false index mismatch.  HOLD waits for any pulse already in flight.
	 */
	control = tandem_read(st, TANDEM_REG_CONTROL);
	resume_auto = control & TANDEM_CONTROL_AUTO;
	if (resume_auto) {
		tandem_write(st, TANDEM_REG_CONTROL, TANDEM_CONTROL_OWN);
		ret = tandem_wait_state(st, ADI_TANDEM_AGC_STATE_ARMED_HOLD);
		if (ret)
			goto err_suppress;
	}
	gain_current = tandem_read(st, TANDEM_REG_GAIN_CURRENT);
	ret = ad9361_tandem_verify(st->phy, st, gain_current,
				   gain_current >> 8);
	if (ret)
		goto err_suppress;
	if (resume_auto) {
		tandem_write(st, TANDEM_REG_CONTROL,
			     TANDEM_CONTROL_OWN | TANDEM_CONTROL_AUTO);
		ret = tandem_wait_state(st, ADI_TANDEM_AGC_STATE_ARMED_AUTO);
		if (ret)
			goto err_suppress;
	}
	return;

err_suppress:
	/* Retain actively-low ownership, but suppress every further pulse. */
	tandem_write(st, TANDEM_REG_CONTROL, TANDEM_CONTROL_OWN);
	if (ret == -EUCLEAN)
		st->software_fault = ADI_TANDEM_AGC_FAULT_INDEX_MISMATCH;
	else if (ret == -EHOSTDOWN)
		st->software_fault = ADI_TANDEM_AGC_FAULT_RADIO_STATE;
	else
		st->software_fault = ADI_TANDEM_AGC_FAULT_RADIO_IO;
}

static void tandem_heartbeat_locked(struct adi_tandem_agc *st)
{
	if (st->acquired)
		mod_delayed_work(system_wq, &st->watchdog_work,
				 TANDEM_WATCHDOG_TIMEOUT);
}

static int tandem_validate_request(struct adi_tandem_agc *st,
				   const struct adi_tandem_agc_request_v1 *req)
{
	unsigned int i;

	if (req->magic != ADI_TANDEM_AGC_REQUEST_MAGIC ||
	    req->version != ADI_TANDEM_AGC_ABI_VERSION ||
	    req->size != sizeof(*req))
		return -EPROTO;
	if (req->required_features != TANDEM_REQUIRED_FEATURES)
		return -EOPNOTSUPP;
	if (req->mode != ADI_TANDEM_AGC_MODE_HOLD &&
	    req->mode != ADI_TANDEM_AGC_MODE_AUTO)
		return -EINVAL;
	if (!req->observation_capacity || !req->event_capacity ||
	    req->event_capacity > st->fifo_depth)
		return -ENOSPC;
	if (!req->power_measurement_samples ||
	    req->power_measurement_samples > GENMASK(19, 0) ||
	    !req->low_power_dwell_periods ||
	    req->low_power_dwell_periods > U8_MAX ||
	    req->cooldown_periods > U8_MAX ||
	    req->pulse_high_cycles < 4 || req->pulse_high_cycles > U8_MAX ||
	    req->pulse_low_cycles < 4 || req->pulse_low_cycles > U8_MAX ||
	    req->detector_blanking_cycles > U16_MAX)
		return -ERANGE;
	if (req->overflow_policy != ADI_TANDEM_AGC_POLICY_FAIL_SESSION ||
	    req->sync_fault_policy != ADI_TANDEM_AGC_POLICY_FAIL_SESSION)
		return -EOPNOTSUPP;
	for (i = 0; i < ARRAY_SIZE(req->reserved); i++)
		if (req->reserved[i])
			return -EINVAL;

	return 0;
}

static int tandem_release_locked(struct adi_tandem_agc *st)
{
	int ret;

	if (!st->acquired && !st->permanent_fault)
		return 0;
	cancel_delayed_work(&st->watchdog_work);

	/* Stop decisions but retain actively-low FPGA pin ownership. */
	tandem_write(st, TANDEM_REG_CONTROL, TANDEM_CONTROL_OWN);
	usleep_range(50, 100);
	ret = ad9361_tandem_release(st->phy, st);
	/* Pin control is disarmed; only now may the mux return to PS/high-Z. */
	tandem_write(st, TANDEM_REG_CONTROL, 0);
	if (ret && ret != -EPERM)
		st->permanent_fault = true;
	st->acquired = false;

	return ret;
}

static void tandem_watchdog_work(struct work_struct *work)
{
	struct adi_tandem_agc *st = container_of(to_delayed_work(work),
						 struct adi_tandem_agc,
						 watchdog_work);

	mutex_lock(&st->lock);
	if (st->acquired) {
		/*
		 * A live provider calls GET_STATUS for every completed frame.  If it
		 * stops making progress, suppress pulses and restore the AD9361 even
		 * though its process still owns the descriptor.
		 */
		tandem_release_locked(st);
		st->software_fault = ADI_TANDEM_AGC_FAULT_WATCHDOG;
	}
	mutex_unlock(&st->lock);
}

static int tandem_acquire_locked(struct adi_tandem_agc *st,
				 struct adi_tandem_agc_acquire *acquire)
{
	const struct adi_tandem_agc_request_v1 *req = &acquire->request;
	struct ad9361_tandem_config radio_config = {
		.minimum_gain_db = req->minimum_gain_db,
		.maximum_gain_db = req->maximum_gain_db,
		.initial_gain_db = req->initial_gain_db,
		.low_power_threshold = req->low_power_threshold,
		.large_lmt_overload_threshold =
			req->large_lmt_overload_threshold,
		.large_adc_overload_threshold =
			req->large_adc_overload_threshold,
		.small_adc_overload_threshold =
			req->small_adc_overload_threshold,
	};
	u32 control;
	int ret;

	if (st->acquired)
		return -EBUSY;
	if (st->permanent_fault)
		return -EIO;
	st->software_fault = 0;
	ret = tandem_validate_request(st, req);
	if (ret)
		return ret;

	/* HOLD and no ownership is the invariant for every validation failure. */
	tandem_write(st, TANDEM_REG_CONTROL, 0);
	ret = ad9361_tandem_prepare(st->phy, st, &radio_config, &st->gain);
	if (ret) {
		int release_ret = ad9361_tandem_release(st->phy, st);

		if (release_ret && release_ret != -EPERM)
			st->permanent_fault = true;
		return ret;
	}

	st->epoch++;
	if (!st->epoch)
		st->epoch++;

	tandem_write(st, TANDEM_REG_EPOCH, st->epoch);
	tandem_write(st, TANDEM_REG_GAIN_LIMITS,
		     st->gain.minimum_gain_index |
		     st->gain.maximum_gain_index << 8 |
		     st->gain.initial_gain_index << 16);
	tandem_write(st, TANDEM_REG_POWER_PERIOD,
		     req->power_measurement_samples);
	tandem_write(st, TANDEM_REG_DWELL_COOLDOWN,
		     req->low_power_dwell_periods | req->cooldown_periods << 8);
	tandem_write(st, TANDEM_REG_PULSE,
		     req->pulse_high_cycles | req->pulse_low_cycles << 8);
	tandem_write(st, TANDEM_REG_BLANKING,
		     req->detector_blanking_cycles);
	tandem_write(st, TANDEM_REG_THRESHOLDS,
		     req->low_power_threshold |
		     req->large_lmt_overload_threshold << 8 |
		     req->large_adc_overload_threshold << 16 |
		     req->small_adc_overload_threshold << 24);
	tandem_write(st, TANDEM_REG_CONTROL, TANDEM_CONTROL_CLEAR);
	ret = readl_poll_timeout(st->regs + TANDEM_REG_FAULT, control,
				 !(control), 1, 10000);
	if (ret)
		goto err_restore;
	tandem_write(st, TANDEM_REG_CONTROL, 0);

	if (tandem_read(st, TANDEM_REG_EPOCH) != st->epoch ||
	    (tandem_read(st, TANDEM_REG_GAIN_LIMITS) & GENMASK(23, 0)) !=
	    (st->gain.minimum_gain_index |
	     st->gain.maximum_gain_index << 8 |
	     st->gain.initial_gain_index << 16)) {
		ret = -EIO;
		goto err_restore;
	}

	control = TANDEM_CONTROL_OWN;
	tandem_write(st, TANDEM_REG_CONTROL, control);
	/*
	 * CONTROL is acknowledged in the AXI clock domain before the request has
	 * necessarily crossed into the receive clock domain.  Do not arm CTRL_IN
	 * until the FPGA confirms that it owns the pins and is driving HOLD-low.
	 */
	ret = tandem_wait_state(st, ADI_TANDEM_AGC_STATE_ARMED_HOLD);
	if (ret)
		goto err_disarm;
	ret = ad9361_tandem_arm(st->phy, st);
	if (ret)
		goto err_disarm;
	if (req->mode == ADI_TANDEM_AGC_MODE_AUTO) {
		control |= TANDEM_CONTROL_AUTO;
		tandem_write(st, TANDEM_REG_CONTROL, control);
		ret = tandem_wait_state(st, ADI_TANDEM_AGC_STATE_ARMED_AUTO);
		if (ret)
			goto err_release;
	}

	if ((tandem_read(st, TANDEM_REG_CONTROL) & control) != control ||
	    (tandem_read(st, TANDEM_REG_STATUS) & TANDEM_STATUS_FAULT)) {
		ret = -EIO;
		goto err_release;
	}

	st->acquired = true;
	tandem_heartbeat_locked(st);
	tandem_fill_status(st, &acquire->status);
	return 0;

err_release:
err_disarm:
	tandem_write(st, TANDEM_REG_CONTROL, TANDEM_CONTROL_OWN);
	usleep_range(50, 100);
err_restore:
	if (ad9361_tandem_release(st->phy, st))
		st->permanent_fault = true;
	tandem_write(st, TANDEM_REG_CONTROL, 0);
	return ret;
}

static int tandem_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct adi_tandem_agc *st = container_of(miscdev,
						 struct adi_tandem_agc, miscdev);
	int ret = 0;

	mutex_lock(&st->lock);
	if (st->session_open)
		ret = -EBUSY;
	else
		st->session_open = true;
	mutex_unlock(&st->lock);
	if (!ret)
		file->private_data = st;

	return ret;
}

static int tandem_release(struct inode *inode, struct file *file)
{
	struct adi_tandem_agc *st = file->private_data;

	mutex_lock(&st->lock);
	tandem_release_locked(st);
	st->session_open = false;
	mutex_unlock(&st->lock);

	return 0;
}

static long tandem_ioctl(struct file *file, unsigned int cmd,
			 unsigned long arg)
{
	struct adi_tandem_agc *st = file->private_data;
	void __user *argp = (void __user *)arg;
	struct adi_tandem_agc_acquire acquire;
	struct adi_tandem_agc_status status;
	struct adi_tandem_agc_caps caps;
	int ret = 0;

	mutex_lock(&st->lock);
	switch (cmd) {
	case ADI_TANDEM_AGC_IOC_GET_CAPS:
		memset(&caps, 0, sizeof(caps));
		caps.version = ADI_TANDEM_AGC_ABI_VERSION;
		caps.size = sizeof(caps);
		caps.features = TANDEM_REQUIRED_FEATURES;
		caps.fpga_identity = tandem_read(st, TANDEM_REG_ID);
		caps.fpga_abi = tandem_read(st, TANDEM_REG_ABI);
		caps.event_size = sizeof(struct adi_tandem_agc_event);
		caps.fifo_depth = st->fifo_depth;
		if (copy_to_user(argp, &caps, sizeof(caps)))
			ret = -EFAULT;
		break;
	case ADI_TANDEM_AGC_IOC_GET_STATUS:
		tandem_heartbeat_locked(st);
		tandem_verify_radio_locked(st);
		tandem_fill_status(st, &status);
		if (copy_to_user(argp, &status, sizeof(status)))
			ret = -EFAULT;
		break;
	case ADI_TANDEM_AGC_IOC_ACQUIRE:
		if (copy_from_user(&acquire, argp, sizeof(acquire))) {
			ret = -EFAULT;
			break;
		}
		ret = tandem_acquire_locked(st, &acquire);
		if (!ret && copy_to_user(argp, &acquire, sizeof(acquire))) {
			tandem_release_locked(st);
			ret = -EFAULT;
		}
		break;
	case ADI_TANDEM_AGC_IOC_RELEASE:
		ret = tandem_release_locked(st);
		break;
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static ssize_t tandem_events_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct adi_tandem_agc *st = file->private_data;
	struct adi_tandem_agc_event event;
	ssize_t copied = 0;
	u32 words[4];

	if (count < sizeof(event) || count % sizeof(event))
		return -EINVAL;

	mutex_lock(&st->lock);
	if (!st->acquired) {
		copied = -ENODATA;
		goto out;
	}
	tandem_heartbeat_locked(st);
	while (count - copied >= sizeof(event) &&
	       tandem_read(st, TANDEM_REG_FIFO_LEVEL)) {
		words[0] = tandem_read(st, TANDEM_REG_EVENT_WORD0);
		words[1] = tandem_read(st, TANDEM_REG_EVENT_WORD1);
		words[2] = tandem_read(st, TANDEM_REG_EVENT_WORD2);
		words[3] = tandem_read(st, TANDEM_REG_EVENT_WORD3_POP);
		memcpy(&event, words, sizeof(event));
		if (copy_to_user(buf + copied, &event, sizeof(event))) {
			copied = copied ? copied : -EFAULT;
			goto out;
		}
		copied += sizeof(event);
	}
	if (!copied)
		copied = -EAGAIN;
out:
	mutex_unlock(&st->lock);
	return copied;
}

static __poll_t tandem_poll(struct file *file, poll_table *wait)
{
	struct adi_tandem_agc *st = file->private_data;
	__poll_t mask = 0;

	mutex_lock(&st->lock);
	if (st->acquired && tandem_read(st, TANDEM_REG_FIFO_LEVEL))
		mask = EPOLLIN | EPOLLRDNORM;
	if (st->permanent_fault ||
	    (st->acquired && tandem_read(st, TANDEM_REG_FAULT)))
		mask |= EPOLLERR;
	mutex_unlock(&st->lock);
	return mask;
}

static const struct file_operations tandem_fops = {
	.owner = THIS_MODULE,
	.open = tandem_open,
	.release = tandem_release,
	.unlocked_ioctl = tandem_ioctl,
	.read = tandem_events_read,
	.poll = tandem_poll,
	.llseek = no_llseek,
};

static ssize_t tandem_attr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adi_tandem_agc *st = iio_priv(indio_dev);
	u32 value;

	mutex_lock(&st->lock);
	switch (this_attr->address) {
	case TANDEM_ATTR_ABI_VERSION:
		value = ADI_TANDEM_AGC_ABI_VERSION;
		break;
	case TANDEM_ATTR_FPGA_IDENTITY:
		value = tandem_read(st, TANDEM_REG_ID);
		break;
	case TANDEM_ATTR_FPGA_ABI:
		value = tandem_read(st, TANDEM_REG_ABI);
		break;
	case TANDEM_ATTR_FEATURES:
		value = TANDEM_REQUIRED_FEATURES;
		break;
	case TANDEM_ATTR_STATE:
		value = (st->permanent_fault || st->software_fault) ?
			ADI_TANDEM_AGC_STATE_FAULTED :
			st->acquired ?
			tandem_read(st, TANDEM_REG_STATUS) & TANDEM_STATUS_STATE_MASK :
			ADI_TANDEM_AGC_STATE_IDLE;
		break;
	case TANDEM_ATTR_EPOCH:
		value = st->acquired ? st->epoch : 0;
		break;
	case TANDEM_ATTR_FAULTS:
		value = tandem_read(st, TANDEM_REG_FAULT) |
			st->software_fault |
			(st->permanent_fault ? ADI_TANDEM_AGC_FAULT_RESTORE : 0);
		break;
	case TANDEM_ATTR_FIFO_DEPTH:
		value = st->fifo_depth;
		break;
	case TANDEM_ATTR_FIFO_LEVEL:
		value = tandem_read(st, TANDEM_REG_FIFO_LEVEL);
		break;
	case TANDEM_ATTR_RX1_GAIN_INDEX:
		value = tandem_read(st, TANDEM_REG_GAIN_CURRENT) & U8_MAX;
		break;
	case TANDEM_ATTR_RX2_GAIN_INDEX:
		value = tandem_read(st, TANDEM_REG_GAIN_CURRENT) >> 8 & U8_MAX;
		break;
	case TANDEM_ATTR_TRANSITIONS:
		value = tandem_read(st, TANDEM_REG_TRANSITION_COUNT);
		break;
	case TANDEM_ATTR_OVERFLOWS:
		value = tandem_read(st, TANDEM_REG_OVERFLOW_COUNT);
		break;
	default:
		mutex_unlock(&st->lock);
		return -EINVAL;
	}
	mutex_unlock(&st->lock);

	return sysfs_emit(buf, "%u\n", value);
}

#define TANDEM_ATTR_RO(_name, _address) \
	IIO_DEVICE_ATTR(_name, 0444, tandem_attr_show, NULL, _address)

static TANDEM_ATTR_RO(abi_version, TANDEM_ATTR_ABI_VERSION);
static TANDEM_ATTR_RO(fpga_identity, TANDEM_ATTR_FPGA_IDENTITY);
static TANDEM_ATTR_RO(fpga_abi, TANDEM_ATTR_FPGA_ABI);
static TANDEM_ATTR_RO(features, TANDEM_ATTR_FEATURES);
static TANDEM_ATTR_RO(state, TANDEM_ATTR_STATE);
static TANDEM_ATTR_RO(ownership_epoch, TANDEM_ATTR_EPOCH);
static TANDEM_ATTR_RO(fault_flags, TANDEM_ATTR_FAULTS);
static TANDEM_ATTR_RO(fifo_depth, TANDEM_ATTR_FIFO_DEPTH);
static TANDEM_ATTR_RO(fifo_level, TANDEM_ATTR_FIFO_LEVEL);
static TANDEM_ATTR_RO(rx1_gain_index, TANDEM_ATTR_RX1_GAIN_INDEX);
static TANDEM_ATTR_RO(rx2_gain_index, TANDEM_ATTR_RX2_GAIN_INDEX);
static TANDEM_ATTR_RO(transition_count, TANDEM_ATTR_TRANSITIONS);
static TANDEM_ATTR_RO(overflow_count, TANDEM_ATTR_OVERFLOWS);

static struct attribute *tandem_attrs[] = {
	&iio_dev_attr_abi_version.dev_attr.attr,
	&iio_dev_attr_fpga_identity.dev_attr.attr,
	&iio_dev_attr_fpga_abi.dev_attr.attr,
	&iio_dev_attr_features.dev_attr.attr,
	&iio_dev_attr_state.dev_attr.attr,
	&iio_dev_attr_ownership_epoch.dev_attr.attr,
	&iio_dev_attr_fault_flags.dev_attr.attr,
	&iio_dev_attr_fifo_depth.dev_attr.attr,
	&iio_dev_attr_fifo_level.dev_attr.attr,
	&iio_dev_attr_rx1_gain_index.dev_attr.attr,
	&iio_dev_attr_rx2_gain_index.dev_attr.attr,
	&iio_dev_attr_transition_count.dev_attr.attr,
	&iio_dev_attr_overflow_count.dev_attr.attr,
	NULL,
};

static const struct attribute_group tandem_attr_group = {
	.attrs = tandem_attrs,
};

static const struct iio_info tandem_iio_info = {
	.attrs = &tandem_attr_group,
};

static void tandem_misc_deregister(void *data)
{
	misc_deregister(data);
}

static void tandem_put_device(void *data)
{
	put_device(data);
}

static int adi_tandem_agc_probe(struct platform_device *pdev)
{
	struct device_node *spi_np;
	struct iio_dev *indio_dev;
	struct adi_tandem_agc *st;
	u32 caps;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;
	mutex_init(&st->lock);
	INIT_DELAYED_WORK(&st->watchdog_work, tandem_watchdog_work);

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);
	if (tandem_read(st, TANDEM_REG_ID) != TANDEM_FPGA_ID ||
	    tandem_read(st, TANDEM_REG_ABI) != TANDEM_FPGA_ABI)
		return -ENODEV;

	spi_np = of_parse_phandle(pdev->dev.of_node, "spibus-connected", 0);
	if (!spi_np)
		return -EINVAL;
	st->spi = of_find_spi_device_by_node(spi_np);
	of_node_put(spi_np);
	if (!st->spi)
		return -EPROBE_DEFER;
	st->phy = ad9361_spi_to_phy(st->spi);
	if (!st->phy) {
		put_device(&st->spi->dev);
		return -EPROBE_DEFER;
	}

	ret = devm_add_action_or_reset(&pdev->dev, tandem_put_device,
				       &st->spi->dev);
	if (ret)
		return ret;

	caps = tandem_read(st, TANDEM_REG_CAPS);
	st->fifo_depth = caps & TANDEM_CAP_FIFO_DEPTH_MASK;
	if (!st->fifo_depth)
		return -EINVAL;
	tandem_write(st, TANDEM_REG_CONTROL, 0);

	st->miscdev.minor = MISC_DYNAMIC_MINOR;
	st->miscdev.name = "tandem-agc-events";
	st->miscdev.fops = &tandem_fops;
	st->miscdev.parent = &pdev->dev;
	ret = misc_register(&st->miscdev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, tandem_misc_deregister,
				       &st->miscdev);
	if (ret)
		return ret;

	indio_dev->name = "tandem-agc";
	indio_dev->info = &tandem_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	platform_set_drvdata(pdev, indio_dev);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id adi_tandem_agc_of_match[] = {
	{ .compatible = "adi,tandem-agc-2.00.a" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_tandem_agc_of_match);

static void adi_tandem_agc_shutdown(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct adi_tandem_agc *st = iio_priv(indio_dev);

	cancel_delayed_work_sync(&st->watchdog_work);
	mutex_lock(&st->lock);
	tandem_release_locked(st);
	/* Also enforce fail-closed when shutdown occurs without a lease. */
	tandem_write(st, TANDEM_REG_CONTROL, 0);
	mutex_unlock(&st->lock);
}

static int adi_tandem_agc_remove(struct platform_device *pdev)
{
	adi_tandem_agc_shutdown(pdev);
	return 0;
}

static struct platform_driver adi_tandem_agc_driver = {
	.driver = {
		.name = "adi-tandem-agc",
		.of_match_table = adi_tandem_agc_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = adi_tandem_agc_probe,
	.remove = adi_tandem_agc_remove,
	.shutdown = adi_tandem_agc_shutdown,
};
module_platform_driver(adi_tandem_agc_driver);

MODULE_AUTHOR("SPF contributors");
MODULE_DESCRIPTION("AD9361 tandem AGC ownership and event driver");
MODULE_LICENSE("GPL");
