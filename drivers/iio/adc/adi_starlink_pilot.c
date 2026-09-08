// SPDX-License-Identifier: GPL-2.0
/* Experimental PIL1 post-decimation capture; not the legacy raw RX ABI. */
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define PIL_ID 0x00
#define PIL_VERSION 0x04
#define PIL_COMMAND 0x08
#define PIL_STATUS 0x0c
#define PIL_FAULT 0x10
#define PIL_SOURCE_RATE 0x14
#define PIL_OUTPUT_RATE 0x18
#define PIL_RATIO 0x1c
#define PIL_VISIT 0x20
#define PIL_EDGE 0x24
#define PIL_SNAPSHOT 0x30
#define PIL_GENERATION 0x98
#define PIL_LIMIT 0x9c
#define PIL_ACTIVE BIT(0)
#define PIL_QUEUED BIT(1)
#define PIL_ARM 1
#define PIL_STOP 2
#define PIL_CLEAR 4
#define PIL_LATCH 8
#define PIL_WORDS 26

struct pilot_state {
	void __iomem *regs;
	struct clk *source_clk;
	struct mutex lock;
	u32 visit;
	u32 limit;
	u32 source_rate;
	int dma_error;
	u32 dma_submitted;
	bool recovery_failed;
};

static u32 pilot_read(struct pilot_state *st, unsigned int offset)
{
	return readl(st->regs + offset);
}

static void pilot_write(struct pilot_state *st, unsigned int offset, u32 value)
{
	writel(value, st->regs + offset);
}

/* Caller owns lock. AXIS acceptance is not DMA or host completion. */
static int pilot_snapshot(struct pilot_state *st, u32 *words, u32 *generation)
{
	u32 prior = pilot_read(st, PIL_GENERATION);
	u32 value;
	int ret, n;

	pilot_write(st, PIL_COMMAND, PIL_LATCH);
	ret = readl_poll_timeout(st->regs + PIL_GENERATION, value,
				value != prior && value != 0, 1, 10000);
	if (ret)
		return ret;
	*generation = value;
	for (n = 0; n < PIL_WORDS; n++)
		words[n] = pilot_read(st, PIL_SNAPSHOT + 4 * n);
	return pilot_read(st, PIL_GENERATION) == value ? 0 : -EIO;
}

static ssize_t pilot_snapshot_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pilot_state *st = iio_priv(dev_to_iio_dev(dev));
	u32 words[PIL_WORDS], generation;
	int ret, n, length = 0;

	mutex_lock(&st->lock);
	ret = pilot_snapshot(st, words, &generation);
	if (!ret) {
		/* Frozen word order follows CAPTURE_ABI.md; one atomic sysfs read. */
		length = scnprintf(buf, PAGE_SIZE,
			"PIL1 00010000 %u 2500000 %u %u %u %d",
			st->source_rate, generation, st->recovery_failed,
			(u32)clk_get_rate(st->source_clk), READ_ONCE(st->dma_error));
		for (n = 0; n < PIL_WORDS; n++)
			length += scnprintf(buf + length, PAGE_SIZE - length,
					    " %08x", words[n]);
		length += scnprintf(buf + length, PAGE_SIZE - length, "\n");
	}
	mutex_unlock(&st->lock);
	return ret ? ret : length;
}

static ssize_t pilot_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pilot_state *st = iio_priv(dev_to_iio_dev(dev));
	u32 value;

	mutex_lock(&st->lock);
	value = to_iio_dev_attr(attr)->address == PIL_VISIT ? st->visit : st->limit;
	mutex_unlock(&st->lock);
	return sysfs_emit(buf, "%u\n", value);
}

static ssize_t pilot_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio = dev_to_iio_dev(dev);
	struct pilot_state *st = iio_priv(indio);
	u32 value;
	int ret;

	ret = kstrtou32(buf, 0, &value);
	if (ret)
		return ret;
	if (to_iio_dev_attr(attr)->address == PIL_VISIT && !value)
		return -EINVAL;
	ret = iio_device_claim_direct_mode(indio);
	if (ret)
		return ret;
	mutex_lock(&st->lock);
	if (to_iio_dev_attr(attr)->address == PIL_VISIT)
		st->visit = value;
	else
		st->limit = value;
	mutex_unlock(&st->lock);
	iio_device_release_direct_mode(indio);
	return count;
}

static ssize_t pilot_fault_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pilot_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%08x\n", pilot_read(st, PIL_FAULT));
}

static IIO_DEVICE_ATTR(capture_snapshot, 0444, pilot_snapshot_show, NULL, 0);
static IIO_DEVICE_ATTR(capture_fault, 0444, pilot_fault_show, NULL, 0);
static IIO_DEVICE_ATTR(capture_visit_id, 0644, pilot_config_show,
			pilot_config_store, PIL_VISIT);
static IIO_DEVICE_ATTR(capture_sample_limit, 0644, pilot_config_show,
			pilot_config_store, PIL_LIMIT);
static IIO_CONST_ATTR(capture_abi, "PIL1-1.0-upper-only");

static struct attribute *pilot_attrs[] = {
	&iio_dev_attr_capture_snapshot.dev_attr.attr,
	&iio_dev_attr_capture_fault.dev_attr.attr,
	&iio_dev_attr_capture_visit_id.dev_attr.attr,
	&iio_dev_attr_capture_sample_limit.dev_attr.attr,
	&iio_const_attr_capture_abi.dev_attr.attr,
	NULL,
};

static const struct attribute_group pilot_attr_group = { .attrs = pilot_attrs };

static int pilot_read_raw(struct iio_dev *indio, const struct iio_chan_spec *chan,
			 int *val, int *val2, long mask)
{
	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;
	*val = 2500000;
	return IIO_VAL_INT;
}

static const struct iio_info pilot_info = {
	.attrs = &pilot_attr_group,
	.read_raw = pilot_read_raw,
};

#define PILOT_CHANNEL(_index, _modifier) { \
	.type = IIO_VOLTAGE, .modified = 1, .channel2 = (_modifier), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = (_index), .scan_type = { .sign = 's', \
	.realbits = 16, .storagebits = 16, .endianness = IIO_LE } }

static const struct iio_chan_spec pilot_channels[] = {
	PILOT_CHANNEL(0, IIO_MOD_I), PILOT_CHANNEL(1, IIO_MOD_Q),
};
static const unsigned long pilot_scan_masks[] = { BIT(0) | BIT(1), 0 };

static int pilot_submit(struct iio_dma_buffer_queue *queue,
			struct iio_dma_buffer_block *block)
{
	struct pilot_state *st = queue->driver_data;
	int ret;

	/* Direction is explicitly capture, not the generic ADC's TX helper. */
	ret = iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
	if (ret)
		WRITE_ONCE(st->dma_error, ret);
	else if (st->dma_submitted != U32_MAX)
		st->dma_submitted++;
	return ret;
}

static const struct iio_dma_buffer_ops pilot_dma_ops = {
	.submit = pilot_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static int pilot_preenable(struct iio_dev *indio)
{
	struct pilot_state *st = iio_priv(indio);
	u32 value;
	int ret = 0;

	mutex_lock(&st->lock);
	if (st->recovery_failed || !st->visit || indio->scan_bytes != 4 ||
	    *indio->active_scan_mask != (BIT(0) | BIT(1)) ||
	    !indio->buffer->channel_mask ||
	    *indio->buffer->channel_mask != (BIT(0) | BIT(1)) ||
	    clk_get_rate(st->source_clk) != st->source_rate) {
		ret = -EINVAL;
		goto out;
	}
	/* Finite captures must complete whole requested IIO buffers. */
	if (st->limit && (!indio->buffer->length ||
	    st->limit % indio->buffer->length)) {
		ret = -EINVAL;
		goto out;
	}
	value = pilot_read(st, PIL_STATUS);
	if (value & (PIL_ACTIVE | PIL_QUEUED)) {
		ret = -EBUSY;
		goto out;
	}
	pilot_write(st, PIL_COMMAND, PIL_CLEAR);
	ret = readl_poll_timeout(st->regs + PIL_GENERATION, value, !value, 1, 10000);
	if (ret)
		goto out;
	pilot_write(st, PIL_VISIT, st->visit);
	pilot_write(st, PIL_LIMIT, st->limit);
	if (pilot_read(st, PIL_FAULT) || pilot_read(st, PIL_VISIT) != st->visit ||
	    pilot_read(st, PIL_LIMIT) != st->limit)
		ret = -EIO;
	WRITE_ONCE(st->dma_error, 0);
	st->dma_submitted = 0;
out:
	mutex_unlock(&st->lock);
	return ret;
}

/* Called with st->lock held, and before the IIO core aborts DMA. */
static int pilot_stop_and_drain(struct pilot_state *st)
{
	u32 value;
	int ret;

	pilot_write(st, PIL_COMMAND, PIL_STOP);
	ret = readl_poll_timeout(st->regs + PIL_STATUS, value,
				!(value & (PIL_ACTIVE | PIL_QUEUED)), 10, 25000);
	if (ret)
		st->recovery_failed = true;
	return ret;
}

static int pilot_postenable(struct iio_dev *indio)
{
	struct pilot_state *st = iio_priv(indio);
	u32 value;
	int ret;

	mutex_lock(&st->lock);
	/* IIO has submitted its DMA buffers before calling postenable. */
	ret = READ_ONCE(st->dma_error);
	if (!ret && !st->dma_submitted)
		ret = -ENOBUFS;
	if (ret)
		goto out;
	pilot_write(st, PIL_COMMAND, PIL_ARM);
	ret = readl_poll_timeout(st->regs + PIL_STATUS, value, value & BIT(4), 1, 10000);
	if (!ret && pilot_read(st, PIL_FAULT))
		ret = -EIO;
	if (ret)
		pilot_stop_and_drain(st);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int pilot_predisable(struct iio_dev *indio)
{
	struct pilot_state *st = iio_priv(indio);
	int ret;

	mutex_lock(&st->lock);
	/* Drain before the IIO core aborts DMA. Never silently clear queued IQ. */
	ret = pilot_stop_and_drain(st);
	mutex_unlock(&st->lock);
	return ret;
}

static const struct iio_buffer_setup_ops pilot_setup_ops = {
	.preenable = pilot_preenable,
	.postenable = pilot_postenable,
	.predisable = pilot_predisable,
};

static int pilot_probe(struct platform_device *pdev)
{
	struct iio_dev *indio;
	struct pilot_state *st;
	struct iio_buffer *buffer;

	indio = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio)
		return -ENOMEM;
	st = iio_priv(indio);
	mutex_init(&st->lock);
	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);
	if (pilot_read(st, PIL_ID) != 0x50494c31 ||
	    pilot_read(st, PIL_VERSION) != 0x00010000 ||
	    pilot_read(st, PIL_EDGE) != 1 || pilot_read(st, PIL_OUTPUT_RATE) != 2500000)
		return -ENODEV;
	st->source_rate = pilot_read(st, PIL_SOURCE_RATE);
	if ((st->source_rate != 15000000 && st->source_rate != 30000000 &&
	     st->source_rate != 60000000) ||
	    pilot_read(st, PIL_RATIO) != st->source_rate / 2500000)
		return -ENODEV;
	st->source_clk = devm_clk_get(&pdev->dev, "rx_sample");
	if (IS_ERR(st->source_clk))
		return PTR_ERR(st->source_clk);
	indio->name = "starlink-pilot-capture";
	indio->info = &pilot_info;
	indio->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio->channels = pilot_channels;
	indio->num_channels = ARRAY_SIZE(pilot_channels);
	indio->available_scan_masks = pilot_scan_masks;
	indio->setup_ops = &pilot_setup_ops;
	indio->dev.parent = &pdev->dev;
	buffer = devm_iio_dmaengine_buffer_alloc(&pdev->dev, "rx", &pilot_dma_ops, st);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);
	if (iio_device_attach_buffer(indio, buffer))
		return -EINVAL;
	return devm_iio_device_register(&pdev->dev, indio);
}

static const struct of_device_id pilot_of_match[] = {
	{ .compatible = "adi,starlink-pilot-capture-1.00.a" }, {},
};
MODULE_DEVICE_TABLE(of, pilot_of_match);
static struct platform_driver pilot_driver = {
	.driver = { .name = "adi-starlink-pilot", .of_match_table = pilot_of_match },
	.probe = pilot_probe,
};
module_platform_driver(pilot_driver);
MODULE_DESCRIPTION("Experimental single-RX post-decimation PIL1 IIO capture");
MODULE_AUTHOR("PlutoSDR experimental receiver contributors");
MODULE_LICENSE("GPL");
