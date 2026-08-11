// SPDX-License-Identifier: GPL-2.0
/*
 * Tandem AGC controller — register access for the FPGA block that owns the
 * AD9361 CTRL_IN pins and steps RX1/RX2 gain together.
 *
 * Copyright 2026 Analog Devices Inc.
 *
 * The block has no channels and produces no samples; it is a register file.
 * It is registered as an IIO device purely so that its registers are reachable
 * through libiio's iio_device_reg_read/write, which is what makes the block
 * controllable from a HOST rather than only from the radio's own ARM.
 *
 * That is the whole reason this driver exists. The alternative — mapping
 * /dev/mem from a userspace tool — works only on the device itself, so every
 * operator action would have to be shelled in over SSH. Exposing the register
 * file through iiod instead means a host opens "ip:<radio>" and talks to the
 * block the same way it already talks to ad9361-phy.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>

#define TANDEM_AGC_REG_ID	0x00
#define TANDEM_AGC_ID_MAGIC	0x54414731	/* "TAG1" */
#define TANDEM_AGC_REG_MAX	0xFF

struct tandem_agc_state {
	void __iomem *regs;
};

/*
 * Registers are 32-bit and word-aligned. A misaligned or out-of-range access
 * is rejected rather than silently masked: this interface is how an operator
 * arms pin control over the gain, and a write that lands somewhere other than
 * where the caller asked is worse than a failed one.
 */
static int tandem_agc_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct tandem_agc_state *st = iio_priv(indio_dev);

	if (reg > TANDEM_AGC_REG_MAX || (reg & 0x3))
		return -EINVAL;

	if (readval)
		*readval = ioread32(st->regs + reg);
	else
		iowrite32(writeval, st->regs + reg);

	return 0;
}

static const struct iio_info tandem_agc_info = {
	.debugfs_reg_access = &tandem_agc_reg_access,
};

static int tandem_agc_probe(struct platform_device *pdev)
{
	struct tandem_agc_state *st;
	struct iio_dev *indio_dev;
	u32 id;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	/*
	 * Verify before registering. A bitstream without the block, or one where
	 * it moved, leaves whatever peripheral happens to live at this address
	 * responding to reads -- and a controller that decodes another block's
	 * registers as gain state is worse than one that is absent. Failing the
	 * probe makes the device simply not appear, which userspace already
	 * treats as "tandem unavailable".
	 */
	id = ioread32(st->regs + TANDEM_AGC_REG_ID);
	if (id != TANDEM_AGC_ID_MAGIC) {
		dev_err(&pdev->dev,
			"no tandem AGC block here: ID reads 0x%08x, expected 0x%08x\n",
			id, TANDEM_AGC_ID_MAGIC);
		return -ENODEV;
	}

	indio_dev->name = "tandem-agc";
	indio_dev->info = &tandem_agc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	/* no channels: this block is a register file, not a data source */
	indio_dev->channels = NULL;
	indio_dev->num_channels = 0;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id tandem_agc_of_match[] = {
	{ .compatible = "adi,tandem-agc-1.00.a" },
	{ }
};
MODULE_DEVICE_TABLE(of, tandem_agc_of_match);

static struct platform_driver tandem_agc_driver = {
	.driver = {
		.name = "tandem-agc",
		.of_match_table = tandem_agc_of_match,
	},
	.probe = tandem_agc_probe,
};
module_platform_driver(tandem_agc_driver);

MODULE_AUTHOR("Analog Devices Inc.");
MODULE_DESCRIPTION("Tandem AGC controller register interface");
MODULE_LICENSE("GPL v2");
