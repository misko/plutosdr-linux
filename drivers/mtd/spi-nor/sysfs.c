// SPDX-License-Identifier: GPL-2.0

#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/sysfs.h>

#include "core.h"

static ssize_t manufacturer_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);

	return sysfs_emit(buf, "%s\n", nor->manufacturer->name);
}
static DEVICE_ATTR_RO(manufacturer);

static ssize_t partname_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);

	return sysfs_emit(buf, "%s\n", nor->info->name);
}
static DEVICE_ATTR_RO(partname);

static ssize_t jedec_id_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);

	return sysfs_emit(buf, "%*phN\n", nor->info->id_len, nor->info->id);
}
static DEVICE_ATTR_RO(jedec_id);

static struct attribute *spi_nor_sysfs_entries[] = {
	&dev_attr_manufacturer.attr,
	&dev_attr_partname.attr,
	&dev_attr_jedec_id.attr,
	NULL
};

static ssize_t sfdp_read(struct file *filp, struct kobject *kobj,
			 struct bin_attribute *bin_attr, char *buf,
			 loff_t off, size_t count)
{
	struct spi_device *spi = to_spi_device(kobj_to_dev(kobj));
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);
	struct sfdp *sfdp = nor->sfdp;
	size_t sfdp_size = sfdp->num_dwords * sizeof(*sfdp->dwords);

	return memory_read_from_buffer(buf, count, &off, nor->sfdp->dwords,
				       sfdp_size);
}
static BIN_ATTR_RO(sfdp, 0);

static ssize_t unique_id_read(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *bin_attr, char *buf,
			      loff_t off, size_t count)
{
	struct spi_device *spi = to_spi_device(kobj_to_dev(kobj));
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);
	u8 unique_id[32];
	int ret;

	if (!nor->params->read_unique_id ||
	    !nor->params->unique_id_len ||
	    nor->params->unique_id_len > sizeof(unique_id))
		return -EOPNOTSUPP;

	ret = spi_nor_lock_and_prep(nor);
	if (ret)
		return ret;

	ret = nor->params->read_unique_id(nor, unique_id,
					 nor->params->unique_id_len);
	spi_nor_unlock_and_unprep(nor);
	if (ret) {
		dev_err(nor->dev, "unique ID read failed: %d\n", ret);
		return ret;
	}

	dev_info(nor->dev, "unique ID read returned %*phN\n",
		 nor->params->unique_id_len, unique_id);

	return memory_read_from_buffer(buf, count, &off, unique_id,
				       nor->params->unique_id_len);
}
static BIN_ATTR_RO(unique_id, 0);

static struct bin_attribute *spi_nor_sysfs_bin_entries[] = {
	&bin_attr_sfdp,
	&bin_attr_unique_id,
	NULL
};

static umode_t spi_nor_sysfs_is_bin_visible(struct kobject *kobj,
					    struct bin_attribute *attr, int n)
{
	struct spi_device *spi = to_spi_device(kobj_to_dev(kobj));
	struct spi_mem *spimem = spi_get_drvdata(spi);
	struct spi_nor *nor = spi_mem_get_drvdata(spimem);

	if (attr == &bin_attr_sfdp && nor->sfdp)
		return 0444;
	if (attr == &bin_attr_unique_id && nor->params->read_unique_id &&
	    nor->params->unique_id_len)
		return 0444;

	return 0;
}

static const struct attribute_group spi_nor_sysfs_group = {
	.name		= "spi-nor",
	.is_bin_visible	= spi_nor_sysfs_is_bin_visible,
	.attrs		= spi_nor_sysfs_entries,
	.bin_attrs	= spi_nor_sysfs_bin_entries,
};

const struct attribute_group *spi_nor_sysfs_groups[] = {
	&spi_nor_sysfs_group,
	NULL
};
