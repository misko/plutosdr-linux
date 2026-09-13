// SPDX-License-Identifier: GPL-2.0
/* Verified extended-address selection for controllers with 24-bit addresses. */
#include <linux/mtd/cfi.h>
#include <linux/sizes.h>
#include <linux/mtd/spi-nor.h>
#include <linux/spi/spi-mem.h>

#include "core.h"

static int spi_nor_ear_opcodes(struct spi_nor *nor, u8 *read, u8 *write,
			       bool *wren)
{
	*read = SPINOR_OP_RDEAR;
	*write = SPINOR_OP_WREAR;
	*wren = true;

	/* CFI_MFR_WINBOND is 0xda, not Winbond's SPI manufacturer byte 0xef. */
	if (nor->manufacturer == &spi_nor_winbond)
		return nor->flags & SNOR_F_HAS_EAR ? 0 : -EOPNOTSUPP;

	switch (nor->jedec_id) {
	case CFI_MFR_AMD:
		*read = SPINOR_OP_BRRD;
		*write = SPINOR_OP_BRWR;
		*wren = false;
		return 0;
	case CFI_MFR_ST:
	case CFI_MFR_MACRONIX:
	case CFI_MFR_PMC:
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

int spi_nor_read_ear(struct spi_nor *nor, u8 *bank)
{
	u8 read, write;
	bool wren;
	unsigned int len = nor->isparallel ? 2 : 1;
	int ret;

	ret = spi_nor_ear_opcodes(nor, &read, &write, &wren);
	if (ret)
		return ret;
	if (nor->reg_proto != SNOR_PROTO_1_1_1)
		return -EOPNOTSUPP;

	if (nor->spimem) {
		struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(read, 1),
				   SPI_MEM_OP_NO_ADDR, SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_IN(len, nor->bouncebuf, 1));

		ret = spi_mem_exec_op(nor->spimem, &op);
	} else {
		ret = nor->controller_ops->read_reg(nor, read, nor->bouncebuf, len);
	}
	if (!ret && len == 2 && nor->bouncebuf[0] != nor->bouncebuf[1])
		ret = -EIO;
	if (!ret)
		*bank = nor->bouncebuf[0];
	return ret;
}

/**
 * spi_nor_write_ear() - Select and verify the bank containing a chip address.
 * @nor: flash, locked and prepared by the caller
 * @addr: per-chip byte address, after parallel/stacked translation
 *
 * A command accepted by the controller can still be ignored by the flash.
 * Cache only a completed, read-back selection. Never wrap an invalid address.
 */
int spi_nor_write_ear(struct spi_nor *nor, u32 addr)
{
	u64 chip_size = nor->mtd.size >> (nor->shift + nor->isstacked);
	u8 read, write, bank = addr >> 24, actual;
	bool wren;
	int ret, cleanup;

	if (addr >= chip_size) {
		ret = -EINVAL;
		goto invalid;
	}
	if (chip_size <= SZ_16M)
		return 0;

	ret = spi_nor_ear_opcodes(nor, &read, &write, &wren);
	if (ret)
		goto invalid;
	if (nor->reg_proto != SNOR_PROTO_1_1_1 || bank > EAR_SEGMENT_MASK) {
		ret = -EOPNOTSUPP;
		goto invalid;
	}
	if (!nor->isstacked && nor->bank_valid && bank == nor->curbank)
		return 0;

	nor->bank_valid = false;
	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		return ret;
	if (wren) {
		ret = spi_nor_write_enable(nor);
		if (ret)
			goto disable;
	}
	nor->bouncebuf[0] = bank;
	if (nor->spimem) {
		struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(write, 1),
				   SPI_MEM_OP_NO_ADDR, SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_OUT(1, nor->bouncebuf, 1));

		ret = spi_mem_exec_op(nor->spimem, &op);
	} else {
		ret = nor->controller_ops->write_reg(nor, write, nor->bouncebuf, 1);
	}
	if (ret)
		goto disable;
	ret = spi_nor_wait_till_ready(nor);
	if (ret)
		goto disable;
	ret = spi_nor_read_ear(nor, &actual);
	if (!ret && actual != bank)
		ret = -EIO;

disable:
	/* Reads also select banks; leave no write-enable latch behind. */
	if (wren) {
		cleanup = spi_nor_write_disable(nor);
		if (!ret)
			ret = cleanup;
	}
	if (!ret) {
		nor->curbank = bank;
		nor->bank_valid = true;
	}
	return ret;

invalid:
	nor->bank_valid = false;
	return ret;
}

int spi_nor_ear_reset(struct spi_nor *nor)
{
	int ret, upper_ret;
	u16 flags = 0;

	nor->bank_valid = false;
	if (nor->isstacked) {
		if (!nor->spimem)
			return -EOPNOTSUPP;
		flags = nor->spimem->spi->master->flags;
		nor->spimem->spi->master->flags &= ~SPI_MASTER_U_PAGE;
	}
	ret = spi_nor_write_ear(nor, 0);
	if (nor->isstacked && nor->spimem) {
		nor->spimem->spi->master->flags |= SPI_MASTER_U_PAGE;
		upper_ret = spi_nor_write_ear(nor, 0);
		nor->spimem->spi->master->flags = flags;
		if (!ret)
			ret = upper_ret;
	}
	/* Each top-level access establishes its own bank state. */
	nor->bank_valid = false;
	return ret;
}
