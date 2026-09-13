// SPDX-License-Identifier: GPL-2.0
/* Included by core.c so tests exercise its production MTD entry points. */
#include <kunit/test.h>
#include <linux/vmalloc.h>

struct ear_fake {
	struct spi_nor nor;
	struct spi_nor_flash_parameter params;
	struct spi_controller controller;
	struct spi_device spi;
	struct spi_mem mem;
	struct device dev;
	struct kunit *test;
	u8 bounce[4096];
	u8 *storage;
	u8 bank[2];
	bool wel, ignore_bank, bad_readback, parallel_mismatch;
	u8 fail_opcode;
	int fail_errno;
	unsigned int data_ops, bank_writes, wren_count, fail_data_after;
	u8 trace[128];
	unsigned int trace_len;
};

static int ear_fake_reg(struct ear_fake *f, u8 opcode, u8 *in,
			const u8 *out, size_t len)
{
	unsigned int chip = !!(f->controller.flags & SPI_MASTER_U_PAGE);

	if (f->trace_len < ARRAY_SIZE(f->trace))
		f->trace[f->trace_len++] = opcode;
	if (opcode == f->fail_opcode)
		return f->fail_errno;
	switch (opcode) {
	case SPINOR_OP_WREN:
		KUNIT_EXPECT_EQ(f->test, len, (size_t)0);
		f->wel = true;
		f->wren_count++;
		break;
	case SPINOR_OP_WRDI:
		f->wel = false;
		break;
	case SPINOR_OP_RDSR:
		memset(in, f->wel ? SR_WEL : 0, len);
		break;
	case SPINOR_OP_WREAR:
	case SPINOR_OP_BRWR:
		KUNIT_EXPECT_EQ(f->test, len, (size_t)1);
		f->bank_writes++;
		if (!f->ignore_bank && (f->wel || opcode == SPINOR_OP_BRWR))
			f->bank[chip] = out[0];
		f->wel = false;
		break;
	case SPINOR_OP_RDEAR:
	case SPINOR_OP_BRRD:
		memset(in, f->bank[chip] ^ f->bad_readback, len);
		if (len == 2 && f->parallel_mismatch)
			in[1] ^= 1;
		break;
	case SPINOR_OP_EN4B:
	case SPINOR_OP_EX4B:
		/* EX4B can leave an EAR different from the software cache. */
		f->bank[chip] = 1;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int ear_fake_read_reg(struct spi_nor *nor, u8 op, u8 *buf, size_t len)
{
	return ear_fake_reg(nor->priv, op, buf, NULL, len);
}

static int ear_fake_write_reg(struct spi_nor *nor, u8 op, const u8 *buf, size_t len)
{
	return ear_fake_reg(nor->priv, op, NULL, buf, len);
}

static ssize_t ear_fake_data(struct ear_fake *f, u8 opcode, u32 addr,
			    size_t len, u8 *in, const u8 *out)
{
	u32 physical = ((u32)f->bank[0] << 24) | (addr & (SZ_16M - 1));
	size_t i;

	f->data_ops++;
	if (opcode == f->fail_opcode && f->data_ops > f->fail_data_after)
		return f->fail_errno;
	if (physical + len > SZ_32M)
		return -EINVAL;
	if (opcode == SPINOR_OP_READ) {
		KUNIT_EXPECT_LE(f->test, (addr & (SZ_16M - 1)) + len, (size_t)SZ_16M);
		memcpy(in, f->storage + physical, len);
	} else {
		if (!f->wel)
			return -EACCES;
		f->wel = false;
		if (opcode == SPINOR_OP_SE) {
			memset(f->storage + physical, 0xff, SZ_64K);
		} else {
			KUNIT_EXPECT_LE(f->test, (addr & 255) + len, (size_t)256);
			for (i = 0; i < len; i++)
				f->storage[physical + i] &= out[i];
		}
	}
	return len;
}

static ssize_t ear_fake_read(struct spi_nor *nor, loff_t addr, size_t len, u8 *buf)
{
	return ear_fake_data(nor->priv, SPINOR_OP_READ, addr, len, buf, NULL);
}

static ssize_t ear_fake_write(struct spi_nor *nor, loff_t addr, size_t len,
			      const u8 *buf)
{
	return ear_fake_data(nor->priv, SPINOR_OP_PP, addr, len, NULL, buf);
}

static int ear_fake_erase(struct spi_nor *nor, loff_t addr)
{
	return ear_fake_data(nor->priv, SPINOR_OP_SE, addr, 0, NULL, NULL);
}

static const struct spi_nor_controller_ops ear_fake_ops = {
	.read_reg = ear_fake_read_reg,
	.write_reg = ear_fake_write_reg,
	.read = ear_fake_read,
	.write = ear_fake_write,
	.erase = ear_fake_erase,
};

static int ear_fake_exec(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct ear_fake *f = spi_mem_get_drvdata(mem);
	ssize_t ret;

	KUNIT_EXPECT_EQ(f->test, op->cmd.nbytes, (u8)1);
	KUNIT_EXPECT_EQ(f->test, op->cmd.buswidth, (u8)1);
	KUNIT_EXPECT_EQ(f->test, op->dummy.nbytes, (u8)0);
	if (op->data.nbytes)
		KUNIT_EXPECT_EQ(f->test, op->data.buswidth, (u8)1);
	if (op->addr.nbytes) {
		KUNIT_EXPECT_EQ(f->test, op->addr.nbytes, (u8)3);
		ret = ear_fake_data(f, op->cmd.opcode, op->addr.val,
				    op->data.nbytes,
				    op->data.dir == SPI_MEM_DATA_IN ? op->data.buf.in : NULL,
				    op->data.dir == SPI_MEM_DATA_OUT ? op->data.buf.out : NULL);
		return ret < 0 ? ret : 0;
	}
	return ear_fake_reg(f, op->cmd.opcode,
			    op->data.dir == SPI_MEM_DATA_IN ? op->data.buf.in : NULL,
			    op->data.dir == SPI_MEM_DATA_OUT ? op->data.buf.out : NULL,
			    op->data.nbytes);
}

static const struct spi_controller_mem_ops ear_fake_mem_ops = {
	.exec_op = ear_fake_exec,
};

static const struct flash_info ear_fake_info = {
	.name = "w25q256",
	.id = {0xef, 0x40, 0x19},
	.id_len = 3,
};

static int ear_test_init(struct kunit *test)
{
	struct ear_fake *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	struct spi_nor *nor;

	if (!f)
		return -ENOMEM;
	f->storage = vzalloc(SZ_32M);
	if (!f->storage)
		return -ENOMEM;
	test->priv = f;
	f->test = test;
	f->dev.init_name = "spi-nor-ear-test";
	f->controller.mem_ops = &ear_fake_mem_ops;
	mutex_init(&f->controller.bus_lock_mutex);
	mutex_init(&f->controller.io_mutex);
	f->spi.controller = &f->controller;
	f->spi.master = &f->controller;
	f->mem.spi = &f->spi;
	init_completion(&f->mem.request_completion);
	spi_mem_set_drvdata(&f->mem, f);
	nor = &f->nor;
	nor->dev = &f->dev;
	nor->priv = f;
	nor->spi = &f->spi;
	nor->params = &f->params;
	nor->params->erase_map.uniform_erase_type = BIT(0);
	nor->info = &ear_fake_info;
	nor->manufacturer = &spi_nor_winbond;
	nor->jedec_id = 0xef;
	nor->flags = SNOR_F_HAS_EAR;
	nor->mtd.priv = nor;
	nor->mtd.size = SZ_32M;
	nor->mtd.erasesize = SZ_64K;
	nor->page_size = 256;
	nor->addr_width = 3;
	nor->read_opcode = SPINOR_OP_READ;
	nor->program_opcode = SPINOR_OP_PP;
	nor->erase_opcode = SPINOR_OP_SE;
	nor->read_proto = nor->write_proto = nor->reg_proto = SNOR_PROTO_1_1_1;
	nor->bouncebuf = f->bounce;
	nor->bouncebuf_size = sizeof(f->bounce);
	nor->controller_ops = &ear_fake_ops;
	mutex_init(&nor->lock);
	return 0;
}

static void ear_test_exit(struct kunit *test)
{
	struct ear_fake *f = test->priv;

	vfree(f->storage);
}

static void ear_select_and_restore_test(struct kunit *test)
{
	struct ear_fake *f = test->priv;
	unsigned int mode, writes;
	static const u8 expected[] = {SPINOR_OP_RDSR, SPINOR_OP_WREN,
		SPINOR_OP_WREAR, SPINOR_OP_RDSR, SPINOR_OP_RDEAR, SPINOR_OP_WRDI};

	for (mode = 0; mode < 2; mode++) {
		f->nor.spimem = mode ? &f->mem : NULL;
		f->nor.bank_valid = false;
		f->trace_len = 0;
		KUNIT_ASSERT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), 0);
		KUNIT_EXPECT_EQ(test, memcmp(f->trace, expected, sizeof(expected)), 0);
		KUNIT_EXPECT_EQ(test, f->trace_len, (unsigned int)sizeof(expected));
		KUNIT_EXPECT_EQ(test, f->bank[0], (u8)1);
		KUNIT_EXPECT_FALSE(test, f->wel);
		writes = f->bank_writes;
		KUNIT_ASSERT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M + 1), 0);
		KUNIT_EXPECT_EQ(test, f->bank_writes, writes);
		KUNIT_ASSERT_EQ(test, spi_nor_write_ear(&f->nor, 0), 0);
		f->bank[0] = 1; /* stale zero cache after a reset/mode change */
		KUNIT_ASSERT_EQ(test, spi_nor_ear_reset(&f->nor), 0);
		KUNIT_EXPECT_EQ(test, f->bank[0], (u8)0);
		KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
	}
}

static void ear_selection_failure_test(struct kunit *test)
{
	struct ear_fake *f = test->priv;
	static const u8 failures[] = {SPINOR_OP_RDSR, SPINOR_OP_WREN,
		SPINOR_OP_WREAR, SPINOR_OP_RDEAR, SPINOR_OP_WRDI};
	unsigned int mode, i, writes;
	u8 byte;
	size_t retlen;

	for (mode = 0; mode < 2; mode++) {
		f->nor.spimem = mode ? &f->mem : NULL;
		for (i = 0; i < ARRAY_SIZE(failures); i++) {
			f->nor.bank_valid = false;
			f->fail_opcode = failures[i];
			f->fail_errno = -ETIMEDOUT;
			f->data_ops = 0;
			retlen = 0;
			KUNIT_EXPECT_EQ(test, spi_nor_read(&f->nor.mtd, SZ_16M, 1, &retlen, &byte), -ETIMEDOUT);
			KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
			KUNIT_EXPECT_EQ(test, f->data_ops, 0U);
			KUNIT_EXPECT_EQ(test, retlen, (size_t)0);
			writes = f->bank_writes;
			f->fail_opcode = 0;
			KUNIT_ASSERT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), 0);
			KUNIT_EXPECT_GT(test, f->bank_writes, writes);
		}
		f->nor.bank_valid = false;
		f->bank[0] = 0;
		f->ignore_bank = true;
		KUNIT_EXPECT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), -EIO);
		KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
		f->ignore_bank = false;
		f->bad_readback = true;
		KUNIT_EXPECT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), -EIO);
		KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
		f->bad_readback = false;
	}
}

static void ear_mtd_boundary_test(struct kunit *test)
{
	struct ear_fake *f = test->priv;
	u8 input[514], output[514];
	struct erase_info erase = {.addr = SZ_16M - SZ_64K, .len = 2 * SZ_64K};
	size_t retlen;
	unsigned int mode, i;

	for (i = 0; i < sizeof(input); i++)
		input[i] = (i * 13 + i / 256) & 255;
	for (mode = 0; mode < 2; mode++) {
		f->nor.spimem = mode ? &f->mem : NULL;
		memset(f->storage, 0xa5, SZ_32M);
		KUNIT_ASSERT_EQ(test, spi_nor_erase(&f->nor.mtd, &erase), 0);
		KUNIT_EXPECT_EQ(test, f->storage[0], (u8)0xa5);
		KUNIT_EXPECT_EQ(test, f->storage[SZ_16M - SZ_64K - 1], (u8)0xa5);
		KUNIT_EXPECT_EQ(test, f->storage[SZ_16M + SZ_64K], (u8)0xa5);
		retlen = 0;
		KUNIT_ASSERT_EQ(test, spi_nor_write(&f->nor.mtd, SZ_16M - 257, sizeof(input), &retlen, input), 0);
		KUNIT_EXPECT_EQ(test, retlen, sizeof(input));
		KUNIT_EXPECT_EQ(test, memcmp(f->storage + SZ_16M - 257, input, sizeof(input)), 0);
		retlen = 0;
		KUNIT_ASSERT_EQ(test, spi_nor_read(&f->nor.mtd, SZ_16M - 257, sizeof(output), &retlen, output), 0);
		KUNIT_EXPECT_EQ(test, memcmp(input, output, sizeof(input)), 0);
		retlen = 0;
		KUNIT_ASSERT_EQ(test, spi_nor_read(&f->nor.mtd, 0, 1, &retlen, output), 0);
		KUNIT_EXPECT_EQ(test, output[0], (u8)0xa5);
		KUNIT_EXPECT_EQ(test, f->bank[0], (u8)0);
	}
}

static void ear_vendor_and_geometry_test(struct kunit *test)
{
	struct ear_fake *f = test->priv;
	static const u8 vendors[] = {CFI_MFR_AMD, CFI_MFR_ST, CFI_MFR_MACRONIX, CFI_MFR_PMC};
	unsigned int i;

	f->nor.manufacturer = NULL;
	for (i = 0; i < ARRAY_SIZE(vendors); i++) {
		f->nor.jedec_id = vendors[i];
		f->nor.bank_valid = false;
		f->wren_count = 0;
		KUNIT_ASSERT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), 0);
		KUNIT_EXPECT_EQ(test, f->wren_count, vendors[i] == CFI_MFR_AMD ? 0U : 1U);
	}
	f->nor.jedec_id = 0xda;
	KUNIT_EXPECT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), -EOPNOTSUPP);
	f->nor.manufacturer = &spi_nor_winbond;
	f->nor.flags = 0;
	KUNIT_EXPECT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), -EOPNOTSUPP);
	f->nor.flags = SNOR_F_HAS_EAR;
	KUNIT_EXPECT_EQ(test, spi_nor_write_ear(&f->nor, SZ_32M), -EINVAL);
	f->nor.isparallel = true;
	f->nor.shift = 1;
	f->nor.mtd.size = SZ_64M;
	f->parallel_mismatch = true;
	f->nor.bank_valid = false;
	KUNIT_EXPECT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), -EIO);
	f->parallel_mismatch = false;
	KUNIT_ASSERT_EQ(test, spi_nor_write_ear(&f->nor, SZ_16M), 0);
	f->nor.isparallel = false;
	f->nor.shift = 0;
	f->nor.isstacked = true;
	f->nor.spimem = &f->mem;
	f->bank[0] = f->bank[1] = 1;
	f->controller.flags |= SPI_MASTER_U_PAGE;
	KUNIT_ASSERT_EQ(test, spi_nor_ear_reset(&f->nor), 0);
	KUNIT_EXPECT_EQ(test, f->bank[0], (u8)0);
	KUNIT_EXPECT_EQ(test, f->bank[1], (u8)0);
	KUNIT_EXPECT_TRUE(test, f->controller.flags & SPI_MASTER_U_PAGE);
}

static void ear_data_error_test(struct kunit *test)
{
	struct ear_fake *f = test->priv;
	u8 data[4] = {0x12, 0x34, 0x56, 0x78};
	struct erase_info erase = {.addr = SZ_16M - SZ_64K, .len = 2 * SZ_64K};
	unsigned int mode;
	size_t retlen;

	for (mode = 0; mode < 2; mode++) {
		f->nor.spimem = mode ? &f->mem : NULL;
		f->fail_errno = -EIO;
		f->fail_data_after = 1;
		f->data_ops = 0;
		f->fail_opcode = SPINOR_OP_READ;
		retlen = 0;
		KUNIT_EXPECT_EQ(test, spi_nor_read(&f->nor.mtd, SZ_16M - 2,
			4, &retlen, data), -EIO);
		KUNIT_EXPECT_EQ(test, retlen, (size_t)2);
		KUNIT_EXPECT_EQ(test, f->data_ops, 2U);
		KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
		f->data_ops = 0;
		f->fail_opcode = SPINOR_OP_PP;
		retlen = 0;
		KUNIT_EXPECT_EQ(test, spi_nor_write(&f->nor.mtd, SZ_16M - 2,
			4, &retlen, data), -EIO);
		KUNIT_EXPECT_EQ(test, retlen, (size_t)2);
		KUNIT_EXPECT_EQ(test, f->data_ops, 2U);
		KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
		f->data_ops = 0;
		f->fail_opcode = SPINOR_OP_SE;
		KUNIT_EXPECT_EQ(test, spi_nor_erase(&f->nor.mtd, &erase), -EIO);
		KUNIT_EXPECT_EQ(test, f->data_ops, 2U);
		KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
	}
}

static int ear_fake_mode_failure(struct spi_nor *nor, bool enable)
{
	return -EIO;
}

static void ear_initialization_test(struct kunit *test)
{
	struct ear_fake *f = test->priv;
	unsigned int mode;

	for (mode = 0; mode < 2; mode++) {
		f->nor.spimem = mode ? &f->mem : NULL;
		f->bank[0] = 1;
		f->nor.curbank = 0;
		f->nor.bank_valid = true;
		KUNIT_ASSERT_EQ(test, spi_nor_init(&f->nor), 0);
		KUNIT_EXPECT_EQ(test, f->bank[0], (u8)0);
		f->fail_opcode = SPINOR_OP_RDEAR;
		f->fail_errno = -ETIMEDOUT;
		KUNIT_EXPECT_EQ(test, spi_nor_init(&f->nor), -ETIMEDOUT);
		KUNIT_EXPECT_FALSE(test, f->nor.bank_valid);
		f->fail_opcode = 0;
	}
	f->nor.addr_width = 4;
	f->params.set_4byte_addr_mode = ear_fake_mode_failure;
	KUNIT_EXPECT_EQ(test, spi_nor_init(&f->nor), -EIO);
}

static void ear_legacy_alias_test(struct kunit *test)
{
	struct ear_fake *f = test->priv;
	u8 input[] = {0x12, 0x34, 0x56, 0x78}, output[4], bank = 1;
	size_t retlen = 0;

	memset(f->storage, 0xff, SZ_32M);
	/* Negative control: the old Winbond path omitted WREN before WREAR. */
	KUNIT_ASSERT_EQ(test, ear_fake_reg(f, SPINOR_OP_WREAR, NULL, &bank, 1), 0);
	KUNIT_EXPECT_EQ(test, f->bank[0], (u8)0);
	f->wel = true;
	KUNIT_ASSERT_EQ(test, ear_fake_data(f, SPINOR_OP_PP, SZ_16M, 4, NULL, input), (ssize_t)4);
	KUNIT_ASSERT_EQ(test, ear_fake_data(f, SPINOR_OP_READ, SZ_16M, 4, output, NULL), (ssize_t)4);
	/* A same-path comparison passes while physical boot bytes are damaged. */
	KUNIT_EXPECT_EQ(test, memcmp(input, output, 4), 0);
	KUNIT_EXPECT_EQ(test, memcmp(input, f->storage, 4), 0);
	KUNIT_EXPECT_EQ(test, f->storage[SZ_16M], (u8)0xff);
	/* The repaired production entry point refuses an ignored bank change. */
	f->ignore_bank = true;
	f->data_ops = 0;
	KUNIT_EXPECT_EQ(test, spi_nor_write(&f->nor.mtd, SZ_16M, 4, &retlen, input), -EIO);
	KUNIT_EXPECT_EQ(test, retlen, (size_t)0);
	KUNIT_EXPECT_EQ(test, f->data_ops, 0U);
}

static struct kunit_case ear_test_cases[] = {
	KUNIT_CASE(ear_select_and_restore_test),
	KUNIT_CASE(ear_selection_failure_test),
	KUNIT_CASE(ear_mtd_boundary_test),
	KUNIT_CASE(ear_vendor_and_geometry_test),
	KUNIT_CASE(ear_data_error_test),
	KUNIT_CASE(ear_initialization_test),
	KUNIT_CASE(ear_legacy_alias_test),
	{}
};

static struct kunit_suite ear_test_suite = {
	.name = "spi-nor-ear",
	.init = ear_test_init,
	.exit = ear_test_exit,
	.test_cases = ear_test_cases,
};
kunit_test_suite(ear_test_suite);
