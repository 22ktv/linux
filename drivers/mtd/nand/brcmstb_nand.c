/*
 * Copyright (C) 2010 Broadcom Corporation
 * Copyright (C) 2015 Jaedon Shin <jaedon.shin@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME	": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_mtd.h>

#define NAND_REVISION			0x00
#define NAND_CMD_START			0x04
#define NAND_CMD_EXT_ADDRESS		0x08
#define NAND_CMD_ADDRESS		0x0c
#define NAND_CMD_END_ADDRESS		0x10
#define NAND_CS_NAND_SELECT		0x14
#define NAND_CS_NAND_XOR		0x18
#define NAND_SPARE_AREA_READ_OFS_0	0x20
#define NAND_SPARE_AREA_READ_OFS_4	0x24
#define NAND_SPARE_AREA_READ_OFS_8	0x28
#define NAND_SPARE_AREA_READ_OFS_C	0x2c
#define NAND_SPARE_AREA_WRITE_OFS_0	0x30
#define NAND_SPARE_AREA_WRITE_OFS_4	0x34
#define NAND_SPARE_AREA_WRITE_OFS_8	0x38
#define NAND_SPARE_AREA_WRITE_OFS_C	0x3c
#define NAND_ACC_CONTROL		0x40
#define NAND_CONFIG			0x48
#define NAND_TIMING_1			0x50
#define NAND_TIMING_2			0x54
#define NAND_SEMAPHORE			0x58
#define NAND_FLASH_DEVICE_ID		0x60
#define NAND_FLASH_DEVICE_ID_EXT	0x64
#define NAND_BLOCK_LOCK_STATUS		0x68
#define NAND_INTFC_STATUS		0x6c
#define NAND_ECC_CORR_EXT_ADDR		0x70
#define NAND_ECC_CORR_ADDR		0x74
#define NAND_ECC_UNC_EXT_ADDR		0x78
#define NAND_ECC_UNC_ADDR		0x7c
#define NAND_READ_ERROR_COUNT		0x80
#define NAND_CORR_STAT_THRESHOLD	0x84
#define NAND_ONFI_STATUS		0x88
#define NAND_ONFI_DEBUG_DATA		0x8c
#define NAND_FLASH_READ_EXT_ADDR	0x90
#define NAND_FLASH_READ_ADDR		0x94
#define NAND_PROGRAM_PAGE_EXT_ADDR	0x98
#define NAND_PROGRAM_PAGE_ADDR		0x9c
#define NAND_COPY_BACK_EXT_ADDR		0xa0
#define NAND_COPY_BACK_ADDR		0xa4
#define NAND_BLOCK_ERASE_EXT_ADDR	0xa8
#define NAND_BLOCK_ERASE_ADDR		0xac
#define NAND_INV_READ_EXT_ADDR		0xb0
#define NAND_INV_READ_ADDR		0xb4
#define NAND_BLK_WR_PROTECT		0xc0
#define NAND_ACC_CONTROL_CS1		0xd0
#define NAND_CONFIG_CS1			0xd4
#define NAND_TIMING_1_CS1		0xd8
#define NAND_TIMING_2_CS1		0xdc
#define NAND_ACC_CONTROL_CS2		0xe0
#define NAND_CONFIG_CS2			0xe4
#define NAND_TIMING_1_CS2		0xe8
#define NAND_TIMING_2_CS2		0xec
#define NAND_ACC_CONTROL_CS3		0xf0
#define NAND_CONFIG_CS3			0xf4
#define NAND_TIMING_1_CS3		0xf8
#define NAND_TIMING_2_CS3		0xfc
#define NAND_ACC_CONTROL_CS4		0x100
#define NAND_CONFIG_CS4			0x104
#define NAND_TIMING_1_CS4		0x108
#define NAND_TIMING_2_CS4		0x10c
#define NAND_SPARE_AREA_READ_OFS_10	0x130
#define NAND_SPARE_AREA_READ_OFS_14	0x134
#define NAND_SPARE_AREA_READ_OFS_18	0x138
#define NAND_SPARE_AREA_READ_OFS_1C	0x13c
#define NAND_SPARE_AREA_WRITE_OFS_10	0x140
#define NAND_SPARE_AREA_WRITE_OFS_14	0x144
#define NAND_SPARE_AREA_WRITE_OFS_18	0x148
#define NAND_SPARE_AREA_WRITE_OFS_1C	0x14c
#define NAND_LL_OP			0x178
#define NAND_LL_RDDATA			0x17c
#define NAND_FLASH_CACHEi_ARRAY_BASE	0x200

#define NAND_INTR_CPU_STATUS		0x0
#define NAND_INTR_CPU_SET		0x4
#define NAND_INTR_CPU_CLEAR		0x8
#define NAND_INTR_CPU_MASK_STATUS	0xc
#define NAND_INTR_CPU_MASK_SET		0x10
#define NAND_INTR_CPU_MASK_CLEAR	0x14
#define NAND_INTR_CTLRDY_MASK		0x01000000

#define CMD_NULL			0x00
#define CMD_PAGE_READ			0x01
#define CMD_SPARE_AREA_READ		0x02
#define CMD_STATUS_READ			0x03
#define CMD_PROGRAM_PAGE		0x04
#define CMD_PROGRAM_SPARE_AREA		0x05
#define CMD_COPY_BACK			0x06
#define CMD_DEVICE_ID_READ		0x07
#define CMD_BLOCK_ERASE			0x08
#define CMD_FLASH_RESET			0x09
#define CMD_BLOCKS_LOCK			0x0a
#define CMD_BLOCKS_LOCK_DOWN		0x0b
#define CMD_BLOCKS_UNLOCK		0x0c
#define CMD_READ_BLOCKS_LOCK_STATUS	0x0d
#define CMD_PARAMETER_READ		0x0e
#define CMD_PARAMETER_CHANGE_COL	0x0f
#define CMD_LOW_LEVEL_OP		0x10

/*
 * 512B flash cache in the NAND controller HW
 */
#define FC_SHIFT			9U
#define FC_BYTES			512U
#define FC_WORDS			(FC_BYTES >> 2)
#define FC(x) \
	(NAND_FLASH_CACHEi_ARRAY_BASE + ((x) << 2))

#define MAX_CONTROLLER_OOB		32

struct brcmstb_nand_cfg {
	u64 device_size;
	u32 block_size;
	u32 page_size;
	u32 spare_area_size;
	u32 device_width;
	u32 col_adr_bytes;
	u32 blk_adr_bytes;
	u32 ful_adr_bytes;
	u32 sector_size_1k;
	u32 ecc_level;
};

struct brcmstb_nfc {
	struct nand_hw_control controller;
	struct nand_chip chip;
	struct mtd_info mtd;

	void __iomem *nand_regs;
	void __iomem *nand_intr_regs;
	int cmd_pending;
	struct completion done;
	unsigned int last_cmd;
	unsigned int last_byte;
	u64 last_addr;
	u32 buf[FC_WORDS];
	int cs;
	struct brcmstb_nand_cfg hwcfg;
};

static struct nand_ecclayout brcmstb_nand_dummy_layout = {
	.eccbytes = 16,
	.eccpos = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
};

static inline u32 reg_read(struct brcmstb_nfc *nfc, u32 offset)
{
	return __raw_readl(nfc->nand_regs + offset);
}

static inline void reg_write(struct brcmstb_nfc *nfc, u32 offset, u32 val)
{
	__raw_writel(val, nfc->nand_regs + offset);
}

static inline u32 config_reg_read(struct brcmstb_nfc *nfc)
{
	u32 offset;

	if (nfc->cs == 0)
		offset = NAND_CONFIG;
	else
		offset = NAND_CONFIG_CS1 + ((nfc->cs - 1) << 4);

	return reg_read(nfc, offset);
}

static inline void config_reg_write(struct brcmstb_nfc *nfc, u32 mask, u32 val)
{
	u32 offset, tmp;

	if (nfc->cs == 0)
		offset = NAND_CONFIG;
	else
		offset = NAND_CONFIG_CS1 + ((nfc->cs - 1) << 4);

	tmp = reg_read(nfc, offset);
	tmp &= ~mask;
	tmp |= val;
	reg_write(nfc, offset, tmp);
}

static inline u32 acc_control_read(struct brcmstb_nfc *nfc)
{
	u32 offset;

	if (nfc->cs == 0)
		offset = NAND_ACC_CONTROL;
	else
		offset = NAND_ACC_CONTROL_CS1 + (((nfc->cs) - 1) << 4);

	return reg_read(nfc, offset);
}

static inline void acc_control_write(struct brcmstb_nfc *nfc, u32 mask, u32 val)
{
	u32 offset, tmp;

	if (nfc->cs == 0)
		offset = NAND_ACC_CONTROL;
	else
		offset = NAND_ACC_CONTROL_CS1 + (((nfc->cs) - 1) << 4);

	tmp = reg_read(nfc, offset);
	tmp &= ~mask;
	tmp |= val;
	reg_write(nfc, offset, tmp);
}

static inline u8 oob_reg_read(struct brcmstb_nfc *nfc, int idx)
{
	u32 offset;

	if (idx >= MAX_CONTROLLER_OOB)
		return 0x77;

	if (idx < 16)
		offset = NAND_SPARE_AREA_READ_OFS_0 + (idx & ~0x03);
	else {
		idx -= 16;
		offset = NAND_SPARE_AREA_READ_OFS_10 + (idx & ~0x03);
	}

	return reg_read(nfc, offset) >> (24 - ((idx & 0x03) << 3));
}

static inline void oob_reg_write(struct brcmstb_nfc *nfc, int idx, u32 data)
{
	u32 offset;

	if (idx >= MAX_CONTROLLER_OOB)
		return;

	if (idx < 16)
		offset = NAND_SPARE_AREA_WRITE_OFS_0 + (idx & ~0x03);
	else {
		idx -= 16;
		offset = NAND_SPARE_AREA_WRITE_OFS_10 + (idx & ~0x03);
	}

	reg_write(nfc, offset, data);
}

static int read_oob_from_regs(struct brcmstb_nfc *nfc, int i, u8 *oob,
		int sas, int sector_1k)
{
	int tbytes = sas << sector_1k;
	int j;

	if (sector_1k && (i & 0x01))
		tbytes = max(0, tbytes - MAX_CONTROLLER_OOB);

	tbytes = min(tbytes, MAX_CONTROLLER_OOB);

	for (j = 0; j < tbytes; j++)
		oob[j] = oob_reg_read(nfc, j);

	return tbytes;
}

static int write_oob_to_regs(struct brcmstb_nfc *nfc, int i, const u8 *oob,
		int sas, int sector_1k)
{
	int tbytes = sas << sector_1k;
	int j;

	if (sector_1k && (i & 0x01))
		tbytes = max(0, tbytes - MAX_CONTROLLER_OOB);

	tbytes = min(tbytes, MAX_CONTROLLER_OOB);

	for (j = 0; j < tbytes; j += 4)
		oob_reg_write(nfc, j,
			      (oob[j + 0] << 24) |
			      (oob[j + 1] << 16) |
			      (oob[j + 2] << 8) |
			      (oob[j + 3] << 0));

	return tbytes;
}

static inline bool is_hamming_ecc(struct brcmstb_nand_cfg *cfg)
{
	return cfg->sector_size_1k == 0 &&
		cfg->spare_area_size == 16 &&
		cfg->ecc_level == 15;
}

static struct nand_ecclayout *brcmstb_nand_create_layout(int ecc_level,
		struct brcmstb_nand_cfg *cfg)
{
	int i, j;
	struct nand_ecclayout *layout;
	int req;
	int sectors;
	int sas;
	int idx1, idx2;

	layout = kzalloc(sizeof(*layout), GFP_KERNEL);
	if (!layout)
		return NULL;

	sectors = cfg->page_size / (512 << cfg->sector_size_1k);
	sas = cfg->spare_area_size << cfg->sector_size_1k;

	if (is_hamming_ecc(cfg)) {
		for (i = 0, idx1 = 0, idx2 = 0; i < sectors; i++) {
			if (i == 0) {
				layout->oobfree[idx2].offset = i * sas + 1;
				if (cfg->page_size == 512)
					layout->oobfree[idx2].offset--;
				layout->oobfree[idx2].length = 5;
			} else {
				layout->oobfree[idx2].offset = i * sas;
				layout->oobfree[idx2].length = 6;
			}
			idx2++;

			layout->eccpos[idx1++] = i * sas + 6;
			layout->eccpos[idx1++] = i * sas + 7;
			layout->eccpos[idx1++] = i * sas + 8;

			layout->oobfree[idx2].offset = i * sas + 9;
			layout->oobfree[idx2].length = 7;
			idx2++;

			if (idx1 >= MTD_MAX_ECCPOS_ENTRIES_LARGE ||
			    idx2 >= MTD_MAX_OOBFREE_ENTRIES_LARGE - 1)
				break;
		}
		goto out;
	}

	req = (ecc_level * 14 + 7) / 8;
	if (req >= sas) {
		pr_info("ECC too large for OOB, using dummy layout\n");
		memcpy(layout, &brcmstb_nand_dummy_layout, sizeof(*layout));
		return layout;
	}

	layout->eccbytes = req * sectors;
	for (i = 0, idx1 = 0, idx2 = 0; i < sectors; i++) {
		for (j = sas - req;
		     j < sas && idx1 < MTD_MAX_ECCPOS_ENTRIES_LARGE;
		     j++, idx1++)
			layout->eccpos[idx1] = i * sas + j;

		if (i == 0) {
			if (cfg->page_size == 512 && (sas - req >= 6)) {
				layout->oobfree[idx2].offset = 0;
				layout->oobfree[idx2].length = 5;
				idx2++;

				if (sas - req > 6) {
					layout->oobfree[idx2].offset = 6;
					layout->oobfree[idx2].length =
					    sas - req - 6;
					idx2++;
				}
			} else if (sas > req + 1) {
				layout->oobfree[idx2].offset = i * sas + 1;
				layout->oobfree[idx2].length = sas - req - 1;
				idx2++;
			}
		} else if (sas > req) {
			layout->oobfree[idx2].offset = i * sas;
			layout->oobfree[idx2].length = sas - req;
			idx2++;
		}

		if (idx1 >= MTD_MAX_ECCPOS_ENTRIES_LARGE ||
		    idx2 >= MTD_MAX_OOBFREE_ENTRIES_LARGE - 1)
			break;
	}

out:
	for (i = 0; i < MTD_MAX_OOBFREE_ENTRIES_LARGE; i++)
		layout->oobavail += layout->oobfree[i].length;

	return layout;
}

static struct nand_ecclayout *brcmstb_choose_ecc_layout(struct brcmstb_nfc *nfc)
{
	struct nand_ecclayout *layout;
	struct brcmstb_nand_cfg *p = &nfc->hwcfg;
	unsigned int ecc_level = p->ecc_level;

	if (p->sector_size_1k)
		ecc_level <<= 1;

	layout = brcmstb_nand_create_layout(ecc_level, p);
	if (!layout) {
		pr_err("no proper ecc_layout for this NAND cfg\n");
		return NULL;
	}

	return layout;
}

static irqreturn_t brcmstb_nand_irq(int irq, void *dev_id)
{
	struct brcmstb_nfc *nfc = dev_id;
	u32 status;

	status = __raw_readl(nfc->nand_intr_regs + NAND_INTR_CPU_STATUS) &
		~__raw_readl(nfc->nand_intr_regs + NAND_INTR_CPU_MASK_STATUS);

	if (status & NAND_INTR_CTLRDY_MASK) {
		__raw_writel(NAND_INTR_CTLRDY_MASK,
			     nfc->nand_intr_regs + NAND_INTR_CPU_CLEAR);
		__raw_readl(nfc->nand_intr_regs + NAND_INTR_CPU_CLEAR);

		complete(&nfc->done);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void brcmstb_nand_send_cmd(struct brcmstb_nfc *nfc, int cmd)
{
	BUG_ON(nfc->cmd_pending != 0);

	nfc->cmd_pending = cmd;

	mb();

	reg_write(nfc, NAND_CMD_START, cmd << 24);
}

static void brcmstb_nand_cmd_ctrl(struct mtd_info *mtd, int dat,
		unsigned int ctrl)
{
	/* intentionally left blank */
}

static int brcmstb_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct nand_chip *chip = mtd->priv;
	struct brcmstb_nfc *nfc = chip->priv;

	if (nfc->cmd_pending) {
		if (wait_for_completion_timeout(&nfc->done, HZ / 10) <= 0)
			pr_err("timeout waiting for command %u\n",
			       nfc->last_cmd);
	}

	nfc->cmd_pending = 0;

	return reg_read(nfc, NAND_INTFC_STATUS) & 0xff;
}

static void brcmstb_nand_cmdfunc(struct mtd_info *mtd, unsigned command,
		int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct brcmstb_nfc *nfc = chip->priv;
	u64 addr = (u64)page_addr << chip->page_shift;
	int native_cmd = 0;

	if (command == NAND_CMD_READID || command == NAND_CMD_PARAM)
		addr = (u64)column;
	else if (page_addr < 0)
		addr = 0;

	nfc->last_cmd = command;
	nfc->last_byte = 0;
	nfc->last_addr = addr;

	switch (command) {
	case NAND_CMD_RESET:
		native_cmd = CMD_FLASH_RESET;
		break;
	case NAND_CMD_STATUS:
		native_cmd = CMD_STATUS_READ;
		break;
	case NAND_CMD_READID:
		native_cmd = CMD_DEVICE_ID_READ;
		break;
	case NAND_CMD_READOOB:
		native_cmd = CMD_SPARE_AREA_READ;
		break;
	case NAND_CMD_ERASE1:
		native_cmd = CMD_BLOCK_ERASE;
		break;
	case NAND_CMD_PARAM:
		native_cmd = CMD_PARAMETER_READ;
		break;
	case NAND_CMD_RNDOUT:
		native_cmd = CMD_PARAMETER_CHANGE_COL;
		addr &= ~((u64)(FC_BYTES - 1));
		break;
	}

	if (!native_cmd)
		return;

	reg_write(nfc, NAND_CMD_EXT_ADDRESS,
			(nfc->cs << 16) | ((addr >> 32) & 0xffff));
	reg_write(nfc, NAND_CMD_ADDRESS, addr & 0xffffffff);

	brcmstb_nand_send_cmd(nfc, native_cmd);
	brcmstb_nand_waitfunc(mtd, chip);
}

static u8 brcmstb_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct brcmstb_nfc *nfc = chip->priv;
	u8 ret = 0;
	int addr, offs;

	switch (nfc->last_cmd) {
	case NAND_CMD_READID:
		if (nfc->last_byte < 4)
			ret = reg_read(nfc, NAND_FLASH_DEVICE_ID) >>
				(24 - (nfc->last_byte << 3));
		else if (nfc->last_byte < 8)
			ret = reg_read(nfc, NAND_FLASH_DEVICE_ID_EXT) >>
				(56 - (nfc->last_byte << 3));
		break;

	case NAND_CMD_READOOB:
		ret = oob_reg_read(nfc, nfc->last_byte);
		break;

	case NAND_CMD_STATUS:
		ret = reg_read(nfc, NAND_INTFC_STATUS) & 0xff;
		break;

	case NAND_CMD_PARAM:
	case NAND_CMD_RNDOUT:
		addr = nfc->last_addr + nfc->last_byte;
		offs = addr & (FC_BYTES - 1);

		if (nfc->last_byte > 0 && offs == 0)
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, addr, -1);

		ret = reg_read(nfc, FC(nfc->last_byte >> 2)) >>
				(24 - ((nfc->last_byte & 0x03) << 3));
		break;
	}

	nfc->last_byte++;

	return ret;
}

static void brcmstb_nand_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++, buf++)
		*buf = brcmstb_nand_read_byte(mtd);
}

static int brcmstb_nand_read_by_pio(struct mtd_info *mtd,
		struct nand_chip *chip, u64 addr, unsigned int trans, u32 *buf,
		u8 *oob, u64 *err_addr)
{
	struct brcmstb_nfc *nfc = chip->priv;
	int i, j, ret = 0;

	reg_write(nfc, NAND_ECC_UNC_ADDR, 0);
	reg_write(nfc, NAND_ECC_CORR_ADDR, 0);

	reg_write(nfc, NAND_CMD_EXT_ADDRESS,
			(nfc->cs << 16) | ((addr >> 32) & 0xffff));

	for (i = 0; i < trans; i++, addr += FC_BYTES) {
		reg_write(nfc, NAND_CMD_ADDRESS, addr & 0xffffffff);
		brcmstb_nand_send_cmd(nfc, CMD_PAGE_READ);
		brcmstb_nand_waitfunc(mtd, chip);

		if (likely(buf))
			for (j = 0; j < FC_WORDS; j++, buf++)
				*buf = le32_to_cpu(reg_read(nfc, FC(j)));

		if (oob)
			oob += read_oob_from_regs(nfc, i,
						  oob, mtd->oobsize / trans,
						  nfc->hwcfg.sector_size_1k);

		if (!ret) {
			*err_addr = reg_read(nfc, NAND_ECC_UNC_ADDR) |
				((u64)(reg_read(nfc, NAND_ECC_UNC_EXT_ADDR) &
					0xffff) << 32);
			if (*err_addr)
				ret = -EBADMSG;
		}

		if (!ret) {
			*err_addr = reg_read(nfc, NAND_ECC_UNC_ADDR) |
				((u64)(reg_read(nfc, NAND_ECC_UNC_EXT_ADDR) &
					0xffff) << 32);
			if (*err_addr)
				ret = -EUCLEAN;
		}
	}

	return ret;
}

static int brcmstb_nand_verify_erased_page(struct mtd_info *mtd,
		struct nand_chip *chip, void *buf, u64 addr)
{
	int i, sas, oob_nbits, data_nbits;
	void *oob = chip->oob_poi;
	unsigned int max_bitflips = 0;
	int page = addr >> chip->page_shift;
	int ret;

	if (!buf) {
		buf = chip->buffers->databuf;
		chip->pagebuf = -1;
	}

	sas = mtd->oobsize / chip->ecc.steps;
	oob_nbits = sas << 3;
	data_nbits = chip->ecc.size << 3;

	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
	ret = chip->ecc.read_page_raw(mtd, chip, buf, true, page);
	if (ret)
		return ret;

	for (i = 0; i < chip->ecc.steps; i++, oob += sas) {
		unsigned int bitflips = 0;

		bitflips += oob_nbits - bitmap_weight(oob, oob_nbits);
		bitflips += data_nbits - bitmap_weight(buf, data_nbits);

		buf += chip->ecc.size;
		addr += chip->ecc.size;

		if (bitflips > chip->ecc.strength)
			return -EBADMSG;

		max_bitflips = max(max_bitflips, bitflips);
	}

	return max_bitflips;
}

static int brcmstb_nand_read(struct mtd_info *mtd, struct nand_chip *chip,
		u64 addr, unsigned int trans, u32 *buf, u8 *oob)
{
	u64 err_addr;
	int err;

	if (oob)
		memset(oob, 0x99, mtd->oobsize);

	err = brcmstb_nand_read_by_pio(mtd, chip, addr, trans, buf, oob,
			&err_addr);

	if (mtd_is_eccerr(err)) {
		int ret = brcmstb_nand_verify_erased_page(mtd, chip, buf, addr);
		if (ret < 0) {
			mtd->ecc_stats.failed++;
			return 0;
		} else {
			if (buf)
				memset(buf, 0xff, FC_BYTES * trans);
			if (oob)
				memset(oob, 0xff, mtd->oobsize);
		}
		return ret;
	}

	if (mtd_is_bitflip(err)) {
		mtd->ecc_stats.corrected++;
		return 1;
	}

	return 0;
}

static int brcmstb_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
		u8 *buf, int oob_required, int page)
{
	struct brcmstb_nfc *nfc = chip->priv;
	u8 *oob = oob_required ? chip->oob_poi : NULL;

	return brcmstb_nand_read(mtd, chip,
			nfc->last_addr, mtd->writesize >> FC_SHIFT,
			(u32 *)buf, oob);
}

static int brcmstb_nand_read_page_raw(struct mtd_info *mtd,
		struct nand_chip *chip, u8 *buf, int oob_required, int page)
{
	struct brcmstb_nfc *nfc = chip->priv;
	u8 *oob = oob_required ? chip->oob_poi : NULL;
	int ret;

	acc_control_write(nfc, 0x80000000, 0);
	acc_control_write(nfc, 0x000f0000, 0);
	ret = brcmstb_nand_read(mtd, chip,
			nfc->last_addr, mtd->writesize >> FC_SHIFT,
			(u32 *)buf, oob);
	acc_control_write(nfc, 0x000f0000, nfc->hwcfg.ecc_level << 16);
	acc_control_write(nfc, 0x80000000, 0x80000000);

	return ret;
}

static int brcmstb_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
		int page)
{
	return brcmstb_nand_read(mtd, chip,
			(u64)page << chip->page_shift,
			mtd->writesize >> FC_SHIFT, NULL, chip->oob_poi);
}

static int brcmstb_nand_read_oob_raw(struct mtd_info *mtd,
		struct nand_chip *chip, int page)
{
	struct brcmstb_nfc *nfc = chip->priv;
	int ret;

	acc_control_write(nfc, 0x80000000, 0);
	acc_control_write(nfc, 0x000f0000, 0);
	ret = brcmstb_nand_read(mtd, chip,
			(u64)page << chip->page_shift,
			mtd->writesize >> FC_SHIFT, NULL, chip->oob_poi);
	acc_control_write(nfc, 0x000f0000, nfc->hwcfg.ecc_level << 16);
	acc_control_write(nfc, 0x80000000, 0x80000000);

	return ret;
}

static int brcmstb_nand_read_subpage(struct mtd_info *mtd,
		struct nand_chip *chip, u32 data_offs, u32 readlen, u8 *bufpoi,
		int page)
{
	struct brcmstb_nfc *nfc = chip->priv;

	return brcmstb_nand_read(mtd, chip,
			nfc->last_addr + data_offs, readlen >> FC_SHIFT,
			(u32 *)bufpoi, NULL);
}

static int brcmstb_nand_write(struct mtd_info *mtd, struct nand_chip *chip,
		u64 addr, const u32 *buf, u8 *oob)
{
	struct brcmstb_nfc *nfc = chip->priv;
	unsigned int i = 0, j, trans = mtd->writesize >> FC_SHIFT;
	int status, ret = 0;

	if (unlikely((u32)buf & 0x03)) {
		pr_warn("unaligned buffer: %p\n", buf);
		buf = (u32 *)((u32)buf & ~0x03);
	}

	for (j = 0; j < MAX_CONTROLLER_OOB; j += 4)
		oob_reg_write(nfc, j, 0xffffffff);

	reg_write(nfc, NAND_CMD_EXT_ADDRESS,
			(nfc->cs << 16) | ((addr >> 32) & 0xffff));

	for (; i < trans; i++, addr += FC_BYTES) {
		reg_write(nfc, NAND_CMD_ADDRESS, addr & 0xffffffff);

		if (buf)
			for (j = 0; j < FC_WORDS; j++, buf++)
				reg_write(nfc, FC(j), cpu_to_le32(*buf));
		else if (oob)
			for (j = 0; j < FC_WORDS; j++)
				reg_write(nfc, FC(j), 0xffffffff);

		if (oob)
			oob += write_oob_to_regs(nfc, i, oob,
					mtd->oobsize / trans,
					nfc->hwcfg.sector_size_1k);

		brcmstb_nand_send_cmd(nfc, CMD_PROGRAM_PAGE);
		status = brcmstb_nand_waitfunc(mtd, chip);

		if (status & NAND_STATUS_FAIL) {
			pr_info("program failed at %llx\n", addr);
			ret = -EIO;
			goto out;
		}
	}

out:
	return ret;
}

static int brcmstb_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
		const u8 *buf, int oob_required)
{
	struct brcmstb_nfc *nfc = chip->priv;
	u8 *oob = oob_required ? chip->oob_poi : NULL;

	return brcmstb_nand_write(mtd, chip, nfc->last_addr, (u32 *)buf, oob);
}

static int brcmstb_nand_write_page_raw(struct mtd_info *mtd,
				       struct nand_chip *chip,
				       const u8 *buf, int oob_required)
{
	struct brcmstb_nfc *nfc = chip->priv;
	u8 *oob = oob_required ? chip->oob_poi : NULL;
	int ret;

	acc_control_write(nfc, 0x40000000, 0);
	ret = brcmstb_nand_write(mtd, chip, nfc->last_addr, (u32 *)buf, oob);
	acc_control_write(nfc, 0x40000000, 0x40000000);

	return ret;
}

static int brcmstb_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
		int page)
{
	return brcmstb_nand_write(mtd, chip, (u64)page << chip->page_shift,
			NULL, chip->oob_poi);
}

static int brcmstb_nand_write_oob_raw(struct mtd_info *mtd,
				      struct nand_chip *chip, int page)
{
	struct brcmstb_nfc *nfc = chip->priv;
	int ret;

	acc_control_write(nfc, 0x40000000, 0);
	ret = brcmstb_nand_write(mtd, chip, (u64)page << chip->page_shift,
			NULL, chip->oob_poi);
	acc_control_write(nfc, 0x40000000, 0x40000000);

	return ret;
}

static const unsigned int block_sizes[] = { 16, 128, 8, 512, 256, 1024, 2048 };
static const unsigned int page_sizes[] = { 512, 2048, 4096, 8192 };

static void brcmstb_nand_set_cfg(struct brcmstb_nfc *nfc,
		struct brcmstb_nand_cfg *cfg)
{
	int i, found;

	for (i = 0, found = 0; i < ARRAY_SIZE(block_sizes); i++)
		if ((block_sizes[i] << 10) == cfg->block_size) {
			config_reg_write(nfc, 0x70000000, (i << 28));
			found = 1;
		}

	if (!found)
		pr_warn("invalid block size %u\n", cfg->block_size);

	for (i = 0, found = 0; i < ARRAY_SIZE(page_sizes); i++)
		if (page_sizes[i] == cfg->page_size) {
			config_reg_write(nfc, 0x00300000, (i << 20));
			found = 1;
		}

	if (!found)
		pr_warn("invalid page size %u\n", cfg->page_size);

	if (fls64(cfg->device_size) < 23)
		pr_warn("invalid device size 0x%llx\n",
			(unsigned long long)cfg->device_size);

	config_reg_write(nfc, 0x0f000000,
			   (fls64(cfg->device_size) - 23) << 24);
	config_reg_write(nfc, 0x00800000,
			   (cfg->device_width == 16 ? 1 : 0) << 23);
	config_reg_write(nfc, 0x00007000, (cfg->col_adr_bytes) << 12);
	config_reg_write(nfc, 0x00000700, (cfg->blk_adr_bytes) << 8);
	config_reg_write(nfc, 0x00070000, (cfg->ful_adr_bytes) << 16);

	acc_control_write(nfc, 0x0000003f, cfg->spare_area_size);
	acc_control_write(nfc, 0x00000040, (cfg->sector_size_1k) << 6);
	acc_control_write(nfc, 0x000f0000, (cfg->ecc_level) << 16);

	reg_write(nfc, NAND_CORR_STAT_THRESHOLD,
			((cfg->ecc_level << cfg->sector_size_1k) * 3 + 2) / 4);
}

static void brcmstb_nand_get_cfg(struct brcmstb_nfc *nfc,
		struct brcmstb_nand_cfg *cfg)
{
	u32 val;

	val = config_reg_read(nfc);

	cfg->block_size = (val & 0x70000000) >> 28;
	cfg->page_size = (val & 0x00300000) >> 20;
	cfg->device_size = (4ULL << 20) << ((val & 0x0f000000) >> 24);
	cfg->device_width = ((val & 0x00800000) >> 23) ? 16 : 8;
	cfg->col_adr_bytes = (val & 0x00007000) >> 12;
	cfg->blk_adr_bytes = (val & 0x00000700) >> 8;
	cfg->ful_adr_bytes = (val & 0x00070000) >> 16;

	val = acc_control_read(nfc);

	cfg->spare_area_size = (val & 0x0000003f);
	cfg->sector_size_1k = (val & 0x00000040) >> 6;
	cfg->ecc_level = (val & 0x000f0000) >> 16;

	if (cfg->block_size < ARRAY_SIZE(block_sizes))
		cfg->block_size = block_sizes[cfg->block_size] << 10;
	else
		cfg->block_size = 128 << 10;

	if (cfg->page_size < ARRAY_SIZE(page_sizes))
		cfg->page_size = page_sizes[cfg->page_size];
	else
		cfg->page_size = 2048;
}

static bool brcmstb_nand_config_match(struct brcmstb_nand_cfg *orig,
		struct brcmstb_nand_cfg *new)
{
	if (orig->device_size != new->device_size)
		return false;
	if (orig->block_size != new->block_size)
		return false;
	if (orig->page_size != new->page_size)
		return false;
	if (orig->device_width != new->device_width)
		return false;
	if (orig->col_adr_bytes != new->col_adr_bytes)
		return false;
	if (orig->blk_adr_bytes != new->blk_adr_bytes)
		return false;
	if (orig->ful_adr_bytes != new->ful_adr_bytes)
		return false;

	if (orig->spare_area_size == new->spare_area_size)
		return true;

	return orig->spare_area_size >= 27 &&
		orig->spare_area_size <= new->spare_area_size;
}

static int brcmstb_nand_ecc_init(struct brcmstb_nfc *nfc,
		struct device_node *np)
{
	struct mtd_info *mtd = &nfc->mtd;
	struct nand_chip *chip = &nfc->chip;
	struct brcmstb_nand_cfg orig_cfg, new_cfg;
	int strength;
	int blk_size;

	brcmstb_nand_get_cfg(nfc, &orig_cfg);
	nfc->hwcfg = orig_cfg;

	memset(&new_cfg, 0, sizeof(new_cfg));
	new_cfg.device_size = mtd->size;
	new_cfg.block_size = mtd->erasesize;
	new_cfg.page_size = mtd->writesize;
	new_cfg.spare_area_size = mtd->oobsize / (mtd->writesize >> FC_SHIFT);
	new_cfg.device_width = (chip->options & NAND_BUSWIDTH_16) ? 16 : 8;
	new_cfg.col_adr_bytes = 2;

	if (mtd->writesize > 512)
		if (mtd->size >= (256 << 20))
			new_cfg.blk_adr_bytes = 3;
		else
			new_cfg.blk_adr_bytes = 2;
	else if (mtd->size >= (64 << 20))
		new_cfg.blk_adr_bytes = 3;
	else
		new_cfg.blk_adr_bytes = 2;

	new_cfg.ful_adr_bytes = new_cfg.blk_adr_bytes + new_cfg.col_adr_bytes;

	if (new_cfg.spare_area_size > MAX_CONTROLLER_OOB)
		new_cfg.spare_area_size = MAX_CONTROLLER_OOB;

	if (!brcmstb_nand_config_match(&orig_cfg, &new_cfg)) {
		new_cfg.sector_size_1k = (new_cfg.page_size >= 1024) ? 1 : 0;

		acc_control_write(nfc, 0x80000000, 0x80000000);
		acc_control_write(nfc, 0x40000000, 0x40000000);

		if (new_cfg.spare_area_size >= 21)
			new_cfg.ecc_level = 12;
		else if (chip->badblockpos == NAND_SMALL_BADBLOCK_POS)
			new_cfg.ecc_level = 5;
		else
			new_cfg.ecc_level = 8;

		brcmstb_nand_set_cfg(nfc, &new_cfg);
		nfc->hwcfg = new_cfg;
	}

	acc_control_write(nfc, 0x10000000, 0);
	acc_control_write(nfc, 0x08000000, 0);
	acc_control_write(nfc, 0x04000000, 0);
	acc_control_write(nfc, 0x01000000, 0x01000000);

	chip->ecc.read_page = brcmstb_nand_read_page;
	chip->ecc.read_page_raw = brcmstb_nand_read_page_raw;
	chip->ecc.read_oob = brcmstb_nand_read_oob;
	chip->ecc.read_oob_raw = brcmstb_nand_read_oob_raw;
	chip->ecc.read_subpage = brcmstb_nand_read_subpage;
	chip->ecc.write_page = brcmstb_nand_write_page;
	chip->ecc.write_page_raw = brcmstb_nand_write_page_raw;
	chip->ecc.write_oob = brcmstb_nand_write_oob;
	chip->ecc.write_oob_raw = brcmstb_nand_write_oob_raw;

	chip->ecc.mode = NAND_ECC_HW;
	blk_size = of_get_nand_ecc_step_size(np);
	strength = of_get_nand_ecc_strength(np);
	if (blk_size > 0 && strength > 0) {
		chip->ecc.size = blk_size << nfc->hwcfg.sector_size_1k;
		chip->ecc.strength = strength << nfc->hwcfg.sector_size_1k;
	} else {
		chip->ecc.size = chip->ecc_step_ds;
		chip->ecc.strength = chip->ecc_strength_ds;
	}
	mtd->bitflip_threshold = 1;

	chip->ecc.layout = brcmstb_choose_ecc_layout(nfc);
	if (!chip->ecc.layout)
		return -ENOMEM;

	return 0;
}

static int brcmstb_nand_probe(struct platform_device *pdev)
{
	struct mtd_part_parser_data ppdata;
	struct device *dev = &pdev->dev;
	struct brcmstb_nfc *nfc;
	unsigned int irq;
	u32 val;

	nfc = devm_kzalloc(dev, sizeof(*nfc), GFP_KERNEL);
	if (IS_ERR(nfc))
		return PTR_ERR(nfc);

	nfc->nand_regs = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(nfc->nand_regs))
		return PTR_ERR(nfc->nand_regs);

	val = reg_read(nfc, NAND_CS_NAND_SELECT);
	val &= ~0x40000000;
	reg_write(nfc, NAND_CS_NAND_SELECT, val);

	val = reg_read(nfc, NAND_CS_NAND_SELECT);
	val &= ~0x000000ff;
	reg_write(nfc, NAND_CS_NAND_SELECT, val);

	val = reg_read(nfc, NAND_CS_NAND_XOR);
	val &= ~0x000000ff;
	reg_write(nfc, NAND_CS_NAND_XOR, val);

	init_completion(&nfc->done);
	spin_lock_init(&nfc->controller.lock);
	init_waitqueue_head(&nfc->controller.wq);

	nfc->nand_intr_regs = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 1));
	if (IS_ERR(nfc->nand_intr_regs))
		return PTR_ERR(nfc->nand_intr_regs);

	val = __raw_readl(nfc->nand_intr_regs + NAND_INTR_CPU_MASK_CLEAR);
	val |= NAND_INTR_CTLRDY_MASK;
	__raw_writel(val, nfc->nand_intr_regs + NAND_INTR_CPU_MASK_CLEAR);
	__raw_readl(nfc->nand_intr_regs + NAND_INTR_CPU_MASK_CLEAR);

	val = __raw_readl(nfc->nand_intr_regs + NAND_INTR_CPU_CLEAR);
	val |= NAND_INTR_CTLRDY_MASK;
	__raw_writel(val, nfc->nand_intr_regs + NAND_INTR_CPU_CLEAR);
	__raw_readl(nfc->nand_intr_regs + NAND_INTR_CPU_CLEAR);

	irq = of_irq_get(dev->of_node, 0);
	if (devm_request_irq(dev, irq, brcmstb_nand_irq, IRQF_SHARED,
			     pdev->name, nfc))
		return -ENOMEM;

	if (of_property_read_u32(dev->of_node, "nand-cs", &nfc->cs))
		nfc->cs = 0;

	platform_set_drvdata(pdev, nfc);

	nfc->mtd.priv = &nfc->chip;
	nfc->mtd.name = "brcmnand";
	nfc->mtd.owner = THIS_MODULE;
	nfc->mtd.dev.parent = &pdev->dev;

	nfc->chip.priv = nfc;
	nfc->chip.controller = &nfc->controller;
	nfc->chip.IO_ADDR_R = (void *)0xdeadbeef;
	nfc->chip.IO_ADDR_W = (void *)0xdeadbeef;
	nfc->chip.cmd_ctrl = brcmstb_nand_cmd_ctrl;
	nfc->chip.cmdfunc = brcmstb_nand_cmdfunc;
	nfc->chip.waitfunc = brcmstb_nand_waitfunc;
	nfc->chip.read_byte = brcmstb_nand_read_byte;
	nfc->chip.read_buf = brcmstb_nand_read_buf;
	nfc->chip.options |= NAND_NO_SUBPAGE_WRITE;

	if (of_get_nand_on_flash_bbt(dev->of_node))
		nfc->chip.bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;

	if (nand_scan_ident(&nfc->mtd, 1, NULL))
		return -ENXIO;

	if (brcmstb_nand_ecc_init(nfc, dev->of_node))
		return -ENOMEM;

	if (nand_scan_tail(&nfc->mtd))
		return -ENXIO;

	ppdata.of_node = dev->of_node;
	mtd_device_parse_register(&nfc->mtd, NULL, &ppdata, NULL, 0);

	return 0;
}

static int brcmstb_nand_remove(struct platform_device *pdev)
{
	struct brcmstb_nfc *nfc = platform_get_drvdata(pdev);

	kfree(nfc->chip.ecc.layout);
	nand_release(&nfc->mtd);

	return 0;
}

static const struct of_device_id of_device_match[] = {
	{ .compatible = "brcm,brcmstb-nand" },
	{ },
};

MODULE_DEVICE_TABLE(of, of_device_match);

static struct platform_driver brcmstb_nand_driver = {
	.probe	= brcmstb_nand_probe,
	.remove	= brcmstb_nand_remove,
	.driver	= {
		.name = "brcmstb-nand",
		.of_match_table = of_device_match,
	},
};

module_platform_driver(brcmstb_nand_driver);
