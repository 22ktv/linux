/*
 * Driver for Broadcom BRCMSTB, NSP,  NS2, Cygnus SPI Controllers
 *
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/cfi.h>
#include <linux/mtd/spi-nor.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#define DRIVER_NAME "bcm_qspi"

#define STATE_IDLE				0
#define STATE_RUNNING				1
#define STATE_SHUTDOWN				2

/* BSPI register offsets */
#define BSPI_REVISION_ID			0x000
#define BSPI_SCRATCH				0x004
#define BSPI_MAST_N_BOOT_CTRL			0x008
#define BSPI_BUSY_STATUS			0x00c
#define BSPI_INTR_STATUS			0x010
#define BSPI_B0_STATUS				0x014
#define BSPI_B0_CTRL				0x018
#define BSPI_B1_STATUS				0x01c
#define BSPI_B1_CTRL				0x020
#define BSPI_STRAP_OVERRIDE_CTRL		0x024
#define BSPI_FLEX_MODE_ENABLE			0x028
#define BSPI_BITS_PER_CYCLE			0x02c
#define BSPI_BITS_PER_PHASE			0x030
#define BSPI_CMD_AND_MODE_BYTE			0x034
#define BSPI_BSPI_FLASH_UPPER_ADDR_BYTE	0x038
#define BSPI_BSPI_XOR_VALUE			0x03c
#define BSPI_BSPI_XOR_ENABLE			0x040
#define BSPI_BSPI_PIO_MODE_ENABLE		0x044
#define BSPI_BSPI_PIO_IODIR			0x048
#define BSPI_BSPI_PIO_DATA			0x04c

/* RAF register offsets */
#define BSPI_RAF_START_ADDR			0x100
#define BSPI_RAF_NUM_WORDS			0x104
#define BSPI_RAF_CTRL				0x108
#define BSPI_RAF_FULLNESS			0x10c
#define BSPI_RAF_WATERMARK			0x110
#define BSPI_RAF_STATUS			0x114
#define BSPI_RAF_READ_DATA			0x118
#define BSPI_RAF_WORD_CNT			0x11c
#define BSPI_RAF_CURR_ADDR			0x120

/* MSPI register offsets */
#define MSPI_SPCR0_LSB				0x000
#define MSPI_SPCR0_MSB				0x004
#define MSPI_SPCR1_LSB				0x008
#define MSPI_SPCR1_MSB				0x00c
#define MSPI_NEWQP				0x010
#define MSPI_ENDQP				0x014
#define MSPI_SPCR2				0x018
#define MSPI_MSPI_STATUS			0x020
#define MSPI_CPTQP				0x024
#define MSPI_SPCR3				0x028
#define MSPI_TXRAM				0x040
#define MSPI_RXRAM				0x0c0
#define MSPI_CDRAM				0x140
#define MSPI_WRITE_LOCK			0x180

#define MSPI_MASTER_BIT			BIT(7)

#define MSPI_NUM_CDRAM				16
#define MSPI_CDRAM_CONT_BIT			BIT(7)
#define MSPI_CDRAM_BITSE_BIT			BIT(6)
#define MSPI_CDRAM_PCS				0xf

#define MSPI_SPCR2_SPE				BIT(6)
#define MSPI_SPCR2_CONT_AFTER_CMD		BIT(7)

#define MSPI_MSPI_STATUS_SPIF			BIT(0)

#define BSPI_ADDRLEN_3BYTES			3
#define BSPI_ADDRLEN_4BYTES			4

#define BSPI_RAF_STATUS_FIFO_EMPTY_MASK	BIT(1)

#define BSPI_RAF_CTRL_START_MASK		BIT(0)
#define BSPI_RAF_CTRL_CLEAR_MASK		BIT(1)

#define BSPI_BPP_MODE_SELECT_MASK		BIT(8)
#define BSPI_BPP_ADDR_SELECT_MASK		BIT(16)

/* HIF INTR2 offsets */
#define HIF_SPI_INTR2_CPU_STATUS		0x00
#define HIF_SPI_INTR2_CPU_SET			0x04
#define HIF_SPI_INTR2_CPU_CLEAR		0x08
#define HIF_SPI_INTR2_CPU_MASK_STATUS		0x0c
#define HIF_SPI_INTR2_CPU_MASK_SET		0x10
#define HIF_SPI_INTR2_CPU_MASK_CLEAR		0x14

#define INTR_BASE_BIT_SHIFT			0x02
#define INTR_COUNT				0x07

/* MSPI Interrupt masks */
#define INTR_MSPI_HALTED_MASK			BIT(6)
#define INTR_MSPI_DONE_MASK                     BIT(5)

/* BSPI interrupt masks */
#define INTR_BSPI_LR_OVERREAD_MASK		BIT(4)
#define INTR_BSPI_LR_SESSION_DONE_MASK		BIT(3)
#define INTR_BSPI_LR_IMPATIENT_MASK		BIT(2)
#define INTR_BSPI_LR_SESSION_ABORTED_MASK	BIT(1)
#define INTR_BSPI_LR_FULLNESS_REACHED_MASK	BIT(0)

/* Override mode masks */
#define BSPI_STRAP_OVERRIDE_CTRL_OVERRIDE	BIT(0)
#define BSPI_STRAP_OVERRIDE_CTRL_DATA_DUAL	BIT(1)
#define BSPI_STRAP_OVERRIDE_CTRL_ADDR_4BYTE	BIT(2)
#define BSPI_STRAP_OVERRIDE_CTRL_DATA_QUAD	BIT(3)
#define BSPI_STRAP_OVERRIDE_CTRL_ENDAIN_MODE	BIT(4)

#define MSPI_INTERRUPTS_ALL			\
	(INTR_MSPI_DONE_MASK |			\
	 INTR_MSPI_HALTED_MASK)

#define BSPI_LR_INTERRUPTS_DATA		\
	(INTR_BSPI_LR_SESSION_DONE_MASK |	\
	 INTR_BSPI_LR_FULLNESS_REACHED_MASK)

#define BSPI_LR_INTERRUPTS_ERROR		\
	(INTR_BSPI_LR_OVERREAD_MASK |		\
	 INTR_BSPI_LR_IMPATIENT_MASK |		\
	 INTR_BSPI_LR_SESSION_ABORTED_MASK)

#define BSPI_LR_INTERRUPTS_ALL			\
	(BSPI_LR_INTERRUPTS_ERROR |		\
	 BSPI_LR_INTERRUPTS_DATA)

#define QSPI_INTERRUPTS_ALL			\
	(MSPI_INTERRUPTS_ALL |			\
	 BSPI_LR_INTERRUPTS_ALL)

#define NUM_CHIPSELECT				4
#define MSPI_BASE_FREQ				27000000UL
#define QSPI_SPBR_MIN				8U
#define QSPI_SPBR_MAX				255U
#define MAX_SPEED_HZ		(MSPI_BASE_FREQ / (QSPI_SPBR_MIN * 2))

#define OPCODE_DIOR				0xBB
#define OPCODE_QIOR				0xEB
#define OPCODE_DIOR_4B				0xBC
#define OPCODE_QIOR_4B				0xEC

#define	MAX_CMD_SIZE				6

#define DWORD_ALIGNED(a)			IS_ALIGNED((uintptr_t)(a), 4)
#define ADDR_TO_4MBYTE_SEGMENT(addr)		(((u32)(addr)) >> 22)

struct bcm_qspi_parms {
	u32 speed_hz;
	u8 mode;
	u8 bits_per_word;
};

static const struct bcm_qspi_parms bcm_qspi_default_parms_cs0 = {
	.speed_hz = MAX_SPEED_HZ,
	.mode = SPI_MODE_3,
	.bits_per_word = 8,
};

struct bcm_xfer_mode {
	bool flex_mode;
	unsigned int width;
	unsigned int addrlen;
	unsigned int hp;
};

enum base_type {
	MSPI,
	BSPI,
	INTR,
	INTR_STATUS,
	CHIP_SELECT,
	BASEMAX,
};

struct bcm_qspi_irq {
	const char *irq_name;
	const irq_handler_t irq_handler;
	u32 mask;
};

struct bcm_qspi_dev_id {
	const struct bcm_qspi_irq *irqp;
	void *dev;
};

struct position {
	struct spi_transfer	*trans;
	int			byte;
};

struct bcm_qspi {
	struct platform_device *pdev;
	struct spi_master *master;
	struct clk *clk;
	u32 base_clk;
	u32 max_speed_hz;
	void __iomem *base[BASEMAX];
	struct bcm_qspi_parms last_parms;
	struct position  pos;
	int state;
	int next_udelay;
	int cs_change;
	int curr_cs;
	int bspi_maj_rev;
	int bspi_min_rev;
	int bspi_enabled;
	int bspi_cs_bmap;
	struct spi_flash_read_message *bspi_rf_msg;
	u32 bspi_rf_msg_idx;
	u32 bspi_rf_msg_len;
	u32 bspi_rf_msg_status;
	struct bcm_xfer_mode xfer_mode;
	u32 s3_intr2_mask;
	u32 s3_strap_override_ctrl;
	bool hif_spi_mode;
	bool bspi_mode;
	bool probed_trans_mode;
	bool use_l2_intc;
	int num_irqs;
	struct bcm_qspi_dev_id *dev_ids;
	struct completion mspi_done;
	struct completion bspi_done;
};

static inline bool has_bspi(struct bcm_qspi *qspi)
{
	return qspi->bspi_mode;
}

/* Read qspi controller register*/
static inline u32 bcm_qspi_read(struct bcm_qspi *qspi, enum base_type type,
				   unsigned int offset)
{
	/*
	 * MIPS endianness is configured by boot strap, which also reverses all
	 * bus endianness (i.e., big-endian CPU + big endian bus ==> native
	 * endian I/O).
	 *
	 * Other architectures (e.g., ARM) either do not support big endian, or
	 * else leave I/O in little endian mode.
	 */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		return ioread32be(qspi->base[type] + offset);
	else
		return readl_relaxed(qspi->base[type] + offset);

}

/* Write qspi controller register*/
static inline void bcm_qspi_write(struct bcm_qspi *qspi, enum base_type type,
			 unsigned int offset, unsigned int data)
{
	/* See brcm_mspi_readl() comments */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		iowrite32be(data, (qspi->base[type] + offset));
	else
		writel_relaxed(data, (qspi->base[type] + offset));
}

/* Interrupt helpers when not using brcm intc driver */
static void bcm_qspi_enable_interrupt(struct bcm_qspi *qspi, u32 mask)
{
	unsigned int val;

	if (qspi->hif_spi_mode) {
		bcm_qspi_write(qspi, INTR, HIF_SPI_INTR2_CPU_MASK_CLEAR, mask);
	} else {
		val = bcm_qspi_read(qspi, INTR, 0);
		val = val | (mask << INTR_BASE_BIT_SHIFT);
		bcm_qspi_write(qspi, INTR, 0, val);
	}
}

static void bcm_qspi_disable_interrupt(struct bcm_qspi *qspi, u32 mask)
{
	unsigned int val;

	if (qspi->hif_spi_mode) {
		bcm_qspi_write(qspi, INTR, HIF_SPI_INTR2_CPU_MASK_SET, mask);
	} else {
		val = bcm_qspi_read(qspi, INTR, 0);
		val = val & ~(mask << INTR_BASE_BIT_SHIFT);
		bcm_qspi_write(qspi, INTR, 0, val);
	}
}

static void bcm_qspi_clear_interrupt(struct bcm_qspi *qspi, u32 mask)
{
	unsigned int val;

	if (qspi->hif_spi_mode) {
		bcm_qspi_write(qspi, INTR_STATUS,
			       HIF_SPI_INTR2_CPU_CLEAR, mask);
	} else {
		for (val = 0; val < INTR_COUNT; val++) {
			if (mask & (1UL << val))
				bcm_qspi_write(qspi, INTR_STATUS,
							(val * 4), 1);
		}
	}
}

static u32 bcm_qspi_read_l2int_status(struct bcm_qspi *qspi)
{
	unsigned int val = 0;
	unsigned int i = 0;

	WARN_ON(!qspi->base[INTR_STATUS]);

	if (qspi->hif_spi_mode) {
		val = bcm_qspi_read(qspi, INTR_STATUS,
				    HIF_SPI_INTR2_CPU_STATUS);
	} else {
		for (i = 0; i < INTR_COUNT; i++) {
			if (bcm_qspi_read(qspi, INTR_STATUS, (i * 4)))
				val |= 1UL << i;
		}
	}
	return val;
}

static int bcm_qspi_bspi_busy_poll(struct bcm_qspi *qspi)
{
	int i;

	/* this should normally finish within 10us */
	for (i = 0; i < 1000; i++) {
		if (!(bcm_qspi_read(qspi, BSPI, BSPI_BUSY_STATUS) & 1))
			return 0;
		udelay(1);
	}
	dev_warn(&qspi->pdev->dev, "timeout waiting for !busy_status\n");
	return -EIO;
}

static inline bool bcm_qspi_bspi_ver_three(struct bcm_qspi *qspi)
{
	if (qspi->bspi_maj_rev < 4)
		return true;
	return false;
}

static void bcm_qspi_flush_prefetch_buffers(struct bcm_qspi *qspi)
{
	bcm_qspi_bspi_busy_poll(qspi);
	/* Force rising edge for the b0/b1 'flush' field */
	bcm_qspi_write(qspi, BSPI, BSPI_B0_CTRL, 1);
	bcm_qspi_write(qspi, BSPI, BSPI_B1_CTRL, 1);
	bcm_qspi_write(qspi, BSPI, BSPI_B0_CTRL, 0);
	bcm_qspi_write(qspi, BSPI, BSPI_B1_CTRL, 0);
}

static int bcm_qspi_lr_is_fifo_empty(struct bcm_qspi *qspi)
{
	return (bcm_qspi_read(qspi, BSPI, BSPI_RAF_STATUS) &
				BSPI_RAF_STATUS_FIFO_EMPTY_MASK);
}

static inline u32 bcm_qspi_lr_read_fifo(struct bcm_qspi *qspi)
{
	u32 data = bcm_qspi_read(qspi, BSPI, BSPI_RAF_READ_DATA);

	/* BSPI v3 LR is LE only, convert data to host endianness */
	if (bcm_qspi_bspi_ver_three(qspi))
		data = le32_to_cpu(data);

	return data;
}

static inline void bcm_qspi_lr_start(struct bcm_qspi *qspi)
{
	bcm_qspi_bspi_busy_poll(qspi);
	bcm_qspi_write(qspi, BSPI, BSPI_RAF_CTRL,
				BSPI_RAF_CTRL_START_MASK);
}

static inline void bcm_qspi_lr_clear(struct bcm_qspi *qspi)
{
	bcm_qspi_write(qspi, BSPI, BSPI_RAF_CTRL,
				BSPI_RAF_CTRL_CLEAR_MASK);
	bcm_qspi_flush_prefetch_buffers(qspi);
}

static void bcm_qspi_bspi_lr_data_read(struct bcm_qspi *qspi)
{
	u32 *buf = (u32 *)qspi->bspi_rf_msg->buf;
	u32 data = 0;

	pr_debug("xfer %p rx %p rxlen %d\n",
	    qspi->bspi_rf_msg, qspi->bspi_rf_msg->buf, qspi->bspi_rf_msg_len);
	while (!bcm_qspi_lr_is_fifo_empty(qspi)) {
		data = bcm_qspi_lr_read_fifo(qspi);
		if (likely(qspi->bspi_rf_msg_len >= 4) &&
		    likely(DWORD_ALIGNED(buf))) {
			buf[qspi->bspi_rf_msg_idx++] = data;
			qspi->bspi_rf_msg_len -= 4;
		} else {
			/* Read out remaining bytes, make sure*/
			u8 *cbuf = (u8 *)&buf[qspi->bspi_rf_msg_idx];

			data = cpu_to_le32(data);
			while (qspi->bspi_rf_msg_len) {
				*cbuf++ = (u8)data;
				data >>= 8;
				qspi->bspi_rf_msg_len--;
			}
		}
	}
}

static inline int bcm_qspi_is_4_byte_mode(struct bcm_qspi *qspi)
{
	return qspi->xfer_mode.addrlen == BSPI_ADDRLEN_4BYTES;
}

static void bcm_qspi_bspi_set_xfer_params(struct bcm_qspi *qspi, u8 cmd_byte,
					  int bpp, int bpc, int flex_mode)
{
	bcm_qspi_write(qspi, BSPI, BSPI_FLEX_MODE_ENABLE, 0);
	bcm_qspi_write(qspi, BSPI, BSPI_BITS_PER_CYCLE, bpc);
	bcm_qspi_write(qspi, BSPI, BSPI_BITS_PER_PHASE, bpp);
	bcm_qspi_write(qspi, BSPI, BSPI_CMD_AND_MODE_BYTE, cmd_byte);
	bcm_qspi_write(qspi, BSPI, BSPI_FLEX_MODE_ENABLE, flex_mode);
}

static int bcm_qspi_bspi_set_flex_mode(struct bcm_qspi *qspi, int width,
				       int addrlen, int hp)
{
	int bpc = 0, bpp = 0;
	u8 command = SPINOR_OP_READ_FAST;
	int flex_mode = 1, rv = 0;
	bool spans_4byte = false;

	pr_debug("set flex mode w %x addrlen %x hp %d\n", width, addrlen, hp);

	if (addrlen == BSPI_ADDRLEN_4BYTES) {
		bpp = BSPI_BPP_ADDR_SELECT_MASK;
		spans_4byte = true;
	}

	bpp |= 8;

	switch (width) {
	case SPI_NBITS_SINGLE:
		if (addrlen == BSPI_ADDRLEN_3BYTES)
			/* default mode, does not need flex_cmd */
			flex_mode = 0;
		else
			command = SPINOR_OP_READ4_FAST;
		break;
	case SPI_NBITS_DUAL:
		bpc = 0x00000001;
		if (hp) {
			bpc |= 0x00010100; /* address and mode are 2-bit */
			bpp = BSPI_BPP_MODE_SELECT_MASK;
			command = OPCODE_DIOR;
			if (spans_4byte == true)
				command = OPCODE_DIOR_4B;
		} else {
			command = SPINOR_OP_READ_1_1_2;
			if (spans_4byte == true)
				command = SPINOR_OP_READ4_1_1_2;
		}
		break;
	case SPI_NBITS_QUAD:
		bpc = 0x00000002;
		if (hp) {
			bpc |= 0x00020200; /* address and mode are 4-bit */
			bpp = 4; /* dummy cycles */
			bpp |= BSPI_BPP_ADDR_SELECT_MASK;
			command = OPCODE_QIOR;
			if (spans_4byte == true)
				command = OPCODE_QIOR_4B;
		} else {
			command = SPINOR_OP_READ_1_1_4;
			if (spans_4byte == true)
				command = SPINOR_OP_READ4_1_1_4;
		}
		break;
	default:
		rv = -EINVAL;
		break;
	}

	if (rv == 0)
		bcm_qspi_bspi_set_xfer_params(qspi, command, bpp, bpc,
					      flex_mode);

	return rv;
}

static int bcm_qspi_bspi_set_override(struct bcm_qspi *qspi, int width,
				      int addrlen, int hp)
{
	u32 data = bcm_qspi_read(qspi, BSPI, BSPI_STRAP_OVERRIDE_CTRL);

	pr_debug("set override mode w %x addrlen %x hp %d\n",
		 width, addrlen, hp);

	switch (width) {
	case SPI_NBITS_QUAD:
		/* clear dual mode and set quad mode */
		data &= ~BSPI_STRAP_OVERRIDE_CTRL_DATA_DUAL;
		data |= BSPI_STRAP_OVERRIDE_CTRL_DATA_QUAD;
		break;
	case SPI_NBITS_DUAL:
		/* clear quad mode set dual mode */
		data &= ~BSPI_STRAP_OVERRIDE_CTRL_DATA_QUAD;
		data |= BSPI_STRAP_OVERRIDE_CTRL_DATA_DUAL;
		break;
	case SPI_NBITS_SINGLE:
		/* clear quad/dual mode */
		data &= ~(BSPI_STRAP_OVERRIDE_CTRL_DATA_QUAD |
			  BSPI_STRAP_OVERRIDE_CTRL_DATA_DUAL);
		break;
	default:
		break;
	}

	if (addrlen == BSPI_ADDRLEN_4BYTES)
		/* set 4byte mode*/
		data |= BSPI_STRAP_OVERRIDE_CTRL_ADDR_4BYTE;
	else
		/* clear 4 byte mode */
		data &= ~BSPI_STRAP_OVERRIDE_CTRL_ADDR_4BYTE;

	/* set the override mode */
	data |=	BSPI_STRAP_OVERRIDE_CTRL_OVERRIDE;
	bcm_qspi_write(qspi, BSPI, BSPI_STRAP_OVERRIDE_CTRL, data);
	bcm_qspi_bspi_set_xfer_params(qspi, SPINOR_OP_READ_FAST, 0, 0, 0);

	return 0;
}

static int bcm_qspi_bspi_set_mode(struct bcm_qspi *qspi,
				   int width, int addrlen, int hp)
{
	int error = 0;

	if (width == -1)
		width = qspi->xfer_mode.width;
	if (addrlen == -1)
		addrlen = qspi->xfer_mode.addrlen;
	if (hp == -1)
		hp = qspi->xfer_mode.hp;

	if (width == -1 && addrlen == -1 && hp == -1)
		qspi->probed_trans_mode = false;

	/* default mode */
	qspi->xfer_mode.flex_mode = true;

	if (!bcm_qspi_bspi_ver_three(qspi)) {
		u32 val, mask;

		val = bcm_qspi_read(qspi, BSPI, BSPI_STRAP_OVERRIDE_CTRL);
		mask = BSPI_STRAP_OVERRIDE_CTRL_OVERRIDE;
		if (val & mask || qspi->s3_strap_override_ctrl & mask) {
			qspi->xfer_mode.flex_mode = false;
			bcm_qspi_write(qspi, BSPI, BSPI_FLEX_MODE_ENABLE,
				       0);

			if ((val | qspi->s3_strap_override_ctrl) &
			    BSPI_STRAP_OVERRIDE_CTRL_DATA_DUAL)
				width = SPI_NBITS_DUAL;
			else if ((val |  qspi->s3_strap_override_ctrl) &
				 BSPI_STRAP_OVERRIDE_CTRL_DATA_QUAD)
				width = SPI_NBITS_QUAD;

			error = bcm_qspi_bspi_set_override(qspi, width, addrlen,
							   hp);
		}
	}

	if (qspi->xfer_mode.flex_mode)
		error = bcm_qspi_bspi_set_flex_mode(qspi, width, addrlen, hp);

	if (error) {
		dev_warn(&qspi->pdev->dev,
			 "INVALID COMBINATION: width=%d addrlen=%d hp=%d\n",
			 width, addrlen, hp);
	} else if (qspi->xfer_mode.width != width ||
		   qspi->xfer_mode.addrlen != addrlen ||
		   qspi->xfer_mode.hp != hp) {
		qspi->xfer_mode.width = width;
		qspi->xfer_mode.addrlen = addrlen;
		qspi->xfer_mode.hp = hp;
		if (!qspi->probed_trans_mode) {
			dev_info(&qspi->pdev->dev,
				 "cs:%d %d-lane output, %d-byte address%s\n",
				 qspi->curr_cs,
				 qspi->xfer_mode.width,
				 qspi->xfer_mode.addrlen,
				 qspi->xfer_mode.hp != -1 ? ", hp mode" : "");
			qspi->probed_trans_mode = true;
		}
	}

	return error;
}

static void bcm_qspi_chip_select(struct bcm_qspi *qspi, int cs)
{
	u32 data = 0;

	if (qspi->curr_cs == cs)
		return;
	if (qspi->base[CHIP_SELECT]) {
		data = bcm_qspi_read(qspi, CHIP_SELECT, 0);
		data = (data & ~0xff) | (1 << cs);
		bcm_qspi_write(qspi, CHIP_SELECT, 0, data);
		udelay(10);
	}
	qspi->curr_cs = cs;
}

static void bcm_qspi_enable_bspi(struct bcm_qspi *qspi)
{

	if ((!qspi->base[BSPI]) || (qspi->bspi_enabled))
		return;

	qspi->bspi_enabled = 1;
	if ((bcm_qspi_read(qspi, BSPI, BSPI_MAST_N_BOOT_CTRL) & 1) == 0)
		return;

	bcm_qspi_flush_prefetch_buffers(qspi);
	udelay(1);
	bcm_qspi_write(qspi, BSPI, BSPI_MAST_N_BOOT_CTRL, 0);
	udelay(1);
}

static void bcm_qspi_disable_bspi(struct bcm_qspi *qspi)
{
	if ((!qspi->base[BSPI]) || (!qspi->bspi_enabled))
		return;

	qspi->bspi_enabled = 0;
	if ((bcm_qspi_read(qspi, BSPI, BSPI_MAST_N_BOOT_CTRL) & 1))
		return;

	bcm_qspi_bspi_busy_poll(qspi);
	bcm_qspi_write(qspi, BSPI, BSPI_MAST_N_BOOT_CTRL, 1);
	udelay(1);
}

static void bcm_qspi_hw_set_parms(struct bcm_qspi *qspi,
				  const struct bcm_qspi_parms *xp)
{
	u32 spcr, spbr = 0;

	if (xp->speed_hz)
		spbr = qspi->base_clk / (2 * xp->speed_hz);

	spcr = clamp_val(spbr, QSPI_SPBR_MIN, QSPI_SPBR_MAX);
	bcm_qspi_write(qspi, MSPI, MSPI_SPCR0_LSB, spcr);

	spcr = MSPI_MASTER_BIT;
	/* for 16 bit the data should be zero */
	if (xp->bits_per_word != 16)
		spcr |= xp->bits_per_word << 2;
	spcr |= xp->mode & 3;
	bcm_qspi_write(qspi, MSPI, MSPI_SPCR0_MSB, spcr);

	qspi->last_parms = *xp;
}

static void bcm_qspi_update_parms(struct bcm_qspi *qspi,
				 struct spi_device *spi,
				 struct spi_transfer *trans)
{
	struct bcm_qspi_parms xp;

	xp.speed_hz = trans->speed_hz;
	xp.bits_per_word = trans->bits_per_word;
	xp.mode = spi->mode;

	bcm_qspi_hw_set_parms(qspi, &xp);
}

static int bcm_qspi_setup(struct spi_device *spi)
{
	struct bcm_qspi_parms *xp;

	if (spi->bits_per_word > 16)
		return -EINVAL;

	xp = spi_get_ctldata(spi);
	if (!xp) {
		xp = kzalloc(sizeof(struct bcm_qspi_parms), GFP_KERNEL);
		if (!xp)
			return -ENOMEM;
		spi_set_ctldata(spi, xp);
	}
	xp->speed_hz = spi->max_speed_hz;
	xp->mode = spi->mode;
	xp->bits_per_word = spi->bits_per_word ? spi->bits_per_word : 8;

	return 0;
}

/* MSPI helpers */

/* stop at end of transfer, no other reason */
#define FNB_BREAK_NONE			0
/* stop at end of spi_message */
#define FNB_BREAK_EOM			1
/* stop at end of spi_transfer if delay */
#define FNB_BREAK_DELAY			2
/* stop at end of spi_transfer if cs_change */
#define FNB_BREAK_CS_CHANGE		4
/* stop if we run out of bytes */
#define FNB_BREAK_NO_BYTES		8

/* events that make us stop filling TX slots */
#define FNB_BREAK_TX			(FNB_BREAK_EOM | FNB_BREAK_DELAY | \
					 FNB_BREAK_CS_CHANGE)

/* events that make us deassert CS */
#define FNB_BREAK_DESELECT		(FNB_BREAK_EOM | FNB_BREAK_CS_CHANGE)

static int find_next_byte(struct bcm_qspi *qspi, struct position *p,
			  int flags)
{
	int ret = FNB_BREAK_NONE;

	if (p->trans->bits_per_word <= 8)
		p->byte++;
	else
		p->byte += 2;

	if (p->byte >= p->trans->len) {
		/* we're at the end of the spi_transfer */

		/* in TX mode, need to pause for a delay or CS change */
		if (p->trans->delay_usecs && (flags & FNB_BREAK_DELAY))
			ret |= FNB_BREAK_DELAY;
		if (p->trans->cs_change && (flags & FNB_BREAK_CS_CHANGE))
			ret |= FNB_BREAK_CS_CHANGE;
		if (ret)
			goto done;

		pr_debug("find_next_byte: advance msg exit\n");
		if (spi_transfer_is_last(qspi->master, p->trans))
			ret = FNB_BREAK_EOM;
		else
			ret = FNB_BREAK_NO_BYTES;

		p->trans = NULL;
	}

done:
	pr_debug("find_next_byte: trans %p len %d byte %d ret %x\n",
		p->trans, p->trans ? p->trans->len : 0, p->byte, ret);
	return ret;
}

static inline u8 read_rxram_slot_u8(struct bcm_qspi *qspi, int slot)
{
	u32 slot_offset = MSPI_RXRAM + (slot << 3) + 0x4;

	/* mask out reserved bits */
	return bcm_qspi_read(qspi, MSPI, slot_offset) & 0xff;
}

static inline u16 read_rxram_slot_u16(struct bcm_qspi *qspi, int slot)
{
	u32 reg_offset = MSPI_RXRAM;
	u32 lsb_offset = reg_offset + (slot << 3) + 0x4;
	u32 msb_offset = reg_offset + (slot << 3);

	return (bcm_qspi_read(qspi, MSPI, lsb_offset) & 0xff) |
		((bcm_qspi_read(qspi, MSPI, msb_offset) & 0xff) << 8);
}

static void read_from_hw(struct bcm_qspi *qspi, int slots)
{
	struct position p;
	int slot;

	bcm_qspi_disable_bspi(qspi);

	if (slots > MSPI_NUM_CDRAM) {
		/* should never happen */
		dev_err(&qspi->pdev->dev, "%s: too many slots!\n", __func__);
		return;
	}

	p = qspi->pos;

	for (slot = 0; slot < slots; slot++) {
		if (p.trans->bits_per_word <= 8) {
			u8 *buf = p.trans->rx_buf;

			if (buf)
				buf[p.byte] = read_rxram_slot_u8(qspi, slot);
			pr_debug("RD %02x\n", buf ? buf[p.byte] : 0xff);
		} else {
			u16 *buf = p.trans->rx_buf;

			if (buf)
				buf[p.byte / 2] = read_rxram_slot_u16(qspi,
								      slot);
			pr_debug("RD %04x\n", buf ? buf[p.byte] : 0xffff);
		}

		find_next_byte(qspi, &p, FNB_BREAK_NONE);
	}

	qspi->pos = p;
}

static inline void write_txram_slot_u8(struct bcm_qspi *qspi, int slot,
		u8 val)
{
	u32 reg_offset = MSPI_TXRAM + (slot << 3);

	/* mask out reserved bits */
	bcm_qspi_write(qspi, MSPI, reg_offset, val);
}

static inline void write_txram_slot_u16(struct bcm_qspi *qspi, int slot,
		u16 val)
{
	u32 reg_offset = MSPI_TXRAM;
	u32 msb_offset = reg_offset + (slot << 3);
	u32 lsb_offset = reg_offset + (slot << 3) + 0x4;

	bcm_qspi_write(qspi, MSPI, msb_offset, (val >> 8));
	bcm_qspi_write(qspi, MSPI, lsb_offset, (val & 0xff));
}

static inline u32 read_cdram_slot(struct bcm_qspi *qspi, int slot)
{
	return bcm_qspi_read(qspi, MSPI, MSPI_CDRAM + (slot << 2));
}

static inline void write_cdram_slot(struct bcm_qspi *qspi, int slot, u32 val)
{
	bcm_qspi_write(qspi, MSPI, (MSPI_CDRAM + (slot << 2)), val);
}

/* Return number of slots written */
static int write_to_hw(struct bcm_qspi *qspi, struct spi_device *spi)
{
	struct position p;
	int slot = 0, fnb = 0;
	u32 mspi_cdram = 0;

	bcm_qspi_disable_bspi(qspi);
	p = qspi->pos;
	bcm_qspi_update_parms(qspi, spi, p.trans);

	/* Run until end of transfer or reached the max data */
	while (!fnb && slot < MSPI_NUM_CDRAM) {
		if (p.trans->bits_per_word <= 8) {
			const u8 *buf = p.trans->tx_buf;
			u8 val = buf ? buf[p.byte] : 0xff;

			write_txram_slot_u8(qspi, slot, val);
			pr_debug("WR %02x\n", val);
		} else {
			const u16 *buf = p.trans->tx_buf;
			u16 val = buf ? buf[p.byte / 2] : 0xffff;

			write_txram_slot_u16(qspi, slot, val);
			pr_debug("WR %04x\n", val);
		}
		mspi_cdram = MSPI_CDRAM_CONT_BIT;
		mspi_cdram |= (~(1 << spi->chip_select) &
			       MSPI_CDRAM_PCS);
		mspi_cdram |= ((p.trans->bits_per_word <= 8) ? 0 :
				MSPI_CDRAM_BITSE_BIT);

		write_cdram_slot(qspi, slot, mspi_cdram);

		/* NOTE: This can update p.trans */
		fnb = find_next_byte(qspi, &p, FNB_BREAK_TX);
		slot++;
	}
	if (!slot) {
		dev_err(&qspi->pdev->dev, "%s: no data to send?", __func__);
		goto done;
	}

	/* in TX mode, need to pause for a delay or CS change */
	if (fnb & FNB_BREAK_CS_CHANGE)
		qspi->cs_change = 1;
	if (fnb & FNB_BREAK_DELAY)
		qspi->next_udelay = p.trans->delay_usecs;

	pr_debug("submitting %d slots\n", slot);
	bcm_qspi_write(qspi, MSPI, MSPI_NEWQP, 0);
	bcm_qspi_write(qspi, MSPI, MSPI_ENDQP, slot - 1);

	if (fnb & FNB_BREAK_DESELECT) {
		mspi_cdram = read_cdram_slot(qspi, slot - 1) &
			~MSPI_CDRAM_CONT_BIT;
		write_cdram_slot(qspi, slot - 1, mspi_cdram);
	}

	if (has_bspi(qspi))
		bcm_qspi_write(qspi, MSPI, MSPI_WRITE_LOCK, 1);

	/* Must flush previous writes before starting MSPI operation */
	mb();
	/* Set cont | spe | spifie */
	bcm_qspi_write(qspi, MSPI, MSPI_SPCR2, 0xe0);
	qspi->state = STATE_RUNNING;

done:
	return slot;
}

static void hw_stop(struct bcm_qspi *qspi)
{
	if (has_bspi(qspi))
		bcm_qspi_write(qspi, MSPI, MSPI_WRITE_LOCK, 0);
	qspi->state = STATE_IDLE;
}

/* BSPI helpers */
static int bcm_qspi_bspi_flash_read(struct spi_device *spi,
				    struct spi_flash_read_message *msg)
{
	struct bcm_qspi *qspi = spi_master_get_devdata(spi->master);
	u32 addr = 0, len, len_words;
	u8 *buf;
	int ret = 0;
	int retry = 3;
	unsigned long timeo = msecs_to_jiffies(100);

	if (bcm_qspi_bspi_ver_three(qspi))
		if (msg->addr_width == BSPI_ADDRLEN_4BYTES)
			return -EIO;

	bcm_qspi_chip_select(qspi, spi->chip_select);

	/*
	 * when using flex mode mode we need to send
	 * the upper address byte to bspi
	 */
	if (bcm_qspi_bspi_ver_three(qspi) == false) {
		addr = msg->from & 0xff000000;
		bcm_qspi_write(qspi, BSPI,
			       BSPI_BSPI_FLASH_UPPER_ADDR_BYTE, addr);
	}

	if (qspi->xfer_mode.flex_mode == false)
		addr = msg->from;
	else
		addr = msg->from & 0x00ffffff;

	/* read result into buffer */
	buf = msg->buf;
	len = msg->len;

	if (bcm_qspi_bspi_ver_three(qspi) == true)
		addr = (addr + 0xc00000) & 0xffffff;

retry:
	reinit_completion(&qspi->bspi_done);
	bcm_qspi_enable_bspi(qspi);
	len_words = (len + 3) >> 2;
	qspi->bspi_rf_msg = msg;
	qspi->bspi_rf_msg_status = 0;
	qspi->bspi_rf_msg_idx = 0;
	qspi->bspi_rf_msg_len = len;
	pr_debug("bspi xfr addr 0x%x len 0x%x", addr, len);

	bcm_qspi_write(qspi, BSPI, BSPI_RAF_START_ADDR, addr);
	bcm_qspi_write(qspi, BSPI, BSPI_RAF_NUM_WORDS, len_words);
	bcm_qspi_write(qspi, BSPI, BSPI_RAF_WATERMARK, 0);
	if (!qspi->use_l2_intc) {
		bcm_qspi_clear_interrupt(qspi, QSPI_INTERRUPTS_ALL);
		bcm_qspi_enable_interrupt(qspi, BSPI_LR_INTERRUPTS_ALL);
	}

	bcm_qspi_lr_start(qspi);
	/* Must flush previous writes before starting BSPI operation */
	mb();
	if (!wait_for_completion_timeout(&qspi->bspi_done, timeo)) {
		if (retry--)
			goto retry;

		dev_err(&qspi->pdev->dev, "timeout waiting for BSPI\n");
		ret = -ETIMEDOUT;
	} else {
		/* set the return length for the caller */
		msg->retlen = len;
	}

	return ret;
}

static int bcm_qspi_flash_read(struct spi_device *spi,
			       struct spi_flash_read_message *msg)
{
	struct bcm_qspi *qspi = spi_master_get_devdata(spi->master);
	int ret = 0;
	bool mspi_read = false;
	u32 nbits, addr, len;
	u_char *buf;

	buf = msg->buf;
	addr = msg->from;
	len = msg->len;

	if (bcm_qspi_bspi_ver_three(qspi) == true) {
		/*
		 * The address coming into this function is a raw flash offset.
		 * But for BSPI <= V3, we need to convert it to a remapped BSPI
		 * address. If it crosses a 4MB boundary, just revert back to
		 * using MSPI.
		 */
		addr = (addr + 0xc00000) & 0xffffff;

		if (ADDR_TO_4MBYTE_SEGMENT(addr) ^
		    ADDR_TO_4MBYTE_SEGMENT(addr + len - 1))
			mspi_read = true;
	}

	/* non-aligned and very short transfers are handled by MSPI */
	if (unlikely(!DWORD_ALIGNED(addr) || !DWORD_ALIGNED(buf) ||
		     len < sizeof(u32)))
		mspi_read = true;

	if (mspi_read)
		/* this will  make the m25p80 fallback to  mspi read */
		return -EAGAIN;

	nbits = msg->data_nbits;
	switch (msg->read_opcode) {
	case SPINOR_OP_READ4_FAST:
		if (!bcm_qspi_is_4_byte_mode(qspi)) {
			ret = bcm_qspi_bspi_set_mode(qspi, nbits,
						     BSPI_ADDRLEN_4BYTES, -1);
			if (ret < 0)
				break;
		}
		/* fall through */
	case SPINOR_OP_READ_FAST:
		ret = bcm_qspi_bspi_flash_read(spi, msg);
		break;
	case OPCODE_QIOR_4B:
	case SPINOR_OP_READ_1_1_4:
	case SPINOR_OP_READ4_1_1_4:
		ret = bcm_qspi_bspi_flash_read(spi, msg);
		break;
	default:
		break;
	}

	return ret;
}

static void bcm_qspi_trans_mode(struct bcm_qspi *qspi,
				struct spi_device *spi,
				struct spi_transfer *trans)
{
	u32 nbits = SPI_NBITS_SINGLE;
	int ret = 0;

	bcm_qspi_chip_select(qspi, spi->chip_select);
	if (trans && trans->len && trans->tx_buf) {
		const u8 *buf = trans->tx_buf;
		u8 command = buf[0];

		if (trans->rx_nbits)
			nbits = trans->rx_nbits;

		switch (command) {
		case SPINOR_OP_EN4B:
			pr_debug("EN4B MODE\n");
			ret = bcm_qspi_bspi_set_mode(qspi, nbits,
						     BSPI_ADDRLEN_4BYTES, -1);
			break;
		case SPINOR_OP_EX4B:
			pr_debug("EX4B MODE\n");
			ret = bcm_qspi_bspi_set_mode(qspi, nbits,
						     BSPI_ADDRLEN_3BYTES, -1);
			break;
		case SPINOR_OP_BRWR:
			command = buf[1];
			pr_debug(" %s 4-BYTE MODE\n",
				 command ? "ENABLE" : "DISABLE");
			ret = bcm_qspi_bspi_set_mode(qspi, nbits,
					   command ? BSPI_ADDRLEN_4BYTES :
					   BSPI_ADDRLEN_3BYTES, -1);
			break;
		default:
			break;
		}
	}

	if (ret < 0)
		dev_warn(&qspi->pdev->dev, "failed to set the read mode\n");
}

static int bcm_qspi_transfer_one(struct spi_master *master,
		   struct spi_device *spi, struct spi_transfer *trans)
{
	struct bcm_qspi *qspi = spi_master_get_devdata(master);
	int slots;
	unsigned long timeo = msecs_to_jiffies(100);

	qspi->pos.trans = trans;
	qspi->pos.byte = 0;
	bcm_qspi_trans_mode(qspi, spi, trans);

	while (qspi->pos.byte < trans->len) {
		reinit_completion(&qspi->mspi_done);

		slots = write_to_hw(qspi, spi);
		if (!wait_for_completion_timeout(&qspi->mspi_done, timeo)) {
			dev_err(&qspi->pdev->dev, "timeout waiting for MSPI\n");
			return -ETIMEDOUT;
		}

		if (qspi->next_udelay) {
			udelay(qspi->next_udelay);
			qspi->next_udelay = 0;
		}

		read_from_hw(qspi, slots);
		if (qspi->cs_change) {
			udelay(10);
			qspi->cs_change = 0;
		}
	}

	if (spi_transfer_is_last(master, trans))
		hw_stop(qspi);

	return 0;
}

static void bcm_qspi_cleanup(struct spi_device *spi)
{
	struct bcm_qspi_parms *xp = spi_get_ctldata(spi);

	kfree(xp);
}

static irqreturn_t bcm_qspi_mspi_l2_isr(int irq, void *dev_id)
{
	struct bcm_qspi_dev_id *qspi_dev_id = dev_id;
	struct bcm_qspi *qspi = qspi_dev_id->dev;
	u32 status = bcm_qspi_read(qspi, MSPI, MSPI_MSPI_STATUS);

	if (status & MSPI_MSPI_STATUS_SPIF) {
		/* clear interrupt */
		status &= ~MSPI_MSPI_STATUS_SPIF;
		bcm_qspi_write(qspi, MSPI, MSPI_MSPI_STATUS, status);
		if (!qspi->use_l2_intc)
			bcm_qspi_clear_interrupt(qspi, INTR_MSPI_DONE_MASK);
		complete(&qspi->mspi_done);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}

static irqreturn_t bcm_qspi_bspi_lr_l2_isr(int irq, void *dev_id)
{
	struct bcm_qspi_dev_id *qspi_dev_id = dev_id;
	struct bcm_qspi *qspi = qspi_dev_id->dev;

	if (qspi->bspi_enabled && qspi->bspi_rf_msg) {
		bcm_qspi_bspi_lr_data_read(qspi);
		if (qspi->bspi_rf_msg_len == 0) {
			qspi->bspi_rf_msg = NULL;
			if (!qspi->use_l2_intc)
				bcm_qspi_disable_interrupt(qspi,
						   BSPI_LR_INTERRUPTS_ALL);
			if (qspi->bspi_rf_msg_status)
				bcm_qspi_lr_clear(qspi);
			else
				bcm_qspi_flush_prefetch_buffers(qspi);

			complete(&qspi->bspi_done);
		}
		if (!qspi->use_l2_intc)
			bcm_qspi_clear_interrupt(qspi, BSPI_LR_INTERRUPTS_ALL);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t bcm_qspi_bspi_lr_err_l2_isr(int irq, void *dev_id)
{
	struct bcm_qspi_dev_id *qspi_dev_id = dev_id;
	struct bcm_qspi *qspi = qspi_dev_id->dev;

	if (qspi_dev_id->irqp->mask & BSPI_LR_INTERRUPTS_ERROR) {
		dev_err(&qspi->pdev->dev, "INT error\n");
		qspi->bspi_rf_msg_status = -EIO;
		if (!qspi->use_l2_intc)
			bcm_qspi_clear_interrupt(qspi,
						 BSPI_LR_INTERRUPTS_ERROR);
		complete(&qspi->bspi_done);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t bcm_qspi_l1_isr(int irq, void *dev_id)
{
	struct bcm_qspi_dev_id *qspi_dev_id = dev_id;
	struct bcm_qspi *qspi = qspi_dev_id->dev;
	u32 status = bcm_qspi_read_l2int_status(qspi);
	irqreturn_t ret = IRQ_NONE;

	if (status & MSPI_INTERRUPTS_ALL)
		ret = bcm_qspi_mspi_l2_isr(irq, dev_id);
	else if (status & BSPI_LR_INTERRUPTS_DATA)
		ret = bcm_qspi_bspi_lr_l2_isr(irq, dev_id);
	else if (status & BSPI_LR_INTERRUPTS_ERROR)
		ret = bcm_qspi_bspi_lr_err_l2_isr(irq, dev_id);

	return ret;
}

static const struct bcm_qspi_irq qspi_irq_tab[] = {
	{
		.irq_name = "spi_lr_fullness_reached",
		.irq_handler = bcm_qspi_bspi_lr_l2_isr,
		.mask = INTR_BSPI_LR_FULLNESS_REACHED_MASK,
	},
	{
		.irq_name = "spi_lr_session_aborted",
		.irq_handler = bcm_qspi_bspi_lr_err_l2_isr,
		.mask = INTR_BSPI_LR_SESSION_ABORTED_MASK,
	},
	{
		.irq_name = "spi_lr_impatient",
		.irq_handler = bcm_qspi_bspi_lr_err_l2_isr,
		.mask = INTR_BSPI_LR_IMPATIENT_MASK,
	},
	{
		.irq_name = "spi_lr_session_done",
		.irq_handler = bcm_qspi_bspi_lr_l2_isr,
		.mask = INTR_BSPI_LR_SESSION_DONE_MASK,
	},
	{
		.irq_name = "spi_lr_overread",
		.irq_handler = bcm_qspi_bspi_lr_err_l2_isr,
		.mask = INTR_BSPI_LR_OVERREAD_MASK,
	},
	{
		.irq_name = "mspi_done",
		.irq_handler = bcm_qspi_mspi_l2_isr,
		.mask = INTR_MSPI_DONE_MASK,
	},
	{
		.irq_name = "mspi_halted",
		.irq_handler = bcm_qspi_mspi_l2_isr,
		.mask = INTR_MSPI_HALTED_MASK,
	},
	{
		/* single muxed L1 interrupt source */
		.irq_name = "spi_l1_intr",
		.irq_handler = bcm_qspi_l1_isr,
		.mask = QSPI_INTERRUPTS_ALL,
	},
};


static void bcm_qspi_hw_init(struct bcm_qspi *qspi)
{
	u32 val = 0;
	struct bcm_qspi_parms parms;

	bcm_qspi_write(qspi, MSPI, MSPI_SPCR1_LSB, 0);
	bcm_qspi_write(qspi, MSPI, MSPI_SPCR1_MSB, 0);
	bcm_qspi_write(qspi, MSPI, MSPI_NEWQP, 0);
	bcm_qspi_write(qspi, MSPI, MSPI_ENDQP, 0);
	bcm_qspi_write(qspi, MSPI, MSPI_SPCR2, 0x20);

	parms.mode = SPI_MODE_3;
	parms.bits_per_word = 8;
	of_property_read_u32(qspi->pdev->dev.of_node, "clock-frequency", &val);
	if (val > 0) {
		parms.speed_hz = val;
		bcm_qspi_hw_set_parms(qspi, &parms);
	} else {
		bcm_qspi_hw_set_parms(qspi, &bcm_qspi_default_parms_cs0);
	}

	if (!qspi->base[BSPI])
		return;
	val = bcm_qspi_read(qspi, BSPI, BSPI_REVISION_ID);
	qspi->bspi_maj_rev = (val >> 8) & 0xff;
	qspi->bspi_min_rev = val & 0xff;
	if (!(bcm_qspi_bspi_ver_three(qspi))) {
		/* Force mapping of BSPI address -> flash offset */
		bcm_qspi_write(qspi, BSPI, BSPI_BSPI_XOR_VALUE, 0);
		bcm_qspi_write(qspi, BSPI, BSPI_BSPI_XOR_ENABLE, 1);
	}
	qspi->bspi_enabled = 1;
	bcm_qspi_disable_bspi(qspi);
	bcm_qspi_write(qspi, BSPI, BSPI_B0_CTRL, 1);
	bcm_qspi_write(qspi, BSPI, BSPI_B1_CTRL, 1);
}

static void bcm_qspi_hw_uninit(struct bcm_qspi *qspi)
{
	bcm_qspi_write(qspi, MSPI, MSPI_SPCR2, 0);
	/* disable irq and enable bits */
	bcm_qspi_enable_bspi(qspi);
}

/* Get BSPI chip-selects info */
static int bcm_qspi_get_bspi_cs(struct bcm_qspi *qspi)
{
	struct device_node *np = qspi->pdev->dev.of_node, *childnode;
	int num_bspi_cs;
	u32 vals[10], i;
	struct spi_master *master = qspi->master;

	qspi->bspi_cs_bmap = 0;
	if (!qspi->base[BSPI])
		return 0;

	if (of_find_property(np, "bspi-sel", NULL)) {
		num_bspi_cs = of_property_count_u32_elems(np, "bspi-sel");
		if (num_bspi_cs) {
			of_property_read_u32_array(np, "bspi-sel", vals,
						   num_bspi_cs);
			for (i = 0; i < num_bspi_cs; i++)
				qspi->bspi_cs_bmap |= (1 << vals[i]);
		}
	} else {
		/*
		 * if using m25p80 compatible driver,
		 * find the chip select info in the child node
		 */
		for_each_child_of_node(np, childnode) {
			if (of_find_property(childnode, "use-bspi", NULL)) {
				const u32 *regp;
				int size;

				/* "reg" field holds chip-select number */
				regp = of_get_property(childnode, "reg", &size);
				if (!regp || size != sizeof(*regp))
					return -EINVAL;
				if (regp[0] < master->num_chipselect)
					qspi->bspi_cs_bmap |=
						(1 << regp[0]);
			}
		}
	}
	pr_debug("bspi chip selects bitmap 0x%x", qspi->bspi_cs_bmap);
	return 0;
}

static int bcm_qspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm_qspi *qspi;
	struct spi_master *master;
	struct resource *res;
	int irq, ret = 0, num_ints = 0;
	u32 val;
	const char *name = NULL;
	int num_irqs = ARRAY_SIZE(qspi_irq_tab);

	master = spi_alloc_master(dev, sizeof(struct bcm_qspi));
	if (!master) {
		dev_err(dev, "error allocating spi_master\n");
		return -ENOMEM;
	}

	qspi = spi_master_get_devdata(master);
	qspi->pdev = pdev;
	qspi->state = STATE_IDLE;
	qspi->pos.trans = NULL;
	qspi->pos.byte = 0;
	qspi->master = master;

	master->bus_num = pdev->id;
	master->mode_bits = SPI_CPHA | SPI_CPOL | SPI_RX_DUAL | SPI_RX_QUAD;
	master->setup = bcm_qspi_setup;
	master->transfer_one = bcm_qspi_transfer_one;
	master->spi_flash_read = bcm_qspi_flash_read;
	master->cleanup = bcm_qspi_cleanup;
	master->dev.of_node = dev->of_node;
	master->num_chipselect = NUM_CHIPSELECT;

	if (!of_property_read_u32(dev->of_node, "num-cs", &val))
		master->num_chipselect = val;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hif_mspi");
	if (!res)
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "mspi");

	if (res) {
		qspi->base[MSPI]  = devm_ioremap_resource(dev, res);
		if (IS_ERR(qspi->base[MSPI])) {
			ret = PTR_ERR(qspi->base[MSPI]);
			goto err2;
		}
	} else
		goto err2;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bspi");
	if (res) {
		qspi->base[BSPI]  = devm_ioremap_resource(dev, res);
		if (IS_ERR(qspi->base[BSPI])) {
			ret = PTR_ERR(qspi->base[BSPI]);
			goto err2;
		}
		qspi->bspi_mode = true;
	} else
		qspi->bspi_mode = false;

	if (!qspi->bspi_mode)
		master->bus_num += 1;

	dev_info(dev, "using %smspi mode\n", qspi->bspi_mode ? "bspi-" : "");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cs_reg");
	if (res) {
		qspi->base[CHIP_SELECT]  = devm_ioremap_resource(dev, res);
		if (IS_ERR(qspi->base[CHIP_SELECT])) {
			ret = PTR_ERR(qspi->base[CHIP_SELECT]);
			goto err2;
		}
	}

	qspi->hif_spi_mode = false;
	qspi->use_l2_intc = false;
	/* SoC based interrupt resource differences are handled here */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intr_regs");
	if (res) {
		qspi->base[INTR]  = devm_ioremap_resource(dev, res);
		if (IS_ERR(qspi->base[INTR])) {
			ret = PTR_ERR(qspi->base[INTR]);
			goto err2;
		}
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "intr_status_reg");
		if (res) {
			qspi->base[INTR_STATUS]  = devm_ioremap_resource(dev,
									 res);
			if (IS_ERR(qspi->base[INTR_STATUS])) {
				ret = PTR_ERR(qspi->base[INTR_STATUS]);
				goto err2;
			}
		}
	} else {
		/* SoCs with hif_spi_intr */
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "hif_spi_intr2");
		if (res) {
			qspi->base[INTR] = devm_ioremap_resource(dev, res);
			if (IS_ERR(qspi->base[INTR])) {
				ret = PTR_ERR(qspi->base[INTR]);
				goto err2;
			}
			qspi->hif_spi_mode = true;
			qspi->base[INTR_STATUS] = qspi->base[INTR];
		} else {
			/* we must be using l2 intc driver */
			qspi->use_l2_intc = true;
		}
	}

	if (!qspi->use_l2_intc) {
		bcm_qspi_disable_interrupt(qspi, QSPI_INTERRUPTS_ALL);
		bcm_qspi_clear_interrupt(qspi, QSPI_INTERRUPTS_ALL);
	}

	qspi->dev_ids = kcalloc(num_irqs, sizeof(struct bcm_qspi_dev_id),
				GFP_KERNEL);
	if (IS_ERR(qspi->dev_ids)) {
		ret = PTR_ERR(qspi->dev_ids);
		goto err2;
	}

	for (val = 0; val < num_irqs; val++) {
		irq = -1;
		name = qspi_irq_tab[val].irq_name;
		if (val <  (num_irqs - 1))
			/* get the l2 interrupts */
			irq = platform_get_irq_byname(pdev, name);
		else if (!num_ints) {
			/* all mspi, bspi intrs muxed to one L1 intr */
			irq = platform_get_irq(pdev, 0);
			of_property_read_string(dev->of_node,
						"interrupt-names",
						&name);
		}

		if (irq  >= 0) {
			ret = devm_request_irq(&pdev->dev, irq,
					       qspi_irq_tab[val].irq_handler, 0,
					       name,
					       &qspi->dev_ids[val]);
			if (ret < 0) {
				dev_err(&pdev->dev, "unable to allocate IRQ\n");
				goto err2;
			}

			qspi->dev_ids[val].dev = qspi;
			qspi->dev_ids[val].irqp = &qspi_irq_tab[val];
			num_ints++;
			dev_dbg(&pdev->dev, "registered IRQ %s %d\n",
				qspi_irq_tab[val].irq_name,
				irq);
		}
	}

	if (!num_ints) {
		dev_err(&pdev->dev, "no IRQs registered, cannot init driver\n");
		goto err2;
	}

	if (!qspi->use_l2_intc)
		bcm_qspi_enable_interrupt(qspi, INTR_MSPI_DONE_MASK);

	qspi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(qspi->clk)) {
		dev_warn(dev, "unable to get clock, using defaults\n");
		qspi->clk = NULL;
	}

	if (qspi->clk) {
		ret = clk_prepare_enable(qspi->clk);
		if (ret) {
			dev_err(dev, "failed to prepare clock\n");
			goto err2;
		}
		qspi->base_clk = clk_get_rate(qspi->clk);
	} else {
		qspi->base_clk = MSPI_BASE_FREQ;
	}

	qspi->max_speed_hz = qspi->base_clk/(QSPI_SPBR_MIN * 2);

	bcm_qspi_hw_init(qspi);
	init_completion(&qspi->mspi_done);
	init_completion(&qspi->bspi_done);
	qspi->curr_cs = -1;

	platform_set_drvdata(pdev, qspi);
	bcm_qspi_get_bspi_cs(qspi);

	qspi->xfer_mode.width = -1;
	qspi->xfer_mode.addrlen = -1;
	qspi->xfer_mode.hp = -1;

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret < 0) {
		dev_err(dev, "can't register master\n");
		goto err1;
	}

	return 0;

err1:
	bcm_qspi_hw_uninit(qspi);
	if (qspi->clk)
		clk_disable_unprepare(qspi->clk);
err2:
	spi_master_put(master);
	kfree(qspi->dev_ids);
	return ret;
}

static int bcm_qspi_remove(struct platform_device *pdev)
{
	struct bcm_qspi *qspi = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	bcm_qspi_hw_uninit(qspi);
	if (qspi->clk)
		clk_disable_unprepare(qspi->clk);
	kfree(qspi->dev_ids);
	spi_unregister_master(qspi->master);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bcm_qspi_suspend(struct device *dev)
{
	struct bcm_qspi *qspi = dev_get_drvdata(dev);

	if (qspi->hif_spi_mode && !qspi->use_l2_intc)
		qspi->s3_intr2_mask = bcm_qspi_read(qspi, INTR,
					    HIF_SPI_INTR2_CPU_MASK_STATUS);
	if (qspi->clk)
		clk_disable(qspi->clk);
	return 0;
};

static int bcm_qspi_resume(struct device *dev)
{
	struct bcm_qspi *qspi = dev_get_drvdata(dev);
	int curr_cs = qspi->curr_cs;
	int ret = 0;

	if (qspi->hif_spi_mode && !qspi->use_l2_intc) {
		bcm_qspi_write(qspi, INTR, HIF_SPI_INTR2_CPU_MASK_CLEAR,
						~qspi->s3_intr2_mask);
		bcm_qspi_read(qspi, INTR, HIF_SPI_INTR2_CPU_MASK_CLEAR);
	}
	bcm_qspi_hw_init(qspi);
	bcm_qspi_bspi_set_mode(qspi, -1, -1, -1);
	qspi->curr_cs = -1;
	bcm_qspi_chip_select(qspi, curr_cs);

	if (qspi->clk)
		ret = clk_enable(qspi->clk);

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(bcm_qspi_pm_ops, bcm_qspi_suspend, bcm_qspi_resume);

static const struct of_device_id bcm_qspi_of_match[] = {
	{ .compatible = "brcm,spi-bcm-qspi" },
	{ .compatible = "brcm,qspi-brcmstb" },
	{ .compatible = "brcm,spi-brcmstb-mspi"},
	{},
};
MODULE_DEVICE_TABLE(of, bcm_qspi_of_match);

static struct platform_driver bcm_qspi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
		.pm = &bcm_qspi_pm_ops,
		.of_match_table = bcm_qspi_of_match,
	},
	.probe = bcm_qspi_probe,
	.remove = bcm_qspi_remove,
};
module_platform_driver(bcm_qspi_driver);

MODULE_AUTHOR("Kamal Dasu");
MODULE_DESCRIPTION("BCM QSPI driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
