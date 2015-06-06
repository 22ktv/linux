/*
 * Copyright (C) 2009 Broadcom Corporation
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#define BSPI_REGS_OFFSET		0x0
#define MSPI_REGS_OFFSET		0x200

#define NUM_CHIPSELECT			4
#define MSPI_BASE_FREQ			27000000UL
#define SPBR_MIN			8U
#define SPBR_MAX			255U
#define MAX_SPEED_HZ \
	(MSPI_BASE_FREQ / (SPBR_MIN * 2))

#define STATE_IDLE			0
#define STATE_RUNNING			1
#define STATE_SHUTDOWN			2

#define FNB_BREAK_NONE			0
#define FNB_BREAK_EOM			1
#define FNB_BREAK_DELAY			2
#define FNB_BREAK_CS_CHANGE		4
#define FNB_BREAK_NO_BYTES		8

#define FNB_BREAK_TX \
	(FNB_BREAK_EOM | FNB_BREAK_DELAY | FNB_BREAK_CS_CHANGE)

#define FNB_BREAK_DESELECT \
	(FNB_BREAK_EOM | FNB_BREAK_CS_CHANGE)

#define NUM_TXRAM			32
#define NUM_RXRAM			32
#define NUM_CDRAM			16

struct mspi_regs {
	u32 spcr0_lsb;
	u32 spcr0_msb;
	u32 spcr1_lsb;
	u32 spcr1_msb;
	u32 newqp;
	u32 endqp;
	u32 spcr2;
	u32 reserved0;
	u32 mspi_status;
	u32 cptqp;
	u32 reserved1[6];
	u32 txram[NUM_TXRAM];
	u32 rxram[NUM_RXRAM];
	u32 cdram[NUM_CDRAM];
	u32 write_lock;
	u32 disable_flush_gen;
};

struct bspi_regs {
	u32 revision_id;
	u32 scratch;
	u32 mast_n_boot_ctrl;
	u32 busy_status;
	u32 intr_status;
	u32 b0_status;
	u32 b0_ctrl;
	u32 b1_status;
	u32 b1_ctrl;
	u32 strap_override_ctrl;
	u32 flex_mode_enable;
	u32 bits_per_cycle;
	u32 bits_per_phase;
	u32 cmd_and_mode_byte;
	u32 flash_upper_addr_byte;
	u32 xor_value;
	u32 xor_enable;
	u32 pio_mode_enable;
	u32 pio_iodir;
	u32 pio_data;
};

struct brcmstb_spi_parms {
	u32 speed_hz;
	u8 chip_select;
	u8 mode;
	u8 bits_per_word;
};

struct brcmstb_spi_position {
	struct spi_message *mesg;
	struct spi_transfer *xfer;
	int byte;
};

struct brcmstb_spi_priv {
	struct spi_master *master;

	struct mspi_regs *mspi_regs;
	struct bspi_regs *bspi_regs;

	struct tasklet_struct tasklet;
	spinlock_t lock;
	struct brcmstb_spi_parms last_parms;
	struct brcmstb_spi_position pos;
	struct list_head mesg_queue;
	int state;
	int outstanding_bytes;
	int delay_usecs;
	int cs_change;
	unsigned int max_speed_hz;
	u32 actual_length;
};

static inline void brcmstb_spi_hw_set_parms(struct brcmstb_spi_priv *priv,
		const struct brcmstb_spi_parms *xp)
{
	u32 lsb = SPBR_MIN, msb = 0;

	if (xp->speed_hz) {
		unsigned int spbr;

		spbr = MSPI_BASE_FREQ / (2 * xp->speed_hz);
		lsb  = max(min(spbr, SPBR_MAX), lsb);
	}

	msb |= 0x80;
	msb |= (xp->bits_per_word ? xp->bits_per_word : 8) << 2;
	msb |= xp->mode & 0x3;

	priv->mspi_regs->spcr0_lsb = lsb;
	priv->mspi_regs->spcr0_msb = msb;

	priv->last_parms = *xp;
}

static int brcmstb_spi_update_parms(struct spi_device *spi,
		struct spi_transfer *t, int override)
{
	struct brcmstb_spi_priv *priv = spi_master_get_devdata(spi->master);
	struct brcmstb_spi_parms xp;
	u32 speed_hz = MAX_SPEED_HZ;

	if (!t->speed_hz) {
		if (spi->max_speed_hz)
			speed_hz = min(spi->max_speed_hz, speed_hz);
	} else
		speed_hz = min(t->speed_hz, speed_hz);

	xp.speed_hz = speed_hz;
	xp.chip_select = spi->chip_select;
	xp.mode = spi->mode;
	xp.bits_per_word = t->bits_per_word ?
			t->bits_per_word :
			(spi->bits_per_word ? spi->bits_per_word : 8);

	if (override ||
	    ((xp.speed_hz == priv->last_parms.speed_hz) &&
	     (xp.chip_select == priv->last_parms.chip_select) &&
	     (xp.mode == priv->last_parms.mode) &&
	     (xp.bits_per_word == priv->last_parms.bits_per_word))) {
		brcmstb_spi_hw_set_parms(priv, &xp);
		return 0;
	}

	return 1;
}

static int brcmstb_spi_setup(struct spi_device *spi)
{
	struct brcmstb_spi_priv *priv = spi_master_get_devdata(spi->master);
	struct brcmstb_spi_parms *xp;

	if (spi->bits_per_word > 16)
		return -EINVAL;

	xp = spi_get_ctldata(spi);
	if (!xp) {
		xp = kzalloc(sizeof(*xp), GFP_KERNEL);
		if (!xp)
			return -ENOMEM;

		spi_set_ctldata(spi, xp);
	}

	if (spi->max_speed_hz < priv->max_speed_hz)
		xp->speed_hz = spi->max_speed_hz;
	else
		xp->speed_hz = 0;

	xp->chip_select = spi->chip_select;
	xp->mode = spi->mode;
	xp->bits_per_word = spi->bits_per_word ?
			spi->bits_per_word : 8;

	return 0;
}

static void brcmstb_spi_cleanup(struct spi_device *spi)
{
	struct brcmstb_spi_parms *xp = spi_get_ctldata(spi);

	kfree(xp);
}

static inline int find_next_byte(struct brcmstb_spi_priv *priv,
		struct brcmstb_spi_position *pos, struct list_head *head,
		int flags)
{
	int ret = FNB_BREAK_NONE;

	pos->byte++;

	while (pos->byte >= pos->xfer->len) {
		if (pos->xfer->delay_usecs && (flags & FNB_BREAK_DELAY))
			ret |= FNB_BREAK_DELAY;

		if (pos->xfer->cs_change && (flags & FNB_BREAK_CS_CHANGE))
			ret |= FNB_BREAK_CS_CHANGE;

		if (ret)
			return ret;

		if (list_is_last(&pos->xfer->transfer_list,
				&pos->mesg->transfers)) {
			struct spi_message *next_mesg = NULL;

			if (!head || (flags & FNB_BREAK_EOM))
				return FNB_BREAK_EOM;

			if (!list_is_last(&pos->mesg->queue, &priv->mesg_queue))
				next_mesg = list_entry(pos->mesg->queue.next,
						struct spi_message, queue);

			list_del(&pos->mesg->queue);
			list_add_tail(&pos->mesg->queue, head);

			pos->mesg = next_mesg;
			pos->byte = 0;
			if (pos->mesg == NULL) {
				pos->xfer = NULL;
				ret = FNB_BREAK_NO_BYTES;
				break;
			}

			pos->xfer = list_entry(pos->mesg->transfers.next,
					struct spi_transfer, transfer_list);
		} else {
			pos->xfer = list_entry(pos->xfer->transfer_list.next,
					struct spi_transfer, transfer_list);
			pos->byte = 0;
		}
	}

	return ret;
}

static void read_from_hw(struct brcmstb_spi_priv *priv,
		struct list_head *head)
{
	struct brcmstb_spi_position pos;
	int slot = 0, n = priv->outstanding_bytes;

	pos = priv->pos;

	while (n > 0) {
		BUG_ON(pos.mesg == NULL);

		if (pos.xfer->bits_per_word <= 8) {
			u8 *buf = pos.xfer->rx_buf;

			if (buf)
				buf[pos.byte] =
				priv->mspi_regs->rxram[(slot<<1)+1]&0xff;
		} else {
			u16 *buf = pos.xfer->rx_buf;

			if (buf)
				buf[pos.byte] =
				((priv->mspi_regs->rxram[(slot<<1)+1]&0xff)<<0)|
				((priv->mspi_regs->rxram[(slot<<1)+0]&0xff)<<8);
		}

		slot++;
		n--;
		pos.mesg->actual_length++;

		find_next_byte(priv, &pos, head, FNB_BREAK_NONE);
	}

	priv->pos = pos;
	priv->outstanding_bytes = 0;
}

static void write_to_hw(struct brcmstb_spi_priv *priv)
{
	struct brcmstb_spi_position pos;
	int slot = 0, fnb = 0;
	struct spi_message *mesg = NULL;

	pos = priv->pos;

	while (1) {
		if (pos.mesg == NULL)
			break;

		if (!mesg) {
			mesg = pos.mesg;
			brcmstb_spi_update_parms(mesg->spi, pos.xfer, 1);
		} else {
			if (brcmstb_spi_update_parms(mesg->spi, pos.xfer, 0))
				break;
		}

		if (pos.xfer->bits_per_word <= 8) {
			const u8 *buf = pos.xfer->tx_buf;

			priv->mspi_regs->txram[slot << 1] =
					buf ? (buf[pos.byte] & 0xff) : 0xff;
		} else {
			const u16 *buf = pos.xfer->tx_buf;

			priv->mspi_regs->txram[(slot<<1)+0] =
					buf ? (buf[pos.byte] >> 8) : 0xff;
			priv->mspi_regs->txram[(slot<<1)+1] =
					buf ? (buf[pos.byte] & 0xff) : 0xff;
		}

		priv->mspi_regs->cdram[slot] =
				((pos.xfer->bits_per_word <= 8) ? 0x8e : 0xce);
		slot++;

		fnb = find_next_byte(priv, &pos, NULL, FNB_BREAK_TX);

		if (fnb & FNB_BREAK_CS_CHANGE)
			priv->cs_change = 1;

		if (fnb & FNB_BREAK_DELAY)
			priv->delay_usecs = pos.xfer->delay_usecs;

		if (fnb || (slot == NUM_CDRAM))
			break;
	}

	if (slot) {
		priv->mspi_regs->newqp = 0;
		priv->mspi_regs->endqp = slot - 1;

		if (fnb & FNB_BREAK_DESELECT)
			priv->mspi_regs->cdram[slot - 1] &= ~0x80;

		priv->mspi_regs->write_lock |= 0x1;
		(void)priv->mspi_regs->write_lock;

		priv->mspi_regs->spcr2 = 0xe0;

		priv->state = STATE_RUNNING;
		priv->outstanding_bytes = slot;
	} else {
		priv->mspi_regs->write_lock &= ~0x1;
		(void)priv->mspi_regs->write_lock;

		priv->state = STATE_IDLE;
	}
}

static int brcmstb_spi_xfer(struct spi_device *spi, struct spi_message *mesg)
{
	struct brcmstb_spi_priv *priv = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	mesg->actual_length = 0;

	list_add_tail(&mesg->queue, &priv->mesg_queue);

	if (priv->state == STATE_IDLE) {
		BUG_ON(priv->pos.mesg != NULL);

		priv->pos.mesg = mesg;
		priv->pos.xfer = list_entry(mesg->transfers.next,
				struct spi_transfer, transfer_list);
		priv->pos.byte = 0;

		write_to_hw(priv);
	}

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void brcmstb_spi_tasklet(unsigned long data)
{
	struct brcmstb_spi_priv *priv = (struct brcmstb_spi_priv *)data;
	struct list_head head;
	struct spi_message *mesg;
	unsigned long flags;

	INIT_LIST_HEAD(&head);

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->delay_usecs) {
		udelay(priv->delay_usecs);
		priv->delay_usecs = 0;
	}

	mesg = priv->pos.mesg;

	read_from_hw(priv, &head);
	if (priv->cs_change) {
		udelay(10);
		priv->cs_change = 0;
	}
	write_to_hw(priv);

	spin_unlock_irqrestore(&priv->lock, flags);

	while (!list_empty(&head)) {
		mesg = list_first_entry(&head, struct spi_message, queue);
		list_del(&mesg->queue);
		mesg->status = 0;
		if (mesg->complete)
			mesg->complete(mesg->context);
	}
}

static irqreturn_t brcmstb_spi_interrupt(int irq, void *dev_id)
{
	struct brcmstb_spi_priv *priv = dev_id;

	if (priv->mspi_regs->mspi_status & 0x1) {
		priv->mspi_regs->mspi_status &= ~0x1;
		tasklet_schedule(&priv->tasklet);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static const struct brcmstb_spi_parms brcmstb_spi_default_parms = {
	.speed_hz = MAX_SPEED_HZ,
	.chip_select = 0,
	.mode = SPI_MODE_3,
	.bits_per_word = 8,
};

static struct spi_board_info brcmstb_spi_board_info = {
	.modalias = "m25p80",
};

static int brcmstb_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct brcmstb_spi_priv *priv;
	struct spi_master *master;
	void __iomem *regs;
	unsigned int irq;
	u32 val;

	master = spi_alloc_master(dev, sizeof(*priv));
	if (!master)
		return -ENOMEM;

	priv = spi_master_get_devdata(master);

	priv->state = STATE_IDLE;
	priv->pos.mesg = NULL;
	priv->master = master;

	master->bus_num = 0;
	master->num_chipselect = NUM_CHIPSELECT;
	master->mode_bits = SPI_MODE_3;
	master->setup = brcmstb_spi_setup;
	master->cleanup = brcmstb_spi_cleanup;
	master->transfer = brcmstb_spi_xfer;
	master->dev.of_node = pdev->dev.of_node;

	regs = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(regs)) {
		spi_master_put(master);
		return PTR_ERR(regs);
	}

	priv->mspi_regs = regs + MSPI_REGS_OFFSET;
	priv->bspi_regs = regs + BSPI_REGS_OFFSET;
	priv->bspi_regs->mast_n_boot_ctrl = 1;

	INIT_LIST_HEAD(&priv->mesg_queue);
	spin_lock_init(&priv->lock);

	tasklet_init(&priv->tasklet, brcmstb_spi_tasklet, (unsigned long)priv);

	irq = of_irq_get(dev->of_node, 0);
	if (devm_request_irq(dev, irq, brcmstb_spi_interrupt, 0, pdev->name,
			priv)) {
		spi_master_put(master);
		return -ENODEV;
	}

	platform_set_drvdata(pdev, priv);

	priv->mspi_regs->spcr1_lsb = 0;
	priv->mspi_regs->spcr1_msb = 0;
	priv->mspi_regs->newqp = 0;
	priv->mspi_regs->endqp = 0;
	priv->mspi_regs->spcr2 = 0x20;

	brcmstb_spi_hw_set_parms(priv, &brcmstb_spi_default_parms);

	if (spi_register_master(master)) {
		spi_master_put(master);
		return -ENODEV;
	}

	/* The bcm7xxx set-top box platforms used for SPI-NOR flash */
	spi_new_device(master, &brcmstb_spi_board_info);

	return 0;
}

static int brcmstb_spi_remove(struct platform_device *pdev)
{
	struct brcmstb_spi_priv *priv = platform_get_drvdata(pdev);

	priv->mspi_regs->spcr2 = 0x0;

	priv->bspi_regs->b0_ctrl = 1;
	priv->bspi_regs->b1_ctrl = 1;
	priv->bspi_regs->mast_n_boot_ctrl = 0;

	spi_unregister_master(priv->master);

	return 0;
}

static const struct of_device_id of_device_match[] = {
	{ .compatible = "brcm,brcmstb-spi" },
	{ },
};

MODULE_DEVICE_TABLE(of, of_device_match);

static struct platform_driver brcmstb_spi_driver = {
	.probe	= brcmstb_spi_probe,
	.remove	= brcmstb_spi_remove,
	.driver	= {
		.name = "brcmstb-spi",
		.of_match_table = of_device_match,
	},
};

module_platform_driver(brcmstb_spi_driver);
