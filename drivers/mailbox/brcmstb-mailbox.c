// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2017-2020, Broadcom */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/arm-smccc.h>

#define BRCM_SCMI_SMC_OEM_FUNC	0x400
#define BRCM_SCMI_MBOX_NUM	0
#define BRCM_FID(ch) ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, \
			IS_ENABLED(CONFIG_ARM64), \
			ARM_SMCCC_OWNER_OEM, \
			BRCM_SCMI_SMC_OEM_FUNC + (ch))
enum {
	A2P_CHAN = 0,
	NUM_CHAN
};

struct chan_priv {
	unsigned int mbox_num;
	unsigned int ch;
};

struct brcm_mbox {
	struct mbox_controller controller;
	int irqs[NUM_CHAN];
};

static struct mbox_chan *brcm_mbox_of_xlate(struct mbox_controller *controller,
					    const struct of_phandle_args *sp)
{
	unsigned int ch = sp->args[0];
	struct brcm_mbox *mbox
		= container_of(controller, struct brcm_mbox, controller);

	if (!mbox || ch >= NUM_CHAN)
		return ERR_PTR(-ENOENT);

	return &mbox->controller.chans[ch];
}

static int announce_msg(unsigned int mbox_num, unsigned int ch)
{
	struct arm_smccc_res res;

	if (ch >= NUM_CHAN)
		return -EIO;
	arm_smccc_smc(BRCM_FID(ch), mbox_num, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EIO;
	return 0;
}

static int brcm_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct chan_priv *priv = chan->con_priv;

	return announce_msg(priv->mbox_num, priv->ch);
}

static int brcm_mbox_startup(struct mbox_chan *chan)
{
	return 0;
}

static const struct mbox_chan_ops brcm_mbox_ops = {
	.send_data = brcm_mbox_send_data,
	.startup = brcm_mbox_startup,
};

static irqreturn_t brcm_a2p_isr(int irq, void *data)
{
	struct mbox_chan *chan = data;

	mbox_chan_received_data(chan, NULL);
	return IRQ_HANDLED;
}

static int brcm_mbox_probe(struct platform_device *pdev)
{
	struct brcm_mbox *mbox;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct chan_priv *chan_priv;
	int ret;

	if (!np)
		return -EINVAL;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	/* Allocate channels */
	mbox->controller.chans = devm_kzalloc(
		&pdev->dev, NUM_CHAN * sizeof(struct mbox_chan), GFP_KERNEL);
	if (!mbox->controller.chans)
		return -ENOMEM;
	chan_priv = devm_kzalloc(
		&pdev->dev, NUM_CHAN * sizeof(struct chan_priv), GFP_KERNEL);
	if (!chan_priv)
		return -ENOMEM;

	mbox->irqs[A2P_CHAN] = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, mbox->irqs[A2P_CHAN], brcm_a2p_isr,
				IRQF_NO_SUSPEND, "brcm: SCMI a2p intr",
				&mbox->controller.chans[A2P_CHAN]);
	if (ret) {
		dev_err(&pdev->dev, "failed to setup SCMI a2p isr\n");
		return ret;
	}
	chan_priv[A2P_CHAN].mbox_num = BRCM_SCMI_MBOX_NUM;
	chan_priv[A2P_CHAN].ch = A2P_CHAN;
	mbox->controller.chans[A2P_CHAN].con_priv = &chan_priv[A2P_CHAN];
	mbox->controller.num_chans++;
	mbox->controller.dev = &pdev->dev;
	mbox->controller.ops = &brcm_mbox_ops;
	mbox->controller.of_xlate = brcm_mbox_of_xlate;
	ret = mbox_controller_register(&mbox->controller);
	if (ret) {
		dev_err(dev, "failed to register BrcmSTB mbox\n");
		return ret;
	}

	platform_set_drvdata(pdev, mbox);
	return 0;
}

static int brcm_mbox_remove(struct platform_device *pdev)
{
	struct brcm_mbox *mbox = platform_get_drvdata(pdev);

	mbox_controller_unregister(&mbox->controller);

	return 0;
}

static const struct of_device_id brcm_mbox_of_match[] = {
	{ .compatible = "brcm,brcmstb-mbox", },
	{}
};
MODULE_DEVICE_TABLE(of, brcm_mbox_of_match);

static struct platform_driver brcm_mbox_driver = {
	.probe = brcm_mbox_probe,
	.remove = brcm_mbox_remove,
	.driver = {
		.name = "brcm_mbox",
		.of_match_table = brcm_mbox_of_match,
	},
};

module_platform_driver(brcm_mbox_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Broadcom STB SCMI Mailbox driver");
MODULE_AUTHOR("Broadcom");
