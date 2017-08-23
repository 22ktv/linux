// SPDX-License-Identifier: GPL-2.0+
/* Copyright (C) 2017-2018 Broadcom */

#include "vc5_drv.h"

static const char *vc5_fence_get_driver_name(struct dma_fence *fence)
{
	return "vc5";
}

static const char *vc5_fence_get_timeline_name(struct dma_fence *fence)
{
	return "vc5-v3d";
}

static bool vc5_fence_enable_signaling(struct dma_fence *fence)
{
	return true;
}

static bool vc5_fence_signaled(struct dma_fence *fence)
{
	struct vc5_fence *f = to_vc5_fence(fence);
	struct vc5_dev *vc5 = to_vc5_dev(f->dev);

	return vc5->finished_seqno >= f->seqno;
}

const struct dma_fence_ops vc5_fence_ops = {
	.get_driver_name = vc5_fence_get_driver_name,
	.get_timeline_name = vc5_fence_get_timeline_name,
	.enable_signaling = vc5_fence_enable_signaling,
	.signaled = vc5_fence_signaled,
	.wait = dma_fence_default_wait,
	.release = dma_fence_free,
};
