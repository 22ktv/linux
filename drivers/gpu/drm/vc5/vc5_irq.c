/*
 * Copyright © 2014 Broadcom
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/**
 * DOC: Interrupt management for the V3D engine
 *
 * When we take a binning flush done interrupt, we need to submit the
 * next frame for binning and move the finished frame to the render
 * thread.
 *
 * When we take a render frame interrupt, we need to wake the
 * processes waiting for some frame to be done, and get the next frame
 * submitted ASAP (so the hardware doesn't sit idle when there's work
 * to do).
 *
 * When we take the binner out of memory interrupt, we need to
 * allocate some new memory and pass it to the binner so that the
 * current job can make progress.
 */

#include "vc5_drv.h"
#include "vc5_regs.h"

#define V3D_CORE_IRQS ((uint32_t)(V3D_INT_OUTOMEM |	\
				  V3D_INT_FLDONE |	\
				  V3D_INT_FRDONE |	\
				  V3D_INT_GMPV))

#define V3D_HUB_IRQS ((uint32_t)(V3D_HUB_INT_MMU_WRV |	\
				 V3D_HUB_INT_MMU_PTI |	\
				 V3D_HUB_INT_MMU_CAP))

static void
vc5_overflow_mem_work(struct work_struct *work)
{
	struct vc5_dev *vc5 =
		container_of(work, struct vc5_dev, overflow_mem_work);
	struct drm_device *dev = vc5->drm;
	struct vc5_bo *bo = vc5_bo_create(dev, NULL /* XXX */, 256 * 1024);
	struct vc5_exec_info *exec;

	if (IS_ERR(bo)) {
		DRM_ERROR("Couldn't allocate binner overflow mem\n");
		return;
	}

	/* If there's a job executing currently, then our previous
	 * overflow allocation is getting used in that job and we need
	 * to queue it to be released when the job is done.  But if no
	 * job is executing at all, then we can free the old overflow
	 * object direcctly.
	 *
	 * No lock necessary for this pointer since we're the only
	 * ones that update the pointer, and our workqueue won't
	 * reenter.
	 */
	if (vc5->overflow_bo) {
		unsigned long irqflags;
		spin_lock_irqsave(&vc5->job_lock, irqflags);
		exec = vc5_first_bin_job(vc5);
		if (!exec)
			exec = vc5_last_render_job(vc5);
		if (exec) {
			vc5->overflow_bo->seqno = exec->seqno;
			list_add_tail(&vc5->overflow_bo->unref_head,
				      &exec->unref_list);
		} else {
			drm_gem_object_put_unlocked(&vc5->overflow_bo->base);
		}
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
	}
	vc5->overflow_bo = bo;

	V3D_WRITE(V3D_PTB_0_BPOA, bo->node.start << PAGE_SHIFT);
	V3D_WRITE(V3D_PTB_0_BPOS, bo->base.size);
}

irqreturn_t
vc5_irq(int irq, void *arg)
{
	struct vc5_dev *vc5 = arg;
	uint32_t intsts;
	irqreturn_t status = IRQ_NONE;

	intsts = V3D_READ(V3D_CTL_0_INT_STS);

	/* Acknowledge the interrupts we're handling here. */
	V3D_WRITE(V3D_CTL_0_INT_CLR, intsts);

	if (intsts & V3D_INT_OUTOMEM) {
		/* Note that the OOM status is edge signaled, so the
		 * interrupt won't happen again until the we actually
		 * add more memory.
		 */
		schedule_work(&vc5->overflow_mem_work);
		status = IRQ_HANDLED;
	}

	if (intsts & V3D_INT_FLDONE) {
		spin_lock(&vc5->job_lock);
		vc5_finish_bin_job(vc5);
		spin_unlock(&vc5->job_lock);
		status = IRQ_HANDLED;
	}

	if (intsts & V3D_INT_FRDONE) {
		spin_lock(&vc5->job_lock);
		vc5_finish_render_job(vc5);
		spin_unlock(&vc5->job_lock);
		status = IRQ_HANDLED;
	}

	/* We shouldn't be triggering these if we have GMP in
	 * always-allowed mode.
	 */
	if (intsts & V3D_INT_GMPV)
		dev_err(vc5->dev, "GMP violation\n");

	return status;
}

irqreturn_t
vc5_hub_irq(int irq, void *arg)
{
	struct vc5_dev *vc5 = arg;
	uint32_t intsts;
	irqreturn_t status = IRQ_NONE;

	intsts = V3D_READ(V3D_HUB_INT_STS);

	/* Acknowledge the interrupts we're handling here. */
	V3D_WRITE(V3D_HUB_INT_CLR, intsts);

	if (intsts & (V3D_HUB_INT_MMU_WRV |
		      V3D_HUB_INT_MMU_PTI |
		      V3D_HUB_INT_MMU_CAP)) {
		uint32_t axi_id = V3D_READ(V3D_MMU_0_VIO_ID);
		uint64_t vio_addr = (uint64_t)V3D_READ(V3D_MMU_0_VIO_ADDR) << 8;

		dev_err(vc5->dev, "MMU error from client %d at 0x%08llx%s%s%s\n",
			axi_id, (long long)vio_addr,
			((intsts & V3D_HUB_INT_MMU_WRV) ?
			 ", write violation" : ""),
			((intsts & V3D_HUB_INT_MMU_PTI) ?
			 ", pte invalid" : ""),
			((intsts & V3D_HUB_INT_MMU_CAP) ?
			 ", cap exceeded" : ""));
	}

	return status;
}

void
vc5_irq_init(struct vc5_dev *vc5)
{
	int ret;

	init_waitqueue_head(&vc5->job_wait_queue);
	INIT_WORK(&vc5->overflow_mem_work, vc5_overflow_mem_work);

	/* Clear any pending interrupts someone might have left around
	 * for us.
	 */
	V3D_WRITE(V3D_CTL_0_INT_CLR, V3D_CORE_IRQS);
	V3D_WRITE(V3D_HUB_INT_CLR, V3D_HUB_IRQS);

	ret = devm_request_irq(vc5->dev, platform_get_irq(vc5->pdev, 1),
			       vc5_hub_irq, IRQF_SHARED,
			       "vc5_hub", vc5);
	ret = devm_request_irq(vc5->dev, platform_get_irq(vc5->pdev, 1),
			       vc5_irq, IRQF_SHARED,
			       "vc5_core", vc5);
	if (ret)
		dev_err(vc5->dev, "IRQ setup failed: %d\n", ret);

	vc5_irq_enable(vc5);
}

void
vc5_irq_enable(struct vc5_dev *vc5)
{
	/* Enable our set of interrupts, masking out any others. */
	V3D_WRITE(V3D_CTL_0_INT_MSK_SET, ~V3D_CORE_IRQS);
	V3D_WRITE(V3D_CTL_0_INT_MSK_CLR, V3D_CORE_IRQS);

	V3D_WRITE(V3D_HUB_INT_MSK_SET, ~V3D_HUB_IRQS);
	V3D_WRITE(V3D_HUB_INT_MSK_CLR, V3D_HUB_IRQS);
}

void
vc5_irq_disable(struct vc5_dev *vc5)
{
	/* Disable all interrupts. */
	V3D_WRITE(V3D_CTL_0_INT_MSK_SET, ~0);
	V3D_WRITE(V3D_HUB_INT_MSK_SET, ~0);

	/* Clear any pending interrupts we might have left. */
	V3D_WRITE(V3D_CTL_0_INT_CLR, V3D_CORE_IRQS);
	V3D_WRITE(V3D_HUB_INT_CLR, V3D_HUB_IRQS);

	cancel_work_sync(&vc5->overflow_mem_work);
}

/** Reinitializes interrupt registers when a GPU reset is performed. */
void vc5_irq_reset(struct vc5_dev *vc5)
{
	unsigned long irqflags;

	/*
	 * Turn all our interrupts on.  Binner out of memory is the
	 * only one we expect to trigger at this point, since we've
	 * just come from poweron and haven't supplied any overflow
	 * memory yet.
	 */
	vc5_irq_enable(vc5);

	spin_lock_irqsave(&vc5->job_lock, irqflags);
	vc5_cancel_bin_job(vc5);
	vc5_finish_render_job(vc5);
	spin_unlock_irqrestore(&vc5->job_lock, irqflags);
}
