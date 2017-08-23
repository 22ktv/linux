// SPDX-License-Identifier: GPL-2.0+
/* Copyright (C) 2014-2018 Broadcom */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/sched/signal.h>

#include "uapi/drm/vc5_drm.h"
#include "vc5_drv.h"
#include "vc5_regs.h"
#include "vc5_trace.h"

struct vc5_hang_state {
	struct drm_vc5_get_hang_state user_state;

	u32 bo_count;
	struct drm_gem_object **bo;
	struct drm_vc5_get_hang_state_reg *regs;
};

static void
vc5_init_core(struct vc5_dev *vc5, int core)
{
	/* Set OVRTMUOUT, which means that the texture sampler uniform
	 * configuration's tmu output type field is used, instead of
	 * using the hardware default behavior based on the texture
	 * type.  If you want the default behavior, you can still put
	 * "2" in the indirect texture state's output_type field.
	 */
	V3D_CORE_WRITE(core, V3D_CTL_MISCCFG, V3D_MISCCFG_OVRTMUOUT);

	/* Whenever we flush the L2T cache, we always want to flush
	 * the whole thing.
	 */
	V3D_CORE_WRITE(core, V3D_CTL_L2TFLSTA, 0);
	V3D_CORE_WRITE(core, V3D_CTL_L2TFLEND, ~0);
}

/* Sets invariant state for the HW. */
static void
vc5_init_hw_state(struct vc5_dev *vc5)
{
	vc5_init_core(vc5, 0);
}

static void
vc5_free_hang_state(struct drm_device *dev, struct vc5_hang_state *state)
{
	unsigned int i;

	for (i = 0; i < state->user_state.bo_count; i++)
		drm_gem_object_put_unlocked(state->bo[i]);

	kfree(state);
}

int
vc5_get_hang_state_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file_priv)
{
	struct drm_vc5_get_hang_state *get_state = data;
	struct drm_vc5_get_hang_state_bo *bo_state;
	struct vc5_hang_state *kernel_state;
	struct drm_vc5_get_hang_state *state;
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	unsigned long irqflags;
	u32 i;
	int ret = 0;

	spin_lock_irqsave(&vc5->job_lock, irqflags);
	kernel_state = vc5->hang_state;
	if (!kernel_state) {
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		return -ENOENT;
	}
	state = &kernel_state->user_state;

	/* If the user's arrays aren't big enough, just return the
	 * required array size.
	 */
	if (get_state->bo_count < state->bo_count ||
	    get_state->reg_count < state->reg_count) {
		get_state->bo_count = state->bo_count;
		get_state->reg_count = state->reg_count;
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		return 0;
	}

	vc5->hang_state = NULL;
	spin_unlock_irqrestore(&vc5->job_lock, irqflags);

	/* Save the user's arrays, so we don't stomp it with the memcpy. */
	state->bos = get_state->bos;
	state->regs = get_state->regs;
	memcpy(get_state, state, sizeof(*state));

	bo_state = kcalloc(state->bo_count, sizeof(*bo_state), GFP_KERNEL);
	if (!bo_state) {
		ret = -ENOMEM;
		goto err_free;
	}

	for (i = 0; i < state->bo_count; i++) {
		struct vc5_bo *vc5_bo = to_vc5_bo(kernel_state->bo[i]);
		u32 handle;

		ret = drm_gem_handle_create(file_priv, kernel_state->bo[i],
					    &handle);

		if (ret) {
			state->bo_count = i;
			goto err_delete_handle;
		}
		bo_state[i].handle = handle;
		bo_state[i].size = vc5_bo->base.size;
	}

	if (copy_to_user(u64_to_user_ptr(get_state->bos), bo_state,
			 state->bo_count * sizeof(*bo_state)))
		ret = -EFAULT;

	if (copy_to_user(u64_to_user_ptr(get_state->regs), kernel_state->regs,
			 state->reg_count * sizeof(*kernel_state->regs)))
		ret = -EFAULT;

err_delete_handle:
	if (ret) {
		for (i = 0; i < state->bo_count; i++)
			drm_gem_handle_delete(file_priv, bo_state[i].handle);
	}

err_free:
	vc5_free_hang_state(dev, kernel_state);
	kfree(bo_state);

	return ret;
}

struct vc5_hang_state_reg_block {
	uint32_t start_reg, end_reg;
};

static const struct vc5_hang_state_reg_block general_reg_blocks[] = {
	{ V3D_HUB_AXICFG, V3D_HUB_INT_MSK_STS },
	{ V3D_MMUC_CONTROL, V3D_MMUC_CONTROL },
};

static const struct vc5_hang_state_reg_block core_reg_blocks[] = {
	{ V3D_CLE_CT0CS, V3D_CLE_CT1QCFG },
	{ V3D_PTB_BPCA,  V3D_PTB_BXCF },
};

static int
vc5_save_hang_state_regs(struct vc5_dev *vc5,
			 struct vc5_hang_state *kernel_state)
{
	struct drm_vc5_get_hang_state *state = &kernel_state->user_state;
	struct drm_vc5_get_hang_state_reg *regs;
	uint32_t reg_count = 0, next_reg, i;

	for (i = 0; i < ARRAY_SIZE(general_reg_blocks); i++) {
		reg_count += (general_reg_blocks[i].end_reg -
			      general_reg_blocks[i].start_reg) / 4 + 1;
	}

	for (i = 0; i < ARRAY_SIZE(core_reg_blocks); i++) {
		/* XXX: Multiply by core count */
		reg_count += (core_reg_blocks[i].end_reg -
			      core_reg_blocks[i].start_reg) / 4 + 1;
	}

	regs = kcalloc(reg_count, sizeof(*regs), GFP_KERNEL);
	if (regs)
		return -ENOMEM;

	kernel_state->regs = regs;
	state->reg_count = reg_count;

	next_reg = 0;

	for (i = 0; i < ARRAY_SIZE(general_reg_blocks); i++) {
		uint32_t reg;
		for (reg = general_reg_blocks[i].start_reg;
		     reg <= general_reg_blocks[i].end_reg;
		     reg += 4) {
			kernel_state->regs[next_reg].offset = reg;
			kernel_state->regs[next_reg].value = V3D_READ(reg);
			next_reg++;
		}
	}

	for (i = 0; i < ARRAY_SIZE(core_reg_blocks); i++) {
		uint32_t reg;

		for (reg = core_reg_blocks[i].start_reg;
		     reg <= core_reg_blocks[i].end_reg;
		     reg += 4) {
			kernel_state->regs[next_reg].offset = reg;
			kernel_state->regs[next_reg].value =
				V3D_CORE_READ(0, reg);
			next_reg++;
		}
	}

	WARN_ON(next_reg != reg_count);

	return 0;
}

static void
vc5_save_hang_state(struct drm_device *dev)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	struct drm_vc5_get_hang_state *state;
	struct vc5_hang_state *kernel_state;
	struct vc5_exec_info *exec[2];
	struct vc5_bo *bo;
	unsigned long irqflags;
	unsigned int i, j, unref_list_count, prev_idx;

	kernel_state = kcalloc(1, sizeof(*kernel_state), GFP_KERNEL);
	if (!kernel_state)
		return;

	state = &kernel_state->user_state;

	spin_lock_irqsave(&vc5->job_lock, irqflags);
	exec[0] = vc5_first_bin_job(vc5);
	exec[1] = vc5_first_render_job(vc5);
	if (!exec[0] && !exec[1]) {
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		return;
	}

	/* Get the bos from both binner and renderer into hang state. */
	state->bo_count = 0;
	for (i = 0; i < 2; i++) {
		if (!exec[i])
			continue;

		unref_list_count = 0;
		list_for_each_entry(bo, &exec[i]->unref_list, unref_head)
			unref_list_count++;
		state->bo_count += exec[i]->bo_count + unref_list_count;
	}

	kernel_state->bo = kcalloc(state->bo_count,
				   sizeof(*kernel_state->bo), GFP_ATOMIC);

	if (!kernel_state->bo) {
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		return;
	}

	prev_idx = 0;
	for (i = 0; i < 2; i++) {
		if (!exec[i])
			continue;

		for (j = 0; j < exec[i]->bo_count; j++) {
			drm_gem_object_get(&exec[i]->bo[j]->base);
			kernel_state->bo[j + prev_idx] = &exec[i]->bo[j]->base;
		}

		list_for_each_entry(bo, &exec[i]->unref_list, unref_head) {
			drm_gem_object_get(&bo->base);
			kernel_state->bo[j + prev_idx] = &bo->base;
			j++;
		}
		prev_idx = j + 1;
	}

	/* XXX
	if (exec[0])
		state->start_bin = exec[0]->ct0ca;
	if (exec[1])
		state->start_render = exec[1]->ct1ca;
	*/

	spin_unlock_irqrestore(&vc5->job_lock, irqflags);

	vc5_save_hang_state_regs(vc5, kernel_state);

	spin_lock_irqsave(&vc5->job_lock, irqflags);
	if (vc5->hang_state) {
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		vc5_free_hang_state(dev, kernel_state);
	} else {
		vc5->hang_state = kernel_state;
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
	}
}

static void
vc5_queue_hangcheck(struct vc5_dev *vc5)
{
	mod_timer(&vc5->hangcheck.timer,
		  round_jiffies_up(jiffies + msecs_to_jiffies(100)));
}

static void
vc5_idle_axi(struct vc5_dev *vc5, int core)
{
	V3D_CORE_WRITE(core, V3D_GMP_CFG, V3D_GMP_CFG_STOP_REQ);

	if (wait_for((V3D_CORE_READ(core, V3D_GMP_STATUS) &
		      (V3D_GMP_STATUS_RD_COUNT_MASK |
		       V3D_GMP_STATUS_WR_COUNT_MASK |
		       V3D_GMP_STATUS_CFG_BUSY)) == 0, 100)) {
		DRM_ERROR("Failed to wait for safe GMP shutdown\n");
	}
}

static void
vc5_idle_gca(struct vc5_dev *vc5)
{
	if (vc5->ver >= 41)
		return;

	V3D_WRITE(V3D_GCA_SAFE_SHUTDOWN, V3D_GCA_SAFE_SHUTDOWN_EN);

	if (wait_for((V3D_READ(V3D_GCA_SAFE_SHUTDOWN_ACK) &
		      V3D_GCA_SAFE_SHUTDOWN_ACK_ACKED) ==
		     V3D_GCA_SAFE_SHUTDOWN_ACK_ACKED, 100)) {
		DRM_ERROR("Failed to wait for safe GCA shutdown\n");
	}
}

static void
vc5_reset_v3d(struct vc5_dev *vc5)
{
	/* XXX: Need to make this a separate register region, as it's
	 * very different on 7278 vs 7268
	 */
	V3D_WRITE(V3D_TOP_GR_BRIDGE_SW_INIT_0,
		  V3D_TOP_GR_BRIDGE_SW_INIT_0_V3D_CLK_108_SW_INIT);
	V3D_WRITE(V3D_TOP_GR_BRIDGE_SW_INIT_0, 0);

	/* GFXH-1383: The SW_INIT may cause a stray write to address 0
	 * of the unit, so reset it to its power-on value here.
	 */
	V3D_WRITE(V3D_HUB_AXICFG, V3D_HUB_AXICFG_MAX_LEN_MASK);

	vc5_init_hw_state(vc5);
}

static void
vc5_reset_work(struct work_struct *work)
{
	struct vc5_dev *vc5 =
		container_of(work, struct vc5_dev, hangcheck.reset_work);
	struct drm_device *dev = vc5->drm;

	vc5_save_hang_state(dev);

	DRM_ERROR("Resetting GPU.\n");
	trace_vc5_reset_begin(dev);

	/* XXX: only needed for safe powerdown, not reset. */
	if (false)
		vc5_idle_axi(vc5, 0);

	vc5_idle_gca(vc5);
	vc5_reset_v3d(vc5);

	vc5_mmu_set_page_table(vc5);
	vc5_irq_reset(vc5);

	trace_vc5_reset_end(dev);

	/* Rearm the hangcheck -- another job might have been waiting
	 * for our hung one to get kicked off, and vc5_irq_reset()
	 * would have started it.
	 */
	vc5_queue_hangcheck(vc5);
}

static void
vc5_hangcheck_elapsed(struct timer_list *t)
{
	struct vc5_dev *vc5 = from_timer(vc5, t, hangcheck.timer);
	struct drm_device *dev = vc5->drm;
	uint32_t ct0ca, ct1ca;
	unsigned long irqflags;
	struct vc5_exec_info *bin_exec, *render_exec;

	spin_lock_irqsave(&vc5->job_lock, irqflags);

	bin_exec = vc5_first_bin_job(vc5);
	render_exec = vc5_first_render_job(vc5);

	/* If idle, we can stop watching for hangs. */
	if (!bin_exec && !render_exec) {
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		return;
	}

	/* XXX: multicore */
	ct0ca = V3D_CORE_READ(0, V3D_CLE_CT0CA);
	ct1ca = V3D_CORE_READ(0, V3D_CLE_CT1CA);

	trace_vc5_hangcheck(dev,
			    bin_exec ? bin_exec->seqno : 0,
			    render_exec ? render_exec->seqno : 0,
			    ct0ca, ct1ca,
			    bin_exec ? bin_exec->last_ct0ca : 0,
			    render_exec ? render_exec->last_ct1ca : 0);
	/* If we've made any progress in execution, rearm the timer
	 * and wait.
	 */
	if ((bin_exec && ct0ca != bin_exec->last_ct0ca) ||
	    (render_exec && ct1ca != render_exec->last_ct1ca)) {
		if (bin_exec)
			bin_exec->last_ct0ca = ct0ca;
		if (render_exec)
			render_exec->last_ct1ca = ct1ca;
		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		vc5_queue_hangcheck(vc5);
		return;
	}

	spin_unlock_irqrestore(&vc5->job_lock, irqflags);

	/* We've gone too long with no progress, reset.  This has to
	 * be done from a work struct, since resetting can sleep and
	 * this timer hook isn't allowed to.
	 */
	schedule_work(&vc5->hangcheck.reset_work);
}

static void
submit_cl(struct drm_device *dev, uint32_t thread, int core,
	  uint32_t start, uint32_t end)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);

	/* Set the current and end address of the control list.
	 * Writing the end register is what starts the job.
	 */
	V3D_CORE_WRITE(core, V3D_CLE_CTNQBA(thread), start);
	V3D_CORE_WRITE(core, V3D_CLE_CTNQEA(thread), end);
}

int
vc5_wait_for_seqno(struct drm_device *dev, uint64_t seqno, uint64_t timeout_ns,
		   bool interruptible)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	int ret = 0;
	unsigned long timeout_expire;
	DEFINE_WAIT(wait);

	if (vc5->finished_seqno >= seqno)
		return 0;

	if (timeout_ns == 0)
		return -ETIME;

	timeout_expire = jiffies + nsecs_to_jiffies(timeout_ns);

	trace_vc5_wait_for_seqno_begin(dev, seqno, timeout_ns);
	for (;;) {
		prepare_to_wait(&vc5->job_wait_queue, &wait,
				interruptible ? TASK_INTERRUPTIBLE :
				TASK_UNINTERRUPTIBLE);

		if (interruptible && signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		if (vc5->finished_seqno >= seqno)
			break;

		if (timeout_ns != ~0ull) {
			if (time_after_eq(jiffies, timeout_expire)) {
				ret = -ETIME;
				break;
			}
			schedule_timeout(timeout_expire - jiffies);
		} else {
			schedule();
		}
	}

	finish_wait(&vc5->job_wait_queue, &wait);
	trace_vc5_wait_for_seqno_end(dev, seqno);

	return ret;
}

static void
vc5_flush_l3(struct vc5_dev *vc5)
{
	if (vc5->ver < 41) {
		uint32_t gca_ctrl = V3D_READ(V3D_GCA_CACHE_CTRL);

		V3D_WRITE(V3D_GCA_CACHE_CTRL,
			  gca_ctrl | V3D_GCA_CACHE_CTRL_FLUSH);

		if (vc5->ver < 33) {
			V3D_WRITE(V3D_GCA_CACHE_CTRL,
				  gca_ctrl & ~V3D_GCA_CACHE_CTRL_FLUSH);
		}
	}
}

/* Invalidates the (read-only) L2 cache. */
static void
vc5_invalidate_l2(struct vc5_dev *vc5, int core)
{
	V3D_CORE_WRITE(core, V3D_CTL_L2CACTL,
		       V3D_L2CACTL_L2CCLR |
		       V3D_L2CACTL_L2CENA);
}

static void
vc5_invalidate_l1td(struct vc5_dev *vc5, int core)
{
	V3D_CORE_WRITE(core, V3D_CTL_L2TCACTL, V3D_L2TCACTL_TMUWCF);
	if (wait_for(!(V3D_CORE_READ(core, V3D_CTL_L2TCACTL) &
		       V3D_L2TCACTL_L2TFLS), 100)) {
		DRM_ERROR("Timeout waiting for L1T write combiner flush\n");
	}
}

/* Invalidates texture L2 cachelines */
static void
vc5_flush_l2t(struct vc5_dev *vc5, int core)
{
	vc5_invalidate_l1td(vc5, core);

	V3D_CORE_WRITE(core, V3D_CTL_L2TCACTL,
		       V3D_L2TCACTL_L2TFLS |
		       VC5_SET_FIELD(V3D_L2TCACTL_FLM_FLUSH, V3D_L2TCACTL_FLM));
	if (wait_for(!(V3D_CORE_READ(core, V3D_CTL_L2TCACTL) &
		       V3D_L2TCACTL_L2TFLS), 100)) {
		DRM_ERROR("Timeout waiting for L2T flush\n");
	}
}

/* Invalidates the slice caches.  These are read-only caches. */
static void
vc5_invalidate_slices(struct vc5_dev *vc5, int core)
{
	V3D_CORE_WRITE(core, V3D_CTL_SLCACTL,
		       VC5_SET_FIELD(0xf, V3D_SLCACTL_TVCCS) |
		       VC5_SET_FIELD(0xf, V3D_SLCACTL_TDCCS) |
		       VC5_SET_FIELD(0xf, V3D_SLCACTL_UCC) |
		       VC5_SET_FIELD(0xf, V3D_SLCACTL_ICC));
}

/* Invalidates texture L2 cachelines */
static void
vc5_invalidate_l2t(struct vc5_dev *vc5, int core)
{
	V3D_CORE_WRITE(core,
		       V3D_CTL_L2TCACTL,
		       V3D_L2TCACTL_L2TFLS |
		       VC5_SET_FIELD(V3D_L2TCACTL_FLM_CLEAR, V3D_L2TCACTL_FLM));
	if (wait_for(!(V3D_CORE_READ(core, V3D_CTL_L2TCACTL) &
		       V3D_L2TCACTL_L2TFLS), 100)) {
		DRM_ERROR("Timeout waiting for L2T invalidate\n");
	}
}

static void
vc5_invalidate_caches(struct vc5_dev *vc5)
{
	vc5_flush_l3(vc5);

	vc5_invalidate_l2(vc5, 0);
	vc5_invalidate_slices(vc5, 0);
	vc5_flush_l2t(vc5, 0);
}

static void
vc5_flush_caches(struct vc5_dev *vc5)
{
	vc5_invalidate_l1td(vc5, 0);
	vc5_invalidate_l2t(vc5, 0);
}

/* Sets the registers for the next job to be actually be executed in
 * the hardware.
 *
 * The job_lock should be held during this.
 */
void
vc5_submit_next_bin_job(struct drm_device *dev)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	struct vc5_exec_info *exec;

again:
	exec = vc5_first_bin_job(vc5);
	if (!exec)
		return;

	vc5_invalidate_caches(vc5);

	/* Either put the job in the binner if it uses the binner, or
	 * immediately move it to the to-be-rendered queue.
	 */
	if (exec->ct0qba != exec->ct0qea) {
		trace_vc5_submit_cl(dev, false,
				    exec->seqno,
				    exec->ct0qba, exec->ct0qea);

		if (exec->qma) {
			V3D_CORE_WRITE(0, V3D_CLE_CT0QMA, exec->qma);
			V3D_CORE_WRITE(0, V3D_CLE_CT0QMS, exec->qms);
		}
		if (exec->qts) {
			V3D_CORE_WRITE(0, V3D_CLE_CT0QTS,
				       V3D_CLE_CT0QTS_ENABLE |
				       exec->qts);
		}

		submit_cl(dev, 0, 0, exec->ct0qba, exec->ct0qea);
	} else {
		vc5_move_job_to_render(dev, exec);
		goto again;
	}
}

void
vc5_submit_next_render_job(struct drm_device *dev)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	struct vc5_exec_info *exec;

again:
	exec = vc5_first_render_job(vc5);
	if (!exec)
		return;

	/* Can we avoid this flush?  We need to be careful of
	 * scheduling, though -- imagine job0 rendering to texture and
	 * job1 reading, and them being executed as bin0, bin1,
	 * render0, render1, so that render1's flush at bin time
	 * wasn't enough.
	 */
	vc5_invalidate_caches(vc5);

	/* XXX: Set the QCFG */

	if (!exec->hung) {
		trace_vc5_submit_cl(dev, true,
				    exec->seqno,
				    exec->ct1qba, exec->ct1qea);
		submit_cl(dev, 1, 0, exec->ct1qba, exec->ct1qea);
	} else {
		vc5_finish_render_job(vc5);
		goto again;
	}
}

void
vc5_move_job_to_render(struct drm_device *dev, struct vc5_exec_info *exec)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	bool was_empty = list_empty(&vc5->render_job_list);

	list_move_tail(&exec->head, &vc5->render_job_list);
	if (was_empty)
		vc5_submit_next_render_job(dev);
}

void
vc5_finish_bin_job(struct vc5_dev *vc5)
{
	struct drm_device *dev = vc5->drm;
	struct vc5_exec_info *exec = vc5_first_bin_job(vc5);

	vc5_flush_caches(vc5);
	trace_vc5_finish_bin_job(dev, exec ? exec->seqno : 0);

	if (!exec)
		return;

	vc5_move_job_to_render(vc5->drm, exec);
	vc5_submit_next_bin_job(vc5->drm);
}

/* Called when we've reset the hung GPU, to move this binner job on
 * through the pipeline without executing any more of it.
 */
void
vc5_cancel_bin_job(struct vc5_dev *vc5)
{
	struct drm_device *dev = vc5->drm;
	struct vc5_exec_info *exec = vc5_first_bin_job(vc5);

	if (!exec)
		return;

	trace_vc5_cancel_bin_job(dev, exec->seqno);
	exec->hung = true;
	vc5_finish_bin_job(vc5);
}

void
vc5_finish_render_job(struct vc5_dev *vc5)
{
	struct drm_device *dev = vc5->drm;
	struct vc5_exec_info *exec = vc5_first_render_job(vc5);

	vc5_flush_caches(vc5);
	trace_vc5_finish_render_job(dev, exec ? exec->seqno : 0);

	if (!exec)
		return;

	vc5->finished_seqno++;
	list_move_tail(&exec->head, &vc5->job_done_list);
	if (exec->fence) {
		dma_fence_signal_locked(exec->fence);
		dma_fence_put(exec->fence);
		exec->fence = NULL;
	}
	vc5_submit_next_render_job(vc5->drm);

	wake_up_all(&vc5->job_wait_queue);
	schedule_work(&vc5->job_done_work);
}

static void
vc5_update_bo_seqnos(struct vc5_exec_info *exec, uint64_t seqno)
{
	struct vc5_bo *bo;
	unsigned i;

	for (i = 0; i < exec->bo_count; i++) {
		bo = to_vc5_bo(&exec->bo[i]->base);
		bo->seqno = seqno;

		reservation_object_add_shared_fence(bo->resv, exec->fence);
	}

	list_for_each_entry(bo, &exec->unref_list, unref_head) {
		bo->seqno = seqno;
	}
}

static void
vc5_unlock_bo_reservations(struct drm_device *dev,
			   struct vc5_exec_info *exec,
			   struct ww_acquire_ctx *acquire_ctx)
{
	int i;

	for (i = 0; i < exec->bo_count; i++) {
		struct vc5_bo *bo = to_vc5_bo(&exec->bo[i]->base);

		ww_mutex_unlock(&bo->resv->lock);
	}

	ww_acquire_fini(acquire_ctx);
}

/* Takes the reservation lock on all the BOs being referenced, so that
 * at queue submit time we can update the reservations.
 *
 * We don't lock the RCL the tile alloc/state BOs, or overflow memory
 * (all of which are on exec->unref_list).  They're entirely private
 * to vc5, so we don't attach dma-buf fences to them.
 */
static int
vc5_lock_bo_reservations(struct drm_device *dev,
			 struct vc5_exec_info *exec,
			 struct ww_acquire_ctx *acquire_ctx)
{
	int contended_lock = -1;
	int i, ret;
	struct vc5_bo *bo;

	ww_acquire_init(acquire_ctx, &reservation_ww_class);

retry:
	if (contended_lock != -1) {
		bo = to_vc5_bo(&exec->bo[contended_lock]->base);
		ret = ww_mutex_lock_slow_interruptible(&bo->resv->lock,
						       acquire_ctx);
		if (ret) {
			ww_acquire_done(acquire_ctx);
			return ret;
		}
	}

	for (i = 0; i < exec->bo_count; i++) {
		if (i == contended_lock)
			continue;

		bo = to_vc5_bo(&exec->bo[i]->base);

		ret = ww_mutex_lock_interruptible(&bo->resv->lock, acquire_ctx);
		if (ret) {
			int j;

			for (j = 0; j < i; j++) {
				bo = to_vc5_bo(&exec->bo[j]->base);
				ww_mutex_unlock(&bo->resv->lock);
			}

			if (contended_lock != -1 && contended_lock >= i) {
				bo = to_vc5_bo(&exec->bo[contended_lock]->base);

				ww_mutex_unlock(&bo->resv->lock);
			}

			if (ret == -EDEADLK) {
				contended_lock = i;
				goto retry;
			}

			ww_acquire_done(acquire_ctx);
			return ret;
		}
	}

	ww_acquire_done(acquire_ctx);

	/* Reserve space for our shared (read-only) fence references,
	 * before we commit the CL to the hardware.
	 */
	for (i = 0; i < exec->bo_count; i++) {
		bo = to_vc5_bo(&exec->bo[i]->base);

		ret = reservation_object_reserve_shared(bo->resv);
		if (ret) {
			vc5_unlock_bo_reservations(dev, exec, acquire_ctx);
			return ret;
		}
	}

	return 0;
}

/* Queues a struct vc5_exec_info for execution.  If no job is
 * currently executing, then submits it.
 *
 * Unlike most GPUs, our hardware only handles one command list at a
 * time.  To queue multiple jobs at once, we'd need to edit the
 * previous command list to have a jump to the new one at the end, and
 * then bump the end address.  That's a change for a later date,
 * though.
 */
static int
vc5_queue_submit(struct drm_device *dev, struct vc5_exec_info *exec,
		 struct ww_acquire_ctx *acquire_ctx)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	uint64_t seqno;
	unsigned long irqflags;
	struct vc5_fence *fence;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return -ENOMEM;
	fence->dev = dev;

	spin_lock_irqsave(&vc5->job_lock, irqflags);

	seqno = ++vc5->emit_seqno;
	exec->seqno = seqno;

	dma_fence_init(&fence->base, &vc5_fence_ops, &vc5->job_lock,
		       vc5->dma_fence_context, exec->seqno);
	fence->seqno = exec->seqno;
	exec->fence = &fence->base;

	vc5_update_bo_seqnos(exec, seqno);

	vc5_unlock_bo_reservations(dev, exec, acquire_ctx);

	list_add_tail(&exec->head, &vc5->bin_job_list);

	/* If no job was executing, kick ours off.  Otherwise, it'll
	 * get started when the previous job's flush done interrupt
	 * occurs.
	 */
	if (vc5_first_bin_job(vc5) == exec) {
		vc5_submit_next_bin_job(dev);
		vc5_queue_hangcheck(vc5);
	}

	spin_unlock_irqrestore(&vc5->job_lock, irqflags);

	return 0;
}

/**
 * vc5_cl_lookup_bos() - Sets up exec->bo[] with the GEM objects
 * referenced by the job.
 * @dev: DRM device
 * @file_priv: DRM file for this fd
 * @exec: V3D job being set up
 *
 * The command validator needs to reference BOs by their index within
 * the submitted job's BO list.  This does the validation of the job's
 * BO list and reference counting for the lifetime of the job.
 *
 * Note that this function doesn't need to unreference the BOs on
 * failure, because that will happen at vc5_complete_exec() time.
 */
static int
vc5_cl_lookup_bos(struct drm_device *dev,
		  struct drm_file *file_priv,
		  struct vc5_exec_info *exec)
{
	struct drm_vc5_submit_cl *args = exec->args;
	uint32_t *handles;
	int ret = 0;
	int i;

	exec->bo_count = args->bo_handle_count;

	if (!exec->bo_count) {
		/* See comment on bo_index for why we have to check
		 * this.
		 */
		DRM_DEBUG("Rendering requires BOs\n");
		return -EINVAL;
	}

	exec->bo = kvmalloc_array(exec->bo_count,
				    sizeof(struct drm_gem_cma_object *),
				    GFP_KERNEL | __GFP_ZERO);
	if (!exec->bo) {
		DRM_DEBUG("Failed to allocate validated BO pointers\n");
		return -ENOMEM;
	}

	handles = kvmalloc_array(exec->bo_count, sizeof(uint32_t), GFP_KERNEL);
	if (!handles) {
		ret = -ENOMEM;
		DRM_DEBUG("Failed to allocate incoming GEM handles\n");
		goto fail;
	}

	if (copy_from_user(handles,
			   (void __user *)(uintptr_t)args->bo_handles,
			   exec->bo_count * sizeof(uint32_t))) {
		ret = -EFAULT;
		DRM_DEBUG("Failed to copy in GEM handles\n");
		goto fail;
	}

	spin_lock(&file_priv->table_lock);
	for (i = 0; i < exec->bo_count; i++) {
		struct drm_gem_object *bo = idr_find(&file_priv->object_idr,
						     handles[i]);
		if (!bo) {
			DRM_DEBUG("Failed to look up GEM BO %d: %d\n",
				  i, handles[i]);
			ret = -ENOENT;
			spin_unlock(&file_priv->table_lock);
			goto fail;
		}
		drm_gem_object_get(bo);
		exec->bo[i] = to_vc5_bo(bo);
	}
	spin_unlock(&file_priv->table_lock);

fail:
	kvfree(handles);
	return ret;
}

static void
vc5_complete_exec(struct vc5_dev *vc5, struct vc5_exec_info *exec)
{
	unsigned i;

	/* If we got force-completed because of GPU reset rather than
	 * through our IRQ handler, signal the fence now.
	 */
	if (exec->fence) {
		dma_fence_signal(exec->fence);
		dma_fence_put(exec->fence);
	}

	for (i = 0; i < exec->bo_count; i++) {
		/* XXX: unpin */
		drm_gem_object_unreference_unlocked(&exec->bo[i]->base);
	}
	kvfree(exec->bo);

	while (!list_empty(&exec->unref_list)) {
		struct vc5_bo *bo = list_first_entry(&exec->unref_list,
						     struct vc5_bo, unref_head);
		list_del(&bo->unref_head);
		drm_gem_object_unreference_unlocked(&bo->base);
	}

	pm_runtime_mark_last_busy(vc5->dev);
	pm_runtime_put_autosuspend(vc5->dev);

	kfree(exec);
}

void
vc5_job_handle_completed(struct vc5_dev *vc5)
{
	unsigned long irqflags;

	spin_lock_irqsave(&vc5->job_lock, irqflags);
	while (!list_empty(&vc5->job_done_list)) {
		struct vc5_exec_info *exec =
			list_first_entry(&vc5->job_done_list,
					 struct vc5_exec_info, head);
		list_del(&exec->head);

		spin_unlock_irqrestore(&vc5->job_lock, irqflags);
		vc5_complete_exec(vc5, exec);
		spin_lock_irqsave(&vc5->job_lock, irqflags);
	}

	spin_unlock_irqrestore(&vc5->job_lock, irqflags);
}

/* Scheduled when any job has been completed, this walks the list of
 * jobs that had completed and unrefs their BOs and frees their exec
 * structs.
 */
static void
vc5_job_done_work(struct work_struct *work)
{
	struct vc5_dev *vc5 =
		container_of(work, struct vc5_dev, job_done_work);

	vc5_job_handle_completed(vc5);
}

static int
vc5_wait_for_seqno_ioctl_helper(struct drm_device *dev,
				uint64_t seqno,
				uint64_t *timeout_ns)
{
	unsigned long start = jiffies;
	int ret = vc5_wait_for_seqno(dev, seqno, *timeout_ns, true);

	if ((ret == -EINTR || ret == -ERESTARTSYS) && *timeout_ns != ~0ull) {
		uint64_t delta = jiffies_to_nsecs(jiffies - start);

		if (*timeout_ns >= delta)
			*timeout_ns -= delta;
	}

	return ret;
}

int
vc5_wait_seqno_ioctl(struct drm_device *dev, void *data,
		     struct drm_file *file_priv)
{
	struct drm_vc5_wait_seqno *args = data;

	return vc5_wait_for_seqno_ioctl_helper(dev, args->seqno,
					       &args->timeout_ns);
}

int
vc5_wait_bo_ioctl(struct drm_device *dev, void *data,
		  struct drm_file *file_priv)
{
	int ret;
	struct drm_vc5_wait_bo *args = data;
	struct drm_gem_object *gem_obj;
	struct vc5_bo *bo;

	if (args->pad != 0)
		return -EINVAL;

	gem_obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!gem_obj) {
		DRM_DEBUG("Failed to look up GEM BO %d\n", args->handle);
		return -EINVAL;
	}
	bo = to_vc5_bo(gem_obj);

	ret = vc5_wait_for_seqno_ioctl_helper(dev, bo->seqno,
					      &args->timeout_ns);

	drm_gem_object_unreference_unlocked(gem_obj);
	return ret;
}

/**
 * vc5_submit_cl_ioctl() - Submits a job (frame) to the VC5.
 * @dev: DRM device
 * @data: ioctl argument
 * @file_priv: DRM file for this fd
 *
 * This is the main entrypoint for userspace to submit a 3D frame to
 * the GPU.  Userspace provides the binner command list (if
 * applicable), and the kernel sets up the render command list to draw
 * to the framebuffer described in the ioctl, using the command lists
 * that the 3D engine's binner will produce.
 */
int
vc5_submit_cl_ioctl(struct drm_device *dev, void *data,
		    struct drm_file *file_priv)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	struct drm_vc5_submit_cl *args = data;
	struct vc5_exec_info *exec;
	struct ww_acquire_ctx acquire_ctx;
	int ret = 0;

	if (args->flags != 0) {
		DRM_DEBUG("Unknown flags: 0x%16llx\n", (long long)args->flags);
		return -EINVAL;
	}

	exec = kcalloc(1, sizeof(*exec), GFP_KERNEL);
	if (!exec)
		return -ENOMEM;

	ret = pm_runtime_get_sync(vc5->dev);
	if (ret < 0) {
		kfree(exec);
		return ret;
	}

	exec->qma = args->qma;
	exec->qms = args->qms;
	exec->qts = args->qts;
	exec->ct0qba = args->bcl_start;
	exec->ct0qea = args->bcl_end;
	exec->ct1qba = args->rcl_start;
	exec->ct1qea = args->rcl_end;
	exec->args = args;
	exec->vc5_priv = file_priv->driver_priv;
	INIT_LIST_HEAD(&exec->unref_list);

	ret = vc5_cl_lookup_bos(dev, file_priv, exec);
	if (ret)
		goto fail;

	ret = vc5_lock_bo_reservations(dev, exec, &acquire_ctx);
	if (ret)
		goto fail;

	/* Clear this out of the struct we'll be putting in the queue,
	 * since it's part of our stack.
	 */
	exec->args = NULL;

	ret = vc5_queue_submit(dev, exec, &acquire_ctx);
	if (ret)
		goto fail;

	return 0;

fail:
	vc5_complete_exec(vc5, exec);

	return ret;
}

int
vc5_gem_init(struct drm_device *dev)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	u32 pt_size = 4096 * 1024;

	vc5->dma_fence_context = dma_fence_context_alloc(1);

	INIT_LIST_HEAD(&vc5->bin_job_list);
	INIT_LIST_HEAD(&vc5->render_job_list);
	INIT_LIST_HEAD(&vc5->job_done_list);
	spin_lock_init(&vc5->job_lock);
	mutex_init(&vc5->bo_lock);

	INIT_WORK(&vc5->hangcheck.reset_work, vc5_reset_work);
	timer_setup(&vc5->hangcheck.timer, vc5_hangcheck_elapsed, 0);

	INIT_WORK(&vc5->job_done_work, vc5_job_done_work);

	/* Note: We don't allocate address 0.  Various bits of HW
	 * treat 0 as special, such as the occlusion query counters
	 * where 0 means "disabled".
	 */
	drm_mm_init(&vc5->mm, 1, pt_size / sizeof(u32) - 1);

	vc5->pt = dma_alloc_wc(vc5->dev, pt_size,
			       &vc5->pt_paddr,
			       GFP_KERNEL | __GFP_NOWARN | __GFP_ZERO);
	if (!vc5->pt) {
		dev_err(vc5->dev,
			"Failed to allocate page tables. "
			"Please ensure you have CMA enabled.\n");
		return -ENOMEM;
	}

	vc5_init_hw_state(vc5);
	vc5_mmu_set_page_table(vc5);

	return 0;
}

void
vc5_gem_destroy(struct drm_device *dev)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);

	/* Waiting for exec to finish would need to be done before
	 * unregistering V3D.
	 */
	WARN_ON(vc5->emit_seqno != vc5->finished_seqno);

	drm_mm_takedown(&vc5->mm);

	dma_free_wc(vc5->dev, 4096 * 1024, vc5->pt, vc5->pt_paddr);

	if (vc5->hang_state)
		vc5_free_hang_state(dev, vc5->hang_state);
}
