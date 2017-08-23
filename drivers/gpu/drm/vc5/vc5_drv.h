// SPDX-License-Identifier: GPL-2.0+
/* Copyright (C) 2015-2018 Broadcom */

#include <linux/reservation.h>
#include <drm/drmP.h>
#include <drm/drm_encoder.h>
#include <drm/drm_gem.h>

#define GMP_GRANULARITY (128 * 1024)

struct vc5_dev {
	struct drm_device *drm;

	/* Short representation (e.g. 33, 41) of the V3D tech version
	 * and revision.
	 */
	int ver;

	struct device *dev;
	struct platform_device *pdev;
	void __iomem *regs;
	struct clk *clk;

	/* Virtual and DMA addresses of the single shared page table. */
	u32 *pt;
	dma_addr_t pt_paddr;

	/* Virtual and DMA addresses of the MMU's scratch page.  When
	 * a read or write is invalid in the MMU, it will be
	 * redirected here.
	 */
	void *mmu_scratch;
	dma_addr_t mmu_scratch_paddr;

	/* Number of V3D cores. */
	uint32_t cores;
	/* Offset from the HUB registers to core 0's CTL regs. */
	uint32_t core_offset;
	/* Offset between cores CTL registers */
	uint32_t core_stride;

	/* Allocator managing the address space.  All units are in
	 * number of pages.
	 */
	struct drm_mm mm;

	struct vc5_bo *overflow_bo;

	struct vc5_hang_state *hang_state;

	uint64_t dma_fence_context;

	/* Sequence number for the last job queued in bin_job_list.
	 * Starts at 0 (no jobs emitted).
	 */
	uint64_t emit_seqno;

	/* Sequence number for the last completed job on the GPU.
	 * Starts at 0 (no jobs completed).
	 */
	uint64_t finished_seqno;

	/* List of all struct vc5_exec_info for jobs to be executed in
	 * the binner.  The first job in the list is the one currently
	 * programmed into ct0qba for execution.
	 */
	struct list_head bin_job_list;

	/* List of all struct vc5_exec_info for jobs that have
	 * completed binning and are ready for rendering.  The first
	 * job in the list is the one currently programmed into ct1qba
	 * for execution.
	 */
	struct list_head render_job_list;

	/* List of the finished vc5_exec_infos waiting to be freed by
	 * job_done_work.
	 */
	struct list_head job_done_list;
	/* Spinlock used to synchronize the job_list and seqno
	 * accesses between the IRQ handler and GEM ioctls.
	 */
	spinlock_t job_lock;
	wait_queue_head_t job_wait_queue;
	struct work_struct job_done_work;

	/* List of struct vc5_seqno_cb for callbacks to be made from a
	 * workqueue when the given seqno is passed.
	 */
	struct list_head seqno_cb_list;

	struct {
		struct timer_list timer;
		struct work_struct reset_work;
	} hangcheck;

	/* Protects bo_stats */
	struct mutex bo_lock;

	struct {
		uint32_t num_allocated;
		uint32_t pages_allocated;
	} bo_stats;

	struct work_struct overflow_mem_work;

	struct semaphore async_modeset;
};

static inline struct vc5_dev *
to_vc5_dev(struct drm_device *dev)
{
	return (struct vc5_dev *)dev->dev_private;
}

/* The per-fd struct, which tracks the MMU mappings. */
struct vc5_file_priv {
	struct vc5_page_table *pt;
	/* Protects drm_mm node allocation/removal and the page table. */
	spinlock_t lock;
};

/* Tracks a mapping of a BO into a per-fd address space */
struct vc5_vma {
	struct vc5_page_table *pt;
	struct list_head list; /* entry in vc5_bo.vmas */
};

struct vc5_bo {
	struct drm_gem_object base;

	struct mutex lock;

	struct drm_mm_node node;

	/* seqno of the last job to render using this BO. */
	uint64_t seqno;

	/* seqno of the last job to use the RCL to write to this BO.
	 *
	 * Note that this doesn't include binner overflow memory
	 * writes.
	 */
	uint64_t write_seqno;

	uint32_t pages_refcount;
	struct page **pages;
	struct sg_table *sgt;
	void *vaddr;

	struct list_head vmas;    /* list of vc5_vma */

	/* List entry for the BO's position in
	 * vc5_exec_info->unref_list
	 */
	struct list_head unref_head;

	/* normally (resv == &_resv) except for imported bo's */
	struct reservation_object *resv;
	struct reservation_object _resv;
};

static inline struct vc5_bo *
to_vc5_bo(struct drm_gem_object *bo)
{
	return (struct vc5_bo *)bo;
}

struct vc5_fence {
	struct dma_fence base;
	struct drm_device *dev;
	/* vc5 seqno for signaled() test */
	uint64_t seqno;
};

static inline struct vc5_fence *
to_vc5_fence(struct dma_fence *fence)
{
	return (struct vc5_fence *)fence;
}

#define V3D_READ(offset) readl(vc5->regs + offset)
#define V3D_WRITE(offset, val) writel(val, vc5->regs + offset)

#define V3D_CORE_REG_ADDR(core, offset) (vc5->regs +			\
					 vc5->core_offset +		\
					 (core) * vc5->core_stride +	\
					 (offset))
#define V3D_CORE_READ(core, offset) readl(V3D_CORE_REG_ADDR(core, offset))
#define V3D_CORE_WRITE(core, offset, val) writel(val, V3D_CORE_REG_ADDR(core, offset))

struct vc5_exec_info {
	struct vc5_file_priv *vc5_priv;

	/* Sequence number for this bin/render job. */
	uint64_t seqno;

	/* Latest write_seqno of any BO that binning depends on. */
	uint64_t bin_dep_seqno;

	struct dma_fence *fence;

	/* Last current addresses the hardware was processing when the
	 * hangcheck timer checked on us.
	 */
	uint32_t last_ct0ca, last_ct1ca;

	/* Kernel-space copy of the ioctl arguments */
	struct drm_vc5_submit_cl *args;

	/* This is the array of BOs that were looked up at the start of exec. */
	struct vc5_bo **bo;
	uint32_t bo_count;

	/* Pointers for our position in vc5->job_list */
	struct list_head head;

	/* List of other BOs used in the job that need to be released
	 * once the job is complete.
	 */
	struct list_head unref_list;

	/* Bitmask of which binner slots are freed when this job completes. */
	uint32_t bin_slots;

	/**
	 * Submitted offsets for the start and end of the bin and render
	 * CLs.
	 */
	uint32_t ct0qba, ct0qea;
	uint32_t ct1qba, ct1qea;

	/* Submitted tile memory allocation start/size, tile state. */
	uint32_t qma, qms, qts;

	/**
	 * Set if the job hung in the BCL and its RCL should be
	 * ignored when it comes time to execute it (since the binner
	 * didn't complete creating tile lists.
	 */
	bool hung;
};

static inline struct vc5_exec_info *
vc5_first_bin_job(struct vc5_dev *vc5)
{
	return list_first_entry_or_null(&vc5->bin_job_list,
					struct vc5_exec_info, head);
}

static inline struct vc5_exec_info *
vc5_first_render_job(struct vc5_dev *vc5)
{
	return list_first_entry_or_null(&vc5->render_job_list,
					struct vc5_exec_info, head);
}

static inline struct vc5_exec_info *
vc5_last_render_job(struct vc5_dev *vc5)
{
	if (list_empty(&vc5->render_job_list))
		return NULL;
	return list_last_entry(&vc5->render_job_list,
			       struct vc5_exec_info, head);
}

/**
 * _wait_for - magic (register) wait macro
 *
 * Does the right thing for modeset paths when run under kdgb or similar atomic
 * contexts. Note that it's important that we check the condition again after
 * having timed out, since the timeout could be due to preemption or similar and
 * we've never had a chance to check the condition before the timeout.
 */
#define _wait_for(COND, MS, W) ({ \
	unsigned long timeout__ = jiffies + msecs_to_jiffies(MS) + 1;	\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (time_after(jiffies, timeout__)) {			\
			if (!(COND))					\
				ret__ = -ETIMEDOUT;			\
			break;						\
		}							\
		if (W && drm_can_sleep())  {				\
			msleep(W);					\
		} else {						\
			cpu_relax();					\
		}							\
	}								\
	ret__;								\
})

#define wait_for(COND, MS) _wait_for(COND, MS, 1)

/* vc5_bo.c */
void vc5_free_object(struct drm_gem_object *gem_obj);
struct vc5_bo *vc5_bo_create(struct drm_device *dev, struct drm_file *file_priv,
			     size_t size);
int vc5_create_bo_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int vc5_mmap_bo_ioctl(struct drm_device *dev, void *data,
		      struct drm_file *file_priv);
int vc5_get_bo_offset_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file_priv);
int vc5_gem_fault(struct vm_fault *vmf);
int vc5_mmap(struct file *filp, struct vm_area_struct *vma);
struct reservation_object *vc5_prime_res_obj(struct drm_gem_object *obj);
int vc5_prime_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma);
struct sg_table *vc5_prime_get_sg_table(struct drm_gem_object *obj);
struct drm_gem_object *vc5_prime_import_sg_table(struct drm_device *dev,
						 struct dma_buf_attachment *attach,
						 struct sg_table *sgt);
void *vc5_prime_vmap(struct drm_gem_object *obj);
void vc5_prime_vunmap(struct drm_gem_object *obj, void *vaddr);

/* vc5_debugfs.c */
int vc5_debugfs_init(struct drm_minor *minor);

/* vc5_fence.c */
extern const struct dma_fence_ops vc5_fence_ops;

/* vc5_gem.c */
int vc5_gem_init(struct drm_device *dev);
void vc5_gem_destroy(struct drm_device *dev);
int vc5_submit_cl_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv);
int vc5_wait_seqno_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file_priv);
int vc5_wait_bo_ioctl(struct drm_device *dev, void *data,
		      struct drm_file *file_priv);
int vc5_get_hang_state_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file_priv);
void vc5_submit_next_bin_job(struct drm_device *dev);
void vc5_submit_next_render_job(struct drm_device *dev);
void vc5_move_job_to_render(struct drm_device *dev, struct vc5_exec_info *exec);
void vc5_cancel_bin_job(struct vc5_dev *vc5);
void vc5_finish_bin_job(struct vc5_dev *vc5);
void vc5_finish_render_job(struct vc5_dev *vc5);
int vc5_wait_for_seqno(struct drm_device *dev, uint64_t seqno,
		       uint64_t timeout_ns, bool interruptible);
void vc5_job_handle_completed(struct vc5_dev *vc5);

/* vc5_irq.c */
void vc5_irq_init(struct vc5_dev *vc5);
void vc5_irq_enable(struct vc5_dev *vc5);
void vc5_irq_disable(struct vc5_dev *vc5);
void vc5_irq_reset(struct vc5_dev *vc5);

/* vc5_mmu.c */
int vc5_mmu_get_offset(struct drm_file *file_priv, struct vc5_bo *bo,
		       uint32_t *offset);
int vc5_mmu_set_page_table(struct vc5_dev *vc5);
void vc5_mmu_insert_ptes(struct vc5_bo *bo);
void vc5_mmu_remove_ptes(struct vc5_bo *bo);
