// SPDX-License-Identifier: GPL-2.0+
/* Copyright (C) 2015-2018 Broadcom */

/**
 * DOC: VC5 GEM BO management support
 *
 * Compared to VC4 (V3D 2.x), V3D 3.3 introduces an MMU between the
 * GPU and the bus, allowing us to use shmem objects for our storage
 * instead of CMA.  We use a virtual address space per DRM FD to
 * isolate clients from each other.
 */

#include <linux/dma-buf.h>
#include <linux/pfn_t.h>

#include "vc5_drv.h"
#include "uapi/drm/vc5_drm.h"

/* Pins the shmem pages, fills in the .pages and .sgt fields of the BO, and maps
 * it for DMA.
 */
static int
vc5_bo_get_pages(struct vc5_bo *bo)
{
	struct drm_gem_object *obj = &bo->base;
	struct drm_device *dev = obj->dev;
	int npages = obj->size >> PAGE_SHIFT;
	int ret = 0;

	mutex_lock(&bo->lock);
	if (bo->pages_refcount++ != 0)
		goto unlock;

	if (!obj->import_attach) {
		bo->pages = drm_gem_get_pages(obj);
		if (IS_ERR(bo->pages)) {
			ret = PTR_ERR(bo->pages);
			goto unlock;
		}

		bo->sgt = drm_prime_pages_to_sg(bo->pages, npages);
		if (IS_ERR(bo->sgt)) {
			ret = PTR_ERR(bo->sgt);
			goto put_pages;
		}
	} else {
		/* XXX: Create pages array. */
	}

	/* Map the pages for use by the GPU. */
	dma_map_sg(dev->dev, bo->sgt->sgl,
		   bo->sgt->nents, DMA_BIDIRECTIONAL);

	mutex_unlock(&bo->lock);

	return 0;

put_pages:
	drm_gem_put_pages(obj, bo->pages, true, true);
	bo->pages = NULL;
unlock:
	bo->pages_refcount--;
	mutex_unlock(&bo->lock);
	return ret;
}

static void
vc5_bo_put_pages(struct vc5_bo *bo)
{
	struct drm_gem_object *obj = &bo->base;

	mutex_lock(&bo->lock);
	if (--bo->pages_refcount == 0) {
		dma_unmap_sg(obj->dev->dev, bo->sgt->sgl,
			     bo->sgt->nents, DMA_BIDIRECTIONAL);
		if (!obj->import_attach) {
			sg_free_table(bo->sgt);
			kfree(bo->sgt);
			drm_gem_put_pages(obj, bo->pages, true, true);
		}
	}
	mutex_unlock(&bo->lock);
}

static struct vc5_bo *vc5_bo_create_struct(struct drm_device *dev,
					   size_t unaligned_size)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	struct drm_gem_object *obj;
	struct vc5_bo *bo;
	size_t size = roundup(unaligned_size, PAGE_SIZE);
	unsigned long irqflags;
	int ret;

	if (size == 0)
		return ERR_PTR(-EINVAL);

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return ERR_PTR(-ENOMEM);
	obj = &bo->base;

	INIT_LIST_HEAD(&bo->vmas);
	mutex_init(&bo->lock);

	ret = drm_gem_object_init(dev, obj, size);
	if (ret)
		goto free_bo;

	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		goto free_obj;

	spin_lock_irqsave(&vc5->job_lock, irqflags);
	ret = drm_mm_insert_node_generic(&vc5->mm, &bo->node,
					 obj->size >> PAGE_SHIFT,
					 GMP_GRANULARITY >> PAGE_SHIFT, 0, 0);
	spin_unlock_irqrestore(&vc5->job_lock, irqflags);
	if (ret)
		goto free_obj;

	return bo;

free_obj:
	drm_gem_object_release(obj);
free_bo:
	kfree(bo);
	return ERR_PTR(ret);
}

struct vc5_bo *vc5_bo_create(struct drm_device *dev, struct drm_file *file_priv,
			     size_t unaligned_size)
{
	struct vc5_dev *vc5 = to_vc5_dev(dev);
	struct drm_gem_object *obj;
	struct vc5_bo *bo;
	int ret;

	bo = vc5_bo_create_struct(dev, unaligned_size);
	if (IS_ERR(bo))
		return bo;
	obj = &bo->base;

	bo->resv = &bo->_resv;
	reservation_object_init(bo->resv);

	ret = vc5_bo_get_pages(bo);
	if (ret)
		goto free_mm;

	vc5_mmu_insert_ptes(bo);

	mutex_lock(&vc5->bo_lock);
	vc5->bo_stats.num_allocated++;
	vc5->bo_stats.pages_allocated += obj->size >> PAGE_SHIFT;
	mutex_unlock(&vc5->bo_lock);

	return bo;

free_mm:
	drm_mm_remove_node(&bo->node);
	drm_gem_object_release(obj);
	kfree(bo);
	return ERR_PTR(ret);
}

/* Called DRM core on the last userspace/kernel unreference of the
 * BO.
 */
void vc5_free_object(struct drm_gem_object *obj)
{
	struct vc5_dev *vc5 = to_vc5_dev(obj->dev);
	struct vc5_bo *bo = to_vc5_bo(obj);
	unsigned long irqflags;

	mutex_lock(&vc5->bo_lock);
	vc5->bo_stats.num_allocated--;
	vc5->bo_stats.pages_allocated -= obj->size >> PAGE_SHIFT;
	mutex_unlock(&vc5->bo_lock);

	reservation_object_fini(&bo->_resv);

	vc5_bo_put_pages(bo);

	if (obj->import_attach)
		drm_prime_gem_destroy(obj, bo->sgt);

	vc5_mmu_remove_ptes(bo);
	spin_lock_irqsave(&vc5->job_lock, irqflags);
	drm_mm_remove_node(&bo->node);
	spin_unlock_irqrestore(&vc5->job_lock, irqflags);

	mutex_destroy(&bo->lock);

	drm_gem_object_release(obj);
	kfree(bo);
}

struct reservation_object *vc5_prime_res_obj(struct drm_gem_object *obj)
{
	struct vc5_bo *bo = to_vc5_bo(obj);

	return bo->resv;
}

static void
vc5_set_mmap_vma_flags(struct vm_area_struct *vma)
{
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;
	vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
}

int vc5_gem_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct drm_gem_object *obj = vma->vm_private_data;
	struct vc5_bo *bo = to_vc5_bo(obj);
	unsigned long pfn;
	pgoff_t pgoff;
	int ret;

	/* XXX: Get a ref on the pages. */

	/* We don't use vmf->pgoff since that has the fake offset: */
	pgoff = (vmf->address - vma->vm_start) >> PAGE_SHIFT;
	pfn = page_to_pfn(bo->pages[pgoff]);

	ret = vm_insert_mixed(vma, vmf->address, __pfn_to_pfn_t(pfn, PFN_DEV));

	switch (ret) {
	case -EAGAIN:
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
	case -EBUSY:
		/*
		 * EBUSY is ok: this just means that another thread
		 * already did the job.
		 */
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

int vc5_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret)
		return ret;

	vc5_set_mmap_vma_flags(vma);

	return ret;
}

int vc5_prime_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	if (ret < 0)
		return ret;

	vc5_set_mmap_vma_flags(vma);

	return 0;
}

void *vc5_prime_vmap(struct drm_gem_object *obj)
{
	return NULL; /* XXX */
}

void vc5_prime_vunmap(struct drm_gem_object *obj, void *vaddr)
{
	/* Nothing to do */
}

struct sg_table *
vc5_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct vc5_bo *bo = to_vc5_bo(obj);
	int npages = obj->size >> PAGE_SHIFT;

	return drm_prime_pages_to_sg(bo->pages, npages);
}

struct drm_gem_object *
vc5_prime_import_sg_table(struct drm_device *dev,
			  struct dma_buf_attachment *attach,
			  struct sg_table *sgt)
{
	struct drm_gem_object *obj;
	struct vc5_bo *bo;

	bo = vc5_bo_create_struct(dev, attach->dmabuf->size);
	if (IS_ERR(bo))
		return ERR_PTR(PTR_ERR(bo));
	obj = &bo->base;

	bo->resv = attach->dmabuf->resv;

	bo->sgt = sgt;
	vc5_bo_get_pages(bo);

	vc5_mmu_insert_ptes(bo);

	return obj;
}

int vc5_create_bo_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct drm_vc5_create_bo *args = data;
	struct vc5_bo *bo = NULL;
	int ret;

	bo = vc5_bo_create(dev, file_priv, PAGE_ALIGN(args->size));
	if (IS_ERR(bo))
		return PTR_ERR(bo);

	args->offset = bo->node.start << PAGE_SHIFT;

	ret = drm_gem_handle_create(file_priv, &bo->base, &args->handle);
	drm_gem_object_unreference_unlocked(&bo->base);

	return ret;
}

int vc5_mmap_bo_ioctl(struct drm_device *dev, void *data,
		      struct drm_file *file_priv)
{
	struct drm_vc5_mmap_bo *args = data;
	struct drm_gem_object *gem_obj;

	gem_obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!gem_obj) {
		DRM_DEBUG("Failed to look up GEM BO %d\n", args->handle);
		return -ENOENT;
	}

	/* The mmap offset was set up at BO allocation time. */
	args->offset = drm_vma_node_offset_addr(&gem_obj->vma_node);

	drm_gem_object_unreference_unlocked(gem_obj);
	return 0;
}

int vc5_get_bo_offset_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file_priv)
{
	struct drm_vc5_get_bo_offset *args = data;
	struct drm_gem_object *gem_obj;
	struct vc5_bo *bo;

	gem_obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!gem_obj) {
		DRM_DEBUG("Failed to look up GEM BO %d\n", args->handle);
		return -ENOENT;
	}
	bo = to_vc5_bo(gem_obj);

	args->offset = bo->node.start << PAGE_SHIFT;

	drm_gem_object_unreference_unlocked(gem_obj);
	return 0;
}
