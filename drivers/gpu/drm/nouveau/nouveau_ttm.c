/*
 * Copyright (c) 2007-2008 Tungsten Graphics, Inc., Cedar Park, TX., USA,
 * All Rights Reserved.
 * Copyright (c) 2009 VMware, Inc., Palo Alto, CA., USA,
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "ttm/ttm_userobj_api.h"

#include "drmP.h"

#include "nouveau_drv.h"

static struct vm_operations_struct nouveau_ttm_vm_ops;
static struct vm_operations_struct *ttm_vm_ops = NULL;

static int
nouveau_ttm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct ttm_buffer_object *bo = vma->vm_private_data;
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bo->bdev);
	int ret;

	ret = ttm_read_lock(&dev_priv->ttm.lock, true);
	if (unlikely(ret != 0))
		return VM_FAULT_NOPAGE;

	ret = ttm_vm_ops->fault(vma, vmf);

	ttm_read_unlock(&dev_priv->ttm.lock);

	return ret;
}

int
nouveau_ttm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_nouveau_private *dev_priv =
		file_priv->minor->dev->dev_private;
	int ret;

	if (unlikely(vma->vm_pgoff < DRM_FILE_PAGE_OFFSET))
		return drm_mmap(filp, vma);

	ret = ttm_bo_mmap(filp, vma, &dev_priv->ttm.bdev);
	if (unlikely(ret != 0))
		return ret;

	if (unlikely(ttm_vm_ops == NULL)) {
		ttm_vm_ops = vma->vm_ops;
		nouveau_ttm_vm_ops = *ttm_vm_ops;
		nouveau_ttm_vm_ops.fault = &nouveau_ttm_fault;
	}

	vma->vm_ops = &nouveau_ttm_vm_ops;
	return 0;
}

int
nouveau_ttm_verify_access(struct ttm_buffer_object *bo, struct file *filp)
{
	struct drm_file *file_priv = filp->private_data;
	struct nouveau_fpriv *fpriv = file_priv->driver_priv;

	return ttm_pl_verify_access(bo, fpriv->tfile);
}

static int
nouveau_ttm_mem_global_init(struct drm_global_reference *ref)
{
	return ttm_mem_global_init(ref->object);
}

static void
nouveau_ttm_mem_global_release(struct drm_global_reference *ref)
{
	ttm_mem_global_release(ref->object);
}

int
nouveau_ttm_global_init(struct drm_nouveau_private *dev_priv)
{
	struct drm_global_reference *global_ref;
	int ret;

	global_ref = &dev_priv->ttm.mem_global_ref;
	global_ref->global_type = DRM_GLOBAL_TTM_MEM;
	global_ref->size = sizeof(struct ttm_mem_global);
	global_ref->init = &nouveau_ttm_mem_global_init;
	global_ref->release = &nouveau_ttm_mem_global_release;

	ret = drm_global_item_ref(global_ref);
	if (unlikely(ret != 0)) {
		DRM_ERROR("Failed referencing a global TTM memory object.\n");
		return ret;
	}

	return 0;
}

void
nouveau_ttm_global_release(struct drm_nouveau_private *dev_priv)
{
	drm_global_item_unref(&dev_priv->ttm.mem_global_ref);
}

