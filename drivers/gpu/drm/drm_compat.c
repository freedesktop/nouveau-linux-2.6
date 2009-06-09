/**************************************************************************
 *
 * This kernel module is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 **************************************************************************/
/*
 * This code provides access to unexported mm kernel features. It is necessary
 * to use the new DRM memory manager code with kernels that don't support it
 * directly.
 *
 * Authors: Thomas Hellstrom <thomas-at-tungstengraphics-dot-com>
 *          Linux kernel mm subsystem authors.
 *          (Most code taken from there).
 */

#include "drmP.h"

#ifdef DRM_IDR_COMPAT_FN
/* only called when idp->lock is held */
static void __free_layer(struct idr *idp, struct idr_layer *p)
{
	p->ary[0] = idp->id_free;
	idp->id_free = p;
	idp->id_free_cnt++;
}

static void free_layer(struct idr *idp, struct idr_layer *p)
{
	unsigned long flags;

	/*
	 * Depends on the return element being zeroed.
	 */
	spin_lock_irqsave(&idp->lock, flags);
	__free_layer(idp, p);
	spin_unlock_irqrestore(&idp->lock, flags);
}

/**
 * idr_for_each - iterate through all stored pointers
 * @idp: idr handle
 * @fn: function to be called for each pointer
 * @data: data passed back to callback function
 *
 * Iterate over the pointers registered with the given idr.  The
 * callback function will be called for each pointer currently
 * registered, passing the id, the pointer and the data pointer passed
 * to this function.  It is not safe to modify the idr tree while in
 * the callback, so functions such as idr_get_new and idr_remove are
 * not allowed.
 *
 * We check the return of @fn each time. If it returns anything other
 * than 0, we break out and return that value.
 *
* The caller must serialize idr_find() vs idr_get_new() and idr_remove().
 */
int idr_for_each(struct idr *idp,
		 int (*fn)(int id, void *p, void *data), void *data)
{
	int n, id, max, error = 0;
	struct idr_layer *p;
	struct idr_layer *pa[MAX_LEVEL];
	struct idr_layer **paa = &pa[0];

	n = idp->layers * IDR_BITS;
	p = idp->top;
	max = 1 << n;

	id = 0;
	while (id < max) {
		while (n > 0 && p) {
			n -= IDR_BITS;
			*paa++ = p;
			p = p->ary[(id >> n) & IDR_MASK];
		}

		if (p) {
			error = fn(id, (void *)p, data);
			if (error)
				break;
		}

		id += 1 << n;
		while (n < fls(id)) {
			n += IDR_BITS;
			p = *--paa;
		}
	}

	return error;
}
EXPORT_SYMBOL(idr_for_each);

/**
 * idr_remove_all - remove all ids from the given idr tree
 * @idp: idr handle
 *
 * idr_destroy() only frees up unused, cached idp_layers, but this
 * function will remove all id mappings and leave all idp_layers
 * unused.
 *
 * A typical clean-up sequence for objects stored in an idr tree, will
 * use idr_for_each() to free all objects, if necessay, then
 * idr_remove_all() to remove all ids, and idr_destroy() to free
 * up the cached idr_layers.
 */
void idr_remove_all(struct idr *idp)
{
       int n, id, max, error = 0;
       struct idr_layer *p;
       struct idr_layer *pa[MAX_LEVEL];
       struct idr_layer **paa = &pa[0];

       n = idp->layers * IDR_BITS;
       p = idp->top;
       max = 1 << n;

       id = 0;
       while (id < max && !error) {
               while (n > IDR_BITS && p) {
                       n -= IDR_BITS;
                       *paa++ = p;
                       p = p->ary[(id >> n) & IDR_MASK];
               }

               id += 1 << n;
               while (n < fls(id)) {
                       if (p) {
                               memset(p, 0, sizeof *p);
                               free_layer(idp, p);
                       }
                       n += IDR_BITS;
                       p = *--paa;
               }
       }
       idp->top = NULL;
       idp->layers = 0;
}
EXPORT_SYMBOL(idr_remove_all);

#endif /* DRM_IDR_COMPAT_FN */


#ifdef DRM_NO_FAULT
unsigned long drm_bo_vm_nopfn(struct vm_area_struct *vma,
			      unsigned long address)
{
	struct drm_buffer_object *bo = (struct drm_buffer_object *) vma->vm_private_data;
	unsigned long page_offset;
	struct page *page = NULL;
	struct drm_ttm *ttm;
	struct drm_device *dev;
	unsigned long pfn;
	int err;
	unsigned long bus_base;
	unsigned long bus_offset;
	unsigned long bus_size;
	unsigned long ret = NOPFN_REFAULT;

	if (address > vma->vm_end)
		return NOPFN_SIGBUS;

	dev = bo->dev;
	err = drm_bo_read_lock(&dev->bm.bm_lock, 1);
	if (err)
		return NOPFN_REFAULT;

	err = mutex_lock_interruptible(&bo->mutex);
	if (err) {
		drm_bo_read_unlock(&dev->bm.bm_lock);
		return NOPFN_REFAULT;
	}

	err = drm_bo_wait(bo, 0, 1, 0, 1);
	if (err) {
		ret = (err != -EAGAIN) ? NOPFN_SIGBUS : NOPFN_REFAULT;
		bo->priv_flags &= ~_DRM_BO_FLAG_UNLOCKED;
		goto out_unlock;
	}

	bo->priv_flags &= ~_DRM_BO_FLAG_UNLOCKED;

	/*
	 * If buffer happens to be in a non-mappable location,
	 * move it to a mappable.
	 */

	if (!(bo->mem.flags & DRM_BO_FLAG_MAPPABLE)) {
		uint32_t new_flags = bo->mem.proposed_flags |
			DRM_BO_FLAG_MAPPABLE |
			DRM_BO_FLAG_FORCE_MAPPABLE;
		err = drm_bo_move_buffer(bo, new_flags, 0, 0);
		if (err) {
			ret = (err != -EAGAIN) ? NOPFN_SIGBUS : NOPFN_REFAULT;
			goto out_unlock;
		}
	}

	err = drm_bo_pci_offset(dev, &bo->mem, &bus_base, &bus_offset,
				&bus_size);

	if (err) {
		ret = NOPFN_SIGBUS;
		goto out_unlock;
	}

	page_offset = (address - vma->vm_start) >> PAGE_SHIFT;

	if (bus_size) {
		struct drm_mem_type_manager *man = &dev->bm.man[bo->mem.mem_type];

		pfn = ((bus_base + bus_offset) >> PAGE_SHIFT) + page_offset;
		vma->vm_page_prot = drm_io_prot(man->drm_bus_maptype, vma);
	} else {
		ttm = bo->ttm;

		drm_ttm_fixup_caching(ttm);
		page = drm_ttm_get_page(ttm, page_offset);
		if (!page) {
			ret = NOPFN_OOM;
			goto out_unlock;
		}
		pfn = page_to_pfn(page);
		vma->vm_page_prot = (bo->mem.flags & DRM_BO_FLAG_CACHED) ?
			vm_get_page_prot(vma->vm_flags) :
			drm_io_prot(_DRM_TTM, vma);
	}

	err = vm_insert_pfn(vma, address, pfn);
	if (err) {
		ret = (err != -EAGAIN) ? NOPFN_OOM : NOPFN_REFAULT;
		goto out_unlock;
	}
out_unlock:
	BUG_ON(bo->priv_flags & _DRM_BO_FLAG_UNLOCKED);
	mutex_unlock(&bo->mutex);
	drm_bo_read_unlock(&dev->bm.bm_lock);
	return ret;
}
#endif
