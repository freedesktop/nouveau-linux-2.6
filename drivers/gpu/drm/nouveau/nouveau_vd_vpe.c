/*
 * Copyright (C) 2010 Jimmy Rentz
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "drmP.h"
#include "drm.h"

#include "nouveau_drv.h"
#include "nouveau_drm.h"
#include "nouveau_vpe_hw.h"

/* VPE MPEG2 HW notes:
 * - There is a 64byte fetch size.  That is why each set of commands must
 * be aligned on a 64 byte boundary for firing.
 * - One fetch of cmds seem to process in 1 microsecond on my nv4e.  
 * However, I presume this can vary based on the hw and nature of commands.
 * - Each firing of a set of commands must be followed by a small delay.
 * The main reason is to avoid overwhelming the hw.  
 * The delays below were determined from testing/measuring.  I doubt they
   are perfect and they could be tweaked a bit.*/

/* Channel/Surface init commands process in little or no time.*/
#define VPE_UDELAY_FIRE_INIT        4

/* Normal firing needs this type of delay.*/
#define VPE_UDELAY_FIRE_NORMAL      35

/* Need a longer delay at the end of the fifo since it takes longer.*/
#define VPE_UDELAY_FIRE_END        100

/* Set if you want to validate vpe user cmds.
 * Otherwise, they are copied asis.
 * The reason this exists is because a user could set a vpe surface to 
 * point to the visible framebuffer, etc.  However, the user could never
 * make a vpe surface use a gart address since it isn't supported by the
 * hardware.*/
/*#define NOUVEAU_VPE_VALIDATE_USER_CMDS*/

/* All these functions up here need to be exported somehow.*/

/* Needed to copy userspace pushbuffers that are sent to the vpe hw.*/
static inline void *
_u_memcpya(uint64_t user, unsigned nmemb, unsigned size)
{
	void *mem;
	void __user *userptr = (void __force __user *)(uintptr_t)user;

	mem = kmalloc(nmemb * size, GFP_KERNEL);
	if (!mem)
		return ERR_PTR(-ENOMEM);

	if (DRM_COPY_FROM_USER(mem, userptr, nmemb * size)) {
		kfree(mem);
		return ERR_PTR(-EFAULT);
	}

	return mem;
}

/* Internal */			 
static inline void
nouveau_vpe_cmd_write(struct nouveau_vd_vpe_channel *vpe_channel, uint32_t value)
{
	nouveau_bo_wr32(vpe_channel->pushbuf_bo, vpe_channel->dma.cur++, value);
	vpe_channel->dma.free--;
	
	if (vpe_channel->dma.cur == vpe_channel->dma.max) {
		vpe_channel->dma.cur = 0;
		vpe_channel->dma.free = vpe_channel->dma.max;
	}
}

static inline void
nouveau_vpe_cmd_align(struct nouveau_vd_vpe_channel *vpe_channel)
{
	uint32_t nop_count;
	uint32_t cmd_sequence_count;
	int i;
	
	/* Alignment is needed when ending cmd sequences.*/
	cmd_sequence_count = vpe_channel->dma.cur - vpe_channel->dma.put;
	nop_count = ALIGN(cmd_sequence_count, NV_VPE_CMD_ALIGNMENT);
	nop_count -= cmd_sequence_count;

	for (i = 0; i < nop_count; i++)
		nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_NOP << NV_VPE_CMD_TYPE_SHIFT);	
}

static inline void
nouveau_vpe_fire(struct nouveau_vd_vpe_channel *vpe_channel, uint64_t delay)
{
	struct drm_device *dev = vpe_channel->dev;
	uint32_t put;
	
	DRM_MEMORYBARRIER();
	
	put = (vpe_channel->dma.cur / NV_VPE_CMD_ALIGNMENT) * NV_VPE_CMD_ALIGNMENT;
	
	nouveau_bo_rd32(vpe_channel->pushbuf_bo, put);
	
	nv_wr32(dev, NV_VPE_MPEG2_USER_PUT, put << 2);
	
	vpe_channel->dma.put = put;
	
	if (delay)
		DRM_UDELAY(delay);
}

static uint32_t
nouveau_vpe_channel_read_get(struct nouveau_vd_vpe_channel *vpe_channel)
{
	struct drm_device *dev = vpe_channel->dev;
	
	return nv_rd32(dev, NV_VPE_MPEG2_USER_GET) >> 2;
}

static int
nouveau_vpe_channel_wait(struct nouveau_vd_vpe_channel *vpe_channel, uint32_t put)
{
	uint32_t get;
	uint32_t prev_get = 0;
	bool is_beg = (put == 0) || (vpe_channel->dma.put == 0);
	uint32_t cnt = 0;
	
	get = prev_get = nouveau_vpe_channel_read_get(vpe_channel);
	
	while ( (!is_beg && (get < put) ) ||
		    (is_beg && (get != 0) ) ) {
		    
		/* reset counter as long as GET is still advancing, this is
		 * to avoid misdetecting a GPU lockup if the GPU happens to
		 * just be processing an operation that takes a long time
		 */
		get = nouveau_vpe_channel_read_get(vpe_channel);
		if (get != prev_get) {
			prev_get = get;
			cnt = 0;
		}

		if ((++cnt & 0xff) == 0) {
			DRM_UDELAY(1);
			if (cnt > 100000) {
				NV_ERROR(vpe_channel->dev, "nouveau_vpe_channel_wait - lockup. cur = 0x%08X, put = 0x%08X, get = 0x%08X, put.seq = %u, get.seq = %u, ec1 = 0x%08X, ec2 = 0x%08X, es = 0x%08X.\n", 
					vpe_channel->dma.cur, put, nouveau_vpe_channel_read_get(vpe_channel), vpe_channel->dma.sequence, 
					nv_rd32(vpe_channel->dev, NV_VPE_MPEG2_SEQUENCE_GET),
					nv_rd32(vpe_channel->dev, NV_VPE_MPEG2_ENGINE_CONFIG_1), 
					nv_rd32(vpe_channel->dev, NV_VPE_MPEG2_ENGINE_CONFIG_2),
					nv_rd32(vpe_channel->dev, NV_VPE_MPEG2_ENGINE_STATUS));
				return -EBUSY;
			}
		}
	}

	return 0;
}

static void
nouveau_vpe_cmd_end_sequence_header(struct nouveau_vd_vpe_channel *vpe_channel)
{	
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_END_SEQUENCE << NV_VPE_CMD_TYPE_SHIFT
					| NV_VPE_CMD_SEQUENCE << 24);

	nouveau_vpe_cmd_write(vpe_channel, ++vpe_channel->dma.sequence);
}

static void
nouveau_vpe_cmd_end_sequence_trailer(struct nouveau_vd_vpe_channel *vpe_channel)
{	
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_END_SEQUENCE << NV_VPE_CMD_TYPE_SHIFT);
}

static void
nouveau_vpe_cmd_end_sequence_finish(struct nouveau_vd_vpe_channel *vpe_channel)
{	
	nouveau_vpe_cmd_align(vpe_channel);
	nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_NORMAL);
}

#ifndef NOUVEAU_VPE_VALIDATE_USER_CMDS
static void
_OUT_RINGp(struct nouveau_vd_vpe_channel *chan, const void *data, unsigned nr_dwords)
{
	bool is_iomem;
	u32 *mem = ttm_kmap_obj_virtual(&chan->pushbuf_bo->kmap, &is_iomem);
	mem = &mem[chan->dma.cur];
	if (is_iomem)
		memcpy_toio((void __force __iomem *)mem, data, nr_dwords * 4);
	else
		memcpy(mem, data, nr_dwords * 4);
	chan->dma.cur += nr_dwords;
}
#endif

static int
nouveau_vpe_cmd_write_user_batch(struct nouveau_vd_vpe_channel *chan, 
									const void *data, unsigned nr_dwords)
{
#ifdef NOUVEAU_VPE_VALIDATE_USER_CMDS
	bool is_iomem;
	u32 *mem = ttm_kmap_obj_virtual(&chan->pushbuf_bo->kmap, &is_iomem);
	u32 *user_data = (u32*) data;
	uint32_t val;
	int i;
	bool in_mb_db = false;
	bool at_end_mb_db = false;
	
	mem = &mem[chan->dma.cur];
	
	for (i = 0; i < nr_dwords; i++) {
		val = user_data[i];
		
		if (in_mb_db) {
			if (at_end_mb_db) {
			  if (val == (NV_VPE_CMD_DCT_SEPARATOR << NV_VPE_CMD_TYPE_SHIFT))
				at_end_mb_db = false;
			  else
				in_mb_db = false;
			}
			else if (val & NV_VPE_DCT_BLOCK_TERMINATOR) 
					at_end_mb_db = true;
		}
		if (!in_mb_db) {
			switch (val & 0xF0000000) {
				case NV_VPE_CMD_DCT_SEPARATOR << NV_VPE_CMD_TYPE_SHIFT:
					in_mb_db = true;
					at_end_mb_db = false;
					break;
				case NV_VPE_CMD_DCT_CHROMA_HEADER << NV_VPE_CMD_TYPE_SHIFT:
				case NV_VPE_CMD_DCT_LUMA_HEADER << NV_VPE_CMD_TYPE_SHIFT:
				case NV_VPE_CMD_DCT_COORDINATE << NV_VPE_CMD_TYPE_SHIFT:
				case NV_VPE_CMD_CHROMA_MOTION_VECTOR_HEADER << NV_VPE_CMD_TYPE_SHIFT:
				case NV_VPE_CMD_LUMA_MOTION_VECTOR_HEADER << NV_VPE_CMD_TYPE_SHIFT:
				case NV_VPE_CMD_MOTION_VECTOR << NV_VPE_CMD_TYPE_SHIFT:
				case NV_VPE_CMD_NOP << NV_VPE_CMD_TYPE_SHIFT:
					break;
				default:
					NV_ERROR(chan->dev, "vpe - invalid cmd 0x%08X detected. Aborting cmd sequence.\n", 
							val);
				return -EINVAL;
			}
		}
		
		/* Always iomem/vram for vpe.*/
		iowrite32_native(val, (void __force __iomem *)&mem[i]);
	}
	
	chan->dma.cur += nr_dwords;
#else
	_OUT_RINGp(chan, data, nr_dwords);
#endif
	
	return 0;
}

static bool
nouveau_vpe_validate_surface(struct nouveau_vd_vpe_channel *vpe_channel, 
							    uint32_t handle, 
							    struct nouveau_bo *target_nvbo)
{
	struct drm_device *dev = vpe_channel->dev;
	struct drm_gem_object *gem;
	struct nouveau_bo *nvbo;
	bool result;
	
	gem = drm_gem_object_lookup(dev, vpe_channel->file_priv, handle);
	if (unlikely(!gem)) {
		result = false;
		NV_ERROR(dev, "nouveau_vpe_validate_gem_handle - Unknown handle 0x%08X.\n", handle);
		goto out;
	}
	nvbo = nouveau_gem_object(gem);
	if (unlikely(!nvbo || (nvbo != target_nvbo))) {
		result = false;
		NV_ERROR(dev, "nouveau_vpe_validate_gem_handle - Unknown bo 0x%08X.\n", handle);
		goto out;
	}
	
	result = true;
		
out:

	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(gem);
	mutex_unlock(&dev->struct_mutex);
	
	return result;
}

static int
nouveau_vpe_pin_surface(struct nouveau_vd_vpe_channel *vpe_channel, uint32_t handle,
                        uint32_t required_size, struct nouveau_bo **pnvbo)
{
	struct drm_device *dev = vpe_channel->dev;
	struct drm_gem_object *gem;
	struct nouveau_bo *nvbo;
	uint32_t mem_type;
	unsigned long size;
	int ret;
	
	gem = drm_gem_object_lookup(dev, vpe_channel->file_priv, handle);
	if (!gem) {
		NV_ERROR(dev, "nouveau_vpe_pin_surface - Unknown handle 0x%08X.\n", handle);
		return -EINVAL;
	}
	nvbo = nouveau_gem_object(gem);
	if (!nvbo) {
		ret = -EINVAL;
		NV_ERROR(dev, "nouveau_vpe_pin_surface - Unknown bo 0x%08X.\n", handle);
		goto out;
	}
	ret = ttm_bo_reserve(&nvbo->bo, false, false, false, 0);
	if (ret)
		goto out;
		
	mem_type = nvbo->bo.mem.mem_type;
	size = nvbo->bo.mem.size;
	
	ttm_bo_unreserve(&nvbo->bo);
		
	if (mem_type != TTM_PL_VRAM) {
		ret = -EINVAL;
		NV_ERROR(dev, "nouveau_vpe_pin_surface - bo must be in vram.\n");
		goto out;
	}
	if (size < required_size) {
		ret = -EINVAL;
		NV_ERROR(dev, "nouveau_vpe_pin_surface - bo 0x%08X has size %lu, required %u.\n", handle,
			size, required_size);
		goto out;
	}
	ret = nouveau_bo_pin(nvbo, TTM_PL_FLAG_VRAM);
	if (ret) {
		NV_ERROR(dev, "nouveau_vpe_pin_surface - Could not pin handle 0x%08X.\n", handle);
		goto out;
	}
	
	*pnvbo = nvbo;
	ret = 0;
	
out:
	
	mutex_lock(&dev->struct_mutex);
	drm_gem_object_unreference(gem);
	mutex_unlock(&dev->struct_mutex);
	
	return ret;
}

static void
nouveau_vpe_unpin_surface(struct nouveau_vd_vpe_channel *vpe_channel, struct nouveau_bo *nvbo)
{
	if (nvbo && nvbo->pin_refcnt)
		nouveau_bo_unpin(nvbo);
}

static void
nouveau_vpe_reset_pushbuf_to_start(struct nouveau_vd_vpe_channel *vpe_channel)
{
	int i;
	uint32_t nop_count;
		
	if (vpe_channel->dma.cur) {
		/* Just write nops till the end since alignment is a non-issue
		 * here.*/
		nop_count = vpe_channel->dma.max - vpe_channel->dma.cur;
		
		for (i = 0; i < nop_count; i++)
			nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_NOP << NV_VPE_CMD_TYPE_SHIFT);
	}
	
	nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_END);
}

static int 
nouveau_vpe_channel_pushbuf_alloc(struct nouveau_vd_vpe_channel *vpe_channel)
{
	struct drm_device *dev = vpe_channel->dev;
	struct nouveau_bo *pushbuf_bo;
	int ret;
	uint32_t flags;
	
	if (0) 
	/*dev_priv->gart_info.type == NOUVEAU_GART_AGP)
	 * agp init is broken right now it seems.*/
		flags = TTM_PL_FLAG_TT;
	else
		flags = TTM_PL_FLAG_VRAM;

	ret = nouveau_gem_new(dev, NULL, NV_VPE_PUSHBUFFER_SIZE, 0,
						 flags, 0, 0x0000, false, true, &pushbuf_bo);
	if (ret)
		return ret;

	ret = nouveau_bo_pin(pushbuf_bo, flags);
	if (ret)
		goto out_err;

	ret = nouveau_bo_map(pushbuf_bo);
	if (ret)
		goto out_err;
	
	vpe_channel->pushbuf_bo = pushbuf_bo;
	vpe_channel->dma.max  = vpe_channel->pushbuf_bo->bo.mem.size >> 2;
	vpe_channel->dma.free = vpe_channel->dma.max;
		
out_err:
	if (ret) {
		mutex_lock(&dev->struct_mutex);
		drm_gem_object_unreference(pushbuf_bo->gem);
		mutex_unlock(&dev->struct_mutex);
	}
	
	return ret;
}

static int 
nouveau_vpe_channel_hw_init(struct nouveau_vd_vpe_channel *vpe_channel)
{
	uint32_t value;
	struct drm_device *dev = vpe_channel->dev;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	uint32_t pushbuf_offset = 0;

	/* Turn off the mpeg2 decoder.*/
	nv_wr32(dev, NV_VPE_MPEG2_USER_CONFIG, 
		NV_VPE_MPEG2_USER_NOT_PRESENT);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_CONTROL, NV_VPE_MPEG2_ENGINE_STOP);
	nv_wr32(dev, NV_VPE_MPEG2_USER_PUT, 0);
	nv_wr32(dev, NV_VPE_MPEG2_USER_OFFSET, 0);
	nv_wr32(dev, NV_VPE_MPEG2_USER_SIZE, 0);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_1, 0);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_2, 0);
	nv_rd32(dev, NV_VPE_MPEG2_ENGINE_CONTROL);
	
	/* Pause a tiny bit to let the hardware reset.  
	 * This might be needed.*/
	DRM_UDELAY(100);
	
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_1, 0x01010000);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_2, 0x01010000);
	nv_wr32(dev, NV_VPE_MPEG2_UNKNOWN_SETUP_3, 0x100);
	
	/* Some type of mpeg2 engine config.
	 * It seems that the hardware automatically sets this to 0x20.
	 * However, I have an nv4a mmio trace where the nvidia driver
	 * actually writes 0x20.  
	 * Also I have noticed that when the mpeg2 engine hw locks
	 * up after playing video, this register gets reset to 0x1.
	 */
	if (nv_rd32(dev, NV_VPE_MPEG2_ENGINE_CONFIG_1) != 0x20)
		nv_wr32(dev, NV_VPE_MPEG2_ENGINE_CONFIG_1, 0x20);
	if (nv_rd32(dev, NV_VPE_MPEG2_ENGINE_CONFIG_2) != 0x20)
		nv_wr32(dev, NV_VPE_MPEG2_ENGINE_CONFIG_2, 0x20);

	/* Make sure the decoder is ready.
	 * So, we check each status register.  
	 * Well, that is what these registers seem to be.
	 */
	value = nv_rd32(dev, NV_VPE_MPEG2_ENGINE_STATUS);

	/* Is the hw still busy? */
	if (value & 0x1)
		if (!nouveau_wait_until(dev, 10000000, NV_VPE_MPEG2_ENGINE_STATUS, 
							0x0FFFFFFF, 0)) {
			NV_ERROR(dev, "nouveau_vpe_channel_hw_init - unknown status value of 0x%08X for engine status reg. Must exit.\n", 
					nv_rd32(dev, NV_VPE_MPEG2_ENGINE_STATUS));
			return -EINVAL;
		}

	/* Make sure the decoder is ready. */
	value = nv_rd32(dev, NV_VPE_MPEG2_ENGINE_STATUS_1);

	/* If we got this value then we might have a problem. */
	if (value & 0x200) {
		NV_ERROR(dev, "nouveau_vpe_channel_hw_init - unknown status value of 0x%08X for engine status 1 reg. Must exit.\n", 
					value);
		return -EINVAL;
	}

	/* Is the status reg still busy? */
	if (value & 0x1)
		if (!nouveau_wait_until(dev, 10000000, NV_VPE_MPEG2_ENGINE_STATUS_1, 
							0x0FFFFFFF, 0)) {
			NV_ERROR(dev, "nouveau_vpe_channel_hw_init - unknown status value of 0x%08X for engine status 1 reg. Must exit.\n", 
					nv_rd32(dev, NV_VPE_MPEG2_ENGINE_STATUS_1));
			return -EINVAL;
		}

	/* Reset the mpeg2 pushbuffer/user. */
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_CONTROL, NV_VPE_MPEG2_ENGINE_STOP);
	nv_wr32(dev, NV_VPE_MPEG2_USER_OFFSET, 0);
	nv_wr32(dev, NV_VPE_MPEG2_USER_SIZE, 0);

	/* The setup of the command buffer is different for agp and pci/pcie. 
	 * NOTE: Agp is not working right now so it is disabled.*/
	if (vpe_channel->pushbuf_bo->bo.mem.mem_type == TTM_PL_TT) {
		
		pushbuf_offset = lower_32_bits(dev_priv->gart_info.aper_base) + 
		    lower_32_bits(vpe_channel->pushbuf_bo->bo.offset);
		
		nv_wr32(dev, NV_VPE_MPEG2_USER_CONFIG,
				NV_VPE_MPEG2_USER_PRESENT | NV_VPE_MPEG2_USER_AGP_OR_PCI);
		/* This needs the agp aperature in the offset.*/
		nv_wr32(dev, NV_VPE_MPEG2_USER_OFFSET,
				pushbuf_offset);
		nv_wr32(dev, NV_VPE_MPEG2_USER_SIZE,
				vpe_channel->dma.max << 2);
		nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_1, 0x01010000);
		nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_2, 0x01010000);
		nv_wr32(dev, NV_VPE_MPEG2_USER_CONFIG,
				NV_VPE_MPEG2_USER_PRESENT | NV_VPE_MPEG2_USER_AGP_OR_PCI | NV_VPE_MPEG2_USER_AGP_OR_PCI_READY);
	} else {
		/* For pci, only the fb offset is used.
		 * However, have to init the pushbuffer/user using the fb size? not sure here.
		 * This is not related to decoding but strictly for reading from
		 * the pushbuffer/user.  It might be caching related. 
		 * The nv driver uses different values but it looks fb size related.
		 * So, I will go with that for now.
		 */
		pushbuf_offset = lower_32_bits(vpe_channel->pushbuf_bo->bo.offset);
		nv_wr32(dev, NV_VPE_MPEG2_USER_CONFIG, 
				NV_VPE_MPEG2_USER_PRESENT | NV_VPE_MPEG2_USER_VRAM);
		nv_wr32(dev, NV_VPE_MPEG2_USER_OFFSET, 0);
		nv_wr32(dev, NV_VPE_MPEG2_USER_SIZE, dev_priv->fb_available_size);
		nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_1, 0x01010000);
		nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_2, 0x01010000);
		nv_wr32(dev, NV_VPE_MPEG2_USER_CONFIG, 
				NV_VPE_MPEG2_USER_PRESENT | NV_VPE_MPEG2_USER_VRAM);
		nv_wr32(dev, NV_VPE_MPEG2_USER_OFFSET,
				pushbuf_offset);
		nv_wr32(dev, NV_VPE_MPEG2_USER_SIZE,
				vpe_channel->dma.max << 2);
	}

	/* Start up the mpeg2 engine */
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_CONTROL, NV_VPE_MPEG2_ENGINE_STOP);
	nv_wr32(dev, NV_VPE_MPEG2_USER_PUT, 0);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_CONTROL, NV_VPE_MPEG2_ENGINE_START);
	nv_rd32(dev, NV_VPE_MPEG2_ENGINE_CONTROL);
	
	return 0;
}

static int 
nouveau_vpe_channel_init(struct nouveau_vd_vpe_channel *vpe_channel)
{
	struct drm_device *dev = vpe_channel->dev;
	int ret;
	int i;
	uint32_t value;
	
	/* Reset decoder to the initial state.*/
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_CHANNEL << NV_VPE_CMD_TYPE_SHIFT
				| NV_VPE_CMD_INIT_CHANNEL_ACCEL << 24 );
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_CHANNEL << NV_VPE_CMD_TYPE_SHIFT);
	/* NOTE: The surface group info value might be tiling related. */
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_CHANNEL << NV_VPE_CMD_TYPE_SHIFT
			| NV_VPE_CMD_INIT_CHANNEL_SURFACE_GROUP_INFO << 24);
			
	nouveau_vpe_cmd_end_sequence_header(vpe_channel);
	/* No body/trailer for the init cmd.*/
	nouveau_vpe_cmd_end_sequence_finish(vpe_channel);
	
	ret = nouveau_vpe_channel_wait(vpe_channel, vpe_channel->dma.put);
	if (ret)
		return ret;
			
	/* Clear out all surface references.*/
	for (i = 0; i < NV_VPE_MAX_SURFACES; i++) {
		
		nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_SURFACE << NV_VPE_CMD_TYPE_SHIFT
			| NV_VPE_CMD_INIT_SURFACE_LUMA(i));
		nouveau_vpe_cmd_align(vpe_channel);
		
		nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_INIT);
		ret = nouveau_vpe_channel_wait(vpe_channel, vpe_channel->dma.put);
		if (ret)
			return ret;
			
		nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_SURFACE << NV_VPE_CMD_TYPE_SHIFT
			| NV_VPE_CMD_INIT_SURFACE_CHROMA(i));
		nouveau_vpe_cmd_align(vpe_channel);
		
		nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_INIT);
		ret = nouveau_vpe_channel_wait(vpe_channel, vpe_channel->dma.put);
		if (ret)
			return ret;
	}
	
	/* Init the decoder channel.*/
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_CHANNEL << NV_VPE_CMD_TYPE_SHIFT
				    | NV_VPE_CMD_INIT_CHANNEL_ACCEL << 24 
				    /* If IDCT is disabled then only MC is done.*/
				    | NV_VPE_CMD_INIT_CHANNEL_ACCEL_IDCT);
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_CHANNEL << NV_VPE_CMD_TYPE_SHIFT
			| (vpe_channel->width << 12 | vpe_channel->height));
	/* NOTE: The surface group info value might be tiling related. */
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_CHANNEL << NV_VPE_CMD_TYPE_SHIFT
			| NV_VPE_CMD_INIT_CHANNEL_SURFACE_GROUP_INFO << 24
			| (ALIGN(vpe_channel->width, 112) / 32));
			
	nouveau_vpe_cmd_end_sequence_header(vpe_channel);
	/* No body/trailer for the init cmd.*/
	nouveau_vpe_cmd_end_sequence_finish(vpe_channel);
	
	ret = nouveau_vpe_channel_wait(vpe_channel, vpe_channel->dma.put);
	if (ret)
		return ret;
		
	/* Make sure hardware context is setup correctly */
	
	value = nv_rd32(dev, NV_VPE_MPEG2_SURFACE_INFO);
	if (value != ( 0x10000 | (ALIGN(vpe_channel->width, 128) ) ) ) {
		NV_ERROR(dev, "nouveau_vpe_channel_init - channel surface setup wrong for width = %d, height = %d, got = 0x%08X.\n", 
				vpe_channel->width, vpe_channel->height, value);
		return -EINVAL;
	}

	value = nv_rd32(dev, NV_VPE_MPEG2_CONTEXT_DIMENSIONS);
	if (value != ( ( (vpe_channel->width & 0xFFF) << 16) | (vpe_channel->height & 0xFFF) ) ) {
		NV_ERROR(dev, "nouveau_vpe_channel_init - channel dimensions wrong for width = %d, height = %d, got = 0x%08X.\n", 
				vpe_channel->width, vpe_channel->height, value);
		return -EINVAL;
	}
	
	return 0;
}

static void 
nouveau_vpe_channel_shutdown(struct nouveau_vd_vpe_channel *vpe_channel)
{
	nouveau_vpe_cmd_end_sequence_header(vpe_channel);
	/* No body/trailer for the init cmd.*/
	nouveau_vpe_cmd_end_sequence_finish(vpe_channel);
}

static void 
nouveau_vpe_channel_hw_shutdown(struct nouveau_vd_vpe_channel *vpe_channel)
{
	struct drm_device *dev = vpe_channel->dev;
	
	nouveau_vpe_channel_shutdown(vpe_channel);
	
	nouveau_vpe_channel_wait(vpe_channel,  vpe_channel->dma.cur);
	
	/* Just a slight pause. This might not be needed. */
	DRM_UDELAY(100);
	
	/* Turn off the mpeg2 decoder.*/
	nv_wr32(dev, NV_VPE_MPEG2_USER_CONFIG, 
		NV_VPE_MPEG2_USER_NOT_PRESENT);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_CONTROL, NV_VPE_MPEG2_ENGINE_STOP);
	nv_wr32(dev, NV_VPE_MPEG2_USER_PUT, 0);
	nv_wr32(dev, NV_VPE_MPEG2_USER_OFFSET, 0);
	nv_wr32(dev, NV_VPE_MPEG2_USER_SIZE, 0);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_1, 0);
	nv_wr32(dev, NV_VPE_MPEG2_ENGINE_SETUP_2, 0);
	nv_rd32(dev, NV_VPE_MPEG2_ENGINE_CONTROL);
}

static int 
nouveau_vpe_channel_alloc(struct drm_device *dev,
				struct drm_nouveau_vd_vpe_channel_alloc *req, 
				struct drm_file *file_priv)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_vd_vpe_channel *vpe_channel;
	int ret;
	
	if (dev_priv->vpe_channel) {
		NV_ERROR(dev, "vpe channel is already in use.\n");
		return -EPERM;   
	}
	
	if ( (dev_priv->card_type != NV_40) &&
	     (dev_priv->card_type != NV_30) ) {
		NV_ERROR(dev, "vpe is not supported on NV%d.\n", 
			dev_priv->card_type);
		return -EINVAL;   
	}
	
	if ( (req->width < NV_VPE_MIN_WIDTH) ||
	     (req->width > NV_VPE_MAX_WIDTH) ||
	     (req->height < NV_VPE_MIN_HEIGHT) ||
	     (req->height > NV_VPE_MAX_HEIGHT) ) {
		NV_ERROR(dev, "vpe does not support width = %d, height = %d\n", req->width,
		req->height);
		return -EINVAL;
	}
	
	vpe_channel = kzalloc(sizeof(*vpe_channel), GFP_KERNEL);
	if (!vpe_channel)
		return -ENOMEM;
		
	req->width = ALIGN(req->width, 16);
	req->height = ALIGN(req->height, 16);	
	vpe_channel->dev = dev;
	vpe_channel->width = req->width;
	vpe_channel->height = req->height;
	
	ret = nouveau_vpe_channel_pushbuf_alloc(vpe_channel);
	if (ret)
		goto out_err;
		
	ret = nouveau_vpe_channel_hw_init(vpe_channel);
	if (ret)
		goto out_err;
	
	ret = nouveau_vpe_channel_init(vpe_channel);
	if (ret)
		goto out_err;
		
	ret = drm_gem_handle_create(file_priv, vpe_channel->pushbuf_bo->gem,
				    &req->pushbuf_handle);
	if (ret)
		goto out_err;
			
	nouveau_debugfs_vpe_channel_init(vpe_channel);
	
	vpe_channel->file_priv = file_priv;
	dev_priv->vpe_channel = vpe_channel;
	
	NV_INFO(dev, "intialized vpe channel\n");
		
out_err:
	if (ret)
		nouveau_vpe_channel_free(vpe_channel);
			
	return ret;
}

void 
nouveau_vpe_channel_free(struct nouveau_vd_vpe_channel *vpe_channel)
{
	struct drm_device *dev;
	struct drm_nouveau_private *dev_priv;
	struct nouveau_vd_vpe_surface *vpe_surface;
	int i;
	
	if (!vpe_channel)
		return;
		
	dev = vpe_channel->dev;
	dev_priv = dev->dev_private;

	nouveau_vpe_channel_hw_shutdown(vpe_channel);
	
	nouveau_debugfs_vpe_channel_fini(vpe_channel);
	
	for (i = 0; i < ARRAY_SIZE(vpe_channel->surface); i++) {
		vpe_surface = &vpe_channel->surface[i];
		if (vpe_surface->luma_bo)
			nouveau_vpe_unpin_surface(vpe_channel, vpe_surface->luma_bo);
		if (vpe_surface->chroma_bo)
			nouveau_vpe_unpin_surface(vpe_channel, vpe_surface->chroma_bo);
	}
	
	if (vpe_channel->pushbuf_bo) {
		nouveau_bo_unmap(vpe_channel->pushbuf_bo);
		mutex_lock(&vpe_channel->dev->struct_mutex);
		drm_gem_object_unreference(vpe_channel->pushbuf_bo->gem);
		mutex_unlock(&vpe_channel->dev->struct_mutex);
	}
	
	NV_INFO(vpe_channel->dev, "shutdown vpe channel\n");
	
	dev_priv->vpe_channel = NULL;
	
	kfree(vpe_channel);
}

static int
nouveau_vpe_reference_surface(struct nouveau_vd_vpe_channel *vpe_channel, 
						uint32_t surface_index, uint64_t addr_offset,
						bool is_luma)
{
	struct drm_device *dev = vpe_channel->dev;
	uint32_t value;
	int ret;
	
	if (vpe_channel->dma.free < 8)
		nouveau_vpe_reset_pushbuf_to_start(vpe_channel);
		
	nouveau_vpe_cmd_write(vpe_channel, NV_VPE_CMD_INIT_SURFACE << NV_VPE_CMD_TYPE_SHIFT
		| (is_luma ? NV_VPE_CMD_INIT_SURFACE_LUMA(surface_index) : 
		             NV_VPE_CMD_INIT_SURFACE_CHROMA(surface_index))
		| NV_VPE_CMD_INIT_SURFACE_OFFSET_DIV(lower_32_bits(addr_offset)));
	nouveau_vpe_cmd_align(vpe_channel);
	
	if (vpe_channel->dma.free >= NV_VPE_CMD_ALIGNMENT)
		nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_INIT);
	else
		nouveau_vpe_reset_pushbuf_to_start(vpe_channel);
		
	ret = nouveau_vpe_channel_wait(vpe_channel, vpe_channel->dma.cur);
	if (ret)
		return ret;
		
	if (is_luma) {	
		value = nv_rd32(dev, NV_VPE_MPEG2_LUMA_SURFACE_OFFSET_GET(surface_index));
		if (lower_32_bits(addr_offset) != value) {
			NV_ERROR(dev, "vpe - surface.luma ref is wrong. Expected 0x%08X, Got 0x%08X.\n", 
				lower_32_bits(addr_offset), value);
			return -EINVAL;
		}
	}
	else {	
		value = nv_rd32(dev, NV_VPE_MPEG2_CHROMA_SURFACE_OFFSET_GET(surface_index));
		if (lower_32_bits(addr_offset) != value) {
			NV_ERROR(dev, "vpe - surface.chroma ref is wrong. Expected 0x%08X, Got 0x%08X.\n", 
				lower_32_bits(addr_offset), value);
			return -EINVAL;
		}
	}
		
	return 0;
}

static int
nouveau_vpe_channel_validate_surfaces(struct nouveau_vd_vpe_channel *vpe_channel,
                        struct drm_nouveau_vd_vpe_surface *surfaces, int nr_surfaces,
                        struct nouveau_vd_vpe_surface **target_vpe_surface)
{
	struct drm_device *dev = vpe_channel->dev;
	int ret;
	int i;
	struct nouveau_vd_vpe_surface *vpe_surface;
	struct drm_nouveau_vd_vpe_surface *surface;
	uint32_t decoder_surface_size = 0;
		
	for (i = 0, surface = surfaces; i < nr_surfaces; i++, surface++) {
		if (unlikely(surface->surface_index >= ARRAY_SIZE(vpe_channel->surface))) {
			NV_ERROR(dev, "nouveau_vpe_channel_validate_surfaces - surface_index %d is invalid.\n", surface->surface_index);
			return -EINVAL;
		}

		vpe_surface = &vpe_channel->surface[surface->surface_index];
		if (!vpe_surface->luma_bo ||
		    !nouveau_vpe_validate_surface(vpe_channel, surface->luma_handle, vpe_surface->luma_bo)) {
			if (!decoder_surface_size)
				decoder_surface_size = vpe_channel->width * vpe_channel->height;
				
			if (vpe_surface->luma_bo) {
				nouveau_vpe_unpin_surface(vpe_channel, vpe_surface->luma_bo);
				vpe_surface->luma_bo = NULL;
			}
			
			ret = nouveau_vpe_pin_surface(vpe_channel, surface->luma_handle,
                        decoder_surface_size, &vpe_surface->luma_bo);
            if (ret) {
				NV_ERROR(dev, "nouveau_vpe_channel_validate_surfaces - could not pin surface_index %d, luma handle 0x%08X, error %d.\n", surface->surface_index,
				surface->luma_handle, ret);
				return ret;
			}
			
			ret = nouveau_vpe_reference_surface(vpe_channel, surface->surface_index, 
										  vpe_surface->luma_bo->bo.offset, true);
			if (ret) {
				NV_ERROR(dev, "nouveau_vpe_channel_validate_surfaces - could not reference surface_index %d, luma handle 0x%08X, error %d.\n", surface->surface_index,
				surface->luma_handle, ret);
				nouveau_vpe_unpin_surface(vpe_channel, vpe_surface->luma_bo);
				vpe_surface->luma_bo = NULL;
				return ret;
			}
			
			vpe_surface->dma_sequence = 0;
		}
		if (unlikely(!vpe_surface->chroma_bo) ||
		    !nouveau_vpe_validate_surface(vpe_channel, surface->chroma_handle, vpe_surface->chroma_bo) ) {
			
			if (!decoder_surface_size)
				decoder_surface_size = vpe_channel->width * vpe_channel->height;
				
			if (vpe_surface->chroma_bo) {
				nouveau_vpe_unpin_surface(vpe_channel, vpe_surface->chroma_bo);
				vpe_surface->chroma_bo = NULL;
			}
			
			ret = nouveau_vpe_pin_surface(vpe_channel, surface->chroma_handle,
                        decoder_surface_size, &vpe_surface->chroma_bo);
            if (ret) {
				NV_ERROR(dev, "nouveau_vpe_channel_validate_surfaces - could not pin surface_index %d, chroma handle 0x%08X, error %d.\n", surface->surface_index,
				surface->luma_handle, ret);
				return ret;
			}
			
			ret = nouveau_vpe_reference_surface(vpe_channel, surface->surface_index, 
			                                    vpe_surface->chroma_bo->bo.offset, false);
			if (ret) {
				NV_ERROR(dev, "nouveau_vpe_channel_validate_surfaces - could not reference surface_index %d, chroma handle 0x%08X, error %d.\n", surface->surface_index,
				surface->luma_handle, ret);
				nouveau_vpe_unpin_surface(vpe_channel, vpe_surface->chroma_bo);
				vpe_surface->chroma_bo = NULL;
				return ret;
			}
			
			vpe_surface->dma_sequence = 0;
		}
		
		/* First surface is considered the target.*/
		if (i == 0)
			*target_vpe_surface = vpe_surface;
	}
	
	return 0;
}

static int 
nouveau_vpe_channel_pushbuf_fire(struct nouveau_vd_vpe_channel *vpe_channel,
				struct drm_nouveau_vd_vpe_pushbuf_fire *req)
{
	int ret;
	uint32_t *pushbuf = NULL;
	uint32_t *batches = NULL;
	struct drm_nouveau_vd_vpe_surface *surfaces = NULL;
	struct nouveau_vd_vpe_surface *vpe_surface = NULL;
	int i;
	uint32_t offset = 0;
	uint32_t batch_size;
	bool is_end_sequence = req->flags & NOUVEAU_VD_VPE_PUSHBUF_FIRE_FLAG_END_SEQUENCE;
	bool is_update_dma_pos = req->flags & NOUVEAU_VD_VPE_PUSHBUF_FIRE_FLAG_UPDATE_DMA_POS;
	bool do_fire_batch;
	
	if (req->nr_surfaces) {
		surfaces = _u_memcpya(req->surfaces, req->nr_surfaces, sizeof(*surfaces));
		if (unlikely(IS_ERR(surfaces))) {
			ret = PTR_ERR(surfaces);
			goto out;
		}
	}
	
	if (req->nr_dwords) {
		pushbuf = _u_memcpya(req->dwords, req->nr_dwords, sizeof(uint32_t));
		if (unlikely(IS_ERR(pushbuf))) {
			ret = PTR_ERR(pushbuf);
			goto out;
		}
	}
	
	if (req->nr_batches) {
		batches = _u_memcpya(req->batches, req->nr_batches, sizeof(uint32_t));
		if (unlikely(IS_ERR(batches))) {
			ret = PTR_ERR(batches);
			goto out;
		}
	}
	
	if (req->nr_surfaces) {
		ret = nouveau_vpe_channel_validate_surfaces(vpe_channel,
										surfaces, req->nr_surfaces, 
										&vpe_surface);
		if (unlikely(ret))
			goto out;
	}
	
	if (is_update_dma_pos) {
		if (req->dma_cur >= vpe_channel->dma.max) {
			ret = -EINVAL;
		    goto out;
		}
		vpe_channel->dma.cur = req->dma_cur;
		vpe_channel->dma.free = vpe_channel->dma.max - vpe_channel->dma.cur;
		if (!is_end_sequence)
			nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_NORMAL);
	}
	
	for (i = 0; i < req->nr_batches; i++) {
		batch_size = batches[i];
		
		do_fire_batch = !(batch_size & NOUVEAU_VD_VPE_PUSHBUF_FIRE_BATCH_DO_NOT_FIRE);
		
		batch_size &= 0xFFFF;
		
		if (unlikely(!batch_size)) {
			ret = -EINVAL;
			goto out;
		}
		
		if (unlikely((batch_size + offset) > req->nr_dwords)) {
			ret = -EINVAL;
			goto out;
		}

		if (batch_size > vpe_channel->dma.free)
			nouveau_vpe_reset_pushbuf_to_start(vpe_channel);
		
		ret = nouveau_vpe_cmd_write_user_batch(vpe_channel, (const void *)((uint64_t)pushbuf + (offset << 2)), batch_size);
		if (ret)
			goto out;
		
		offset += batch_size;
		vpe_channel->dma.free -= batch_size;
		
		if (!vpe_channel->dma.free) {
			vpe_channel->dma.cur = 0;
			vpe_channel->dma.free = vpe_channel->dma.max;
			nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_END);
		}
		
		if (do_fire_batch)
			nouveau_vpe_fire(vpe_channel, VPE_UDELAY_FIRE_NORMAL);
	}
	
	if (req->nr_dwords) {
		if (vpe_channel->dma.free < NV_VPE_MAX_MB) 
			nouveau_vpe_reset_pushbuf_to_start(vpe_channel);
	}

	if (is_end_sequence) {
		if (vpe_channel->dma.free < NV_VPE_CMD_ALIGNMENT)
			nouveau_vpe_reset_pushbuf_to_start(vpe_channel);
		nouveau_vpe_cmd_end_sequence_header(vpe_channel);
		nouveau_vpe_cmd_end_sequence_trailer(vpe_channel);
		nouveau_vpe_cmd_end_sequence_finish(vpe_channel);
		
		if (vpe_surface) 
			vpe_surface->dma_sequence = vpe_channel->dma.sequence;
	}
	
	req->dma_free = vpe_channel->dma.free;
	req->dma_cur = vpe_channel->dma.cur;
	ret = 0;
out:
	if (!IS_ERR(surfaces) && surfaces)
		kfree(surfaces);
	if (!IS_ERR(batches) && batches)
		kfree(batches);
	if (!IS_ERR(pushbuf) && pushbuf)
		kfree(pushbuf);
		
	return ret;
}

static int 
nouveau_vpe_surface_query(struct nouveau_vd_vpe_channel *vpe_channel,
				struct drm_nouveau_vd_vpe_surface_query *req)
{
	struct drm_device *dev = vpe_channel->dev;
	struct nouveau_vd_vpe_surface *vpe_surface;
	uint32_t i;
	uint32_t value;
	
	if (unlikely(req->surface_index >= ARRAY_SIZE(vpe_channel->surface))) {
		NV_ERROR(dev, "nouveau_vpe_surface_query - invalid surface index %d.\n", 
			req->surface_index);
		return -EINVAL; 
	}
	
	req->is_busy = 0;
	
	vpe_surface = &vpe_channel->surface[req->surface_index];
	
	/* This is set when cmds are being written for the target surface.*/
	if (vpe_surface->dma_sequence) {
		/* Read the current sequence and see if any surfaces have finished rendering.*/
		value = nv_rd32(dev, NV_VPE_MPEG2_SEQUENCE_GET);
		for (i = 0; i < ARRAY_SIZE(vpe_channel->surface); i++) {
			if (vpe_channel->surface[i].luma_bo || 
			    vpe_channel->surface[i].chroma_bo) {
				if (value >= vpe_channel->surface[i].dma_sequence)
					vpe_channel->surface[i].dma_sequence = 0;
				else if (i == req->surface_index) {
					req->is_busy = 1;
				}
			}
		}
	}
	
	return 0;
}

/* IOCtls.*/

int
nouveau_vd_vpe_ioctl_channel_alloc(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	
	struct drm_nouveau_vd_vpe_channel_alloc *req = data;

	NOUVEAU_CHECK_INITIALISED_WITH_RETURN;
		
	return nouveau_vpe_channel_alloc(dev, req, file_priv);
}

int
nouveau_vd_vpe_ioctl_channel_free(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct nouveau_vd_vpe_channel *vpe_channel;
	
	NOUVEAU_CHECK_INITIALISED_WITH_RETURN;
	
	NOUVEAU_GET_VPE_CHANNEL_WITH_RETURN(file_priv, vpe_channel);
	
	nouveau_vpe_channel_free(vpe_channel);
			
	return 0;
}

int nouveau_vd_vpe_ioctl_pushbuf_fire(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	struct nouveau_vd_vpe_channel *vpe_channel;
	struct drm_nouveau_vd_vpe_pushbuf_fire *req = data;
	
	NOUVEAU_CHECK_INITIALISED_WITH_RETURN;
	
	NOUVEAU_GET_VPE_CHANNEL_WITH_RETURN(file_priv, vpe_channel);
	
	return nouveau_vpe_channel_pushbuf_fire(vpe_channel, req);
}

int nouveau_vd_vpe_ioctl_surface_query(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	struct nouveau_vd_vpe_channel *vpe_channel;
	struct drm_nouveau_vd_vpe_surface_query *req = data;
	
	NOUVEAU_CHECK_INITIALISED_WITH_RETURN;
	
	NOUVEAU_GET_VPE_CHANNEL_WITH_RETURN(file_priv, vpe_channel);
	
	return nouveau_vpe_surface_query(vpe_channel, req);
}
