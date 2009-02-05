/*
 * Copyright (C) 2007 Ben Skeggs.
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

#include "ttm/ttm_fence_driver.h"

#include "drmP.h"
#include "drm.h"

#include "nouveau_drv.h"
#include "nouveau_dma.h"

struct nouveau_channel *
nouveau_fence_channel(struct drm_device *dev, uint32_t class)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;

	if (class == 0)
		class = dev_priv->channel->id;

	return dev_priv->fifos[class];
}

static bool
nouveau_fence_has_irq(struct ttm_fence_device *fdev, uint32_t class,
		      uint32_t flags)
{
	struct drm_nouveau_private *dev_priv = nouveau_fdev(fdev);

	if (dev_priv->card_type < NV_10)
		return true;
	return false;
}

static int
nouveau_fence_emit(struct ttm_fence_device *fdev, uint32_t class,
		   uint32_t flags, uint32_t *sequence,
		   unsigned long *timeout_jiffies)
{
	struct drm_nouveau_private *dev_priv = nouveau_fdev(fdev);
	struct drm_device *dev = dev_priv->dev;
	struct nouveau_channel *chan = nouveau_fence_channel(dev, class);
	int ret;

	ret = RING_SPACE(chan, 2);
	if (ret)
		return ret;

	*sequence  = ++chan->next_sequence;
	*timeout_jiffies = jiffies + 3 * DRM_HZ;

	if (dev_priv->card_type >= NV_10)
		BEGIN_RING(chan, NvSubM2MF, NV_MEMORY_TO_MEMORY_FORMAT_SET_REF, 1);
	else
		BEGIN_RING(chan, NvSubM2MF, 0x0150, 1);
	OUT_RING  (chan, *sequence);
	FIRE_RING (chan);

	return 0;
}

static void
nouveau_fence_poll(struct ttm_fence_device *fdev, uint32_t class,
		   uint32_t waiting_types)
{
	struct drm_nouveau_private *dev_priv = nouveau_fdev(fdev);
	struct drm_device *dev = dev_priv->dev;
	struct nouveau_channel *chan = nouveau_fence_channel(dev, class);
	uint32_t sequence;

	if (unlikely(!chan)) {
		static int done = 0;
		if (!done) {
			NV_ERROR(dev, "AIII channel %d inactive\n", class);
			WARN_ON(1);
			done = 1;
		}
		ttm_fence_handler(fdev, class, ~0, TTM_FENCE_TYPE_EXE, 0);
		return;
	}

	if (unlikely(!waiting_types))
		return;

	if (dev_priv->card_type >= NV_10)
		sequence = nvchan_rd32(0x48);
	else
		sequence = chan->last_sequence_irq;

	ttm_fence_handler(fdev, class, sequence, TTM_FENCE_TYPE_EXE, 0);
}

void
nouveau_fence_handler(struct drm_device *dev, int channel)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct ttm_fence_class_manager *fc =
		&dev_priv->ttm.fdev.fence_class[channel];

	write_lock(&fc->lock);
	nouveau_fence_poll(&dev_priv->ttm.fdev, channel,
			   fc->waiting_types | TTM_FENCE_TYPE_EXE);
	write_unlock(&fc->lock);
}

struct ttm_fence_driver nouveau_fence_driver = {
	.has_irq	= nouveau_fence_has_irq,
	.emit		= nouveau_fence_emit,
	.flush          = NULL,
	.poll           = nouveau_fence_poll,
	.needed_flush   = NULL,
	.wait           = NULL,
	.signaled	= NULL,
	.lockup		= NULL
};

struct ttm_fence_class_init nouveau_fence_class_init = {
	.wrap_diff	= (1 << 30),
	.flush_diff	= (1 << 29),
	.sequence_mask	= 0xffffffffU,
};

int
nouveau_fence_ttm_device_init(struct drm_device *dev)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	int ret;

	ret = ttm_fence_device_init(dev_priv->engine.fifo.channels,
				    dev_priv->ttm.mem_global_ref.object,
				    &dev_priv->ttm.fdev,
				    &nouveau_fence_class_init, true,
				    &nouveau_fence_driver);
	return ret;
}

