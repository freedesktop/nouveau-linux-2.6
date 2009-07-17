/*
 * Copyright (C) 2008 Maarten Maathuis.
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
#include "drm_crtc_helper.h"
#include "nouveau_reg.h"
#include "nouveau_drv.h"
#include "nouveau_dma.h"
#include "nouveau_encoder.h"
#include "nouveau_crtc.h"
#include "nv50_display.h"

static void
nv50_dac_disconnect(struct nouveau_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_channel *evo = &dev_priv->evo.chan;
	int ret;

	NV_DEBUG(dev, "or %d\n", encoder->or);

	ret = RING_SPACE(evo, 2);
	if (ret) {
		NV_ERROR(dev, "no space while disconnecting DAC\n");
		return;
	}
	BEGIN_RING(evo, 0, NV50_EVO_DAC(encoder->or, MODE_CTRL), 1);
	OUT_RING  (evo, 0);
}

static int
nv50_dac_set_clock_mode(struct nouveau_encoder *encoder,
			struct drm_display_mode *mode)
{
	struct drm_device *dev = encoder->base.dev;

	NV_DEBUG(dev, "or %d\n", encoder->or);

	nv_wr32(NV50_PDISPLAY_DAC_CLK_CTRL2(encoder->or),  0);
	return 0;
}

static enum drm_connector_status
nv50_dac_detect(struct drm_encoder *drm_encoder,
		struct drm_connector *drm_connector)
{
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);
	struct drm_device *dev = encoder->base.dev;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	enum drm_connector_status status = connector_status_disconnected;
	uint32_t dpms_state, load_pattern, load_state;
	int or = encoder->or;

	nv_wr32(NV50_PDISPLAY_DAC_REGS_CLK_CTRL1(or), 0x00000001);
	dpms_state = nv_rd32(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or));

	nv_wr32(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or),
		0x00150000 | NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_PENDING);
	if (!nv_wait(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or),
		     NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_PENDING, 0)) {
		NV_ERROR(dev, "timeout: DAC_DPMS_CTRL_PENDING(%d) == 0\n", or);
		NV_ERROR(dev, "DAC_DPMS_CTRL(%d) = 0x%08x\n", or,
			  nv_rd32(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or)));
		return status;
	}

	/* Use bios provided value if possible. */
	if (dev_priv->vbios->dactestval) {
		load_pattern = dev_priv->vbios->dactestval;
		NV_DEBUG(dev, "Using bios provided load_pattern of %d\n",
			  load_pattern);
	} else {
		load_pattern = 340;
		NV_DEBUG(dev, "Using default load_pattern of %d\n",
			 load_pattern);
	}

	nv_wr32(NV50_PDISPLAY_DAC_REGS_LOAD_CTRL(or),
		NV50_PDISPLAY_DAC_REGS_LOAD_CTRL_ACTIVE | load_pattern);
	mdelay(45); /* give it some time to process */
	load_state = nv_rd32(NV50_PDISPLAY_DAC_REGS_LOAD_CTRL(or));

	nv_wr32(NV50_PDISPLAY_DAC_REGS_LOAD_CTRL(or), 0);
	nv_wr32(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or), dpms_state |
		NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_PENDING);

	if ((load_state & NV50_PDISPLAY_DAC_REGS_LOAD_CTRL_PRESENT) ==
			  NV50_PDISPLAY_DAC_REGS_LOAD_CTRL_PRESENT)
		status = connector_status_connected;

	if (status == connector_status_connected)
		NV_DEBUG(dev, "Load was detected on output with or %d\n", or);
	else
		NV_DEBUG(dev, "Load was not detected on output with or %d\n", or);

	return status;
}

static void nv50_dac_dpms(struct drm_encoder *drm_encoder, int mode)
{
	struct drm_device *dev = drm_encoder->dev;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);
	uint32_t val;
	int or = encoder->or;

	NV_DEBUG(dev, "or %d\n", or);

	if (dev_priv->in_modeset) {
		nv50_dac_disconnect(encoder);
		return;
	}

	/* wait for it to be done */
	if (!nv_wait(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or),
		     NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_PENDING, 0)) {
		NV_ERROR(dev, "timeout: DAC_DPMS_CTRL_PENDING(%d) == 0\n", or);
		NV_ERROR(dev, "DAC_DPMS_CTRL(%d) = 0x%08x\n", or,
			  nv_rd32(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or)));
		return;
	}

	val = nv_rd32(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or)) & ~0x7F;

	if (mode != DRM_MODE_DPMS_ON)
		val |= NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_BLANKED;

	switch (mode) {
	case DRM_MODE_DPMS_STANDBY:
		val |= NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_HSYNC_OFF;
		break;
	case DRM_MODE_DPMS_SUSPEND:
		val |= NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_VSYNC_OFF;
		break;
	case DRM_MODE_DPMS_OFF:
		val |= NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_OFF;
		val |= NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_HSYNC_OFF;
		val |= NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_VSYNC_OFF;
		break;
	default:
		break;
	}

	nv_wr32(NV50_PDISPLAY_DAC_REGS_DPMS_CTRL(or),
		val | NV50_PDISPLAY_DAC_REGS_DPMS_CTRL_PENDING);
}

static void nv50_dac_save(struct drm_encoder *drm_encoder)
{
	NV_ERROR(drm_encoder->dev, "!!\n");
}

static void nv50_dac_restore(struct drm_encoder *drm_encoder)
{
	NV_ERROR(drm_encoder->dev, "!!\n");
}

static bool nv50_dac_mode_fixup(struct drm_encoder *drm_encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void nv50_dac_prepare(struct drm_encoder *drm_encoder)
{
}

static void nv50_dac_commit(struct drm_encoder *drm_encoder)
{
}

static void nv50_dac_mode_set(struct drm_encoder *drm_encoder,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode)
{
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);
	struct drm_device *dev = drm_encoder->dev;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_channel *evo = &dev_priv->evo.chan;
	struct nouveau_crtc *crtc = to_nouveau_crtc(drm_encoder->crtc);
	uint32_t mode_ctl = 0, mode_ctl2 = 0;
	int ret;

	NV_DEBUG(dev, "or %d\n", encoder->or);

	ret = dev_priv->in_modeset;
	dev_priv->in_modeset = false;
	nv50_dac_dpms(drm_encoder, DRM_MODE_DPMS_ON);
	dev_priv->in_modeset = ret;

	if (crtc->index == 1)
		mode_ctl |= NV50_EVO_DAC_MODE_CTRL_CRTC1;
	else
		mode_ctl |= NV50_EVO_DAC_MODE_CTRL_CRTC0;

	/* Lacking a working tv-out, this is not a 100% sure. */
	if (encoder->base.encoder_type == DRM_MODE_ENCODER_DAC) {
		mode_ctl |= 0x40;
	} else
	if (encoder->base.encoder_type == DRM_MODE_ENCODER_TVDAC) {
		mode_ctl |= 0x100;
	}

	if (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC)
		mode_ctl2 |= NV50_EVO_DAC_MODE_CTRL2_NHSYNC;

	if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
		mode_ctl2 |= NV50_EVO_DAC_MODE_CTRL2_NVSYNC;

	ret = RING_SPACE(evo, 3);
	if (ret) {
		NV_ERROR(dev, "no space while connecting DAC\n");
		return;
	}
	BEGIN_RING(evo, 0, NV50_EVO_DAC(encoder->or, MODE_CTRL), 2);
	OUT_RING  (evo, mode_ctl);
	OUT_RING  (evo, mode_ctl2);
}

static const struct drm_encoder_helper_funcs nv50_dac_helper_funcs = {
	.dpms = nv50_dac_dpms,
	.save = nv50_dac_save,
	.restore = nv50_dac_restore,
	.mode_fixup = nv50_dac_mode_fixup,
	.prepare = nv50_dac_prepare,
	.commit = nv50_dac_commit,
	.mode_set = nv50_dac_mode_set,
	.detect = nv50_dac_detect
};

static void nv50_dac_destroy(struct drm_encoder *drm_encoder)
{
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);

	NV_DEBUG(drm_encoder->dev, "\n");

	if (!drm_encoder)
		return;

	drm_encoder_cleanup(&encoder->base);

	kfree(encoder);
}

static const struct drm_encoder_funcs nv50_dac_encoder_funcs = {
	.destroy = nv50_dac_destroy,
};

int nv50_dac_create(struct drm_device *dev, struct dcb_entry *entry)
{
	struct nouveau_encoder *encoder = NULL;

	NV_DEBUG(dev, "\n");
	NV_INFO(dev, "Detected a DAC output\n");

	encoder = kzalloc(sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	encoder->dcb = entry;
	encoder->or = ffs(entry->or) - 1;

	/* Set function pointers. */
	encoder->set_clock_mode = nv50_dac_set_clock_mode;

	drm_encoder_init(dev, &encoder->base, &nv50_dac_encoder_funcs,
			 DRM_MODE_ENCODER_DAC);
	drm_encoder_helper_add(&encoder->base, &nv50_dac_helper_funcs);

	encoder->base.possible_crtcs = entry->heads;
	encoder->base.possible_clones = 0;
	return 0;
}

