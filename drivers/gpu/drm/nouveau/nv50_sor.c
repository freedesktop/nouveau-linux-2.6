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
#include "nouveau_connector.h"
#include "nouveau_crtc.h"
#include "nv50_display.h"
#include "nv50_display_commands.h"

extern int nouveau_duallink;

static void
nv50_sor_disconnect(struct nouveau_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_channel *evo = &dev_priv->evo.chan;
	uint32_t offset = encoder->or * 0x40;
	int ret;

	NV_DEBUG(dev, "Disconnecting SOR %d\n", encoder->or);

	ret = RING_SPACE(evo, 2);
	if (ret) {
		NV_ERROR(dev, "no space while disconnecting SOR\n");
		return;
	}
	BEGIN_RING(evo, 0, NV50_SOR0_MODE_CTRL + offset, 1);
	OUT_RING  (evo, NV50_SOR_MODE_CTRL_OFF);
}

static int
nv50_sor_set_clock_mode(struct nouveau_encoder *encoder,
			struct drm_display_mode *mode)
{
	struct drm_device *dev = encoder->base.dev;
	uint32_t limit = encoder->dcb->type == OUTPUT_LVDS ? 112000 : 165000;

	NV_DEBUG(dev, "or %d\n", encoder->or);

	/* We don't yet know what to do, if anything at all. */
	if (encoder->dcb->type == OUTPUT_LVDS)
		return 0;

	/* 0x70000 was a late addition to nv, mentioned as fixing tmds
	 * initialisation on certain gpu's. I presume it's some kind of
	 * clock setting, but what precisely i do not know.
	 */
	nv_wr32(NV50_PDISPLAY_SOR_CLK_CTRL2(encoder->or),
		0x70000 | ((mode->clock > limit) ? 0x101 : 0));

	return 0;
}

static void nv50_sor_dpms(struct drm_encoder *drm_encoder, int mode)
{
	struct drm_device *dev = drm_encoder->dev;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);
	uint32_t val;
	int or = encoder->or;

	NV_DEBUG(dev, "or %d\n", encoder->or);

	if (dev_priv->in_modeset) {
		nv50_sor_disconnect(encoder);
		return;
	}

	/* wait for it to be done */
	if (!nv_wait(NV50_PDISPLAY_SOR_REGS_DPMS_CTRL(or),
		     NV50_PDISPLAY_SOR_REGS_DPMS_CTRL_PENDING, 0)) {
		NV_ERROR(dev, "timeout: SOR_DPMS_CTRL_PENDING(%d) == 0\n", or);
		NV_ERROR(dev, "SOR_DPMS_CTRL(%d) = 0x%08x\n", or,
			 nv_rd32(NV50_PDISPLAY_SOR_REGS_DPMS_CTRL(or)));
	}

	val = nv_rd32(NV50_PDISPLAY_SOR_REGS_DPMS_CTRL(or));

	if (mode == DRM_MODE_DPMS_ON)
		val |= NV50_PDISPLAY_SOR_REGS_DPMS_CTRL_ON;
	else
		val &= ~NV50_PDISPLAY_SOR_REGS_DPMS_CTRL_ON;

	nv_wr32(NV50_PDISPLAY_SOR_REGS_DPMS_CTRL(or), val |
		NV50_PDISPLAY_SOR_REGS_DPMS_CTRL_PENDING);
	if (!nv_wait(NV50_PDISPLAY_SOR_REGS_DPMS_STATE(or),
		     NV50_PDISPLAY_SOR_REGS_DPMS_STATE_WAIT, 0)) {
		NV_ERROR(dev, "timeout: SOR_DPMS_STATE_WAIT(%d) == 0\n", or);
		NV_ERROR(dev, "SOR_DPMS_STATE(%d) = 0x%08x\n", or,
			 nv_rd32(NV50_PDISPLAY_SOR_REGS_DPMS_STATE(or)));
	}
}

static void nv50_sor_save(struct drm_encoder *drm_encoder)
{
	NV_ERROR(drm_encoder->dev, "!!\n");
}

static void nv50_sor_restore(struct drm_encoder *drm_encoder)
{
	NV_ERROR(drm_encoder->dev, "!!\n");
}

struct nouveau_connector *
nouveau_encoder_connector_get(struct nouveau_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_connector *drm_connector;

	list_for_each_entry(drm_connector, &dev->mode_config.connector_list, head) {
		if (drm_connector->encoder == &encoder->base)
			return to_nouveau_connector(drm_connector);
	}

	return NULL;
}

static bool nv50_sor_mode_fixup(struct drm_encoder *drm_encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);
	struct nouveau_connector *connector;

	connector = nouveau_encoder_connector_get(encoder);
	if (!connector)
		return false;

	if ((connector->scaling_mode != DRM_MODE_SCALE_NON_GPU &&
	     connector->scaling_mode != DRM_MODE_SCALE_NO_SCALE) &&
	     connector->native_mode) {
		int id = adjusted_mode->base.id;
		*adjusted_mode = *connector->native_mode;
		adjusted_mode->base.id = id;
	}

	return true;
}

static void nv50_sor_prepare(struct drm_encoder *drm_encoder)
{
}

static void nv50_sor_commit(struct drm_encoder *drm_encoder)
{
}

static void nv50_sor_mode_set(struct drm_encoder *drm_encoder,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode)
{
	struct drm_nouveau_private *dev_priv = drm_encoder->dev->dev_private;
	struct nouveau_channel *evo = &dev_priv->evo.chan;
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);
	struct drm_device *dev = drm_encoder->dev;
	struct nouveau_crtc *crtc = to_nouveau_crtc(drm_encoder->crtc);
	uint32_t offset = encoder->or * 0x40;
	uint32_t mode_ctl = NV50_SOR_MODE_CTRL_OFF;
	int ret;

	NV_DEBUG(dev, "or %d\n", encoder->or);

	ret = dev_priv->in_modeset;
	dev_priv->in_modeset = false;
	nv50_sor_dpms(drm_encoder, DRM_MODE_DPMS_ON);
	dev_priv->in_modeset = ret;

	if (encoder->base.encoder_type == DRM_MODE_ENCODER_LVDS) {
		mode_ctl |= NV50_SOR_MODE_CTRL_LVDS;
	} else {
		mode_ctl |= NV50_SOR_MODE_CTRL_TMDS;
		if (adjusted_mode->clock > 165000)
			mode_ctl |= NV50_SOR_MODE_CTRL_TMDS_DUAL_LINK;
	}

	if (crtc->index == 1)
		mode_ctl |= NV50_SOR_MODE_CTRL_CRTC1;
	else
		mode_ctl |= NV50_SOR_MODE_CTRL_CRTC0;

	if (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC)
		mode_ctl |= NV50_SOR_MODE_CTRL_NHSYNC;

	if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
		mode_ctl |= NV50_SOR_MODE_CTRL_NVSYNC;

	ret = RING_SPACE(evo, 2);
	if (ret) {
		NV_ERROR(dev, "no space while connecting SOR\n");
		return;
	}
	BEGIN_RING(evo, 0, NV50_SOR0_MODE_CTRL + offset, 1);
	OUT_RING  (evo, mode_ctl);
}

static const struct drm_encoder_helper_funcs nv50_sor_helper_funcs = {
	.dpms = nv50_sor_dpms,
	.save = nv50_sor_save,
	.restore = nv50_sor_restore,
	.mode_fixup = nv50_sor_mode_fixup,
	.prepare = nv50_sor_prepare,
	.commit = nv50_sor_commit,
	.mode_set = nv50_sor_mode_set,
	.detect = NULL
};

static void nv50_sor_destroy(struct drm_encoder *drm_encoder)
{
	struct nouveau_encoder *encoder = to_nouveau_encoder(drm_encoder);

	NV_DEBUG(drm_encoder->dev, "\n");

	if (!drm_encoder)
		return;

	drm_encoder_cleanup(&encoder->base);

	kfree(encoder);
}

static const struct drm_encoder_funcs nv50_sor_encoder_funcs = {
	.destroy = nv50_sor_destroy,
};

int nv50_sor_create(struct drm_device *dev, struct dcb_entry *entry)
{
	struct nouveau_encoder *encoder = NULL;
	bool dum;
	int type;

	NV_DEBUG(dev, "\n");

	switch (entry->type) {
	case OUTPUT_TMDS:
		NV_INFO(dev, "Detected a TMDS output\n");
		type = DRM_MODE_ENCODER_TMDS;
		break;
	case OUTPUT_LVDS:
		NV_INFO(dev, "Detected a LVDS output\n");
		type = DRM_MODE_ENCODER_LVDS;

		if (nouveau_bios_parse_lvds_table(dev, 0, &dum, &dum)) {
			NV_ERROR(dev, "Failed parsing LVDS table\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	encoder = kzalloc(sizeof(*encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	encoder->dcb = entry;
	encoder->or = ffs(entry->or) - 1;

	encoder->dual_link = nouveau_duallink;

	/* Set function pointers. */
	encoder->set_clock_mode = nv50_sor_set_clock_mode;

	drm_encoder_init(dev, &encoder->base, &nv50_sor_encoder_funcs, type);
	drm_encoder_helper_add(&encoder->base, &nv50_sor_helper_funcs);

	encoder->base.possible_crtcs = entry->heads;
	encoder->base.possible_clones = 0;

	/* Some default state, unknown what it precisely means. */
	if (encoder->base.encoder_type == DRM_MODE_ENCODER_TMDS) {
		int or = encoder->or;

		nv_wr32(NV50_PDISPLAY_SOR_REGS_UNK_00C(or), 0x03010700);
		nv_wr32(NV50_PDISPLAY_SOR_REGS_UNK_010(or), 0x0000152f);
		nv_wr32(NV50_PDISPLAY_SOR_REGS_UNK_014(or), 0x00000000);
		nv_wr32(NV50_PDISPLAY_SOR_REGS_UNK_018(or), 0x00245af8);
	}

	return 0;
}
