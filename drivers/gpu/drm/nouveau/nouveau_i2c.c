/*
 * Copyright 2009 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Ben Skeggs
 */

#include "drmP.h"
#include "nouveau_drv.h"
#include "nouveau_i2c.h"
#include "nouveau_hw.h"

static void
nv04_i2c_setscl(void *data, int state)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;
	uint8_t val;

	val = (NVReadVgaCrtc(dev, 0, i2c->wr) & 0xd0) | (state ? 0x20 : 0);
	NVWriteVgaCrtc(dev, 0, i2c->wr, val | 0x01);
}

static void
nv04_i2c_setsda(void *data, int state)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;
	uint8_t val;

	val = (NVReadVgaCrtc(dev, 0, i2c->wr) & 0xe0) | (state ? 0x10 : 0);
	NVWriteVgaCrtc(dev, 0, i2c->wr, val | 0x01);
}

static int
nv04_i2c_getscl(void *data)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	return !!(NVReadVgaCrtc(dev, 0, i2c->rd) & 4);
}

static int
nv04_i2c_getsda(void *data)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	return !!(NVReadVgaCrtc(dev, 0, i2c->rd) & 8);
}

static void
nv4e_i2c_setscl(void *data, int state)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;
	uint8_t val;

	val = (nv_rd32(i2c->wr) & 0xd0) | (state ? 0x20 : 0);
	nv_wr32(i2c->wr, val | 0x01);
}

static void
nv4e_i2c_setsda(void *data, int state)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;
	uint8_t val;

	val = (nv_rd32(i2c->wr) & 0xe0) | (state ? 0x10 : 0);
	nv_wr32(i2c->wr, val | 0x01);
}

static int
nv4e_i2c_getscl(void *data)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	return !!((nv_rd32(i2c->rd) >> 16) & 4);
}

static int
nv4e_i2c_getsda(void *data)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	return !!((nv_rd32(i2c->rd) >> 16) & 8);
}

static int
nv50_i2c_getscl(void *data)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	return !!(nv_rd32(i2c->rd) & 1);
}


static int
nv50_i2c_getsda(void *data)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	return !!(nv_rd32(i2c->rd) & 2);
}

static void
nv50_i2c_setscl(void *data, int state)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	nv_wr32(i2c->wr, 4 | (i2c->data ? 2 : 0) | (state ? 1 : 0));
}

static void
nv50_i2c_setsda(void *data, int state)
{
	struct nouveau_i2c_chan *i2c = data;
	struct drm_device *dev = i2c->dev;

	nv_wr32(i2c->wr, (nv_rd32(i2c->rd) & 1) | 4 | (state ? 2 : 0));
	i2c->data = state;
}

int
nouveau_i2c_new(struct drm_device *dev, const char *name, unsigned index,
		struct nouveau_i2c_chan **pi2c)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct dcb_i2c_entry *dcbi2c = &dev_priv->vbios->dcb->i2c[index];
	struct nouveau_i2c_chan *i2c;
	int ret;

	if (dcbi2c->chan) {
		*pi2c = dcbi2c->chan;
		return 0;
	}

	i2c = drm_calloc(1, sizeof(*i2c), DRM_MEM_DRIVER);
	if (i2c == NULL)
		return -ENOMEM;

	snprintf(i2c->adapter.name, sizeof(i2c->adapter.name),
		 "nouveau-%s-%s", pci_name(dev->pdev), name);
	i2c->adapter.owner = THIS_MODULE;
	i2c->adapter.algo_data = &i2c->algo;
	i2c->dev = dev;

	switch (dcbi2c->port_type) {
	case 0:
		i2c->algo.setsda = nv04_i2c_setsda;
		i2c->algo.setscl = nv04_i2c_setscl;
		i2c->algo.getsda = nv04_i2c_getsda;
		i2c->algo.getscl = nv04_i2c_getscl;
		i2c->rd = dcbi2c->read;
		i2c->wr = dcbi2c->write;
		break;
	case 4:
		i2c->algo.setsda = nv4e_i2c_setsda;
		i2c->algo.setscl = nv4e_i2c_setscl;
		i2c->algo.getsda = nv4e_i2c_getsda;
		i2c->algo.getscl = nv4e_i2c_getscl;
		i2c->rd = 0x600800 + dcbi2c->read;
		i2c->wr = 0x600800 + dcbi2c->write;
		break;
	case 5:
		i2c->algo.setsda = nv50_i2c_setsda;
		i2c->algo.setscl = nv50_i2c_setscl;
		i2c->algo.getsda = nv50_i2c_getsda;
		i2c->algo.getscl = nv50_i2c_getscl;
		/* not 100% convinced this is correct everywhere, but
		 * the best guess so far..
		 */
		if (dcbi2c->read <= 3)
			i2c->rd = 0xe138 + (dcbi2c->read * 24);
		else
		if (dev_priv->chipset < 0x90)
			i2c->rd = 0xe1e0 + (dcbi2c->read * 24);
		else
			i2c->rd = 0xe1d4 + (dcbi2c->read * 32);
		i2c->wr = i2c->rd;
		break;
	default:
		NV_ERROR(dev, "DCB I2C port type %d unknown\n",
			 dcbi2c->port_type);
		drm_free(i2c, sizeof(*i2c), DRM_MEM_DRIVER);
		return -EINVAL;
	}
	i2c->algo.udelay = 20;
	i2c->algo.timeout = usecs_to_jiffies(2200);
	i2c->algo.data = i2c;

	i2c_set_adapdata(&i2c->adapter, i2c);

	ret = i2c_bit_add_bus(&i2c->adapter);
	if (ret) {
		NV_ERROR(dev, "Failed to register i2c %s\n", name);
		drm_free(i2c, sizeof(*i2c), DRM_MEM_DRIVER);
		return ret;
	}

	*pi2c = dcbi2c->chan = i2c;
	return 0;

}

void
nouveau_i2c_del(struct nouveau_i2c_chan **pi2c)
{
	struct nouveau_i2c_chan *i2c = *pi2c;

	if (!i2c)
		return;

	*pi2c = NULL;
	i2c_del_adapter(&i2c->adapter);
	drm_free(i2c, sizeof(*i2c), DRM_MEM_DRIVER);
}

bool
nouveau_i2c_detect(struct nouveau_connector *connector)
{
	/* kindly borrrowed from the intel driver, hope it works. */
	uint8_t out_buf[] = { 0x0, 0x0};
	uint8_t buf[2];
	bool ret;
	struct i2c_msg msgs[] = {
		{
			.addr = 0x50,
			.flags = 0,
			.len = 1,
			.buf = out_buf,
		},
		{
			.addr = 0x50,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = buf,
		}
	};

	if (!connector->i2c_chan)
		return false;

	ret = (i2c_transfer(&connector->i2c_chan->adapter, msgs, 2) == 2);
	return ret;
}
