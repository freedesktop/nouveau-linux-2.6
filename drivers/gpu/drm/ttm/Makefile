#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm
ttm-y := ttm_agp_backend.o ttm_bo.o ttm_bo_util.o ttm_bo_vm.o \
	 ttm_execbuf_util.o ttm_fence.o ttm_fence_user.o ttm_lock.o \
	 ttm_memory.o ttm_object.o ttm_pat_compat.o ttm_placement_user.o \
	 ttm_tt.o ttm_stub.o

obj-$(CONFIG_DRM_TTM) += ttm.o
