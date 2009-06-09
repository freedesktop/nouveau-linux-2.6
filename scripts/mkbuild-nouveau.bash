#!/bin/bash

# 395e0ddc44005ced5e4fed9bfc2e4bdf63d37627 shmem_file_setup export

# Use e.g. CONFIG_FB_SAVAGE to get FB_CFB_FILLRECT, FB_CFB_COPYAREA
# and FB_CFB_IMAGEBLIT. CONFIG_DRM must be a module.

# cd into an empty directory, and call this script from there.

# Path to the kernel tree, from which DRM is built.
NKDIR="/home/pq/linux/linux-2.6-torvalds"

# Path to the configured kernel tree you normally use
KDIR="$(readlink -f /lib/modules/`uname -r`/source)"

# All resulting files *should* be written to OUTDIR, but it seems
# the external module directory does not honour that. I.e. the
# the external modules are built into DRMDIR.

DRMDIR="$NKDIR/drivers/gpu/drm"
OUTDIR="$(pwd)"
CONFIG="$KDIR/.config"

# --- nothing configurable below this line ---

CMD="$1"

function config_check_ym {
	OPTION="$1"
	grep -q -e "CONFIG_$OPTION=[ym]" "$CONFIG" && return 0
	echo "Option CONFIG_$OPTION is not set in your kernel config."
	return 1
}

function config_require {
	local FAIL=0
	for REQ in "$@"; do
		config_check_ym "$REQ" || FAIL=1
	done
	return $FAIL
}

function die {
	echo "$@"
	exit 1
}

function kmake {
	make -C "$KDIR" O="$OUTDIR" "$@"
}

function kmake_drm {
	kmake M="$DRMDIR" EXTRA_CFLAGS="-I$NKDIR/include/drm" "$@"
}

echo "KDIR     $KDIR"
echo "DRMDIR   $DRMDIR"
echo "OUTDIR   $OUTDIR"
echo ""

REQCFG="DRM FB BACKLIGHT_CLASS_DEVICE FB_CFB_FILLRECT FB_CFB_COPYAREA FB_CFB_IMAGEBLIT"

case "$CMD" in
	init)
		cp "$CONFIG" "$OUTDIR/.config"
		cp -a "$KDIR/scripts" "$OUTDIR/scripts"
		kmake silentoldconfig
		#echo -e "\nCONFIG_DRM_NOUVEAU=m" >> "$NEWCONFIG"
		#echo -e "\nCONFIG_DRM_TTM=m" >> "$NEWCONFIG"
		config_require $REQCFG || \
			die "Please, fix you kernel config to have: $REQCFG"
		echo -e "\n#define CONFIG_DRM_TTM_MODULE 1" >> "$OUTDIR/include/linux/autoconf.h"
		echo -e "\nCONFIG_DRM_TTM=m" >> "$OUTDIR/include/config/auto.conf"
		echo -e "\n#define CONFIG_DRM_NOUVEAU_MODULE 1" >> "$OUTDIR/include/linux/autoconf.h"
		echo -e "\nCONFIG_DRM_NOUVEAU=m" >> "$OUTDIR/include/config/auto.conf"
		;;
	build)
		kmake_drm drm.ko || \
			die "making drm.ko failed"
		kmake_drm ttm/ttm.ko || \
			die "making ttm.ko failed"
		kmake_drm nouveau/nouveau.ko || \
			die "making nouveau.ko failed"
		;;
	clean)
		kmake_drm clean
		;;
	*)
		echo "Unknown command: '$CMD'"
		echo "Supported commands: init build clean"
		echo "Please edit the script to configure it."
		exit 1
esac

