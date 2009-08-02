#ifndef _DRM_COMPAT_H_
#define _DRM_COMPAT_H_

#include <linux/version.h>

#ifndef pgprot_writecombine
#define pgprot_writecombine(p) pgprot_noncached(p)
#endif

#if defined(CONFIG_X86)
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31)) || defined(kmap_atomic_prot)
#define CONFIG_X86_EXPORTED_KMAP_ATOMIC_PROT
#endif
#endif

#endif
