#ifndef _DRM_COMPAT_H_
#define _DRM_COMPAT_H_

#ifndef pgprot_writecombine
#define pgprot_writecombine(p) pgprot_noncached(p)
#endif

#endif
