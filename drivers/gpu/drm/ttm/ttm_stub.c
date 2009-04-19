#include <linux/module.h>

static int __init ttm_init(void)
{
	return 0;
}

static void __exit ttm_exit(void)
{
}

module_init(ttm_init);
module_exit(ttm_exit);

MODULE_LICENSE("GPL and additional rights");
