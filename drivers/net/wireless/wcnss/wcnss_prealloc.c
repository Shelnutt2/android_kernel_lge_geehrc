#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/wcnss_wlan.h>

struct wcnss_prealloc wcnss_allocs[] = {
	{0, 8 * 1024, NULL},
	{0, 8 * 1024, NULL},
	{0, 8 * 1024, NULL},
	{0, 8 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 32 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 64 * 1024, NULL},
	{0, 128 * 1024, NULL},
};

void wcnss_prealloc_init (void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++) {
		wcnss_allocs[i].occupied = 0;
		wcnss_allocs[i].ptr = kmalloc (wcnss_allocs[i].size, GFP_KERNEL);
	}
}

void wcnss_prealloc_deinit (void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++) {
		kfree(wcnss_allocs[i].ptr);
	}
}

void * wcnss_prealloc (unsigned int size)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++) {
		if (wcnss_allocs[i].occupied) continue;;
		if (wcnss_allocs[i].size > size) {
			/* we found the slot */
			//printk(KERN_INFO"%s : prealloc index%d %d -> %d\n", __func__,
			//		i, size, wcnss_allocs[i].size);
			wcnss_allocs[i].occupied = 1;
			return wcnss_allocs[i].ptr;
		}
	}
	printk (KERN_INFO"%s : prealloc failed\n", __func__);

	return NULL;
}
EXPORT_SYMBOL(wcnss_prealloc);

int wcnss_prefree (void * ptr)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(wcnss_allocs); i++) {
		if (wcnss_allocs[i].ptr == ptr) {
			//printk(KERN_INFO"%s : free index%d\n", __func__, i);
			wcnss_allocs[i].occupied = 0;
			return 1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(wcnss_prefree);
