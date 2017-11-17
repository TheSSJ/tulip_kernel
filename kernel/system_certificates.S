#include <linux/export.h>
#include <linux/init.h>

#define GLOBAL(name)	\
	.globl VMLINUX_SYMBOL(name);	\
	VMLINUX_SYMBOL(name):

	__INITRODATA

GLOBAL(system_certificate_list)
	.incbin "/root/kernel/kernel/x509_certificate_list"
GLOBAL(system_certificate_list_end)
