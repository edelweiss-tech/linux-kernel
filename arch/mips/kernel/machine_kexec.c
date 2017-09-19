/*
 * machine_kexec.c for kexec
 * Created by <nschichan@corp.free.fr> on Thu Oct 12 15:15:06 2006
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */
#include <linux/compiler.h>
#include <linux/kexec.h>
#include <linux/mm.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>
#include <asm/page.h>
#include <asm/machine_kexec.h>

extern unsigned long kexec_start_address;
extern unsigned long kexec_indirection_page;

int (*_machine_kexec_prepare)(struct kimage *) = NULL;
void (*_machine_kexec_shutdown)(void) = NULL;
void (*_machine_crash_shutdown)(struct pt_regs *regs) = NULL;
#ifdef CONFIG_SMP
void (*relocated_kexec_smp_wait) (void *);
atomic_t kexec_ready_to_reboot = ATOMIC_INIT(0);
#endif

int
machine_kexec_prepare(struct kimage *kimage)
{
	if (_machine_kexec_prepare)
		return _machine_kexec_prepare(kimage);
	return 0;
}

void
machine_kexec_cleanup(struct kimage *kimage)
{
}

void
machine_shutdown(void)
{
	if (_machine_kexec_shutdown)
		_machine_kexec_shutdown();
}

void
machine_crash_shutdown(struct pt_regs *regs)
{
	if (_machine_crash_shutdown)
		_machine_crash_shutdown(regs);
	else
		default_machine_crash_shutdown(regs);
}

typedef void (*noretfun_t)(void) __noreturn;

void
machine_kexec(struct kimage *image)
{
	unsigned long reboot_code_buffer;
	unsigned long entry;
	unsigned long *ptr;

	/*
	 * XXX: from arm/kernel/machine_kexec.c:
	 * This can only happen if machine_shutdown() failed to disable some
	 * CPU, and that can only happen if the checks in
	 * machine_kexec_prepare() were not correct. If this fails, we can't
	 * reliably kexec anyway, so BUG_ON is appropriate.
	 */
	BUG_ON(num_online_cpus() > 1);

	pr_info("machine_kexec - machine_kexec: image = %p\n", image);

	reboot_code_buffer =
	  (unsigned long)page_address(image->control_code_page);
	pr_debug("machine_kexec: reboot_code_buffer @ 0x%lx\n", reboot_code_buffer);

	kexec_start_address =
		(unsigned long) phys_to_virt(image->start);
	pr_debug("machine_kexec: kexec_start_address (\"kernel_entry\") 0x%lx\n", kexec_start_address);
	pr_debug("machine_kexec: kexec_relocate_new_kernel code @ 0x%p\n", (void *)kexec_relocate_new_kernel);
	pr_debug("machine_kexec: kexec_relocate_new_kernel size %lu\n", KEXEC_RELOCATE_NEW_KERNEL_SIZE);

	if (image->type == KEXEC_TYPE_DEFAULT) {
		kexec_indirection_page =
			(unsigned long) phys_to_virt(image->head & PAGE_MASK);
	} else {
		kexec_indirection_page = (unsigned long)&image->head;
	}

	pr_info("machine_kexec: copy kexec_relocate_new_kernel code to 0x%p\n", (void *)reboot_code_buffer);
	memcpy((void*)reboot_code_buffer, kexec_relocate_new_kernel,
	       KEXEC_RELOCATE_NEW_KERNEL_SIZE);

	/*
	 * The generic kexec code builds a page list with physical
	 * addresses. they are directly accessible through KSEG0 (or
	 * CKSEG0 or XPHYS if on 64bit system), hence the
	 * phys_to_virt() call.
	 */
	for (ptr = &image->head; (entry = *ptr) && !(entry &IND_DONE);
	     ptr = (entry & IND_INDIRECTION) ?
	       phys_to_virt(entry & PAGE_MASK) : ptr + 1) {
		if (*ptr & IND_SOURCE || *ptr & IND_INDIRECTION ||
		    *ptr & IND_DESTINATION)
			*ptr = (unsigned long) phys_to_virt(*ptr);
	}

	/*
	 * we do not want to be bothered.
	 */
	local_irq_disable();

	printk("Will call new kernel at %08lx\n", image->start);
	printk("Bye ...\n");
	__flush_cache_all();
#ifdef CONFIG_SMP
	/* All secondary cpus now may jump to kexec_wait cycle */
	relocated_kexec_smp_wait = reboot_code_buffer +
		(void *)(kexec_smp_wait - kexec_relocate_new_kernel);
	smp_wmb();
	atomic_set(&kexec_ready_to_reboot, 1);
#endif
	((noretfun_t) reboot_code_buffer)();
}
