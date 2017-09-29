/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015,2016 Baikal Electronics JSC.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 *  BAIKAL MIPS boards specific PCI support.
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/msi.h>
#include <linux/spinlock.h>

#include "pci-baikal.h"


#define	MAX_DW_MSI_IRQS		256
#define HW_MSI_IRQ_OFF		512

typedef struct {
	unsigned int		msi_data;
	struct irq_domain	*irq_domain;
	spinlock_t		msi_lock;
	DECLARE_BITMAP(msi_irqs, MAX_DW_MSI_IRQS);
} dw_msi_data_t;

dw_msi_data_t 	dw_msi_data;

int arch_setup_msi_irq(struct pci_dev *dev, struct msi_desc *desc)
{
	struct msi_msg msg;
	int virq, hwirq;
	unsigned int reg;
	unsigned long flags;

	spin_lock_irqsave(&dw_msi_data.msi_lock, flags);

	if ((hwirq = bitmap_find_free_region(dw_msi_data.msi_irqs, MAX_DW_MSI_IRQS, 0)) < 0) {
		spin_unlock_irqrestore(&dw_msi_data.msi_lock, flags);
		pr_err("PCI MSI: Cannot allocate MSI HW interrupt number.\n");
		return -ENOSPC;
	}

	/* Get a virtual interrupt number for MSI interrupt. */
	if ((virq = irq_find_mapping(dw_msi_data.irq_domain, HW_MSI_IRQ_OFF + hwirq)) == 0) {
		spin_unlock_irqrestore(&dw_msi_data.msi_lock, flags);
		pr_err("PCI MSI: Cannot allocate MSI virtual interrupt number.\n");
		return -ENOSPC;
	}

	/* Get MSI interrupt register based on virq */
	reg = READ_PCIE_REG(PCIE_MSI_CTRL_INT_0_EN_OFF + MSI_INTERRUPT_OFF * (hwirq / 32));

	/* Enable MSI interrupt. */
	reg |= (1 << (hwirq % 32));
	WRITE_PCIE_REG(PCIE_MSI_CTRL_INT_0_EN_OFF + MSI_INTERRUPT_OFF * (hwirq / 32), reg);

	spin_unlock_irqrestore(&dw_msi_data.msi_lock, flags);

	pr_info("PCI MSI: setup hwirq:%d  virq:%d\n", hwirq, virq);

	/* Construct message. */
	msg.data = hwirq;
	msg.address_lo = virt_to_phys((void *)dw_msi_data.msi_data);
	msg.address_hi = 0;

	wmb();

	irq_set_msi_desc(virq, desc);
	pci_write_msi_msg(virq, &msg);

	return 0;
}

/**
 * Called when a device no longer needs its MSI interrupts. All
 * MSI interrupts for the device are freed.
 *
 * @irq:    The devices first irq number. There may be multple in sequence.
 */
void arch_teardown_msi_irq(unsigned int irq)
{
	unsigned int reg;
	unsigned long flags;

	pr_info("PCI MSI: free irq %d\n", irq);

	spin_lock_irqsave(&dw_msi_data.msi_lock, flags);

	bitmap_release_region(dw_msi_data.msi_irqs, irq, 0);

	/* Get MSI interrupt register based on virq */
	reg = READ_PCIE_REG(PCIE_MSI_CTRL_INT_0_EN_OFF + MSI_INTERRUPT_OFF * (irq / 32));

	/* Disable MSI interrupt. */
	reg &= ~(1 << (irq % 32));
	WRITE_PCIE_REG(PCIE_MSI_CTRL_INT_0_EN_OFF + MSI_INTERRUPT_OFF * (irq / 32), reg);

	spin_unlock_irqrestore(&dw_msi_data.msi_lock, flags);
}

static struct irq_chip dw_irq_chip_msi = {
	.name = "PCI MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

/*
 * Called by the interrupt handling code when an MSI interrupt
 * occurs.
 */
irqreturn_t dw_msi_interrupt(int id, void *dev_id) 
{
	int irq, bit, i;
	irqreturn_t ret = IRQ_NONE;
	unsigned long reg;

	pr_debug("PCI MSI: irq=%d\n", id);

	/* Check all MSI interrupts */
	for (i = 0; i < 8; i++) {
		if ((reg = READ_PCIE_REG(PCIE_MSI_CTRL_INT_0_STATUS_OFF + MSI_INTERRUPT_OFF * (i))) != 0) {
			ret = IRQ_HANDLED;
			bit = 0;
			while ((bit = find_next_bit(&reg, 32, bit)) != 32) {
				irq = irq_find_mapping(dw_msi_data.irq_domain, HW_MSI_IRQ_OFF + i * 32 + bit);

				pr_debug("PCI MSI: msi_stat_reg=0x%lx hwirq=%d, virq=%d\n", reg,
					(i * 32 + bit), irq);

				/* Ack an interrupt. */
				WRITE_PCIE_REG(PCIE_MSI_CTRL_INT_0_STATUS_OFF + MSI_INTERRUPT_OFF * (i),
					(1 << bit));

				/* Generate irq. */
				do_IRQ(irq);

				bit++;

				wmb();

			}
		}

	}

	return ret;
}

static int dw_msi_map(struct irq_domain *domain, unsigned int irq, irq_hw_number_t hwirq)
{
        irq_set_chip_and_handler(irq, &dw_irq_chip_msi, handle_simple_irq);
        irq_set_chip_data(irq, domain->host_data);

        return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
        .map = dw_msi_map,
};


/*
 * Initializes the MSI interrupt handling code
 */
int dw_msi_init(void)
{
	int i;

	dw_msi_data.msi_data = PHYS_PCI_MSI_BASE_ADDR;
	spin_lock_init(&dw_msi_data.msi_lock);

	/* Set data address. */
	WRITE_PCIE_REG(PCIE_MSI_CTRL_ADDR_OFF, virt_to_phys((void *)dw_msi_data.msi_data));
	WRITE_PCIE_REG(PCIE_MSI_CTRL_UPPER_ADDR_OFF, 0);

	/* Register MSI interrupts. */
	dw_msi_data.irq_domain = irq_domain_add_linear(NULL, (HW_MSI_IRQ_OFF + MAX_DW_MSI_IRQS),
							&msi_domain_ops, &dw_msi_data);
	if (!dw_msi_data.irq_domain) {
		pr_err("PCI MSI: Cannot create irq domain.\n");
		return -ENXIO;
	}

	for (i = HW_MSI_IRQ_OFF; i < (HW_MSI_IRQ_OFF + MAX_DW_MSI_IRQS); i++) {
		irq_create_mapping(dw_msi_data.irq_domain, i);
		irq_set_chip_and_handler(i, &dw_irq_chip_msi, handle_simple_irq);
	}

	return 0;
}

