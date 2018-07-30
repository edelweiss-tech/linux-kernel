/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015-2017 Baikal Electronics JSC.
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
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>	/* dev_err */
#include <linux/module.h>
#include <linux/of_platform.h>	/* open firmware functioons */
#include <linux/delay.h>

#include <asm/mips-cm.h>
#include <asm/mips-boards/generic.h>
#include <asm/mach-baikal/pci-baikal.h>

static struct resource dw_mem_resource = {
	.name	= "DW PCI MEM",
	.start	= PHYS_PCIMEM_BASE_ADDR,
	.end	= PHYS_PCIMEM_LIMIT_ADDR,
	.flags	= IORESOURCE_MEM,
};

static struct resource dw_io_resource = {
	.name	= "DW PCI I/O",
	.start	= PHYS_PCIIO_BASE_ADDR,
	.end	= PHYS_PCIIO_LIMIT_ADDR,
	.flags	= IORESOURCE_IO,
};

static struct resource dw_busn_resource = {
	.name	= "DW PCI busn",
	.start	= PCIE_ROOT_BUS_NUM, /* It's going to be overwritten anyway */
	.end	= 255,
	.flags	= IORESOURCE_BUS,
};

extern struct pci_ops dw_pci_ops;
static int dw_pcie_get_busn(void);
extern int dw_pcie_init(void);

static struct pci_controller dw_controller = {
	.pci_ops	= &dw_pci_ops,
	.io_resource	= &dw_io_resource,
	.mem_resource	= &dw_mem_resource,
	.busn_resource	= &dw_busn_resource,
	.get_busno  = dw_pcie_get_busn,
};

#ifdef CONFIG_PCI_MSI
static int dw_msi_irq;
#endif /* CONFIG_PCI_MSI */

static int dw_aer_irq;

/* Retrieve the secondary bus number of the RC */
static int dw_pcie_get_busn(void)
{
   return PCIE_ROOT_BUS_NUM;
}

void baikal_find_vga_mem_init(void);

void __init mips_pcibios_init(void)
{
	struct pci_controller *controller;

	if (dw_pcie_init()) {
		pr_err("%s: Init DW PCI controller failed\n", __func__);
		return;
	}

#ifdef CONFIG_PCI_MSI
	if (dw_msi_init()) {
		pr_err("%s: Init DW PCI MSI failed\n", __func__);
		return;
	}
#endif /* CONFIG_PCI_MSI */

	pci_set_flags(PCI_REASSIGN_ALL_RSRC);

	/* Register PCI controller */
	controller = &dw_controller;

	iomem_resource.end &= 0xfffffffffULL;
	ioport_resource.end = controller->io_resource->end;
	controller->io_map_base = IO_BASE;
	controller->io_offset = 0;
	register_pci_controller(controller);

#ifdef CONFIG_CPU_SUPPORTS_UNCACHED_ACCELERATED
	baikal_find_vga_mem_init();
#endif /* CONFIG_CPU_SUPPORTS_UNCACHED_ACCELERATED */
}

#ifdef CONFIG_PCIEAER
irqreturn_t aer_irq(int irq, void *context);
#endif /* CONFIG_PCIEAER */

irqreturn_t dw_aer_interrupt(int id, void *dev_id) 
{
#ifdef CONFIG_PCIEAER	
	aer_irq(id, dev_id);
#endif /* CONFIG_PCIEAER */

	return IRQ_HANDLED;
}

static int dw_pci_drv_probe(struct platform_device *pdev)
{

#ifdef CONFIG_PCI_MSI
	if ((dw_msi_irq = platform_get_irq(pdev, 0)) < 0) {
		dev_err(&pdev->dev, "There is no MSI IRQ resource specified.\n");
		return -EINVAL;
	}

	if (request_irq(dw_msi_irq, dw_msi_interrupt, IRQF_SHARED, "MSI PCI", pdev)) {
		dev_err(&pdev->dev, "Cannot request MSI irq %d.\n", dw_msi_irq);
		return -ENXIO;
	}
#endif /* CONFIG_PCI_MSI */

	if ((dw_aer_irq = platform_get_irq(pdev, 1)) < 0) {
		dev_err(&pdev->dev, "There is no AER IRQ resource specified.\n");
		return -EINVAL;
	}

	if (request_irq(dw_aer_irq, dw_aer_interrupt, IRQF_SHARED, "AER PCI", pdev)) {
		dev_err(&pdev->dev, "Cannot request AER irq %d.\n", dw_aer_irq);
		return -ENXIO;
	}

	dev_info(&pdev->dev, "DW PCIe driver successfully loaded.\n");
	dev_info(&pdev->dev, "MSI IRQ:%d   AER IRQ:%d\n", dw_msi_irq, dw_aer_irq);

	/* Return success */
	return 0;
}

static int dw_pci_drv_remove(struct platform_device *pdev)
{
#ifdef CONFIG_PCI_MSI
		/* Free IRQ resource */
		free_irq(dw_msi_irq, pdev);
#endif /* CONFIG_PCI_MSI */

	free_irq(dw_aer_irq, pdev);

	/* Return success */
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dw_pci_of_match[] = {
	{ .compatible = "be,baikal-pci", },
	{ .compatible = "snps,dw-pci", },
	{ .compatible = "snps,dw-pcie", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw_pci_of_match);
#endif

static struct platform_driver dw_pci_driver = {
	.probe          = dw_pci_drv_probe,
	.remove         = dw_pci_drv_remove,
	.driver         = {
		.name   = "dw_pci",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw_pci_of_match),
#endif /* CONFIG_OF */
		},
};

module_platform_driver(dw_pci_driver);
MODULE_VERSION("1.2");
MODULE_DESCRIPTION("Baikal Electronics PCIe Driver.");
MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Alexey Malakhov");
MODULE_ALIAS("platform:dw_pci");
