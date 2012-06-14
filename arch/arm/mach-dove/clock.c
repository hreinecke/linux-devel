/*
 *  linux/arch/arm/mach-dove/clock.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <mach/pm.h>
#include <mach/hardware.h>
#include "clock.h"

/* Create a clkdev entry for a given device/clk */
void __init dove_clkdev_add(const char *con_id, const char *dev_id,
			     struct clk *clk)
{
	struct clk_lookup *cl;

	cl = clkdev_alloc(clk, con_id, dev_id);
	if (cl)
		clkdev_add(cl);
}

static DEFINE_SPINLOCK(gating_lock);

static struct clk __init *dove_register_gate(const char *name, u8 bit_idx)
{
	return clk_register_gate(NULL, name, "tclk", 0,
				 (void __iomem *)CLOCK_GATING_CONTROL,
				 bit_idx, CLK_GATE_SET_TO_DISABLE,
				 &gating_lock);
}

/*
 * Create clkdev entries for dove.
 * Dove (like Kirkwood) has gated clocks for some of its peripherals,
 * so it need to create its own clkdev entries. For all the other orion
 * devices, create clkdev entries to the tclk.
 */
void __init dove_clkdev_init(struct clk *tclk)
{
	struct clk *clk_usb0, *clk_usb1, *clk_gbe, *clk_sata;
	struct clk *clk_pci0, *clk_pci1, *clk_sdio0, *clk_sdio1;
	struct clk *clk_nand, *clk_camera, *clk_i2s0, *clk_i2s1;
	struct clk *clk_cesa, *clk_ac97, *clk_pdma, *clk_xor0, *clk_xor1;

	clk_usb0 = dove_register_gate("usb.0", CLOCK_GATING_USB0_BIT);
	clk_usb1 = dove_register_gate("usb.1", CLOCK_GATING_USB1_BIT);
	clk_gbe = dove_register_gate("ge0",CLOCK_GATING_GIGA_PHY_BIT);
	clk_sata = dove_register_gate("sata.0", CLOCK_GATING_SATA_BIT);
	clk_pci0 = dove_register_gate("pex.0", CLOCK_GATING_PCIE0_BIT);
	clk_pci1 = dove_register_gate("pex.1", CLOCK_GATING_PCIE1_BIT);
	clk_sdio0 = dove_register_gate("sdio.0", CLOCK_GATING_SDIO0_BIT);
	clk_sdio1 = dove_register_gate("sdio.1", CLOCK_GATING_SDIO1_BIT);
	clk_nand = dove_register_gate("nand", CLOCK_GATING_NAND_BIT);
	clk_camera = dove_register_gate("camera", CLOCK_GATING_CAMERA_BIT);
	clk_i2s0 = dove_register_gate("i2s.0", CLOCK_GATING_I2S0_BIT);
	clk_i2s1 = dove_register_gate("i2s.1", CLOCK_GATING_I2S1_BIT);
	clk_cesa = dove_register_gate("cesa", CLOCK_GATING_CRYPTO_BIT);
	clk_ac97 = dove_register_gate("ac97", CLOCK_GATING_AC97_BIT);
	clk_pdma = dove_register_gate("pdma", CLOCK_GATING_PDMA_BIT);
	clk_xor0 = dove_register_gate("xor.0", CLOCK_GATING_XOR0_BIT);
	clk_xor1 = dove_register_gate("xor.1", CLOCK_GATING_XOR1_BIT);

	dove_clkdev_add("orion-ehci.0", NULL, clk_usb0);
	dove_clkdev_add("orion-ehci.1", NULL, clk_usb1);
	dove_clkdev_add(NULL, "PCI0", clk_pci0);
	dove_clkdev_add(NULL, "PCI1", clk_pci1);
	dove_clkdev_add("sdhci-dove.0", NULL, clk_sdio0);
	dove_clkdev_add("sdhci-dove.1", NULL, clk_sdio1);
	dove_clkdev_add("sata_mv.0", NULL, clk_sata);
	dove_clkdev_add("dove-nand", NULL, clk_nand);
	dove_clkdev_add("crypto", NULL, clk_cesa);
	dove_clkdev_add(NULL, "AC97CLK", clk_ac97);
	dove_clkdev_add(NULL, "PDMA", clk_pdma);
	dove_clkdev_add("mv_xor_shared.0", NULL, clk_xor0);
	dove_clkdev_add("mv_xor_shared.1", NULL, clk_xor1);
}

