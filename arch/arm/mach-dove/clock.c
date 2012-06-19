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
#include <plat/mv_xor.h>
#include <plat/ehci-orion.h>
#include <plat/common.h>
#include "clock.h"

static DEFINE_SPINLOCK(gating_lock);

static struct clk __init *dove_register_gate(const char *name, u8 bit_idx)
{
	return clk_register_gate(NULL, name, "tclk", 0,
				 (void __iomem *)CLOCK_GATING_CONTROL,
				 bit_idx, 0, &gating_lock);
}

/*
 * Create clkdev entries for dove.
 * Dove (like Kirkwood) has gated clocks for some of its peripherals,
 * so it need to create its own clkdev entries. For all the other orion
 * devices, create clkdev entries to the tclk.
 */
void __init dove_clkdev_init(struct clk *tclk)
{
	struct clk *clk_usb0, *clk_usb1, *clk_gbe, *clk_sata, *clk_phy;
	struct clk *clk_pci0, *clk_pci1, *clk_sdio0, *clk_sdio1;
	struct clk *clk_nand, *clk_camera, *clk_i2s0, *clk_i2s1;
	struct clk *clk_cesa, *clk_ac97, *clk_pdma, *clk_xor0, *clk_xor1;

	clk_usb0 = dove_register_gate("usb.0", CLOCK_GATING_USB0_BIT);
	clk_usb1 = dove_register_gate("usb.1", CLOCK_GATING_USB1_BIT);
	clk_gbe = dove_register_gate("ge0", CLOCK_GATING_GBE_BIT);
	clk_phy = clk_register_gate(NULL, "phy0", "ge0", 0,
				(void __iomem *)CLOCK_GATING_CONTROL,
				CLOCK_GATING_GIGA_PHY_BIT, 0, &gating_lock);
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

	orion_clkdev_add(NULL, "orion_spi.0", tclk);
	orion_clkdev_add(NULL, "orion_spi.1", tclk);
	orion_clkdev_add(NULL, MV643XX_ETH_NAME ".0", clk_phy);
	orion_clkdev_add(NULL, "orion_wdt", tclk);
	orion_clkdev_add(NULL, "orion-ehci.0", clk_usb0);
	orion_clkdev_add(NULL, "orion-ehci.1",  clk_usb1);
	orion_clkdev_add("PCI0", NULL, clk_pci0);
	orion_clkdev_add("PCI1", NULL, clk_pci1);
	orion_clkdev_add(NULL, "sdhci-dove.0", clk_sdio0);
	orion_clkdev_add(NULL, "sdhci-dove.1", clk_sdio1);
	orion_clkdev_add(NULL, "sata_mv.0", clk_sata);
	orion_clkdev_add(NULL, "sata_mv.1", clk_sata);
	orion_clkdev_add(NULL, "dove-nand", clk_nand);
	orion_clkdev_add(NULL, "cafe1000-ccic.0", clk_camera);
	orion_clkdev_add(NULL, "kirkwood-i2s.0", clk_i2s0);
	orion_clkdev_add(NULL, "kirkwood-i2s.1", clk_i2s1);
	orion_clkdev_add(NULL, "crypto", clk_cesa);
	orion_clkdev_add("AC97CLK", NULL, clk_ac97);
	orion_clkdev_add("PDMA", NULL, clk_pdma);
	orion_clkdev_add(NULL, MV_XOR_SHARED_NAME "0", clk_xor0);
	orion_clkdev_add(NULL, MV_XOR_SHARED_NAME "1", clk_xor1);
}

