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

static u32 dove_clocks_get_bits(u32 addr, u32 start_bit, u32 end_bit)
{
	u32 mask;
	u32 value;

	value = readl(addr);
	mask = ((1 << (end_bit + 1 - start_bit)) - 1) << start_bit;
	value = (value & mask) >> start_bit;
	return value;
}

static void dove_clocks_set_bits(u32 addr, u32 start_bit, u32 end_bit,
				 u32 value)
{
	u32 mask;
	u32 new_value;
	u32 old_value;


	old_value = readl(addr);

	mask = ((1 << (end_bit + 1 - start_bit)) - 1) << start_bit;
	new_value = old_value & (~mask);
	new_value |= (mask & (value << start_bit));
	writel(new_value, addr);
}

static void dove_clocks_set_axi_clock(u32 divider)
{
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL1_REG, 10, 10, 1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 1, 6, divider);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 7, 7, 1);
	udelay(1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 7, 7, 0);
}

static void dove_clocks_set_gpu_clock(u32 divider)
{
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL1_REG, 10, 10, 1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 8, 13, divider);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 14, 14, 1);
	udelay(1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 14, 14, 0);
}

static long gpu_round_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *prate)
{
	unsigned long divider;
	unsigned long best_rate;

	divider = *prate;
	do_div(divider, rate);
	if (divider > 63)
		divider = 63;
	if (divider < 1)
		divider = 1;
	best_rate = *prate;
	do_div(best_rate, divider);
	return best_rate;
}

static unsigned long gpu_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	u32 divider;
	unsigned long rate = parent_rate;

	divider = dove_clocks_get_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 8, 13);
	do_div(rate, divider);

	return rate;
}

static int gpu_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned long divider = parent_rate;

	do_div(divider, rate);
	if (divider < 1 || divider > 64) {
		printk(KERN_ERR "Unsupported gpu clock %lu\n", rate);
		return -EINVAL;
	}
	printk(KERN_INFO "Setting gpu clock to %lu (divider: %u)\n",
	       rate, divider);
	dove_clocks_set_gpu_clock(divider);
	return 0;
}

struct clk_ops gpu_clk_ops = {
	.round_rate	= gpu_round_rate,
	.recalc_rate	= gpu_recalc_rate,
	.set_rate	= gpu_set_rate,
};

u32 axi_divider[] = {-1, 2, 1, 3, 4, 6, 5, 7, 8, 10, 9};

static long axi_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	unsigned long divider;
	unsigned long best_rate;

	divider = *prate;
	do_div(divider, rate);
	if (divider > 10)
		divider = 0;
	if (divider < 1)
		divider = 1;
	best_rate = *prate;
	do_div(best_rate, divider);
	return best_rate;
}

static unsigned long axi_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	u32 divider;
	unsigned long rate = parent_rate;

	divider = dove_clocks_get_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 1, 6);
	do_div(rate, axi_divider[divider]);

	return rate;
}

static int axi_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	u32 divider = 0, i;
	unsigned long div;

	for (i = 1; i < 11; i++) {
		div = parent_rate;
		do_div(div, axi_divider[i]);
		if (div == rate) {
			divider = i;
			break;
		}
	}

	if (i == 11) {
		printk(KERN_ERR "Unsupported AXI clock %lu\n",
			 rate);

		return -EINVAL;
	}
	printk(KERN_INFO "Setting axi clock to %lu (divider: %u)\n",
		 rate, divider);
	dove_clocks_set_axi_clock(divider);
	return 0;
}

struct clk_ops axi_clk_ops = {
	.round_rate	= axi_round_rate,
	.recalc_rate	= axi_recalc_rate,
	.set_rate	= axi_set_rate,
};

static struct clk __init *dove_register_clk(const char *name,
					    struct clk_ops *ops,
					    const char *parent_name)
{
	struct clk_init_data init;
	struct clk_hw *hw;

	hw = kzalloc(sizeof(struct clk_hw), GFP_KERNEL);
	if (!hw) {
		printk(KERN_ERR "Cannot allocate clock\n");
		return NULL;
	}
	init.name = name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	hw->init = &init;

	return clk_register(NULL, hw);
}

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
	struct clk *clk_gpu, *clk_axi;

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
	clk_axi = dove_register_clk("axi", &axi_clk_ops, "pll_clk");
	clk_gpu = dove_register_clk("gpu", &gpu_clk_ops, "pll_clk");

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
	orion_clkdev_add("GCCLK", NULL, clk_gpu);
	orion_clkdev_add("AXICLK", NULL, clk_axi);
}

