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

static void dove_clocks_set_gpu_clock(u32 divider)
{
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL1_REG, 10, 10, 1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 8, 13, divider);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 14, 14, 1);
	udelay(1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 14, 14, 0);
}

static void dove_clocks_set_vmeta_clock(u32 divider)
{
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL1_REG, 10, 10, 1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 15, 20, divider);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 21, 21, 1);
	udelay(1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 21, 21, 0);
}

static void dove_clocks_set_lcd_clock(u32 divider)
{
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL1_REG, 10, 10, 1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 22, 27, divider);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 28, 28, 1);
	udelay(1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 28, 28, 0);
}

static void dove_clocks_set_axi_clock(u32 divider)
{
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL1_REG, 10, 10, 1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 1, 6, divider);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 7, 7, 1);
	udelay(1);
	dove_clocks_set_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 7, 7, 0);
}

static long pll_round_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *prate)
{
	unsigned long divider;
	unsigned long best_rate;

	divider = *prate;
	do_div(divider, rate);
	if (divider > 31)
		divider = 31;
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
	if (divider < 1 || divider > 31) {
		printk(KERN_ERR "Unsupported gpu clock %lu\n", rate);
		return -EINVAL;
	}
	printk(KERN_INFO "Setting gpu clock to %lu (divider: %lu)\n",
	       rate, divider);
	dove_clocks_set_gpu_clock(divider);
	return 0;
}

struct clk_ops gpu_clk_ops = {
	.round_rate	= pll_round_rate,
	.recalc_rate	= gpu_recalc_rate,
	.set_rate	= gpu_set_rate,
};

static int vmeta_clk_enable(struct clk_hw *hw)
{
	unsigned int reg;

	/* power on */
	reg = readl(PMU_PWR_SUPLY_CTRL_REG);
	reg &= ~PMU_PWR_VPU_PWR_DWN_MASK;
	writel(reg, PMU_PWR_SUPLY_CTRL_REG);
	/* un-reset unit */
	reg = readl(PMU_SW_RST_CTRL_REG);
	reg |= PMU_SW_RST_VIDEO_MASK;
	writel(reg, PMU_SW_RST_CTRL_REG);
	/* disable isolators */
	reg = readl(PMU_ISO_CTRL_REG);
	reg |= PMU_ISO_VIDEO_MASK;
	writel(reg, PMU_ISO_CTRL_REG);

	return 0;
}

static void vmeta_clk_disable(struct clk_hw *hw)
{
	unsigned int reg;

	/* enable isolators */
	reg = readl(PMU_ISO_CTRL_REG);
	reg &= ~PMU_ISO_VIDEO_MASK;
	writel(reg, PMU_ISO_CTRL_REG);
	/* reset unit */
	reg = readl(PMU_SW_RST_CTRL_REG);
	reg &= ~PMU_SW_RST_VIDEO_MASK;
	writel(reg, PMU_SW_RST_CTRL_REG);
	/* power off */
	reg = readl(PMU_PWR_SUPLY_CTRL_REG);
	reg |= PMU_PWR_VPU_PWR_DWN_MASK;
	writel(reg, PMU_PWR_SUPLY_CTRL_REG);
}

static int vmeta_clk_is_enabled(struct clk_hw *hw)
{
	u32 reg;

	/* Read power down bit */
	reg = readl(PMU_PWR_SUPLY_CTRL_REG);
	reg &= PMU_PWR_VPU_PWR_DWN_MASK;

	return reg ? 0 : 1;
}

static unsigned long vmeta_recalc_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	u32 divider;
	unsigned long rate = parent_rate;

	divider = dove_clocks_get_bits(PMU_PLL_CLK_DIV_CTRL0_REG, 15, 20);
	do_div(rate, divider);

	return rate;
}

static int vmeta_set_rate(struct clk_hw *hw, unsigned long rate,
			  unsigned long parent_rate)
{
	unsigned long divider = parent_rate;

	do_div(divider, rate);
	if (divider < 1 || divider > 31) {
		printk(KERN_ERR "Unsupported vmeta clock %lu\n", rate);
		return -EINVAL;
	}
	printk(KERN_INFO "Setting vmeta clock to %lu (divider: %lu)\n",
	       rate, divider);
	dove_clocks_set_vmeta_clock(divider);
	return 0;
}

struct clk_ops vmeta_clk_ops = {
	.enable		= vmeta_clk_enable,
	.disable	= vmeta_clk_disable,
	.is_enabled	= vmeta_clk_is_enabled,
	.round_rate	= pll_round_rate,
	.recalc_rate	= vmeta_recalc_rate,
	.set_rate	= vmeta_set_rate,
};

static void set_lcd_internal_ref_clock(u32 clock_div, u32 is_half_div)
{
	u32	reg;
	u32	old_clock_div, old_half_div;

	/* disable preemption, the gen conf regs might be accessed by other
	** drivers.
	*/
	preempt_disable();

	/*
	 * If current setting is right, just return.
	 */
	reg = readl(DOVE_GLOBAL_CONFIG_1);
	old_clock_div = (reg & (0x3F << 10)) >> 10;
	old_half_div = (reg & (1 << 16)) >> 16;

	if (clock_div == old_clock_div && is_half_div == old_half_div) {
		preempt_enable();
		return;
	}

	/* Clear LCD_Clk_Enable (Enable LCD Clock).			*/
	reg &= ~(1 << 17);
	writel(reg, DOVE_GLOBAL_CONFIG_1);

	/* Set LCD_CLK_DIV_SEL in LCD TWSI and CPU Configuration 1	*/
	reg = readl(DOVE_GLOBAL_CONFIG_1);
	reg &= ~(1 << 9);
	writel(reg, DOVE_GLOBAL_CONFIG_1);

	/* Configure division factor (N = LCD_EXT_DIV[5:0], N<32) in    */
	/* Config 1 Register.                                           */
	reg &= ~(0x3F << 10);
	reg |= (clock_div << 10);

	/* Set LCD_Half_integer_divider = 1 in LCD TWSI and CPU Config 1*/
	if (is_half_div)
		reg |= (1 << 16);
	else
		reg &= ~(1 << 16);

	writel(reg, DOVE_GLOBAL_CONFIG_1);

	/* Set LCD_Ext_Clk_Div_Load in LCD TWSI and CPU Config 2.	*/
	reg = readl(DOVE_GLOBAL_CONFIG_2);
	reg |= (1 << 24);
	writel(reg, DOVE_GLOBAL_CONFIG_2);

	preempt_enable();

	/* Insert S/W delay of at least 200 nsec.			*/
	udelay(1);

	preempt_disable();
	/* Clear LCD_Ext_Clk_Div_Load.					*/
	reg = readl(DOVE_GLOBAL_CONFIG_2);
	reg &= ~(1 << 24);
	writel(reg, DOVE_GLOBAL_CONFIG_2);

	/* Set LCD_Clk_Enable (Enable LCD Clock).			*/
	reg = readl(DOVE_GLOBAL_CONFIG_1);
	reg |= (1 << 17);
	writel(reg, DOVE_GLOBAL_CONFIG_1);
	preempt_enable();

	return;
}


static inline u64 calc_diff(u64 a, u64 b)
{
	if (a > b)
		return a - b;
	else
		return b - a;
}

static void calc_best_clock_div(unsigned long parent_rate,
				u32 tar_freq, u32 *axi_div,
				u32 *is_ext_rem)
{
	u64 req_div;
	u64 best_rem = 0xFFFFFFFFFFFFFFFFll;
	unsigned int best_axi_div = 0;
	unsigned int best_lcd_div = 0;
	u64 tmp_lcd_div;
	int ext_rem = 0;
	u32 i, borders;
	u64 rem;
	u64 temp;
	int override = 0;	/* Used to mark special cases where the LCD */
	int div_2_skip = 3;	/* divider value is not recommended.	    */
				/* (in our case it's divider 3).	    */

	/* Calculate required dividor */
	req_div = parent_rate;
	do_div(req_div, tar_freq);

	/* Look for the whole division with the smallest remainder */
	for (i = 5; i < 64; i++) {
		temp = (u64)tar_freq * (u64)i;
		borders = req_div;
		do_div(borders, i);
		/* The LCD divsion must be smaller than 64K */
		if (borders < SZ_64K) {
			tmp_lcd_div = parent_rate;
			/* We cannot do 64-bit / 64-bit operations,
			** thus... */
			do_div(tmp_lcd_div, i);
			do_div(tmp_lcd_div, tar_freq);
			rem = calc_diff(parent_rate, (temp * tmp_lcd_div));
			if ((rem < best_rem) ||
			    ((override == 1) && (rem == best_rem))) {
				best_rem = rem;
				best_axi_div = i;
				best_lcd_div = tmp_lcd_div;
				override = ((best_lcd_div == div_2_skip) ?
						1 : 0);
			}
			if ((best_rem == 0) && (override == 0))
				break;
			/* Check the next LCD divider */
			tmp_lcd_div++;
			rem = calc_diff((temp * tmp_lcd_div), parent_rate);
			if ((rem < best_rem) ||
			    ((override == 1) && (rem == best_rem))) {
				best_rem = rem;
				best_axi_div = i;
				best_lcd_div = tmp_lcd_div;
				override = ((best_lcd_div == div_2_skip) ?
						1 : 0);
			}
			if ((best_rem == 0) && (override == 0))
				break;
		}
	}

	/* Look for the extended division with the smallest remainder */
	if (best_rem != 0) {
		req_div = parent_rate * 10;
		do_div(req_div, tar_freq);
		/* Half div can be between 12.5 & 31.5 */
		for (i = 55; i <= 315; i += 10) {
			temp = (u64)tar_freq * (u64)i;
			borders = req_div;
			do_div(borders, i);
			if (borders < SZ_64K) {
				tmp_lcd_div = parent_rate * 10;
				/* We cannot do 64-bit / 64-bit operations,
				** thus... */
				do_div(tmp_lcd_div, i);
				do_div(tmp_lcd_div, tar_freq);

				rem = calc_diff(parent_rate * 10,
						(tmp_lcd_div * temp));
				do_div(rem, 10);
				if ((rem < best_rem) ||
				    ((override == 1) && (rem == best_rem))) {
					ext_rem = 1;
					best_rem = rem;
					best_axi_div = i / 10;
					best_lcd_div = tmp_lcd_div;
					override = ((best_lcd_div == div_2_skip)
							? 1 : 0);
				}
				if ((best_rem == 0) && (override == 0))
					break;
				/* Check next LCD divider */
				tmp_lcd_div++;
				rem = calc_diff((tmp_lcd_div * temp),
						parent_rate * 10);
				do_div(rem, 10);
				if ((rem < best_rem) ||
				    ((override == 1) && (rem == best_rem))) {
					ext_rem = 1;
					best_rem = rem;
					best_axi_div = i / 10;
					best_lcd_div = tmp_lcd_div;
					override = ((best_lcd_div == div_2_skip)
							? 1 : 0);
				}
				if ((best_rem == 0) && (override == 0))
					break;
			}
		}
	}

	*is_ext_rem = ext_rem;
	*axi_div = best_axi_div;
	return;
}

static long lcd_round_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *prate)
{
	unsigned long divider;
	unsigned long best_rate;

	divider = *prate;
	do_div(divider, rate);
	if (divider > 63)
		divider = 63;
	if (divider < 5)
		divider = 5;
	best_rate = *prate;
	do_div(best_rate, divider);
	return best_rate;
}

static unsigned long lcd_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	u32 reg;
	u32 old_clock_div, old_half_div;
	unsigned long pll_src = parent_rate * 2, divider;

	/* disable preemption, the gen conf regs might be accessed by other
	** drivers.
	*/
	preempt_disable();

	/*
	 * If current setting is right, just return.
	 */
	reg = readl(DOVE_GLOBAL_CONFIG_1);
	old_clock_div = (reg & (0x3F << 10)) >> 10;
	old_half_div = (reg & (1 << 16)) >> 16;

	preempt_enable();
	divider = (old_clock_div * 2) + old_half_div;
	do_div(pll_src, divider);

	return pll_src;
}

#ifndef CONFIG_FB_DOVE_CLCD_SCLK_VALUE
#define LCD_SCLK	(1000*1000*1000)
#else
#define LCD_SCLK	(CONFIG_FB_DOVE_CLCD_SCLK_VALUE*1000*1000)
#endif

static int lcd_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned long axi_div = parent_rate;
	int is_ext = 0;

	rate = LCD_SCLK;
	do_div(axi_div, rate);
	printk(KERN_INFO "set internal refclk divider to %lu.%d\n",
	       axi_div, is_ext ? 5 : 0);
	set_lcd_internal_ref_clock(axi_div, is_ext);

	return 0;
}

static int accrt_lcd_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	u32 axi_div, is_ext = 0;

	calc_best_clock_div(parent_rate, rate, &axi_div, &is_ext);

	printk(KERN_INFO "set internal refclk divider to %d.%d."
	       "(accurate mode)\n", axi_div, is_ext ? 5 : 0);
	set_lcd_internal_ref_clock(axi_div, is_ext);

	return 0;
}

static int lcd_clk_enable(struct clk_hw *hw)
{
	u32	reg;
	reg = readl(DOVE_GLOBAL_CONFIG_1);
	reg |= (1 << 17);
	writel(reg, DOVE_GLOBAL_CONFIG_1);

	/* We keep original PLL output 2G clock. */
	dove_clocks_set_lcd_clock(1);
	return 0;
}

static void lcd_clk_disable(struct clk_hw *hw)
{
	u32	reg;
	reg = readl(DOVE_GLOBAL_CONFIG_1);
	reg &= ~(1 << 17);
	writel(reg, DOVE_GLOBAL_CONFIG_1);
	dove_clocks_set_lcd_clock(0);
	return;
}

static int lcd_clk_is_enabled(struct clk_hw *hw)
{
	u32	reg;

	reg = readl(DOVE_GLOBAL_CONFIG_1);
	reg &= (1 << 17);

	return reg ? 1 : 0;
}

struct clk_ops lcd_clk_ops = {
	.enable		= lcd_clk_enable,
	.disable	= lcd_clk_disable,
	.is_enabled	= lcd_clk_is_enabled,
	.round_rate	= lcd_round_rate,
	.recalc_rate	= lcd_recalc_rate,
	.set_rate	= lcd_set_rate,
};

struct clk_ops accrt_lcd_clk_ops = {
	.enable		= lcd_clk_enable,
	.disable	= lcd_clk_disable,
	.is_enabled	= lcd_clk_is_enabled,
	.round_rate	= lcd_round_rate,
	.recalc_rate	= lcd_recalc_rate,
	.set_rate	= accrt_lcd_set_rate,
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
		printk(KERN_ERR "Unsupported AXI clock %lu\n", rate);

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
	struct clk *clk_gpu, *clk_axi, *clk_vmeta, *clk_lcd, *clk_accrt_lcd;

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
	clk_vmeta = dove_register_clk("vmeta", &gpu_clk_ops, "pll_clk");
	clk_lcd = dove_register_clk("lcd", &lcd_clk_ops, "pll_clk");
	clk_accrt_lcd = dove_register_clk("accurate_lcd",
					  &accrt_lcd_clk_ops, "pll_clk");

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
	orion_clkdev_add("LCDCLK", NULL, clk_lcd);
	orion_clkdev_add("accurate_LCDCLK", NULL, clk_accrt_lcd);
	orion_clkdev_add("VMETA_CLK", NULL, clk_vmeta);
}

