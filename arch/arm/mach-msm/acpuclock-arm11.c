/* arch/arm/mach-msm/acpuclock.c
 *
 * MSM architecture clock driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/clk.h>

#include "proc_comm.h"
#include "acpuclock.h"
#include <asm/mach-types.h>

#define A11S_CLK_CNTL_ADDR (MSM_CSR_BASE + 0x100)
#define A11S_CLK_SEL_ADDR (MSM_CSR_BASE + 0x104)
#define A11S_VDD_SVS_PLEVEL_ADDR (MSM_CSR_BASE + 0x124)
#define A11S_CLK_CTL_PLL2 (MSM_CLK_CTL_BASE + 0x33C)

/*
 * ARM11 clock configuration for specific ACPU speeds
 */

#define ACPU_PLL_TCXO	-1
#define ACPU_PLL_0	0
#define ACPU_PLL_1	1
#define ACPU_PLL_2	2
#define ACPU_PLL_3	3

struct clock_state
{
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
	unsigned long			power_collapse_khz;
	unsigned long			wait_for_irq_khz;
};

static struct clk *ebi1_clk;
static struct clock_state drv_state = { 0 };

static void __init acpuclk_init(void);

/* MSM7201A Levels 3-6 all correspond to 1.2V, level 7 corresponds to 1.325V. */
enum {
	VDD_0 = 0,
	VDD_1 = 1,
	VDD_2 = 2,
	VDD_3 = 3,
	VDD_4 = 4,
	VDD_5 = 5,
	VDD_6 = 6,
	VDD_7 = 7,
	VDD_END
};

static const int vdd_mv_table[VDD_END] = {
	[VDD_0] = 1000,
	[VDD_1] = 1050,
	[VDD_2] = 1100,
	[VDD_3] = 1150,
	[VDD_4] = 1200,
	[VDD_5] = 1225,
	[VDD_6] = 1300,
	[VDD_7] = 1350,
};

/*
 * Compile-time macro to specify custom voltages in millivolts directly
 * in the frequency table (e.g. VDD_MV(1250)).
 */
#define VDD_MV(mv) \
	((mv) <= 1025 ? VDD_0 : \
	 (mv) <= 1075 ? VDD_1 : \
	 (mv) <= 1125 ? VDD_2 : \
	 (mv) <= 1175 ? VDD_3 : \
	 (mv) <= 1212 ? VDD_4 : \
	 (mv) <= 1237 ? VDD_5 : \
	 (mv) <= 1287 ? VDD_6 : VDD_7)

static int vdd_to_mv(int vdd)
{
	if (vdd < 0)
		return vdd_mv_table[0];
	if (vdd >= VDD_END)
		return vdd_mv_table[VDD_END - 1];
	return vdd_mv_table[vdd];
}

static int mv_to_vdd(int mv)
{
	int i;
	if (mv <= vdd_mv_table[0])
		return VDD_0;
	if (mv >= vdd_mv_table[VDD_END - 1])
		return VDD_END - 1;

	for (i = 0; i < VDD_END - 1; i++) {
		if (mv <= (vdd_mv_table[i] + vdd_mv_table[i + 1]) / 2)
			return i;
	}
	return VDD_END - 1;
}

struct clkctl_acpu_speed {
	unsigned int	a11clk_khz;
	int		pll;
	unsigned int	a11clk_src_sel;
	unsigned int	a11clk_src_div;
	unsigned int	ahbclk_khz;
	unsigned int	ahbclk_div;
	int		vdd;
	unsigned int	axiclk_khz;
	unsigned long	lpj; /* loops_per_jiffy */
/* Index in acpu_freq_tbl[] for steppings. */
	short		down;
	short		up;
	short		pll2_lval;
};

/*
 * ACPU speed table. Complete table is shown but certain speeds are commented
 * out to optimized speed switching. Initalize loops_per_jiffy to 0.
 *
 * Table stepping up/down is optimized for 256mhz jumps while staying on the
 * same PLL.
 */
static struct clkctl_acpu_speed *acpu_freq_tbl;

/* 7200a turbo mode, PLL0(mpll):245.76, PLL1(gpll):960, PLL2(bpll0):1056 */
static struct clkctl_acpu_speed  msm72xx_tbl[] = {
#if defined(CONFIG_TURBO_MODE)
/* 0*/	{ 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, VDD_MV(1000), 30720, 0, 0, 3 },
/* 1*/	{ 122880, ACPU_PLL_0, 4, 1, 61440, 1, VDD_MV(1050), 61440, 0, 0, 4 },
/* 2*/	{ 160000, ACPU_PLL_1, 1, 5, 53333, 2, VDD_MV(1050), 61440, 0, 0, 6 },
/* 3*/	{ 176000, ACPU_PLL_2, 2, 5, 88000, 1, VDD_MV(1050), 61440, 0, 0, 6 },
/* 4*/	{ 245760, ACPU_PLL_0, 4, 0, 81920, 2, VDD_MV(1100), 61440, 0, 0, 6 },
/* 5*/	{ 264000, ACPU_PLL_2, 2, 3, 88000, 2, VDD_MV(1100), 128000, 0, 0, 8 },
/* 6*/	{ 352000, ACPU_PLL_2, 2, 2, 117333, 2, VDD_MV(1150), 128000, 0, 3, 13 },
/* 7*/	{ 480000, ACPU_PLL_1, 1, 1, 120000, 3, VDD_MV(1200), 128000, 0, 4, 20 },
/* 8*/	{ 518400, ACPU_PLL_2, 2, 1, 129600, 3, VDD_MV(1225), 128000, 0, 5, -1, 0x1b },
/* 9*/	{ 528000, ACPU_PLL_2, 2, 1, 132000, 3, VDD_MV(1225), 128000, 0, 5, -1 },
/*10*/	{ 537600, ACPU_PLL_2, 2, 1, 134400, 3, VDD_MV(1250), 160000, 0, 6, -1, 0x1c },
/*11*/	{ 556800, ACPU_PLL_2, 2, 1, 139200, 3, VDD_MV(1250), 160000, 0, 6, -1, 0x1d },
/*12*/	{ 576000, ACPU_PLL_2, 2, 1, 144000, 3, VDD_MV(1250), 160000, 0, 6, -1, 0x1e },
/*13*/	{ 595200, ACPU_PLL_2, 2, 1, 148800, 3, VDD_MV(1250), 160000, 0, 6, -1, 0x1f },
/*14*/	{ 614400, ACPU_PLL_2, 2, 1, 153600, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x20 },
/*15*/	{ 633600, ACPU_PLL_2, 2, 1, 158400, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x21 },
/*16*/	{ 652800, ACPU_PLL_2, 2, 1, 163200, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x22 },
/*17*/	{ 672000, ACPU_PLL_2, 2, 1, 168000, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x23 },
/*18*/	{ 691200, ACPU_PLL_2, 2, 1, 172800, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x24 },
/*19*/	{ 710400, ACPU_PLL_2, 2, 1, 177600, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x25 },
/*20*/	{ 729600, ACPU_PLL_2, 2, 1, 182400, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x26 },
/*21*/	{ 748800, ACPU_PLL_2, 2, 1, 187200, 3, VDD_MV(1250), 160000, 0, 8, -1, 0x27 },
/*22*/	{ 768000, ACPU_PLL_2, 2, 1, 192000, 3, VDD_MV(1275), 200000, 0, 8, -1, 0x28 },
/*23*/	{ 787200, ACPU_PLL_2, 2, 1, 196800, 3, VDD_MV(1350), 200000, 0, 10, -1, 0x29 },
/*24*/	{ 806400, ACPU_PLL_2, 2, 1, 201600, 3, VDD_MV(1350), 200000, 0, 11, -1, 0x2a },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
#else
	{ 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, VDD_0, 30720, 0, 0, 4 },
	{ 122880, ACPU_PLL_0, 4, 1, 61440, 1, VDD_1, 61440, 0, 0, 4 },
	{ 128000, ACPU_PLL_1, 1, 5, 64000, 1, VDD_1, 61440, 0, 0, 6 },
	{ 176000, ACPU_PLL_2, 2, 5, 88000, 1, VDD_1, 61440, 0, 0, 5 },
	{ 245760, ACPU_PLL_0, 4, 0, 81920, 2, VDD_2, 61440, 0, 0, 5 },
	{ 352000, ACPU_PLL_2, 2, 2, 88000, 3, VDD_3, 128000, 0, 3, 7 },
	{ 384000, ACPU_PLL_1, 1, 1, 128000, 2, VDD_4, 128000, 0, 2, -1 },
	{ 528000, ACPU_PLL_2, 2, 1, 132000, 3, VDD_6, 128000, 0, 5, -1 },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
#endif
};

static unsigned long max_axi_rate;

#ifdef CONFIG_CPU_FREQ
static struct cpufreq_frequency_table *freq_table;

static struct cpufreq_frequency_table msm72xx_freq_table[] = {
#if defined(CONFIG_TURBO_MODE)
	{ 0, 176000 },
	{ 1, 264000 },
	{ 2, 352000 },
	{ 3, 518400 },
	{ 4, 576000 },
	{ 5, 614400 },
	{ 6, 633600 },
	{ 7, 652800 },
	{ 8, 672000 },
	{ 9, 691200 },
	{10, 710400 },
	{11, 729600 },
	{12, 748800 },
	{13, 768000 },
	{14, 787200 },
	{15, 806400 },
	{16, CPUFREQ_TABLE_END },
#else
	{ 0, 19200 },
	{ 1, 122880 },
	{ 2, 128000 },
	{ 3, 245760 },
	{ 4, 384000 },
	{ 5, 528000 },
	{ 6, CPUFREQ_TABLE_END },
#endif
};

#endif

#define TABLE_CONFIG(m0, m1) { \
	MACH_TYPE_##m0, \
	msm##m1##_tbl, \
	msm##m1##_freq_table \
}

struct freq_tbl_map {
	unsigned int machine;
	struct clkctl_acpu_speed *tbl;
	struct cpufreq_frequency_table *freq_tbl;
};

static struct freq_tbl_map acpu_freq_tbl_list[] = {
	TABLE_CONFIG(BUZZ, 72xx),
	TABLE_CONFIG(BEE, 72xx),
	TABLE_CONFIG(BUZZC, 72xx),
	{ 0, 0, 0}
};

static int pc_pll_request(unsigned id, unsigned on)
{
	on = !!on;
	return msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
}


/*----------------------------------------------------------------------------
 * ARM11 'owned' clock control
 *---------------------------------------------------------------------------*/
module_param_call(pwrc_khz, param_set_int, param_get_int,
		&drv_state.power_collapse_khz, S_IWUSR | S_IRUGO);
module_param_call(wfi_khz, param_set_int, param_get_int,
		&drv_state.wait_for_irq_khz, S_IWUSR | S_IRUGO);

unsigned long acpuclk_power_collapse(int from_idle)
{
	int ret = acpuclk_get_rate();
	ret *= 1000;
	if (ret > drv_state.power_collapse_khz) {
		if (from_idle)
			acpuclk_set_rate(drv_state.power_collapse_khz * 1000,
					SETRATE_PC_IDLE);
		else
			acpuclk_set_rate(drv_state.power_collapse_khz * 1000,
					SETRATE_PC);
	}
	return ret;
}

unsigned long acpuclk_get_wfi_rate(void)
{
	return drv_state.wait_for_irq_khz;
}

unsigned long acpuclk_wait_for_irq(void)
{
	int ret = acpuclk_get_rate();
	ret *= 1000;
	if (ret > drv_state.wait_for_irq_khz)
		acpuclk_set_rate(drv_state.wait_for_irq_khz * 1000,
				SETRATE_SWFI);
	return ret;
}

static int acpuclk_set_vdd_level(int vdd)
{
	writel((1 << 7) | (vdd << 3), A11S_VDD_SVS_PLEVEL_ADDR);
	udelay(drv_state.vdd_switch_time_us);
	if ((readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x7) != vdd) {
		printk(KERN_ERR "acpuclock: VDD set failed\n");
		return -EIO;
	}

	return 0;
}

/* Set proper dividers for the given clock speed. */
static void acpuclk_set_div(const struct clkctl_acpu_speed *hunt_s) {
	uint32_t reg_clkctl, reg_clksel, clk_div, a11_div;

	/* AHB_CLK_DIV */
	clk_div = (readl(A11S_CLK_SEL_ADDR) >> 1) & 0x03;

	/* OC BEGIN */
	a11_div = hunt_s->a11clk_src_div;

	if (hunt_s->a11clk_khz >= 518400 && hunt_s->pll2_lval > 0) {
		a11_div = 0;
		if (readl(A11S_CLK_CTL_PLL2) != hunt_s->pll2_lval) {
			writel(hunt_s->pll2_lval, A11S_CLK_CTL_PLL2);
			udelay(20);
		}
	}
	/* OC END */

	/*
	 * If the new clock divider is higher than the previous, then
	 * program the divider before switching the clock
	 */
	if (hunt_s->ahbclk_div > clk_div) {
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}
	if ((readl(A11S_CLK_SEL_ADDR) & 0x01) == 0) {
		/* SRC0 */

		/* Program clock source */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~(0x07 << 4);
		reg_clkctl |= (hunt_s->a11clk_src_sel << 4);
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock divider */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~0xf;
		reg_clkctl |= a11_div;
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock source selection */
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel |= 1; /* CLK_SEL_SRC1NO  == SRC1 */
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	} else {
		/* SRC1 */

		/* Program clock source */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~(0x07 << 12);
		reg_clkctl |= (hunt_s->a11clk_src_sel << 12);
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock divider */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
		reg_clkctl &= ~(0xf << 8);
		reg_clkctl |= (a11_div << 8);
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock source selection */
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel &= ~1; /* CLK_SEL_SRC1NO  == SRC0 */
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}

	/*
	 * If the new clock divider is lower than the previous, then
	 * program the divider after switching the clock
	 */
	if (hunt_s->ahbclk_div < clk_div) {
		reg_clksel = readl(A11S_CLK_SEL_ADDR);
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}
	/*
	 * "Clear" PLL2
	 */
	if (hunt_s->a11clk_khz < 518400 || hunt_s->pll2_lval <= 0){
		/* Check we are not already "cleared" */
		if (readl(A11S_CLK_CTL_PLL2) != 0x37){
			writel(0x37, A11S_CLK_CTL_PLL2);
			udelay(20);
		}
	}
}

int acpuclk_set_rate(unsigned long rate, enum setrate_reason reason)
{
	uint32_t reg_clkctl;
	struct clkctl_acpu_speed *cur_s, *tgt_s, *strt_s;
	int rc = 0;
	unsigned int plls_enabled = 0, pll;

	strt_s = cur_s = drv_state.current_speed;

	WARN_ONCE(cur_s == NULL, "acpuclk_set_rate: not initialized\n");
	if (cur_s == NULL)
		return -ENOENT;

	if (rate == (cur_s->a11clk_khz * 1000))
		return 0;

	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++) {
		if (tgt_s->a11clk_khz == (rate / 1000))
			break;
	}

	if (tgt_s->a11clk_khz == 0)
		return -EINVAL;

	/* Choose the highest speed speed at or below 'rate' with same PLL. */
	if (reason != SETRATE_CPUFREQ
	    && tgt_s->a11clk_khz < cur_s->a11clk_khz) {
		while (tgt_s->pll != ACPU_PLL_TCXO && tgt_s->pll != cur_s->pll)
			tgt_s--;
	}

	if (strt_s->pll != ACPU_PLL_TCXO)
		plls_enabled |= 1 << strt_s->pll;

	if (reason == SETRATE_CPUFREQ) {
		mutex_lock(&drv_state.lock);
		if (strt_s->pll != tgt_s->pll && tgt_s->pll != ACPU_PLL_TCXO) {
			rc = pc_pll_request(tgt_s->pll, 1);
			if (rc < 0) {
				pr_err("PLL%d enable failed (%d)\n",
					tgt_s->pll, rc);
				goto out;
			}
			plls_enabled |= 1 << tgt_s->pll;
		}
		/* Increase VDD if needed. */
		if (tgt_s->vdd > cur_s->vdd) {
			if ((rc = acpuclk_set_vdd_level(tgt_s->vdd)) < 0) {
				printk(KERN_ERR "Unable to switch ACPU vdd\n");
				goto out;
			}
		}
	} else {
		/* Power collapse should also increase VDD. */
		if (tgt_s->vdd > cur_s->vdd) {
			if ((rc = acpuclk_set_vdd_level(tgt_s->vdd)) < 0) {
				printk(KERN_ERR "Unable to switch ACPU vdd\n");
				goto out;
			}
		}
	}

	/* Set wait states for CPU inbetween frequency changes */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl &= ~(0xff << 16);
	reg_clkctl |= (32 << 16); /* set WT_ST_CNT */
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

	/* Direct frequency jump without intermediate stepping */
	if (tgt_s->pll != ACPU_PLL_TCXO && !(plls_enabled & (1 << tgt_s->pll))) {
		rc = pc_pll_request(tgt_s->pll, 1);
		if (rc < 0) {
			pr_err("PLL%d enable failed (%d)\n", tgt_s->pll, rc);
			goto out;
		}
		plls_enabled |= 1 << tgt_s->pll;
	}

	acpuclk_set_div(tgt_s);
	drv_state.current_speed = tgt_s;
	loops_per_jiffy = tgt_s->lpj;
	udelay(drv_state.acpu_switch_time_us);

	/* Nothing else to do for power collapse. */
	if (reason == SETRATE_SWFI || reason == SETRATE_PC_IDLE)
		return 0;

	/* Change the AXI bus frequency if we can. */
	if (strt_s->axiclk_khz != tgt_s->axiclk_khz) {
		rc = clk_set_rate(ebi1_clk, tgt_s->axiclk_khz * 1000);
		if (rc < 0)
			pr_err("Setting AXI min rate failed!\n");
	}

	if (reason == SETRATE_PC)
		return 0;

	/* Disable PLLs we are not using anymore. */
	plls_enabled &= ~(1 << tgt_s->pll);
	for (pll = ACPU_PLL_0; pll <= ACPU_PLL_2; pll++)
		if (plls_enabled & (1 << pll)) {
			rc = pc_pll_request(pll, 0);
			if (rc < 0) {
				pr_err("PLL%d disable failed (%d)\n", pll, rc);
				goto out;
			}
		}

	if (reason == SETRATE_PC)
		return 0;

	/* Drop VDD level if we can. */
	if (tgt_s->vdd < strt_s->vdd) {
		if (acpuclk_set_vdd_level(tgt_s->vdd) < 0)
			printk(KERN_ERR "acpuclock: Unable to drop ACPU vdd\n");
	}

out:
	if (reason == SETRATE_CPUFREQ)
		mutex_unlock(&drv_state.lock);
	return rc;
}

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *speed, *max_s;
	uint32_t div, sel;
	int rc;

	/*
	 * Determine the rate of ACPU clock
	 */

	if (!(readl(A11S_CLK_SEL_ADDR) & 0x01)) { /* CLK_SEL_SRC1N0 */
		/* CLK_SRC0_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 12) & 0x7;
		/* CLK_SRC0_DIV */
		div = (readl(A11S_CLK_CNTL_ADDR) >> 8) & 0x0f;
	} else {
		/* CLK_SRC1_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 4) & 0x07;
		/* CLK_SRC1_DIV */
		div = readl(A11S_CLK_CNTL_ADDR) & 0x0f;
	}

	for (speed = acpu_freq_tbl; speed->a11clk_khz != 0; speed++) {
		if (speed->a11clk_src_sel == sel
		 && (speed->a11clk_src_div == div))
			break;
	}
	if (speed->a11clk_khz == 0) {
		printk(KERN_WARNING "Warning - ACPU clock reports invalid speed\n");
		return;
	}

	drv_state.current_speed = speed;

	rc = clk_set_rate(ebi1_clk, speed->axiclk_khz * 1000);
	if (rc < 0)
		pr_err("Setting AXI min rate failed!\n");

	max_s = speed - 1;
	max_axi_rate = max_s->axiclk_khz * 1000;

	printk(KERN_INFO "ACPU running at %d KHz\n", speed->a11clk_khz);
}

unsigned long acpuclk_get_max_axi_rate(void)
{
	return max_axi_rate;
}
EXPORT_SYMBOL(acpuclk_get_max_axi_rate);

unsigned long acpuclk_get_rate(void)
{
	WARN_ONCE(drv_state.current_speed == NULL,
		  "acpuclk_get_rate: not initialized\n");
	if (drv_state.current_speed)
		return drv_state.current_speed->a11clk_khz;
	else
		return 0;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

static ssize_t acpuclk_tbl_show(struct device *dev,
		struct device_attribute *attr, char *buffer)
{
	char buf[16];
	ssize_t result = 0;
	int i = 0;

	if (freq_table == NULL) {
		pr_info("No frequency table!\n");
		return 0;
	}

	while (freq_table[i].frequency != CPUFREQ_TABLE_END) {
		if (freq_table[i].frequency >=
		    CONFIG_MSM_CPU_FREQ_ONDEMAND_MIN) {
			memset(buf, 0, 16);
			result += sprintf(buf, "%u,", freq_table[i].frequency);
			strcat(buffer, buf);
		}

		i++;
	}

	buffer[result-1] = '\n';
	return result;
}
static ssize_t vdd_levels_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t len = 0;

	if (!acpu_freq_tbl)
		return 0;

	mutex_lock(&drv_state.lock);
	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%u: %u\n",
				 acpu_freq_tbl[i].a11clk_khz,
				 vdd_to_mv(acpu_freq_tbl[i].vdd));
	}
	mutex_unlock(&drv_state.lock);

	return len;
}

static ssize_t vdd_levels_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int khz = 0;
	unsigned int mv = 0;
	int matched;
	int i, new_vdd;
	bool found = false;

	/*
	 * Accept format: "<khz>: <mv>" or "<khz> <mv>"
	 * If only "<mv>" is provided, apply to current running frequency.
	 */
	matched = sscanf(buf, "%u: %u", &khz, &mv);
	if (matched != 2)
		matched = sscanf(buf, "%u %u", &khz, &mv);

	if (matched == 1) {
		/* Single argument: treat as mV for current frequency */
		mv = khz;
		khz = drv_state.current_speed ? drv_state.current_speed->a11clk_khz : 0;
	} else if (matched != 2) {
		return -EINVAL;
	}

	if (!khz || !mv || !acpu_freq_tbl)
		return -EINVAL;

	new_vdd = mv_to_vdd(mv);

	mutex_lock(&drv_state.lock);
	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {
		if (acpu_freq_tbl[i].a11clk_khz == khz) {
			acpu_freq_tbl[i].vdd = new_vdd;
			found = true;

			/* If updating currently active frequency, apply immediately */
			if (drv_state.current_speed &&
			    drv_state.current_speed->a11clk_khz == khz) {
				acpuclk_set_vdd_level(new_vdd);
			}
			break;
		}
	}
	mutex_unlock(&drv_state.lock);

	return found ? count : -EINVAL;
}

/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/
static DEVICE_ATTR(acpuclk_tbl, 0444, acpuclk_tbl_show, NULL);
static DEVICE_ATTR(vdd_levels, 0644, vdd_levels_show, vdd_levels_store);

static struct kobject *acpuclock_kobj;

static void __init acpu_freq_tbl_fixup(void)
{
	int ret;
	struct freq_tbl_map *lst;

	/* Select the right table to use. */
	for (lst = acpu_freq_tbl_list; lst->tbl != 0; lst++) {
		if (lst->machine == machine_arch_type) {
			acpu_freq_tbl = lst->tbl;
			freq_table = lst->freq_tbl;
			break;
		}
	}

	if (acpu_freq_tbl == NULL) {
		pr_info("Unknown frequency table!\n");
		return;
	}

	acpuclock_kobj = kobject_create_and_add("acpuclock", NULL);
	if (acpuclock_kobj == NULL) {
		pr_info("msm_acpu_clock_init: subsystem_register failed\n");
		return;
	}

	ret = sysfs_create_file(acpuclock_kobj, &dev_attr_acpuclk_tbl.attr);
	if (ret)
		pr_info("msm_acpu_clock_init: sysfs_create_file acpuclk_tbl failed\n");

	ret = sysfs_create_file(acpuclock_kobj, &dev_attr_vdd_levels.attr);
	if (ret)
		pr_info("msm_acpu_clock_init: sysfs_create_file vdd_levels failed\n");
}

/* Initalize the lpj field in the acpu_freq_tbl. */
static void __init lpj_init(void)
{
	int i;
	const struct clkctl_acpu_speed *base_clk = drv_state.current_speed;
	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {
		acpu_freq_tbl[i].lpj = cpufreq_scale(loops_per_jiffy,
						base_clk->a11clk_khz,
						acpu_freq_tbl[i].a11clk_khz);
	}
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	pr_info("acpu_clock_init()\n");

	ebi1_clk = clk_get(NULL, "ebi1_clk");

	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = min(clkdata->acpu_switch_time_us, 20U);
	drv_state.max_speed_delta_khz = clkdata->max_speed_delta_khz;
	drv_state.vdd_switch_time_us = min(clkdata->vdd_switch_time_us, 20U);
	drv_state.power_collapse_khz = clkdata->power_collapse_khz;
	drv_state.wait_for_irq_khz = clkdata->wait_for_irq_khz;
	acpu_freq_tbl_fixup();
	acpuclk_init();
	lpj_init();
#ifdef CONFIG_CPU_FREQ
	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());
#endif
}
