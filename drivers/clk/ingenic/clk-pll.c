#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clkdev.h>
#include "clk.h"
#include "clk-pll.h"

#define PLL_TIMEOUT_MS		10

struct ingenic_clk_pll {
	struct clk_hw		hw;
	void __iomem		*lock_reg;
	void __iomem		*con_reg;
	unsigned int		rate_count;

	struct ingenic_pll_hwdesc *hwdesc;
	struct ingenic_pll_rate_table *rate_table;

};

#define to_clk_pll(_hw) container_of(_hw, struct ingenic_clk_pll, hw)

static long ingenic_pll_round_rate(struct clk_hw *hw, unsigned long drate, unsigned long *prate)
{
	struct ingenic_clk_pll *pll = to_clk_pll(hw);
	const struct ingenic_pll_rate_table *rate_table = pll->rate_table;
	int i;
	unsigned int rate;

    /* if pll can't find rate in rate_table, pll can set to big than rate */
	for (i = 0; i < pll->rate_count; i++) {
		if (drate == rate_table[i].rate)
			return  rate_table[i].rate;
	}

	for (i = 0; i < pll->rate_count; i++) {
		if (drate >= rate_table[i].rate)
			return  rate_table[i].rate;
	}

    return rate_table[i - 1].rate;

}

static unsigned long ingenic_tx_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct ingenic_clk_pll *pll = to_clk_pll(hw);
	unsigned od0, m, n, od1;
	uint32_t val;
	uint32_t rate;

	val = readl(pll->con_reg);

	od0 = (val >> pll->hwdesc->od0_sft) & GENMASK(pll->hwdesc->od0_width - 1, 0);
	od1 = (val >> pll->hwdesc->od1_sft) & GENMASK(pll->hwdesc->od1_width - 1, 0);
	n = (val >> pll->hwdesc->n_sft) & GENMASK(pll->hwdesc->n_width - 1, 0);
	m = (val >> pll->hwdesc->m_sft) & GENMASK(pll->hwdesc->m_width - 1, 0);

	rate = (parent_rate / 1000000 ) * m / n / od0 / od1;

	//printk("od0: %d, od1: %d, m: %d, n: %d, tmp: %x, tmp2: %x, rate:%ld\n", od0, od1, m, n, tmp, tmp2, rate);
    rate = rate*1000000;

	return rate;
}

static int wait_pll_stable(void __iomem *reg, u32 shift)
{
	unsigned int timeout = 0xffff;

	while ((!((readl(reg) >> shift) & 1)) && timeout--);
	if (!timeout) {
		printk("WARNING : why cannot wait pll stable ???\n");
		return -EIO;
	} else {
		return 0;
	}
}

static int ingenic_tx_pll_set_rate(struct clk_hw *hw, unsigned long drate, unsigned long prate)
{
	struct ingenic_clk_pll *pll = to_clk_pll(hw);

	unsigned int m, n, od1, od0;
	unsigned int val, i;

	for (i = 0; i < pll->rate_count; i++) {
		if (drate == pll->rate_table[i].rate) {
			m = pll->rate_table[i].m;
			n = pll->rate_table[i].n;
			od1 = pll->rate_table[i].od1;
			od0 = pll->rate_table[i].od0;
		}
	}

	writel(0, pll->con_reg);
	val = (m << pll->hwdesc->m_sft) | (n << pll->hwdesc->n_sft) | (od1 << pll->hwdesc->od1_sft) | (od0 << pll->hwdesc->od0_sft) | 1;
	writel(val, pll->con_reg);

	wait_pll_stable(pll->con_reg, pll->hwdesc->on_bit);

	return 0;
}

static const struct clk_ops ingenic_tx_pll_clk_ops = {
	.recalc_rate = ingenic_tx_pll_recalc_rate,
	.round_rate = ingenic_pll_round_rate,
	.set_rate = ingenic_tx_pll_set_rate,
};

static void __init _ingenic_clk_register_pll(struct ingenic_clk_provider *ctx,
				const struct ingenic_pll_clock *pll_clk,
				void __iomem *base)
{
	struct ingenic_clk_pll *pll;
	struct clk *clk;
	struct clk_init_data init;
	int ret, len;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll) {
		pr_err("%s: could not allocate pll clk %s\n",
			__func__, pll_clk->dev_name);
		return;
	}

	init.name = pll_clk->dev_name;
//	init.flags = pll_clk->flags;
	init.parent_names = &pll_clk->parent_name;
	init.num_parents = 1;
	init.ops = &ingenic_tx_pll_clk_ops;


	pll->hw.init = &init;
	pll->hwdesc = pll_clk->hwdesc;
	pll->con_reg = base + pll_clk->hwdesc->regoff;

	if (pll_clk->rate_table) {
		/* find count of rates in rate_table */
		for (len = 0; pll_clk->rate_table[len].rate != 0; )
			len++;

		pll->rate_count = len - 1;
		pll->rate_table = kmemdup(pll_clk->rate_table,
				pll->rate_count *
				sizeof(struct ingenic_pll_rate_table),
				GFP_KERNEL);
		WARN(!pll->rate_table,
				"%s: could not allocate rate table for %s\n",
				__func__, pll_clk->name);
	}

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register pll clock %s : %ld\n",
			__func__, pll_clk->dev_name, PTR_ERR(clk));
		kfree(pll);
		return;
	}

    __clk_set_flags(clk, 1);
	ingenic_clk_add_lookup(ctx, clk, pll_clk->id);


	ret = clk_register_clkdev(clk, pll_clk->dev_name, NULL);
	if (ret)
		pr_err("%s: failed to register lookup for %s : %d",
			__func__, pll_clk->dev_name, ret);
}

void __init ingenic_clk_register_pll(struct ingenic_clk_provider *ctx,
			const struct ingenic_pll_clock *pll_list,
			unsigned int nr_pll, void __iomem *base)
{
	int cnt;

	for (cnt = 0; cnt < nr_pll; cnt++)
		_ingenic_clk_register_pll(ctx, &pll_list[cnt], base);
}
