#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>

#include <soc/cache.h>
#include <soc/base.h>
#include <asm/io.h>
#include <soc/ddr.h>
#include <soc/base.h>
#include <soc/cpm.h>



#define SLEEP_MEMORY_START		0xb2400000
#define SLEEP_MEMORY_END		0xb2407ffc
#define SLEEP_RESUME_SP			SLEEP_MEMORY_END
#define SLEEP_RESUME_BOOTUP_TEXT	SLEEP_MEMORY_START

#define SLEEP_CPU_RESUME_BOOTUP_TEXT	SLEEP_RESUME_BOOTUP_TEXT
#define SLEEP_CPU_RESUME_BOOTUP_LEN	32 // 8 instructions
#define SLEEP_CPU_RESUME_TEXT		(SLEEP_CPU_RESUME_BOOTUP_TEXT + SLEEP_CPU_RESUME_BOOTUP_LEN)
#define SLEEP_CPU_RESUME_LEN		(1024)
#define SLEEP_CPU_SLEEP_TEXT		(SLEEP_CPU_RESUME_TEXT + SLEEP_CPU_RESUME_LEN)
#define SLEEP_CPU_SLEEP_LEN		(3072)
#define SLEEP_CPU_RESUME_SP		SLEEP_RESUME_SP


/*************************************** debug ***************************************/

#define PRINT_DEBUG

#ifdef PRINT_DEBUG

#define OFF_TDR         (0x00)
#define OFF_LCR         (0x0C)
#define OFF_LSR         (0x14)
#define LSR_TDRQ        (1 << 5)
#define LSR_TEMT        (1 << 6)

#define U_IOBASE (UART0_IOBASE + 0xa0000000)
#define TCSM_PCHAR(x)                                                   \
	*((volatile unsigned int*)(U_IOBASE+OFF_TDR)) = x;              \
while ((*((volatile unsigned int*)(U_IOBASE + OFF_LSR)) & (LSR_TDRQ | LSR_TEMT)) != (LSR_TDRQ | LSR_TEMT))
#else
#define TCSM_PCHAR(x)
#endif

#define TCSM_DELAY(x)                                           \
	do{                                                     \
		register unsigned int i = x;                    \
		while(i--)                                      \
		__asm__ volatile(".set mips32\n\t"      \
				"nop\n\t"              \
				".set mips32");        \
	}while(0)                                               \

static inline void serial_put_hex(unsigned int x) {
	int i;
	unsigned int d;
	for(i = 7;i >= 0;i--) {
		d = (x  >> (i * 4)) & 0xf;
		if(d < 10) d += '0';
		else d += 'A' - 10;
		TCSM_PCHAR(d);
	}
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
}

/*************************************************************************************/

/*-----------------------------------------------------------------------------
 *  extern function declare
 *-----------------------------------------------------------------------------*/
extern long long save_goto(unsigned int func, suspend_state_t state, unsigned int cpu_id);
extern int restore_goto(unsigned int func, suspend_state_t state, unsigned int cpu_id);


static void load_func_to_tcsm(unsigned int *tcsm_addr,unsigned int *f_addr,unsigned int size)
{
	unsigned int instr;
	int offset;
	int i;
#ifdef DEBUG_PM
	printk("tcsm addr = %p %p size = %d\n",tcsm_addr,f_addr,size);
#endif
	for(i = 0;i < size / 4;i++) {
		instr = f_addr[i];
		if((instr >> 26) == 2){
			offset = instr & 0x3ffffff;
			offset = (offset << 2) - ((unsigned int)f_addr & 0xfffffff);
			if(offset > 0) {
				offset = ((unsigned int)tcsm_addr & 0xfffffff) + offset;
				instr = (2 << 26) | (offset >> 2);
			}
		}
		tcsm_addr[i] = instr;
	}
}


static int soc_pm_idle(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &=~ 0x3; // low power mode: IDLE
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~(1 << 30);
	opcr &= ~(1 << 31);
	cpm_outl(opcr, CPM_OPCR);

	return 0;
}

#ifdef X2000_IDLE_PD
static int soc_pm_idle_pd(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &=~ 0x3; // low power mode: IDLE
	lcr |= 2;
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~ (1<<31);
	opcr |= (1 << 30);
	opcr |= (1 << 26);	//l2c retention
	opcr |= 1 << 2; // select RTC clk
	cpm_outl(opcr, CPM_OPCR);

	return 0;
}
#endif

static int soc_pm_sleep(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);
	unsigned int opcr = cpm_inl(CPM_OPCR);

	lcr &=~ 0x3;
	lcr |= 1 << 0; // low power mode: SLEEP
	cpm_outl(lcr, CPM_LCR);

	opcr &= ~(1 << 31);
	opcr |= (1 << 30);
	opcr |= (1 << 26); //L2C retention.
	opcr |= (1 << 21); // cpu 32k ram retention.
	opcr |= (1 << 3); // power down CPU
	opcr &= ~(1 << 4); // exclk disable;
	opcr |= (1 << 2); // select RTC clk
	cpm_outl(opcr, CPM_OPCR);

	{
		/* before power down cpu by set PD in OPCR, reduce cpu's frequency as the same as L2C's freq */
		unsigned int val, div;

		/* change disable */
		val = cpm_inl(CPM_CPCCR);
		val &= ~(1 << 22);
		cpm_outl(val, CPM_CPCCR);

		/* div cpu = div l2c */
		div = val & (0xf << 4);
		val &= ~0xf;
		val |= div;
		cpm_outl(val, CPM_CPCCR);

		/* change enable */
		val |= (1 << 22);
		cpm_outl(val, CPM_CPCCR);
	}

	return 0;
}

static int soc_post_wakeup(void)
{
	unsigned int lcr = cpm_inl(CPM_LCR);

	printk("post wakeup!\n");

	lcr &= ~0x3; // low power mode: IDLE
	cpm_outl(lcr, CPM_LCR);

	return 0;
}

static noinline void cpu_resume_bootup(void)
{
	/* set reset entry */
	*(volatile unsigned int *)0xb2200f00 = 0xbfc00000;

	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"move $29, %0	\n\t"
		"jr.hb   %1	\n\t"
		"nop		\n\t"
		".set pop	\n\t"
		:
		:"r" (SLEEP_CPU_RESUME_SP), "r"(SLEEP_CPU_RESUME_TEXT)
		:
		);
}

static noinline void cpu_resume(suspend_state_t state)
{
	unsigned int cpu_id;
	unsigned int ddrc_ctrl = ddr_readl(DDRC_CTRL);

	if (state == PM_SUSPEND_MEM) {
		ddrc_ctrl &= ~(1<<5);
		ddrc_ctrl |= 1 << 1;
		ddr_writel(ddrc_ctrl, DDRC_CTRL);

		while(ddr_readl(DDRC_STATUS) & (1<<2));
	}

	cpu_id = read_c0_ebase() & 0x3ff;

	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"jr.hb %0	\n\t"
		"nop		\n\t"
		".set pop 	\n\t"
		:
		: "r" (restore_goto),
		  "r" (state),
		  "r" (cpu_id)
		:
		);
}


static noinline void cpu_sleep(suspend_state_t state)
{

	unsigned int ddrc_ctrl = ddr_readl(DDRC_CTRL);

	blast_dcache32();
	blast_scache64();
	__sync();
	__fast_iob();

	if (state == PM_SUSPEND_MEM) {
		/* DDR self refresh, */
		ddrc_ctrl |= 1 << 5;
		ddrc_ctrl &= ~(1<<1);
		ddr_writel(ddrc_ctrl, DDRC_CTRL);
		while(!(ddr_readl(DDRC_STATUS) & (1<<2)));
	}

	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"wait		\n\t"
		"nop		\n\t"
		"nop		\n\t"
		"nop		\n\t"
		".set pop	\n\t"
	);

	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');
	TCSM_PCHAR('?');
	TCSM_PCHAR('?');
	TCSM_PCHAR('?');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	__asm__ volatile(
		".set push	\n\t"
		".set mips32r2	\n\t"
		"jr.hb %0	\n\t"
		"nop		\n\t"
		".set pop 	\n\t"
		:
		: "r" (SLEEP_CPU_RESUME_BOOTUP_TEXT)
		:
		);
}


int x2000_pm_enter(suspend_state_t state)
{
	unsigned int cpu_id;

	printk("x2000 pm enter!!\n");

	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_BOOTUP_TEXT, (unsigned int *)cpu_resume_bootup, SLEEP_CPU_RESUME_BOOTUP_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_RESUME_TEXT, (unsigned int *)cpu_resume, SLEEP_CPU_RESUME_LEN);
	load_func_to_tcsm((unsigned int *)SLEEP_CPU_SLEEP_TEXT, (unsigned int *)cpu_sleep, SLEEP_CPU_SLEEP_LEN);

	if (state == PM_SUSPEND_STANDBY) {
#ifdef X2000_IDLE_PD
		soc_pm_idle_pd();
#else
		soc_pm_idle();
#endif
	} else if (state == PM_SUSPEND_MEM) {
		soc_pm_sleep();
	} else {
		printk("WARNING : unsupport pm suspend state\n");
	}

#ifdef DEBUG_PM
	printk("LCR: %08x\n", cpm_inl(CPM_LCR));
	printk("OPCR: %08x\n", cpm_inl(CPM_OPCR));
#endif

	cpu_id = read_c0_ebase() & 0x1ff;

	/* set reset entry */
	*(volatile unsigned int *)0xb2200f00 = SLEEP_CPU_RESUME_BOOTUP_TEXT;

	mb();
	save_goto((unsigned int)SLEEP_CPU_SLEEP_TEXT, state, cpu_id);
	mb();

	soc_post_wakeup();

	return 0;
}

static int x2000_pm_begin(suspend_state_t state)
{
	printk("x2000 suspend begin\n");
	return 0;
}

static void x2000_pm_end(void)
{
	printk("x2000 pm end!\n");
}

static int ingenic_pm_valid(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;

		default:
			return 0;
	}
}
static const struct platform_suspend_ops x2000_pm_ops = {
	.valid		= ingenic_pm_valid,
	.begin		= x2000_pm_begin,
	.enter		= x2000_pm_enter,
	.end		= x2000_pm_end,
};

/*
 * Initialize suspend interface
 */
static int __init pm_init(void)
{

	suspend_set_ops(&x2000_pm_ops);


	return 0;
}

late_initcall(pm_init);
