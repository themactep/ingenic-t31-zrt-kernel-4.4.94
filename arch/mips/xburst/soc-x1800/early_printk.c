/*
 * INGENIC SOC serial routines for early_printk.
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 *
 */
#include <linux/moduleparam.h>
#include <linux/serial_reg.h>
#include <asm/setup.h>
#include <asm/io.h>
#include <soc/base.h>

#define UART_OFF	(0x1000)

static void check_uart(char c);

static volatile u32 *uart_base;
typedef void (*putchar_f_t)(char);

static putchar_f_t putchar_f = check_uart;

static void putchar(char ch)
{
	int timeout = 10000;
	volatile u32 *base = uart_base;

	/* Wait for fifo to shift out some bytes */
	while ((base[UART_LSR] & (UART_LSR_THRE | UART_LSR_TEMT))
	       != (UART_LSR_THRE | UART_LSR_TEMT) && timeout--)
		;
	base[UART_TX] = (u8)ch;
}

static void putchar_dummy(char ch)
{
	return;
}

static void check_uart(char c)
{
	/* We Couldn't use ioremap() here */
	volatile u32 *base = (volatile u32*)CKSEG1ADDR(UART0_IOBASE);
	int i;
	for(i=0; i < 3; i++) {
		if(base[UART_LCR])
			break;
		base += (UART_OFF/sizeof(u32));
	}

	if(i < 3) {
		uart_base = base;
		putchar_f = putchar;
		putchar_f(c);
	} else {
		putchar_f = putchar_dummy;
	}
}

/* used by early printk */
void prom_putchar(char c)
{
	putchar_f(c);
}

static int __init early_parse_console(char *p)
{
	unsigned char *param = "null";

	if(strstr(p, param)){
		putchar_f = putchar_dummy;
	}
	return 0;
}
early_param("console", early_parse_console);

#if 0
void prom_putstr(char *s)
{
        while(*s) {
                if(*s == '\n')
                        putchar_f('\r');
                putchar_f(*s);
                s++;
        }
}

static char pbuffer[4096];
void prom_printk(const char *fmt, ...) {
        va_list args;

        va_start(args, fmt);
        vsnprintf(pbuffer, 4096, fmt, args);
        va_end(args);

        prom_putstr(pbuffer);
}
#endif

