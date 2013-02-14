#include "inc/LPC11xx.h"
#include "hdr/hdr_uart.h"

#include "uart.h"

char* itoa(uint16_t number, uint8_t base) {
    if(!number) {
        return "0";
    }
    static char buf[16] = {0};
        register char i = 14;
        char m = 0;
        if(number < 0) {
                number = number * (-1);
                m = 1;
        }
        for(; number && i ; --i, number /= base)
                buf[i] = "0123456789ABCDEF"[number % base];
        if(m) {
                buf[i] = '-';
                return &buf[i];
        } else {
                return &buf[i+1];
        }
}

void
putChar(char c)
{
	while(!(LPC_UART->LSR & UART_LSR_THRE));

	LPC_UART->THR = c;
}

void
putHex8(uint8_t v)
{
        char *buf;
        buf = itoa(v, 16);
        printf(buf);
}

void
printf(const char *str)
{
	while(*str != '\0') {
		putChar(*str);
		str++;
	}
}
