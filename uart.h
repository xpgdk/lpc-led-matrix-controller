#ifndef _UART_H
#define _UART_H

#include <stdint.h>

void putChar(char c);
void printf(const char *str);
void putHex8(uint8_t v);
void putHex16(uint16_t v);
void putHex32(uint32_t v);

#endif
