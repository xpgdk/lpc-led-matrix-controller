#ifndef _LED_MATRIX_CONFIG_H
#define _LED_MATRIX_CONFIG_H

#include "inc/LPC11xx.h"

#undef OLIMEX

#ifdef OLIMEX
// Olimex PIN 20
#define CLK_OUT_PORT		LPC_GPIO2
#define CLK_OUT_PIN		(1 << 5)

// Olimex PIN 19
#define LATCH_PORT		LPC_GPIO2
#define LATCH_PIN		(1 << 4)

// Olimex PIN 21
#define SER_OUT_PORT		LPC_GPIO3
#define SER_OUT_PIN		(1 << 5)

// Olimex PIN 22
#define ROW_SER_OUT_PORT	LPC_GPIO0
#define ROW_SER_OUT_PIN		(1 << 6)

// Olimex PIN 23
#define ROW_CLK_OUT_PORT	LPC_GPIO0
#define ROW_CLK_OUT_PIN		(1 << 7)

// Olimex PIN 24
#define ROW_LATCH_PORT		LPC_GPIO2
#define ROW_LATCH_PIN		(1 << 9)

// Olimex PIN 25
#define ROW_ENABLE_PORT		LPC_GPIO2
#define ROW_ENABLE_PIN		(1 << 10)

#else

#define CLK_OUT_PORT		LPC_GPIO1
#define CLK_OUT_PIN		(1 << 1)

#define SER_OUT_PORT		LPC_GPIO1
#define SER_OUT_PIN		(1 << 5)

#define LATCH_PORT		LPC_GPIO1
#define LATCH_PIN		(1 << 3)

#define ROW_SER_OUT_PORT	LPC_GPIO1
#define ROW_SER_OUT_PIN		(1 << 2)

#define ROW_CLK_OUT_PORT	LPC_GPIO1
#define ROW_CLK_OUT_PIN		(1 << 8)

#define ROW_LATCH_PORT		LPC_GPIO1
#define ROW_LATCH_PIN		(1 << 4)

#define ROW_ENABLE_PORT		LPC_GPIO1
#define ROW_ENABLE_PIN		(1 << 0)
#endif

#ifndef FAST_GPIOPinWrite
#define FAST_GPIOPinWrite(ulPort, ucPins, ucVal)  (ulPort->MASKED_ACCESS[ucPins] = ucVal)
#endif

#if 0
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_gpio.h>
#include <driverlib/gpio.h>

#define CLK_OUT_PORT		GPIO_PORTA_BASE
#define CLK_OUT_PIN		GPIO_PIN_4

#define LATCH_PORT		GPIO_PORTA_BASE
#define LATCH_PIN		GPIO_PIN_3

#define SER_OUT_PORT		GPIO_PORTA_BASE
#define SER_OUT_PIN		GPIO_PIN_2

#ifndef FAST_GPIOPinWrite
#define FAST_GPIOPinWrite(ulPort, ucPins, ucVal) HWREG(ulPort + (GPIO_O_DATA + (ucPins << 2))) = ucVal
#endif
#endif
#endif
