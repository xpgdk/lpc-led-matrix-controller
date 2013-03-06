/** \file main.c
 * \brief Sample LPC1114 project
 * \details This file holds a very basic code for LPC1114. This code configures
 * flash access time, enables the PLL and all required clocks and peripherals
 * to achieve the highest allowed frequency for LPC1114 (50MHz). Main code
 * block just blinks the LED. The LED port and pin are defined in config.h
 * file. Target core frequency and quartz crystal resonator frequency are
 * defined there as well.
 *
 * \author Freddie Chopin, http://www.freddiechopin.info/
 * \date 2012-01-08
 */

/******************************************************************************
* project: lpc1114_blink_led
* chip: LPC1114
* compiler: arm-none-eabi-gcc (Sourcery CodeBench Lite 2011.09-69) 4.6.1
*
* prefix: (none)
*
* available global functions:
* 	int main(void)
*
* available local functions:
* 	static void flash_access_time(uint32_t frequency)
* 	static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
* 	static void system_init(void)
*
* available interrupt handlers:
******************************************************************************/

/*
+=============================================================================+
| includes
+=============================================================================+
*/

#include <stdint.h>
#include <string.h>
extern "C" {
#include "inc/LPC11xx.h"
#include "config.h"
#include "uart.h"
}

#include "hdr/hdr_syscon.h"
#include "hdr/hdr_power.h"
#include "hdr/hdr_tmr.h"
#include "hdr/hdr_uart.h"
#include "hdr/hdr_ssp.h"

extern "C" {
//#include "led_matrix.h"
#include "led_matrix_config.h"
}

#include "led-matrix-lib/LedMatrix.hpp"
#include "led-matrix-lib/LedMatrixSimpleFont.hpp"
#include "led-matrix-lib/TestAnimation.hpp"
#include "led-matrix-lib/PulseAnimation.hpp"

LedMatrixFrameBuffer<8,32,32>	frameBuffer;
LedMatrixSimpleFont		defaultFont;
LedMatrixScrollAnimation	scrollAnim(defaultFont);
LedMatrix 			matrix(frameBuffer, defaultFont);

LedMatrixTestAnimation		testAnimation(matrix, scrollAnim);
PulseAnimation			pulseAnimation;

/*
+=============================================================================+
| module variables
+=============================================================================+
*/

/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

static void flash_access_time(uint32_t frequency);
static uint32_t pll_start(uint32_t crystal, uint32_t frequency);
static void system_init(void);
static void system_sleep(void);

/*
+=============================================================================+
| global functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief main code block
* \details Call some static initialization functions and blink the led with
* frequency defined via count_max variable.
*//*-------------------------------------------------------------------------*/

int main(void)
{
	pll_start(CRYSTAL, FREQUENCY);			// start the PLL
	system_init();							// initialize other necessary elements

	// We want 50 frames per second.
	// Each frame takes ROW_COUNT*COLOR_LEVELS updates
	// = 16 * 64 = 512 updates
	// We thus need a frequency of 512 * 50 = 25600 = 25.6KHz
	LPC_TMR16B0->PR = 25; // We run at 50MHz, scale down to 1.1MHz
	LPC_TMR16B0->MR0 = 100; // 27.78KHz
	LPC_TMR16B0->MCR = TMR16_MCR_MR0I | TMR16_MCR_MR0R;
	LPC_TMR16B0->TCR |= TMR16_TCR_CEN; // Enable timer

#if 0
	LED_GPIO->DIR |= LED;					// set the direction of the LED pin to output
	LED_gma = 0;
#endif

	CLK_OUT_PORT->DIR |= CLK_OUT_PIN;
	LATCH_PORT->DIR |= LATCH_PIN;
	SER_OUT_PORT->DIR |= SER_OUT_PIN;

	ROW_SER_OUT_PORT->DIR |= ROW_SER_OUT_PIN;
	ROW_CLK_OUT_PORT->DIR |= ROW_CLK_OUT_PIN;
	ROW_LATCH_PORT->DIR |= ROW_LATCH_PIN;
	ROW_ENABLE_PORT->DIR |= ROW_ENABLE_PIN;

	FAST_GPIOPinWrite(ROW_ENABLE_PORT, ROW_ENABLE_PIN, 0);
	//displayInit();

	//char s[] = "#F00H#220e#550l#FF0l#BF0o #0F0W#F00orld  ";
	//char s[] = "#3F00J#003Fo#203Fn#3F20a#3F00than #0A3FFleischer  ";
	//char s[] = "Jonathan Fleischer  ";
	//char s[] = "#FF0EEEE     ";
	char s[] = "#3F00Hello #003FWorld";

	matrix.setAnimation(&scrollAnim, 6);
	scrollAnim.setMessage(s, strlen(s));

	//matrix.setMessage(s, strlen(s));
	//LedMatrixColor color(0xA, 0x3F, 0x00);
	//matrix.setChar('H', color);
	//matrix.clear(color);
	//set_message(s, strlen(s));
	//set_char('E', COLOR(0x3, 0xF, 0));
	//displayFillColor(COLOR(20, 0, 0));
	//msg_mode = MODE_ANIM;

	NVIC_EnableIRQ(TIMER_16_0_IRQn);
	NVIC_EnableIRQ(SSP1_IRQn);

	printf("Entering loop\r\n");

	while (1)
	{
		system_sleep();
	}
}

/*
+=============================================================================+
| local functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief Configures flash access time.
* \details Configures flash access time which allows the chip to run at higher
* speeds.
*
* \param [in] frequency defines the target frequency of the core
*//*-------------------------------------------------------------------------*/

static void flash_access_time(uint32_t frequency)
{
	uint32_t access_time, flashcfg_register;

	if (frequency < 20000000ul)				// 1 system clock for core speed below 20MHz
		access_time = FLASHCFG_FLASHTIM_1CLK;
	else if (frequency < 40000000ul)		// 2 system clocks for core speed between 20MHz and 40MHz
		access_time = FLASHCFG_FLASHTIM_2CLK;
	else									// 3 system clocks for core speed over 40MHz
		access_time = FLASHCFG_FLASHTIM_3CLK;

	// do not modify reserved bits in FLASHCFG register
	flashcfg_register = FLASHCFG;			// read register
	flashcfg_register &= ~(FLASHCFG_FLASHTIM_mask << FLASHCFG_FLASHTIM_bit);	// mask the FLASHTIM field
	flashcfg_register |= access_time << FLASHCFG_FLASHTIM_bit;	// use new FLASHTIM value
	FLASHCFG = flashcfg_register;			// save the new value back to the register
}

/*------------------------------------------------------------------------*//**
* \brief Starts the PLL.
* \details Configure and enable PLL to achieve some frequency with some
* crystal. Before the speed change flash access time is configured via
* flash_access_time(). Main oscillator is configured and started. PLL
* parameters m and p are based on function parameters. The PLL is configured,
* started and selected as the main clock. AHB clock divider is set to 1.
*
* \param [in] crystal is the frequency of the crystal resonator connected to
* the LPC1114 chip.
* \param [in] frequency is the desired target frequency after enabling the PLL
*
* \return real frequency that was set
*//*-------------------------------------------------------------------------*/

static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
{
	uint32_t m, p = 0, fcco;

	flash_access_time(frequency);			// configure flash access time first

	// SYSOSCCTRL_FREQRANGE should be 0 for crystals in range 1 - 20MHz
	// SYSOSCCTRL_FREQRANGE should be 1 for crystals in range 15 - 25MHz
	if (crystal < 17500000)					// divide the ranges on 17.5MHz then
		LPC_SYSCON->SYSOSCCTRL = 0;			// "lower speed" crystals
	else
		LPC_SYSCON->SYSOSCCTRL = SYSOSCCTRL_FREQRANGE;	// "higher speed" crystals

	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSOSC_PD;	// power-up main oscillator

	LPC_SYSCON->SYSPLLCLKSEL = SYSPLLCLKSEL_SEL_IRC;	// select main oscillator as the input clock for PLL
	LPC_SYSCON->SYSPLLCLKUEN = 0;			// confirm the change of PLL input clock by toggling the...
	LPC_SYSCON->SYSPLLCLKUEN = SYSPLLUEN_ENA;	// ...ENA bit in LPC_SYSCON->SYSPLLCLKUEN register

	// calculate PLL parameters
	m = frequency / crystal;				// M is the PLL multiplier
	fcco = m * crystal * 2;					// FCCO is the internal PLL frequency

	frequency = crystal * m;

	while (fcco < 156000000)
	{
		fcco *= 2;
		p++;								// find P which gives FCCO in the allowed range (over 156MHz)
	}

	LPC_SYSCON->SYSPLLCTRL = ((m - 1) << SYSPLLCTRL_MSEL_bit) | (p << SYSPLLCTRL_PSEL_bit);	// configure PLL
	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSPLL_PD; // power-up PLL

	while (!(LPC_SYSCON->SYSPLLSTAT & SYSPLLSTAT_LOCK));	// wait for PLL lock

	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_SEL_PLLOUT;	// select PLL output as the main clock
	LPC_SYSCON->MAINCLKUEN = 0;				// confirm the change of main clock by toggling the...
	LPC_SYSCON->MAINCLKUEN = MAINCLKUEN_ENA;	// ...ENA bit in LPC_SYSCON->MAINCLKUEN register

	LPC_SYSCON->SYSAHBCLKDIV = 1;			// set AHB clock divider to 1

	return frequency;
}

/*------------------------------------------------------------------------*//**
* \brief Initializes system.
* \details Enables clock for IO configuration block.
*//*-------------------------------------------------------------------------*/

static void system_init(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_IOCON;	// Enable clock for IO configuration block
	LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_CT16B0;	// Enable clock for Timer
	LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_UART;	// Enable clock for UART
	LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_SSP1;	// Enable clock for SSP0

	/* Pin configuration */
	LPC_IOCON->PIO1_6 = 0x01; // UART RX
	LPC_IOCON->PIO1_7 = 0x01; // UART TX

	LPC_IOCON->PIO2_0 = 0x02; // SSP1 SSEL -- Olimex pin 2
	LPC_IOCON->PIO2_1 = 0x02; // SSP0 SCK  -- Olimex pin 13
	LPC_IOCON->PIO2_2 = 0x02; // SSP1 MISO -- Olimex pin 26
	LPC_IOCON->PIO2_3 = 0x02; // SSP0 MOSI -- Olimex pin 38

	/* Configure UART */
	// Select UART_PCLK to 50Mhz
	LPC_SYSCON->UARTCLKDIV = 1; // 8-bit divider
	LPC_SYSCON->SSP1CLKDIV = 1;

	LPC_UART->LCR |= UART_LCR_DLAB;
	// Baud rate is defined by:
	// UART_rate = UART_PCLK / (16 * ( 256 * U0DLM + U0DLL) * (1 + DivAddVal/MulVal))
	// We want 115200 BAUD and achieve it by 50MHz / 27
	LPC_UART->DLM = 0;
	LPC_UART->DLL = 27;

	LPC_UART->LCR &= ~UART_LCR_DLAB;
	LPC_UART->LCR = UART_LCR_WLS_8BIT | UART_LCR_SBS_1BIT;

	// Enable UART (no hardware flow control)
	LPC_UART->TER |= UART_TER_TXEN;
	LPC_UART->MCR = 0;
	LPC_UART->FCR |= UART_FCR_FIFOEN;

	printf("UART initialized\r\n");

	/* Configure SSP1 for slave mode */
	LPC_SYSCON->PRESETCTRL |= PRESETCTRL_SSP1_RST_N;
	LPC_SSP1->CR0 = SSP_CR0_DSS_8_BIT | SSP_CR0_FRF_SPI | SSP_CR0_CPHA | SSP_CR0_CPOL;
	LPC_SSP1->IMSC = SSP_IMSC_RXIM | SSP_IMSC_RTIM;
	LPC_SSP1->CR1 = SSP_CR1_SSE | SSP_CR1_MS | SSP_CR1_SOD;

	if( LPC_SSP1->SR & SSP_SR_TFE ) {
		printf("Transmit FIFO is empty\r\n");
	} else {
		printf("Transmit FIFO is NOT empty\r\n");
	}
	// Ensure that we have something to send
	//LPC_SSP1->DR = 0x42;

	printf("SPI Initialized\r\n");
}

static void system_sleep(void)
{
	LPC_PMU->PCON &= ~PCON_DPDEN;
	SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
	__WFI();
}

/*
+=============================================================================+
| ISRs
+=============================================================================+
*/

#ifdef __cplusplus
extern "C" {
#endif

void TIMER16_0_IRQHandler(void)
{
	//LED_gma = 0;
	//NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);
	if( LPC_TMR16B0->IR & 0x01) {
		LPC_TMR16B0->IR = 0x01;
		matrix.update();
	}

}

#define CMD_RESET		0x01
#define CMD_APPEND_MSG		0x02
#define CMD_CLEAR_MSG		0x03

#define CMD_RESP_OK		0x01
#define CMD_RESP_ERR		0xFF

#define STATE_IDLE			0x00
#define STATE_MESSAGE_CMD			0x01
#define STATE_MESSAGE_CMD_RDY_FOR_DATA	0x02
#define STATE_MESSAGE_CMD_DATA		0x03
#define STATE_RESP_TO_IDLE1		0xFD
#define STATE_RESP_TO_IDLE2		0xFE
#define STATE_INVALID			0xFF

static uint16_t state = STATE_IDLE;
static uint8_t data_buffer[50];
static uint8_t data_count;
static uint8_t current_data_count;

void SSP1_IRQHandler(void)
{
	if( LPC_SSP1->MIS & SSP_MIS_RXMIS ||
	    LPC_SSP1->MIS & SSP_MIS_RTMIS ) {
		if( LPC_SSP1->MIS & SSP_MIS_RTMIS ) {
			LPC_SSP1->ICR |= SSP_ICR_RTIC;
		}
		while( LPC_SSP1->SR & SSP_SR_RNE ) {
			uint8_t data = (uint8_t)(LPC_SSP1->DR & 0xFF);
			uint8_t nextOutByte = 0x00;
			bool slaveEnable = false;
#ifdef DEBUG
			printf("State is: ");
			putHex8(state);
			printf("\r\n");
			printf("Read a byte: ");
			putHex8(data);
			printf("\r\n");
#endif
			switch(state) {
				case STATE_IDLE:
					if( data == CMD_APPEND_MSG ) {
						state = STATE_MESSAGE_CMD;
					} else if( data == CMD_CLEAR_MSG ) {
						nextOutByte = CMD_RESP_OK;
						slaveEnable = true;
						scrollAnim.setMessage((char*)"", 0);
						state = STATE_RESP_TO_IDLE1;
					} else if( data == CMD_RESET ) {
						state = STATE_IDLE;
					} else {
						state = STATE_INVALID;
					}
				break;
				case STATE_RESP_TO_IDLE1:
					state = STATE_RESP_TO_IDLE2;
				break;
				case STATE_RESP_TO_IDLE2:
					state = STATE_IDLE;
				break;
				case STATE_MESSAGE_CMD:
					state = STATE_MESSAGE_CMD_DATA;
					data_count = data;
					current_data_count = 0;
#ifdef DEBUG
					printf("Expecting ");
					putHex8(data_count);
					printf(" bytes\r\n");
#endif
				break;
				case STATE_MESSAGE_CMD_DATA:
					data_buffer[current_data_count] = data;
					current_data_count++;
					if( current_data_count == data_count - 1) {
						nextOutByte = 0x42;
						slaveEnable = true;
					}
					if( current_data_count >= data_count ) {
						//LPC_SSP1->CR1 &= ~SSP_CR1_SOD;
						//LPC_SSP1->DR = CMD_RESP_OK;
						nextOutByte = CMD_RESP_OK;
						slaveEnable = true;
						//append_message((char*)data_buffer, data_count);
						scrollAnim.appendMessage((char*)data_buffer, data_count);
#ifdef DEBUG
						printf("Got all data\r\n");
						for(int i=0; i<data_count; i++) {
							putHex8(data_buffer[i]);
							printf(" ");
						}
						printf("\r\n");
#endif
						state = STATE_RESP_TO_IDLE1;
					}
				break;
				case STATE_INVALID:
					if( data == CMD_RESET ) {
						state = STATE_IDLE;
						printf("Reset\r\n");
					} else {
						/*LPC_SSP1->CR1 &= ~SSP_CR1_SOD;
						LPC_SSP1->DR = CMD_RESP_ERR;*/
						/*printf("Ignoring data in invalid state: ");
						putHex8(data);
						printf("\r\n");*/
					}
				break;
			}
			if( slaveEnable ) {
				LPC_SSP1->DR = nextOutByte;
				LPC_SSP1->CR1 &= ~SSP_CR1_SOD;
			} else {
				LPC_SSP1->CR1 |= SSP_CR1_SOD;
			}
		}
	}	
}

#ifdef __cplusplus
}
#endif

/******************************************************************************
* END OF FILE
******************************************************************************/
