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

#include <mcu++/gpio.hpp>
#include <mcu++/lpc11xx_gpio.hpp>

#include "CommandProcessor.hpp"

#include "board_config.hpp"

//typedef LPC1114DipConfig BoardConfig;
//typedef MCU::StaticLPCGPIO<LPC_GPIO0_BASE, 2> SlaveSelect;
typedef LPC1114OlimexConfig BoardConfig;
typedef BoardConfig::SlaveSelect SlaveSelect;

typedef LedMatrixFrameBuffer<BoardConfig::LedConfig>	FrameBuffer;

FrameBuffer					frameBuffer[2];
LedMatrixSimpleFont				defaultFont;
LedMatrixScrollAnimation<FrameBuffer>		scrollAnim(defaultFont);
LedMatrix<FrameBuffer> 				matrix(frameBuffer[0], defaultFont);

LedMatrixTestAnimation<FrameBuffer>		testAnimation(matrix, scrollAnim);
PulseAnimation<FrameBuffer>			pulseAnimation;

struct CommandProcessorConfig {
public:
	typedef ::FrameBuffer 			FrameBuffer;
	typedef ::LedMatrix<FrameBuffer>	Matrix;
	
	static const int	ScreenOffsetX = 0;
	static const int	ScreenOffsetY = 0;
	static const bool	ScreenRotate = false;
};

CommandProcessor<CommandProcessorConfig> processor(frameBuffer, matrix);

#undef DEBUG


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

	matrix.init();
#if 0
	CLK_OUT_PORT->DIR |= CLK_OUT_PIN;
	LATCH_PORT->DIR |= LATCH_PIN;
	SER_OUT_PORT->DIR |= SER_OUT_PIN;

	ROW_SER_OUT_PORT->DIR |= ROW_SER_OUT_PIN;
	ROW_CLK_OUT_PORT->DIR |= ROW_CLK_OUT_PIN;
	ROW_LATCH_PORT->DIR |= ROW_LATCH_PIN;
	ROW_ENABLE_PORT->DIR |= ROW_ENABLE_PIN;

	FAST_GPIOPinWrite(ROW_ENABLE_PORT, ROW_ENABLE_PIN, 0);
#endif
	//displayInit();

	//char s[] = "#F00H#220e#550l#FF0l#BF0o #0F0W#F00orld  ";
	//char s[] = "#3F00J#003Fo#203Fn#3F20a#3F00than #0A3FFleischer  ";
	//char s[] = "Jonathan Fleischer  ";
	//char s[] = "#FF0EEEE     ";
	char s[] = "#3F00Hello #003FWorld";

	LedMatrixColor yellow(5, 32, 0);

	//matrix.setAnimation(&scrollAnim, 6);
	matrix.clear();
	scrollAnim.setMessage(s, strlen(s));

	//matrix.changeFrameBuffer(&frameBuffer[currentFrameBuffer]);

	//matrix.setMessage(s, strlen(s));
	//LedMatrixColor color(0xA, 0x3F, 0x00);
	//matrix.setChar('H', color);
	//matrix.clear(color);
	//set_message(s, strlen(s));
	//set_char('E', COLOR(0x3, 0xF, 0));
	//displayFillColor(COLOR(20, 0, 0));
	//msg_mode = MODE_ANIM;

#if 1
	//SlaveSelect::ConfigureDirection(MCU::GPIO::Input);
	/*SlaveSelect::ConfigureInterrupt(MCU::GPIO::EdgeRising);
	SlaveSelect::EnableInterrupt();
	SlaveSelect::ClearInterrupt();*/
#else
#ifdef SPI1
        LPC_GPIO2->IS &= ~(1<<0);
        LPC_GPIO2->IBE &= ~(1<<0);
        LPC_GPIO2->IEV |= (1<<0);
        LPC_GPIO2->IE |= (1<<0);
#else
        LPC_GPIO0->IS &= ~(1<<2);
        LPC_GPIO0->IBE &= ~(1<<2);
        LPC_GPIO0->IEV |= (1<<2);
        LPC_GPIO0->IE |= (1<<2);
#endif
#endif

	NVIC_EnableIRQ(TIMER_16_0_IRQn);
	/*NVIC_EnableIRQ(SSP0_IRQn);
	NVIC_EnableIRQ(SSP1_IRQn);*/
        //NVIC_EnableIRQ(EINT2_IRQn);
        //NVIC_EnableIRQ(EINT0_IRQn);

	printf("Entering loop\r\n");

	LPC_SSP_TypeDef *SSP = BoardConfig::GetSSP();

	uint16_t count = 0;
	uint16_t match = 0;
	volatile uint8_t value, prev_value;

	value = prev_value = 1;


	while (1)
	{
		//system_sleep();
		value = SlaveSelect::Read();
		while( SSP->SR & SSP_SR_RNE ) {
			uint8_t data = (uint8_t)(SSP->DR & 0xFF);
			processor.handleByte(data);
			count++;
		}
		if( value != prev_value && value == 1 ) {
			processor.reset();
			//printf("De-select\r\n");
			/*while( SSP->SR & SSP_SR_RNE ) {
				uint8_t data = (uint8_t)(SSP->DR & 0xFF);
				count++;
			}*/
			if( count != 0x20E ) {
				printf("Got: ");
				putHex16(count);
				printf("\r\n");
			}
			count = 0;
		}
		prev_value = value;
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

static void system_init(void) {
	BoardConfig::EnableClocks();
	BoardConfig::ConfigurePins();

	/* Configure UART */
	// Select UART_PCLK to 50Mhz
	LPC_SYSCON->UARTCLKDIV = 1; // 8-bit divider

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

	BoardConfig::SetupSPI();

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
	//NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);
	if( LPC_TMR16B0->IR & 0x01) {
		LPC_TMR16B0->IR = 0x01;
		matrix.update();
	}
}

#ifdef __cplusplus
}
#endif

/******************************************************************************
* END OF FILE
******************************************************************************/
