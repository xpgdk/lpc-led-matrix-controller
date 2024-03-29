#ifndef _BOARD_CONFIG_HPP
#define _BOARD_CONFIG_HPP

template<unsigned SSP_BASE>
class CommonBoardConfig {
public:
	static void EnableClocks() {
		LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_IOCON;	// Enable clock for IO configuration block
		LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_CT16B0;	// Enable clock for Timer
		LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_UART;	// Enable clock for UART
	}

	static void ConfigurePins() {
		/* Pin configuration */
		LPC_IOCON->PIO1_6 = 0x01; // UART RX
		LPC_IOCON->PIO1_7 = 0x01; // UART TX
	}

	static void SetupSPI() {
		LPC_SSP_TypeDef *ssp = GetSSP();

		ssp->CR0 = SSP_CR0_DSS_8_BIT | SSP_CR0_FRF_SPI | SSP_CR0_CPHA | SSP_CR0_CPOL;
		ssp->IMSC = SSP_IMSC_RXIM | SSP_IMSC_RTIM;
		ssp->CR1 = SSP_CR1_SSE | SSP_CR1_MS | SSP_CR1_SOD;

		if( ssp->SR & SSP_SR_TFE ) {
			printf("Transmit FIFO is empty\r\n");
		} else {
			printf("Transmit FIFO is NOT empty\r\n");
		}
	}

	static LPC_SSP_TypeDef * GetSSP() {
		return (LPC_SSP_TypeDef*) SSP_BASE;
	}
};


class LedOlimexConfig {
public:
	static const uint16_t Rows = 8;
	static const uint16_t Cols = 16;
	static const uint16_t Levels = 32;

	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE,10>	GPIORowEnable;
	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE,9>	GPIORowLatch;
	typedef MCU::StaticLPCGPIO<LPC_GPIO0_BASE,7>	GPIORowClock;
	typedef MCU::StaticLPCGPIO<LPC_GPIO0_BASE,6>	GPIORowOutput;

	typedef MCU::StaticLPCGPIO<LPC_GPIO3_BASE,5>	GPIOColOutput;
	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE,5>	GPIOColClock;
	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE,4>	GPIOColLatch;
};

class LPC1114OlimexConfig{
public:
	typedef LedOlimexConfig LedConfig;
	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE, 0> SlaveSelect;

	static void EnableClocks() {
		Parent::EnableClocks();
		LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_SSP1;	// Enable clock for SSP1
	}

	static void ConfigurePins() {
		Parent::ConfigurePins();
	}

	static void SetupSPI() {
		LPC_SYSCON->SSP1CLKDIV = 1;

		LPC_IOCON->PIO2_0 = 0x02; // SSP1 SSEL -- Olimex pin 2
		LPC_IOCON->PIO2_1 = 0x02; // SSP1 SCK  -- Olimex pin 13
		LPC_IOCON->PIO2_2 = 0x02; // SSP1 MISO -- Olimex pin 26
		LPC_IOCON->PIO2_3 = 0x02; // SSP1 MOSI -- Olimex pin 38
		LPC_SYSCON->PRESETCTRL |= PRESETCTRL_SSP1_RST_N;

		Parent::SetupSPI();
	}

	static LPC_SSP_TypeDef * GetSSP() {
		return (LPC_SSP_TypeDef*) LPC_SSP1_BASE;
	}
private:
	typedef CommonBoardConfig<LPC_SSP1_BASE> Parent;
};


class LedDipConfig {
public:
	static const uint16_t Rows = 8;
	static const uint16_t Cols = 16;
	static const uint16_t Levels = 32;

	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,0>	GPIORowEnable;
	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,4>	GPIORowLatch;
	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,8>	GPIORowClock;
	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,2>	GPIORowOutput;

	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,5>	GPIOColOutput;
	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,1>	GPIOColClock;
	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,3>	GPIOColLatch;
};

class LPC1114DipConfig {
public:
	typedef LedDipConfig LedConfig;
	typedef MCU::StaticLPCGPIO<LPC_GPIO0_BASE, 2> SlaveSelect;

	static void EnableClocks() {
		Parent::EnableClocks();
		LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_SSP0;	// Enable clock for SSP0
	}

	static void ConfigurePins() {
		Parent::ConfigurePins();

		/* Choose PIO functionality */
		LPC_IOCON->R_PIO1_0 = 0x01;
		LPC_IOCON->R_PIO1_1 = 0x01;
		LPC_IOCON->R_PIO1_2 = 0x01;
		LPC_IOCON->SWDIO_PIO1_3 = 0x01;
	}

	static void SetupSPI() {
		LPC_SYSCON->SSP0CLKDIV = 1;

		LPC_IOCON->SCK_LOC = 0x02;
		LPC_IOCON->PIO0_2 = 0x01; // SSP0 SSEL -- PIO0_2
		LPC_IOCON->PIO0_6 = 0x02; // SSP0 SCK  -- PIO0_6
		LPC_IOCON->PIO0_8 = 0x01; // SSP0 MISO -- PIO0_8
		LPC_IOCON->PIO0_9 = 0x01; // SSP0 MOSI -- PIO0_9
		LPC_SYSCON->PRESETCTRL |= PRESETCTRL_SSP0_RST_N;

		Parent::SetupSPI();
	}

	static LPC_SSP_TypeDef * GetSSP() {
		return (LPC_SSP_TypeDef*) LPC_SSP0_BASE;
	}
private:
	typedef CommonBoardConfig<LPC_SSP0_BASE> Parent;
};


class LedLPC1113BreakoutConfig {
public:
	static const uint16_t Rows = 8;
	static const uint16_t Cols = 16;
	static const uint16_t Levels = 32;

	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE,9>	GPIORowEnable;
	typedef MCU::StaticLPCGPIO<LPC_GPIO3_BASE,4>	GPIORowLatch;
	typedef MCU::StaticLPCGPIO<LPC_GPIO1_BASE,9>	GPIORowClock;
	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE,5>	GPIORowOutput;

	typedef MCU::StaticLPCGPIO<LPC_GPIO0_BASE,3>	GPIOColOutput;
	typedef MCU::StaticLPCGPIO<LPC_GPIO3_BASE,5>	GPIOColClock;
	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE,4>	GPIOColLatch;
};

class LPC1113BreakoutConfig{
public:
	typedef LedLPC1113BreakoutConfig LedConfig;
	typedef MCU::StaticLPCGPIO<LPC_GPIO2_BASE, 0> SlaveSelect;

	static void EnableClocks() {
		Parent::EnableClocks();
		LPC_SYSCON->SYSAHBCLKCTRL |= SYSAHBCLKCTRL_SSP1;	// Enable clock for SSP1
	}

	static void ConfigurePins() {
		Parent::ConfigurePins();

		LPC_IOCON->PIO0_4 = 0x100;
		LPC_IOCON->PIO0_5 = 0x100;
		LPC_IOCON->PIO2_4 = 0x00;
		LPC_IOCON->PIO2_5 = 0x00;
		LPC_IOCON->PIO2_9 = 0x00;
		LPC_IOCON->PIO3_5 = 0x00;
	}

	static void SetupSPI() {
		LPC_SYSCON->SSP1CLKDIV = 1;

		LPC_IOCON->PIO2_0 = 0x02; // SSP1 SSEL -- Olimex pin 2
		LPC_IOCON->PIO2_1 = 0x02; // SSP1 SCK  -- Olimex pin 13
		LPC_IOCON->PIO2_2 = 0x02; // SSP1 MISO -- Olimex pin 26
		LPC_IOCON->PIO2_3 = 0x02; // SSP1 MOSI -- Olimex pin 38
		LPC_SYSCON->PRESETCTRL |= PRESETCTRL_SSP1_RST_N;

		Parent::SetupSPI();
	}

	static LPC_SSP_TypeDef * GetSSP() {
		return (LPC_SSP_TypeDef*) LPC_SSP1_BASE;
	}
private:
	typedef CommonBoardConfig<LPC_SSP1_BASE> Parent;
};

#endif
