#ifndef _HDR_UART_H
#define _HDR_UART_H

/* UART FIFO COntrol Register (FCR) */
#define UART_FCR_FIFOEN_bit	0
#define UART_FCR_FIFOEN		(1 << UART_FCR_FIFOEN_bit)
#define UART_FCR_RXFIFORES_bit	1
#define UART_FCR_RXFIFORES	(1 << UART_FCR_RXFIFORES_bit)
#define UART_FCR_TXFIFORES_bit	2
#define UART_FCR_TXFIFORES	(1 << UART_FCR_TXFIFORES_bit)
#define UART_FCR_RXTL_bit	6
#define UART_FCR_RXTL_mask	3
#define UART_FCR_RXTL_LEVEL_0_value	0
#define UART_FCR_RXTL_LEVEL_1_value	1
#define UART_FCR_RXTL_LEVEL_2_value	2
#define UART_FCR_RXTL_LEVEL_3_value	3
#define UART_FCR_RXTL_LEVEL_0		(1<<UART_FCR_RXTL_LEVEL_0_bit)
#define UART_FCR_RXTL_LEVEL_1		(1<<UART_FCR_RXTL_LEVEL_1_bit)
#define UART_FCR_RXTL_LEVEL_2		(1<<UART_FCR_RXTL_LEVEL_2_bit)
#define UART_FCR_RXTL_LEVEL_3		(1<<UART_FCR_RXTL_LEVEL_3_bit)

/* UART Lice Control Register (LCR) */
#define UART_LCR_WLS_bit_mask	3
#define UART_LCR_WLS_bit		0
#define UART_LCR_WLS_5BIT_value	0
#define UART_LCR_WLS_6BIT_value	1
#define UART_LCR_WLS_7BIT_value	2
#define UART_LCR_WLS_8BIT_value	3
#define UART_LCR_WLS_5BIT	(UART_LCR_WLS_5BIT_value << UART_LCR_WLS_bit)
#define UART_LCR_WLS_6BIT	(UART_LCR_WLS_6BIT_value << UART_LCR_WLS_bit)
#define UART_LCR_WLS_7BIT	(UART_LCR_WLS_7BIT_value << UART_LCR_WLS_bit)
#define UART_LCR_WLS_8BIT	(UART_LCR_WLS_8BIT_value << UART_LCR_WLS_bit)

#define UART_LCR_SBS_bit		2
#define UART_LCR_SBS_1BIT		(0 << UART_LCR_SBS_bit)
#define UART_LCR_SBS_2BIT		(1 << UART_LCR_SBS_bit)

#define UART_LCR_PE_bit		3
#define UART_LCR_PE			(1 << UART_LCR_PE_bit)

#define UART_LCR_PS_bit		4
#define UART_LCR_PS_mask		3
#define UART_LCR_PS_ODD_value	0
#define UART_LCR_PS_EVEN_value	1
#define UART_LCR_PS_STICK_1_value	2
#define UART_LCR_PS_STICK_0_value	3
#define UART_LCR_PS_ODD		(UART_LCR_PS_ODD_value << UART_LCR_PS_bit)
#define UART_LCR_PS_EVEN		(UART_LCR_PS_EVEN_value << UART_LCR_PS_bit)
#define UART_LCR_PS_STICK_1		(UART_LCR_PS_STICK_1_value << UART_LCR_PS_bit)
#define UART_LCR_PS_STICK_0		(UART_LCR_PS_STICK_0_value << UART_LCR_PS_bit)

#define UART_LCR_BC_bit		6
#define UART_LCR_BC			(1<<UART_LCR_BC_bit)

#define UART_LCR_DLAB_bit		7
#define UART_LCR_DLAB		(1<<UART_LCR_DLAB_bit)

/* UART Modem Control Register (MCR) */
#define UART_MCR_DTRC_bit		0
#define UART_MCR_RTSC_bit		1
#define UART_MCR_LMS_bit		4
#define UART_MCR_RTSEN_bit		6
#define UART_MCR_CTSEN_bit		7

#define UART_MCR_DTRC			(1<<UART_MCR_DTRC_bit)
#define UART_MCR_RTSC			(1<<UART_MCR_RTSC_bit)
#define UART_MCR_LMS			(1<<UART_MCR_LMS_bit)
#define UART_MCR_RTSEN			(1<<UART_MCR_RTSEN_bbit)
#define UART_MCR_CTSEN			(1<<UART_MCR_CTSEN_bit)

/* UART Line Status Register (LSR) */
#define UART_LSR_RDR_bit		0
#define UART_LSR_OE_bit			1
#define UART_LSR_PE_bit			2
#define UART_LSR_FE_bit			3
#define UART_LSR_BI_bit			4
#define UART_LSR_THRE_bit		5
#define UART_LSR_TEMT_bit		6
#define UART_LSR_RXFE_bit		7
#define UART_LSR_RDR			(1 << UART_LSR_RDR_bit)
#define UART_LSR_OE			(1 << UART_LSR_OE_bit)
#define UART_LSR_PE			(1 << UART_LSR_PE_bit)
#define UART_LSR_FE			(1 << UART_LSR_FE_bit)
#define UART_LSR_THRE			(1 << UART_LSR_THRE_bit)
#define UART_LSR_TEMT			(1 << UART_LSR_TEMT_bit)
#define UART_LSR_RXFE			(1 << UART_LSR_RXFE_bit)

/* UART Fractional Divider Register (FDR) */
#define UART_FDR_DIVADDVALL_bit	0
#define UART_FDR_DIVADDVALL_mask 	0xF
#define UART_FDR_MULVAL_bit		4
#define UART_FDR_MULVAL_mask		0xF

#define UART_FDR_value(div, mul)	( (((div) & UART_FDR_DIVADDVALL_mask) << UART_FDR_DIVADDVALL_bit) | (((mul) & UART_FDR_MULVAL_mask) << UART_FDR_MULVAL_bit))


/* UART Transmit Enable Register (TER) */
#define UART_TER_TXEN_bit	7
#define UART_TER_TXEN		(1 << UART_TER_TXEN_bit)
#endif
