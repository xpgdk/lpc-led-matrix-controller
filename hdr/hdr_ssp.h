#ifndef _HDR_SSP_H
#define _HDR_SSP_H

/* SSP Control Register 0 (CR0) */
#define SSP_CR0_DSS_bit			0
#define SSP_CR0_DSS_mask		0xF
#define SSP_CR0_DSS_4_BIT_value		0x3
#define SSP_CR0_DSS_5_BIT_value		0x4
#define SSP_CR0_DSS_6_BIT_value		0x5
#define SSP_CR0_DSS_7_BIT_value		0x6
#define SSP_CR0_DSS_8_BIT_value		0x7
#define SSP_CR0_DSS_9_BIT_value		0x8
#define SSP_CR0_DSS_10_BIT_value	0x9
#define SSP_CR0_DSS_11_BIT_value	0xA
#define SSP_CR0_DSS_12_BIT_value	0xB
#define SSP_CR0_DSS_13_BIT_value	0xC
#define SSP_CR0_DSS_14_BIT_value	0xD
#define SSP_CR0_DSS_15_BIT_value	0xE
#define SSP_CR0_DSS_16_BIT_value	0xF

#define SSP_CR0_DSS_3_BIT		(SSP_CR0_DSS_3_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_4_BIT		(SSP_CR0_DSS_4_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_5_BIT		(SSP_CR0_DSS_5_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_6_BIT		(SSP_CR0_DSS_6_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_7_BIT		(SSP_CR0_DSS_7_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_8_BIT		(SSP_CR0_DSS_8_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_9_BIT		(SSP_CR0_DSS_9_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_10_BIT		(SSP_CR0_DSS_10_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_11_BIT		(SSP_CR0_DSS_11_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_12_BIT		(SSP_CR0_DSS_12_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_13_BIT		(SSP_CR0_DSS_13_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_14_BIT		(SSP_CR0_DSS_14_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_15_BIT		(SSP_CR0_DSS_15_BIT_value << SSP_CR0_DSS_bit)
#define SSP_CR0_DSS_16_BIT		(SSP_CR0_DSS_16_BIT_value << SSP_CR0_DSS_bit)

#define SSP_CR0_FRF_bit			4
#define SSP_CR0_FRF_mask		3
#define SSP_CR0_FRF_SPI_value		0x0
#define SSP_CR0_FRF_TI_value		0x1
#define SSP_CR0_FRF_MICROWIRE_value	0x2
#define SSP_CR0_FRF_SPI			(SSP_CR0_FRF_SPI_value << SSP_CR0_FRF_bit)
#define SSP_CR0_FRF_TI			(SSP_CR0_FRF_TI_value << SSP_CR0_FRF_bit)
#define SSP_CR0_FRF_MICROWIRE		(SSP_CR0_FRF_MICROWIRE_value << SSP_CR0_FRF_bit)

#define SSP_CR0_CPOL_bit		6
#define SSP_CR0_CPOL			(1<<SSP_CR0_CPOL_bit)

#define SSP_CR0_CPHA_bit		7
#define SSP_CR0_CPHA			(1<<SSP_CR0_CPHA_bit)

#define SSP_CR0_SCR_bit			8
#define SSP_CR0_SCR_mask		0xFF

/* SSP Control Register 1 (CR1) */
#define SSP_CR1_LPM_bit			0
#define SSP_CR1_LPM			(1<<SSP_CR1_LPM_bit)
#define SSP_CR1_SSE_bit			1
#define SSP_CR1_SSE			(1<<SSP_CR1_SSE_bit)
#define SSP_CR1_MS_bit			2
#define SSP_CR1_MS			(1<<SSP_CR1_MS_bit)
#define SSP_CR1_SOD_bit			3
#define SSP_CR1_SOD			(1<<SSP_CR1_SOD_bit)

/* SSP Data Register (DR) */
#define SSP_DR_DATA_bit			0
#define SSP_DR_DATA_mask		0xFFFF

/* SSP Status Register (SR) */
#define SSP_SR_TFE_bit			0
#define SSP_SR_TFE			(1<<SSP_SR_TFE_bit)
#define SSP_SR_TNF_bit			1
#define SSP_SR_TNF			(1<<SSP_SR_TNF_bit)
#define SSP_SR_RNE_bit			2
#define SSP_SR_RNE			(1<<SSP_SR_RNE_bit)
#define SSP_SR_RFF_bit			3
#define SSP_SR_RFF			(1<<SSP_SR_RFF_bit)
#define SSP_SR_BSY_bit			4
#define SSP_SR_BSY			(1<<SSP_SR_BSY_bit)

/* SSP Clock Prescale Register (CPSR) */
#define SSP_CPSR_CPSDVSR_bit		0
#define SSP_CPSR_CPSDVSR_mask		0xF

/* SSP Interrupt Mask Set/Clear Register (IMSC) */
#define SSP_IMSC_RORIM_bit		0
#define SSP_IMSC_RORIM			(1<<SSP_IMSC_RORIM_bit)
#define SSP_IMSC_RTIM_bit		1
#define SSP_IMSC_RTIM			(1<<SSP_IMSC_RTIM_bit)
#define SSP_IMSC_RXIM_bit		2
#define SSP_IMSC_RXIM			(1<<SSP_IMSC_RXIM_bit)
#define SSP_IMSC_TXIM_bit		3
#define SSP_IMSC_TXIM			(1<<SSP_IMSC_TXIM_bit)

/* SSP Raw Interrupt Status Register (RIS) */
#define SSP_RIS_RORRIS_bit		0
#define SSP_RIS_RORRIS			(1<<SSP_RIS_RORRIS_bit)
#define SSP_RIS_RTRIS_bit		1
#define SSP_RIS_RTRIS			(1<<SSP_RIS_RTRIS_bit)
#define SSP_RIS_RXRIS_bit		2
#define SSP_RIS_RXRIS			(1<<SSP_RIS_RXRIS_bit)
#define SSP_RIS_TXRIS_bit		3
#define SSP_RIS_TXRIS			(1<<SSP_RIS_TXRIS_bit)

/* SSP Masked Interrupt Status Register (MIS) */
#define SSP_MIS_RORMIS_bit		0
#define SSP_MIS_RORMIS			(1<<SSP_MIS_RORMIS_bit)
#define SSP_MIS_RTMIS_bit		1
#define SSP_MIS_RTMIS			(1<<SSP_MIS_RTMIS_bit)
#define SSP_MIS_RXMIS_bit		2
#define SSP_MIS_RXMIS			(1<<SSP_MIS_RXMIS_bit)
#define SSP_MIS_TXMIS_bit		3
#define SSP_MIS_TXMIS			(1<<SSP_MIS_TXMIS_bit)

/* SSP Interrupt Clear Register (ICR) */
#define SSP_ICR_RORIC_bit		0
#define SSP_ICR_RORIC			(1<<SSP_ICR_RORIC_bit)
#define SSP_ICR_RTIC_bit		1
#define SSP_ICR_RTIC			(1<<SSP_ICR_RTIC_bit)
#endif
