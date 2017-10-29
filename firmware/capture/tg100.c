#include <string.h>
#include <unicore-mx/stm32/rcc.h>
#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/timer.h>
#include <unicore-mx/cm3/nvic.h>
#include <unicore-mx/stm32/exti.h>
#include <unicore-mx/stm32/dma.h>

#include "led.h"
#include "cdcacm.h"
#include "usart.h"

void dma_setup(void);
void timer_setup(void);

#define DMABUFSIZE 16
uint16_t dmabuf[DMABUFSIZE], dmabuf2[DMABUFSIZE];
uint32_t timbuf[DMABUFSIZE], timbuf2[DMABUFSIZE];

void dma_setup(void) {
	// Fill them in, to check if anything is being written at all
	memset(dmabuf, 0xdd, sizeof(dmabuf));
	memset(dmabuf2, 0xcc, sizeof(dmabuf2));
	memset(timbuf, 0xbb, sizeof(timbuf));
	memset(timbuf2, 0xaa, sizeof(timbuf2));

	rcc_periph_clock_enable(RCC_DMA2);

	nvic_enable_irq(NVIC_DMA2_STREAM4_IRQ);
	DMA_SPAR(DMA2, DMA_STREAM4) = &GPIOD_IDR;  // Read 16 bits from GPIOD port
	DMA_SM0AR(DMA2, DMA_STREAM4) = dmabuf;
	DMA_SM1AR(DMA2, DMA_STREAM4) = dmabuf2;    // Double buffering
	DMA_SNDTR(DMA2, DMA_STREAM4) = DMABUFSIZE; // Set number of words to read
//	DMA_SFCR(DMA2, DMA_STREAM4) = DMA_SxFCR_FTH_4_4_FULL | DMA_SxFCR_DMDIS;
	DMA_SCR(DMA2, DMA_STREAM4) = 0
		| DMA_SxCR_CHSEL(6)                    // Stream 1, Channel 6 = TIM1_CH1
		| DMA_SxCR_DBM                         // Enable double buffering
		| DMA_SxCR_PL_VERY_HIGH                     // High priority
		| DMA_SxCR_PSIZE_16BIT                 // Read 16 bits, not 8
		| DMA_SxCR_MSIZE_16BIT                 // Increment 16 bits
		| DMA_SxCR_MINC                        // Increment memory location
		| DMA_SxCR_CIRC                        // Circular mode
		| DMA_SxCR_DIR_PERIPHERAL_TO_MEM
		| DMA_SxCR_TCIE                        // Enable Transfer Complete Interrupt
		| DMA_SxCR_EN                          // Enable this DMA stream
		;

	nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);
	DMA_SPAR(DMA2, DMA_STREAM7) = &TIM5_CNT;
	DMA_SM0AR(DMA2, DMA_STREAM7) = timbuf;
	DMA_SM1AR(DMA2, DMA_STREAM7) = timbuf2;
	DMA_SNDTR(DMA2, DMA_STREAM7) = DMABUFSIZE;
//	DMA_SFCR(DMA2, DMA_STREAM7) = DMA_SxFCR_FTH_4_4_FULL | DMA_SxFCR_DMDIS;
	DMA_SCR(DMA2, DMA_STREAM7) = 0
		| DMA_SxCR_CHSEL(7)                    // Stream 7, Channel 7 = TIM8_TRG
		| DMA_SxCR_DBM
		| DMA_SxCR_PL_HIGH
		| DMA_SxCR_PSIZE_32BIT
		| DMA_SxCR_MSIZE_32BIT
		| DMA_SxCR_MINC
		| DMA_SxCR_CIRC
		| DMA_SxCR_DIR_PERIPHERAL_TO_MEM
		| DMA_SxCR_TCIE                        // Enable Transfer Complete Interrupt
		| DMA_SxCR_EN
		;

	// Setup timer as a slave to Timer 4
	rcc_periph_clock_enable(RCC_TIM1);
	TIM1_DIER = TIM_DIER_TDE;
	TIM1_SMCR = TIM_SMCR_SMS_ECM1 | TIM_SMCR_TS_ITR3; // TIM1 ITR3 = TIM4_TRGO
	TIM1_CR1 = TIM_CR1_CEN;

	rcc_periph_clock_enable(RCC_TIM8);
	TIM8_DIER = TIM_DIER_TDE;
	TIM8_SMCR = TIM_SMCR_SMS_ECM1 | TIM_SMCR_TS_ITR2; // TIM8 ITR2 = TIM4_TRGO
	TIM8_CR1 = TIM_CR1_CEN;

	// Setup Timer 4 capture and compare mode for channel 2
	// It is used to trigger DMA above
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOB, GPIO_AF2, GPIO7); // PB7 Alternate Function 2 = TIM4_CH2

	rcc_periph_clock_enable(RCC_TIM4);
	TIM4_CCMR1 = TIM_CCMR1_CC1S_IN_TI2;
	TIM4_CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;
	TIM4_DIER = 0;
	TIM4_SMCR = TIM_SMCR_SMS_ECM1 | TIM_SMCR_TS_TI2FP2;
	TIM4_CR2 = TIM_CR2_MMS_COMPARE_PULSE;
	TIM4_CR1 = TIM_CR1_CEN;
}

static bool balls = false;
void dma2_stream7_isr(void) {
	if(dma_get_interrupt_flag(DMA2, DMA_STREAM7, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA2, DMA_STREAM7, DMA_TCIF);
		gpio_set(GPIOC, GPIO7);

		while(!balls)
			;

		uint16_t *readybuf = (DMA_SCR(DMA2, DMA_STREAM4) & DMA_SxCR_CT) ? dmabuf2 : dmabuf;
		uint32_t *timerbuf = (DMA_SCR(DMA2, DMA_STREAM7) & DMA_SxCR_CT) ? timbuf2 : timbuf;

// 0x61 nn nn
// 0x62 735
// 0x63 882
// 0x7n n+1

		uint8_t buf[] = { 0x61, 0xb5 };
		static uint32_t lasttim = 0;
		cdcacm_write(buf, sizeof(buf));
		uint8_t buf2[4];
		for(int i = 0; i < DMABUFSIZE; i++) {
			uint16_t b = readybuf[i];
			uint32_t tim;
			tim = timerbuf[i] - lasttim;
			lasttim = timerbuf[i];
			if(tim > 0xffff) tim = 0xffff;
			buf2[0] = tim;
			buf2[1] = tim >> 8;
			buf2[2] = (b >> 8) & 0x0f;
			buf2[3] = b;
			cdcacm_write(buf2, sizeof(buf2));  // send data
		}

		gpio_clear(GPIOC, GPIO7);
		balls = false;
	}
}

void dma2_stream4_isr(void) {
	if(dma_get_interrupt_flag(DMA2, DMA_STREAM4, DMA_TCIF)) {
		dma_clear_interrupt_flags(DMA2, DMA_STREAM4, DMA_TCIF);
		gpio_set(GPIOC, GPIO8);

		balls = true;

		gpio_clear(GPIOC, GPIO8);
	}
}

void timer_setup(void) {
	// TIM2 and TIM5 have 32-bit counters
	rcc_periph_clock_enable(RCC_TIM5);
	timer_reset(TIM5);
	TIM5_ARR = 0xffffffff;
	TIM5_PSC = 3810;
	TIM5_CNT = 0xffffffff;
	TIM5_CR1 = TIM_CR1_CEN;
}

int main(void) {
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	// Just some status pin to probe with the scope
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7 | GPIO8);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO7 | GPIO8);

	led_setup();
	cdcacm_setup();
	usart_setup();
	dma_setup();

	// The data read port
	// D0-D7, A0-A3, /RD, /CS
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO15);
	gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO14 | GPIO15);

	timer_setup();

	while(1) {
		cdcacm_poll();
		usart_poll();
	}
}
