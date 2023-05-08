#include <errno.h>
#include <unicore-mx/cm3/scb.h>
#include <unicore-mx/stm32/rcc.h>
#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/usart.h>

#include "usart.h"
#include "led.h"

static char debugbuf[5000];
static int debugbuf_top = 0, debugbuf_bottom = 0;

void usart_setup(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	usart_set_baudrate(USART1, 3000000);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_enable(USART1);
}

#if 0
int _write(int file, char *ptr, int len) {
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART1, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}
#else
int _write(int file, char *ptr, int len) {
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++) {
			debugbuf[debugbuf_top] = ptr[i];
			debugbuf_top++;
			if(debugbuf_top >= sizeof(debugbuf))
				debugbuf_top = 0;
			if(debugbuf_top == debugbuf_bottom) {
				led_on();
				debugbuf_bottom++;
				if(debugbuf_bottom >= sizeof(debugbuf))
					debugbuf_bottom = 0;
			}
		}
		return i;
	}

	errno = EIO;
	return -1;
}
#endif
void usart_poll(void) {
	if((USART_SR(USART1) & USART_SR_TXE) != 0) {
		if(debugbuf_bottom != debugbuf_top) {
			usart_send(USART1, debugbuf[debugbuf_bottom]);
			debugbuf_bottom++;
			if(debugbuf_bottom >= sizeof(debugbuf))
				debugbuf_bottom = 0;
		}
	}
}

void usart_flush(void) {
	while(debugbuf_bottom != debugbuf_top)
		usart_poll();
}
