#include "led.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void led_init() {
	rcc_periph_clock_enable(LED_RCC);
	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
	gpio_set(LED_PORT, LED_PIN);
}

static int led;
void led_on() {
	gpio_clear(LED_PORT, LED_PIN);
	led++;
}

void led_off() {
	led--;
	if(led <= 0) {
		led = 0;
		gpio_set(LED_PORT, LED_PIN);
	}
}

void led_blink_forever() {
	while(1) {
		gpio_toggle(LED_PORT, LED_PIN);
		for(int i = 0; i < 0x100000; i++)
			__asm__("nop");
	}
}
