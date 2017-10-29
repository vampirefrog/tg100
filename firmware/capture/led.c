#include <unicore-mx/stm32/rcc.h>
#include <unicore-mx/stm32/gpio.h>

#include "led.h"

static int led_state = 0;

void led_setup() {
	rcc_periph_clock_enable(LED_RCC_PERIPH);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
#ifdef LED_INVERTED
	gpio_set(LED_PORT, LED_PIN);
#else
	gpio_clear(LED_PORT, LED_PIN);
#endif
}

void led_on() {
	led_state++;
#ifdef LED_INVERTED
	gpio_clear(LED_PORT, LED_PIN);
#else
	gpio_set(LED_PORT, LED_PIN);
#endif
}

void led_off() {
	led_state--;
	if(led_state <= 0) {
		led_state = 0;
#ifdef LED_INVERTED
		gpio_set(LED_PORT, LED_PIN);
#else
		gpio_clear(LED_PORT, LED_PIN);
#endif
	}
}

void led_blink_forever() {
	while(1) {
		gpio_toggle(LED_PORT, LED_PIN);
		for(int i = 0; i < 0x800000; i++)
			__asm__("nop");
	}
}
