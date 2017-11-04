#ifndef LED_H_
#define LED_H_

#define LED_RCC RCC_GPIOC
#define LED_PORT GPIOC
#define LED_PIN GPIO13

void led_init(void);
void led_on(void);
void led_off(void);
void led_blink_forever(void);

#endif /* LED_H_ */
