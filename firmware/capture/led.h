#define LED_PORT GPIOE
#define LED_RCC_PERIPH RCC_GPIOE
#define LED_PIN GPIO0
#define LED_INVERTED 1 // uncomment if anode is tied to Vcc
#define LED_BLINK_DELAY 0x80000

void led_setup(void);
void led_on(void);
void led_off(void);
void led_blink_forever(void);
