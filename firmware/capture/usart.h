void usart_setup(void);
void usart_poll(void);
void usart_flush(void);

#ifdef DEBUG
#include <unicore-mx/stm32/usart.h>
#include <stdio.h>
#include <errno.h>
int _write(int file, char *ptr, int len);
#define debugf(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define debugf(fmt, ...)
#endif

