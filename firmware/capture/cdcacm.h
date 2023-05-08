#ifndef CDCACM_H_
#define CDCACM_H_

void cdcacm_setup(void);
void cdcacm_poll(void);
void cdcacm_write(uint8_t *buf, int length);

#endif /* CDCACM_H_ */
