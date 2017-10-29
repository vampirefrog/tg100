#include "usart.h"
#include "led.h"

#include <unicore-mx/cm3/scb.h>
#include <unicore-mx/stm32/gpio.h>

void hard_fault_handler(void);

void hard_fault_handler(void) {
	debugf("\n\nHARD FAULT OCCURRED!\n\n");
	debugf("SCB HFSR:%08lx", SCB_HFSR);
	if(SCB_HFSR & SCB_HFSR_DEBUG_VT) debugf(" DEBUG_VT");
	if(SCB_HFSR & SCB_HFSR_FORCED) debugf(" FORCED");
	if(SCB_HFSR & SCB_HFSR_VECTTBL) debugf(" VECTTBL");
	debugf(" CFSR:%08lx", SCB_CFSR);
	if(SCB_CFSR & SCB_CFSR_DIVBYZERO) debugf(" DIVBYZERO");
	if(SCB_CFSR & SCB_CFSR_UNALIGNED) debugf(" UNALIGNED");
	if(SCB_CFSR & SCB_CFSR_NOCP) debugf(" NOCP");
	if(SCB_CFSR & SCB_CFSR_INVPC) debugf(" INVPC");
	if(SCB_CFSR & SCB_CFSR_INVSTATE) debugf(" INVSTATE");
	if(SCB_CFSR & SCB_CFSR_UNDEFINSTR) debugf(" UNDEFINSTR");
	if(SCB_CFSR & SCB_CFSR_BFARVALID) debugf(" BFARVALID");
	if(SCB_CFSR & SCB_CFSR_STKERR) debugf(" STKERR");
	if(SCB_CFSR & SCB_CFSR_UNSTKERR) debugf(" UNSTKERR");
	if(SCB_CFSR & SCB_CFSR_IMPRECISERR) debugf(" IMPRECISERR");
	if(SCB_CFSR & SCB_CFSR_PRECISERR) debugf(" PRECISERR");
	if(SCB_CFSR & SCB_CFSR_IBUSERR) debugf(" IBUSERR");
	if(SCB_CFSR & SCB_CFSR_MMARVALID) debugf(" MMARVALID");
	if(SCB_CFSR & SCB_CFSR_MSTKERR) debugf(" MSTKERR");
	if(SCB_CFSR & SCB_CFSR_MUNSTKERR) debugf(" MUNSTKERR");
	if(SCB_CFSR & SCB_CFSR_DACCVIOL) debugf(" DACCVIOL");
	if(SCB_CFSR & SCB_CFSR_IACCVIOL) debugf(" IACCVIOL");
	debugf(" BFAR:%08lx\n\n", SCB_BFAR);
//	usart_flush();
	led_blink_forever();
}
