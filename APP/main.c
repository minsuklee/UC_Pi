#include "uart.h"
#include "regs.h"
#include "timer.h"
#include "ucos/includes.h"
//#include "rpi.h"

OS_STK  userAppTaskStk1[1000];
OS_STK  userAppTaskStk2[1000];

extern void userApp1(void *);
extern void userApp2(void *);
extern unsigned int wtf_delay(unsigned int micro_second);

void main()
{
	uart_init();

	InitInterruptController();

	DisableInterrupts();

	timer_init();

    /*
    if(map_peripheral(&gpio) == -1) {
        uart_string("Failed to map the physical GPIO registers into the virtual memory space.\n");
	    return;
    }
    */

	OSInit();

    /*
	int i = 0; 
	while(1){
		wtf_delay(1000000);
        hexstring(i);
		uart_string(" second?\n");
        ++i;
	}
    */

	OSTaskCreate(userApp1, (void *) 0, &userAppTaskStk1[1000-1],5);

	OSTaskCreate(userApp2, (void *) 0, &userAppTaskStk2[1000-1],6);

	OSStart();

	while(1);
}
