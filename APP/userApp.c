#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "uart.h"
#include "ucos/includes.h"
#include "interrupts.h"

extern void PUT32 ( unsigned int, unsigned int );
extern unsigned int GET32 ( unsigned int );

#define GPBASE 0x20200000

#define GPSET0 0x2020001C
#define GPCLR0 0x20200028
#define GPLEV0 0x20200034


static volatile uint32_t *gpio_addr = (uint32_t *)0x20200000;
#define INP_GPIO(g) *(gpio_addr + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)   *(gpio_addr + ((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_READ(g)  (*(gpio_addr + 13) & (1<<(g)))
#define GPIO_SET  *(gpio_addr + 7)
#define GPIO_CLR  *(gpio_addr + 10)

#define MAXTIMINGS 	85
#define DHTPIN 		9
#define HIGH		1
#define LOW		0
int dht11_dat[5] = {0, 0, 0, 0, 0};

static uint32_t dht_mask = 1 << DHTPIN;


void read_dht11_dat();
unsigned int wtf_delay(unsigned int micro_second);

void userApp2(void * args)
{
    int i;
    INP_GPIO(16);
    OUT_GPIO(16);
	while(1)
	{
        //DisableInterrupts();
		read_dht11_dat();
        //EnableInterrupts();

		PUT32(GPCLR0, 1 << 16);
		OSTimeDly(1000);
		uart_string("in userApp2");
		OSTimeDly(1000);
	}
}

#define LOOPS_PER_MICROSEC 3 
unsigned int wtf_delay(unsigned int micro_second){
	unsigned int count = 0, i;
	for(i = 0; i < micro_second * LOOPS_PER_MICROSEC; i++){
		count++;
	}
	return count;
}
	


unsigned int row_count = 0;
uint8_t digitalRead(int pin_num){
    pin_num &= 63;

    uint32_t * read_addr; 
    if(pin_num < 32)
        read_addr = (uint32_t *) 0x20200034;
    else
        read_addr = (uint32_t *) 0x20200038;

    pin_num &= 31;

    /*
    uart_string("reading: ");
    hexstring(*(read_addr));
    */
    uint32_t res = *read_addr & (dht_mask);
/*
    if(row_count < 7){
        hexstrings( res > 0 );
        row_count++;
    }
    else{
        hexstring( res  > 0);
        row_count = 0;
    }
    */

    if( res != 0)
        return HIGH;
    else
        return LOW;
}


void print_dht(){
    uint32_t * read_addr = (uint32_t *) 0x20200034;
    if(row_count < 7){
        row_count++;
        hexstrings( (*read_addr & dht_mask)  > 0);
    }
    else{
        row_count = 0;
        hexstring( (*read_addr & dht_mask)  > 0);
    }
}

void read_dht11_dat()
{
	uint8_t laststate	= HIGH;
	uint8_t counter		= 0;
	uint8_t j		= 0, i;
	float	f; 
    uint32_t * read_addr = (uint32_t *) 0x20200034;
    uint32_t dht_mask = 1 << DHTPIN;
 
	// init dht11_dat to ZERO
	dht11_dat[0] = dht11_dat[1] = dht11_dat[2] = dht11_dat[3] = dht11_dat[4] = 0;

	/* host send start signal */
	// pinMode( DHTPIN, OUTPUT );
    INP_GPIO(DHTPIN);
    OUT_GPIO(DHTPIN); 

    //uart_string("bef clr: ");
    //print_dht();
	// digitalWrite( DHTPIN, LOW );
    GPIO_CLR = dht_mask;

    //uart_string("aft clr: ");
    //print_dht();
    

    int wtf_delay_cnt = 0;
    while(1){
        if(wtf_delay_cnt == 2)
            break;
        OSTimeDly(10);
        wtf_delay_cnt++;
    }
        


	//delay( 18 );
	//wtf_delay(18000);
	// digitalWrite( DHTPIN, HIGH );

    //uart_string("bef set: ");
    //print_dht();
    GPIO_SET = dht_mask;
    //uart_string("aft set: ");
    //print_dht();
	//delayMicroseconds( 40 );
	wtf_delay(40);
 
    INP_GPIO(DHTPIN);
	// receive data
    DisableInterrupts();
	for ( i = 0; i < MAXTIMINGS; i++ )
	{
		counter = 0;
		while ( digitalRead(DHTPIN) == laststate )
		{
			counter++;
			//delayMicroseconds( 1 );
			wtf_delay(1);
			if ( counter == 255 )
			{
				break;
			}
		}
		laststate = digitalRead( DHTPIN );
 
		if ( counter == 255 )
			break;
 
		/* ilk 3 donusumu atla */
		if ( (i >= 4) && (i % 2 == 0) )
		{
			dht11_dat[j / 8] <<= 1;
			if ( counter > 16 )
				dht11_dat[j / 8] |= 1;
			j++;
		}
	}
    EnableInterrupts();

 
	if ( (j >= 40) &&
	     (dht11_dat[4] == ( (dht11_dat[0] + dht11_dat[1] + dht11_dat[2] + dht11_dat[3]) & 0xFF) ) )
	{
		f = dht11_dat[2] * 9. / 5. + 32;
		//printf( "Nem = %d.%d %% Sıcaklık = %d.%d *C (%.1f *F)\n",
	//		dht11_dat[0], dht11_dat[1], dht11_dat[2], dht11_dat[3], f );
		uart_string("Nem & Sicaklik: ");
		hexstrings(dht11_dat[0]);
		uart_string("\n");
		hexstrings(dht11_dat[2]);
		uart_string("\n");
	}else  {
		uart_string("data arriving.\n");
	}
}


char digit[10][8] =  //The increasing number
    {
        {0,0,0,0,0,0,1,1}, //0
        {1,0,0,1,1,1,1,1}, //1
        {0,0,1,0,0,1,0,1}, //2
        {0,0,0,0,1,1,0,1}, //3
        {1,0,0,1,1,0,0,1}, //4
        {0,1,0,0,1,0,0,1}, //5
        {0,1,0,0,0,0,0,1}, //6
        {0,0,0,1,1,1,1,1}, //7
        {0,0,0,0,0,0,0,1}, //8
        {0,0,0,0,1,0,0,1}  //9
    };

int node[4] = {8, 23, 18, 22};

int pin[8] = {
    25, //A
    10, //B
    17, //C
    3,  //D
    2,  //E
    24, //F
    27, //G
    4  //DP
};
    
void digitalWrite(int n_gpio, int level){
    if(level == LOW)
        GPIO_CLR = 1 << n_gpio;
    else
        GPIO_SET = 1 << n_gpio;
}

void userApp1(void * args)
{
    int n_pin;
    // set all the seg not to illume
    for (n_pin = 0; n_pin < 8; ++n_pin){
        INP_GPIO(pin[n_pin]);
        OUT_GPIO(pin[n_pin]);
        digitalWrite(pin[n_pin], HIGH);
    }

    // set all the node not to off 
    for(n_pin = 0; n_pin < 4; ++n_pin){
        INP_GPIO(node[n_pin]);
        OUT_GPIO(node[n_pin]);
        digitalWrite(node[n_pin], LOW);
    }

    INP_GPIO(16);
    OUT_GPIO(16);

    int i_node = 0;
    int disp_buffer[4] = {5, 0, 5, 0};
    int disp_num = 0;
    int last_dht11_dat0 = dht11_dat[0], last_dht11_dat2 = dht11_dat[2];

    int i, update_flag;

	while(1)
	{
        if(last_dht11_dat0 != dht11_dat[0] || last_dht11_dat2 != dht11_dat[2]){
            disp_buffer[1] = dht11_dat[0]%10;
            disp_buffer[0] = dht11_dat[0]/10;
            disp_buffer[3] = dht11_dat[2]%10;
            disp_buffer[2] = dht11_dat[2]/10;
        }

        last_dht11_dat0 = dht11_dat[0];
        last_dht11_dat2 = dht11_dat[2];

        digitalWrite(node[i_node], LOW);
        i_node = (i_node+1) % 4; 
        disp_num = disp_buffer[i_node];

        for(n_pin = 0; n_pin < 8; ++n_pin){
            digitalWrite(pin[n_pin], digit[disp_num][n_pin]);
        } 

        digitalWrite(node[i_node], HIGH);

        //wtf_delay(100);
        OSTimeDly(1);
	}
}

