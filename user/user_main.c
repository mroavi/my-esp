/* ===========================================================================
 * my-esp features
 * ===========================================================================
 * Uses the NEC Infrared Transmission Protocol to receive IR codes:
 * 	http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
 *
 *
 *
 */


#include <esp8266.h>
#include <config.h>

#include <user_config.h>
#include "mem.h"
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_config.h"
#include "ip_addr.h"
#include "uart.h"
#include "pwm.h"
#include "espconn.h"
#include "user_interface.h"



/* ====================================== */
/* SOFTWARE TIMER                         */
/* ====================================== */

#define user_procTaskPrio		0
#define user_procTaskQueueLen  	1

#define RELAY_PIN				BIT5

os_event_t    user_procTaskQueue[user_procTaskQueueLen];
static void user_procTask(os_event_t *events);

static volatile os_timer_t some_timer;

static struct espconn *pUdpServer;

void some_timerfunc(void *arg)
{
    // Toggle PIN
    if (GPIO_REG_READ(GPIO_OUT_ADDRESS) & RELAY_PIN)
    {
        //Set GPIO0 to LOW
//        gpio_output_set(0, BIT0, BIT0, 0);
    }
    else
    {
        //Set GPIO0 to HIGH
//        gpio_output_set(BIT0, 0, BIT0, 0);
    }
}

/* ====================================== */
/* HARDWARE TIMER                         */
/* ====================================== */

#define FRC1_ENABLE_TIMER  BIT7
#define FRC1_AUTO_LOAD  BIT6

//TIMER PREDIVED MODE
typedef enum {
    DIVDED_BY_1 = 0,		//timer clock
    DIVDED_BY_16 = 4,	//divided by 16
    DIVDED_BY_256 = 8,	//divided by 256
} TIMER_PREDIVED_MODE;

typedef enum {			//timer interrupt mode
    TM_LEVEL_INT = 1,	// level interrupt
    TM_EDGE_INT   = 0,	//edge interrupt
} TIMER_INT_MODE;

typedef enum {
    FRC1_SOURCE = 0,
    NMI_SOURCE = 1,
} FRC1_TIMER_SOURCE_TYPE;


static uint32 	intervalArr[70] = {0};
static uint32	edgeIndex = 0;

/* initialize to a very big number */
static uint32	minInterval = 0xFFFFFFFF;

/* this array will contain the final IR code */
static uint32 irCode[100] = {0};
static uint32 irCodeLen = 0;


/* assumes timer clk of 5MHz */
static uint32 usToTicks( uint32_t us )
{
	return ( us * 10) >> 1;
}

/* assumes timer clk of 5MHz */
static uint32 ticksToUs( uint32_t ticks )
{
	return ( ticks << 1 ) / 10;
}


/* ====================================== */
/* HARDWARE TIMER                         */
/* ====================================== */

void hwTimerCallback( void )
{
	int i, j;
	int state = 1;
	int stateLen = 0;

	/* stop the HW TIMER */
	RTC_REG_WRITE(FRC1_CTRL_ADDRESS, DIVDED_BY_16 | TM_EDGE_INT);

	//Set GPIO0 to LOW
	gpio_output_set(0, BIT0, BIT0, 0);

	/* load the HW TIMER for next IR code */
	uint32 ticks = usToTicks(70000);
	RTC_REG_WRITE(FRC1_LOAD_ADDRESS, ticks);

	/* derive the IR code */
	for( i = 0 ; i < ( edgeIndex - 1 ) ; i++)
	{
		/* find number of bits in current interval */
		stateLen = ( intervalArr[i] / minInterval );

		for( j = 0 ; j < stateLen ; j++)
		{
			irCode[ irCodeLen ] = state;
			irCodeLen++;
		}

		/* toggle state */
		state ^= 1;

//		os_printf( "\r\nDuration of interval %d: %d us\r\n", i, intervalArr[i] );
	}

	/* print the received IR code */
	os_printf( "\r\nIR CODE: ");
	for ( i = 0 ; i < irCodeLen ; i++ )
	{
		os_printf( "%d", irCode[i] );
	}

	/* reset index */
	edgeIndex = 0;

	os_printf("\r\nEnd of RF code\r\n");
}

/* ====================================== */
/* GPIO INTERRPUT                         */
/* ====================================== */

void gpioCallback(void *arg)
{
	uint16 			gpio_status = 0;
	static uint32	currTicks = 0;
	static uint32	prevTicks = 0;

	/* get the GPIO interrupt status */
	gpio_status = GPIO_REG_READ( GPIO_STATUS_ADDRESS );

	/* clear the interrupt */
	GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, gpio_status);

	/* did GPIO 12 (connected to IR receiver) generate the ISR? */
	if( gpio_status == BIT(12) )
	{
		/* yes, and is it the first edge of the IR code? */
		if ( edgeIndex == 0 )
		{
			/* yes, then store counter value */
			prevTicks = RTC_REG_READ(FRC1_COUNT_ADDRESS);

			/* and start the HW TIMER */
			RTC_REG_WRITE(FRC1_CTRL_ADDRESS,
					DIVDED_BY_16 | FRC1_ENABLE_TIMER | TM_EDGE_INT);

			/* reset relevant variables */
			minInterval = 0xFFFFFFFF;
			irCodeLen = 0;

			os_printf("\n\nBeginning of IR code detected");

			//Set GPIO0 to HIGH
			gpio_output_set( BIT0, 0, BIT0, 0 );
		}
		else
		{
			/* record time of current edge */
			currTicks = RTC_REG_READ( FRC1_COUNT_ADDRESS );

			/* record time interval between current edge and previous edge */
			intervalArr[ edgeIndex - 1 ] = ticksToUs( prevTicks - currTicks );

			/* keep track the shortest interval */
			minInterval = ( intervalArr[ edgeIndex - 1 ] < minInterval ) ? intervalArr[ edgeIndex - 1 ] : minInterval;

			/* save time of current edge */
			prevTicks = currTicks;
		}

		edgeIndex++;
	}
}


/* ====================================== */
/* SOFTWARE TIMER                         */
/* ====================================== */

static void ICACHE_FLASH_ATTR
user_procTask(os_event_t *events)
{
    os_delay_us(10);
}


void ICACHE_FLASH_ATTR charrx( uint8_t c )
{
	//Called from UART.
}


/* ====================================== */
/* UDP SERVER							  */
/* ====================================== */

/* usage: echo "foo" | nc -w1 -u 192.168.1.187 7777 */

static void ICACHE_FLASH_ATTR
udpServerRxCb(void *arg, char *pData, unsigned short len)
{
	int i;

	for ( i = 0 ; i < len ; i++)
	{
		os_printf("%c", pData[i]);
	}

	/* Toggle PIN */
	if (GPIO_REG_READ(GPIO_OUT_ADDRESS) & RELAY_PIN)
	{
		//Set GPIO0 to LOW
		gpio_output_set(0, RELAY_PIN, BIT0, 0);
	}
	else
	{
		//Set GPIO0 to HIGH
		gpio_output_set(RELAY_PIN, 0, RELAY_PIN, 0);
	}
}



// initialize the custom stuff that goes beyond esp-link
void app_init()
{
	// Initialize the GPIO subsystem.
	gpio_init();

	/* ====================================== */
	/* UART                                   */
	/* ====================================== */

	// Initialize UART0 and UART1
//	uart_init_2( BIT_RATE_115200, BIT_RATE_115200 );

//	uart0_sendStr( "\nUART0 - USED TO PROGRAM THE MODULE\n" );

	os_printf("\n===================\nUART1 - DEBUG OUPUT\n===================\n");

	/* NOTE: PWM CANNOT BE USED SIMULTANEOUSLY WITH HW TIMER */
#if 0
	/* ====================================== */
	/* PWM                                    */
	/* ====================================== */

    uint32  pwm_period = 1000;
	uint32 pwm_duty[PWM_CHANNEL] = {0};


    uint32 io_info[][3] = {   {PWM_0_OUT_IO_MUX,PWM_0_OUT_IO_FUNC,PWM_0_OUT_IO_NUM},
    		                  {PWM_1_OUT_IO_MUX,PWM_1_OUT_IO_FUNC,PWM_1_OUT_IO_NUM},
    		              };

	/*PIN FUNCTION INIT FOR PWM OUTPUT*/
	pwm_init(pwm_period, pwm_duty, PWM_CHANNEL, io_info);

	/* Set pwm_duty cycle */
	pwm_set_duty (14185, 0);
	pwm_set_duty (22222, 1); // todo: explain why 22222 is the highest possible value

	/* Start PWM */
	pwm_start(); // NOTE: PWM causes spikes in other GPIOs
#endif


	/* ====================================== */
	/* GPIO INTERRPUT                         */
	/* ====================================== */

	/* Set GPIO12 as GPIO */
	PIN_FUNC_SELECT( PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12 );

	/* Set GPIO12 as input */
	GPIO_DIS_OUTPUT( GPIO_ID_PIN(12) );

	/* Disable all IO interrupts */
	ETS_GPIO_INTR_DISABLE();

	/* Set a GPIO callback function */
	ETS_GPIO_INTR_ATTACH( gpioCallback, NULL );

	/* Configure the type of edge */
	gpio_pin_intr_state_set( GPIO_ID_PIN(12), GPIO_PIN_INTR_ANYEDGE );

	ETS_GPIO_INTR_ENABLE();


	/* ====================================== */
	/* SOFTWARE TIMER                         */
	/* ====================================== */

	// Set GPIO0 to output mode
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);

	//Set GPIO0 low
	gpio_output_set(0, RELAY_PIN, RELAY_PIN, 0);

	//Disarm timer
	os_timer_disarm((ETSTimer*)&some_timer);

	//Setup timer
	os_timer_setfn((ETSTimer*)&some_timer, (os_timer_func_t *) some_timerfunc, NULL);

	//Arm the timer
	//&some_timer is the pointer
	//1000 is the fire time in ms
	//0 for once and 1 for repeating
	os_timer_arm((ETSTimer*)&some_timer, 1000, 1);

	//Start os task
	system_os_task(user_procTask, user_procTaskPrio, user_procTaskQueue,
	user_procTaskQueueLen);


	/* ====================================== */
	/* HARDWARE TIMER                         */
	/* ====================================== */

	/* load the HW TIMER */
	uint32 ticks = usToTicks(70000);
	RTC_REG_WRITE(FRC1_LOAD_ADDRESS, ticks);

	/* register callback function */
	ETS_FRC_TIMER1_INTR_ATTACH( hwTimerCallback, NULL );

	/* enable interrupts */
	TM1_EDGE_INT_ENABLE();
	ETS_FRC1_INTR_ENABLE();

	/* don't start timer yet */
	/* the timer is started inside the GPIO INT callback */


	/* ====================================== */
	/* UDP SERVER                         	  */
	/* ====================================== */

	/* usage: echo "foo" | nc -w1 -u 192.168.1.187 7777 */

	/* allocate space for server */
	pUdpServer = (struct espconn *) os_zalloc(sizeof(struct espconn));

	/* reset allocated memory to 0 */
	ets_memset(pUdpServer, 0, sizeof(struct espconn));

	/* create the UDP server */
	espconn_create(pUdpServer);

	/* set the type of server */
	pUdpServer->type = ESPCONN_UDP;

	/**/
	pUdpServer->proto.udp = (esp_udp *) os_zalloc(sizeof(esp_udp));

	/* set the port that the server will be listening to */
	pUdpServer->proto.udp->local_port = 7777;

	/* register the callback */
	espconn_regist_recvcb(pUdpServer, udpServerRxCb);

	/* start listening */
	if (espconn_create(pUdpServer))
	{
		while (1) { os_printf("Error creating a UDP server\n"); }
	}


	/* ====================================== */
	/* WIFI                         	  	  */
	/* ====================================== */

	struct station_config stconf;

	wifi_set_opmode(STATION_MODE);

	wifi_station_get_config_default(&stconf);

	os_strncpy((char*) stconf.ssid, "TP-LINK_2.4GHz_FC2E51", 32);
	os_strncpy((char*) stconf.password, "tonytony", 64);

	stconf.bssid_set = 0;
	wifi_station_set_config(&stconf);


	return;

}
