/* ===========================================================================
 * my-esp features
 * ===========================================================================
 * - Uses the NEC Infrared Transmission Protocol to receive IR codes:
 *     http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
 *
 * - Uses I2S to control the color of the LEDs in a ws2812_i2s strip.
 *
 *
 */


// #include <esp8266.h>
// #include <config.h>

#include "user_config.h"
#include "mem.h"
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "ip_addr.h"
#include "driver/uart.h"
#include "pwm.h"
#include "espconn.h"
#include "user_interface.h"
#include "ws2812_i2s.h"

/* ====================================== */
/* UART                                   */
/* ====================================== */

#define DBG  uart1_sendStr_no_wait // mrv

#define SYNC_PATTERN_SIZE		(3)

static uint16_t sync_idx = 0;

const char rx_sync_pattern[] = "mrv";

static uint8_t uart_input_buff[256] = {};

/* ====================================== */
/* TCP/UDP CONNECTIONS                    */
/* ====================================== */

static struct espconn *pUdpServer;

static struct espconn *pTcpConn;
uint8 server_ip_address[4] = { 184, 106, 153, 149 };

static void ICACHE_FLASH_ATTR
tcpConnCb(void *arg);

static void ICACHE_FLASH_ATTR
tcpSentCb(void *arg);

static void ICACHE_FLASH_ATTR
tcpRecvCb(void *arg, char *pData, unsigned short len);

static void ICACHE_FLASH_ATTR
tcpReconnCb(void *arg, sint8 err);

static void ICACHE_FLASH_ATTR
tcpDisconnCb(void *arg);


/* ====================================== */
/* WIFI                         	  	  */
/* ====================================== */

struct station_config stconf;


/* ====================================== */
/* SOFTWARE TIMER                         */
/* ====================================== */

#define user_procTaskPrio		0
#define user_procTaskQueueLen  	1

#define RELAY_PIN				BIT5

os_event_t    user_procTaskQueue[user_procTaskQueueLen];
static void user_procTask(os_event_t *events);

static volatile os_timer_t some_timer;

static volatile os_timer_t wifi_setup_timer;

static volatile os_timer_t post_timer;

/*
 * An ADC measurement is made every time this timer triggers.
 * The ADC input is connected to a:
 * SparkFun MEMS Microphone Breakout - INMP401 (ADMP401)
 * https://www.sparkfun.com/products/9868
 *
 * This callback also contains the logic that simulates an
 * equalizer by changing the LED colors of a ws2812 LED strip
 */
void softwareTimerCallback(void *arg)
{
    /* ====================================== */
	/* ADC	                         	  	  */
	/* ====================================== */

	uint16 i;

    /* Current ADC value (volume level of the most recent ADC measurement)*/
	uint16 adc_value;

	/* Previous ADC value (volume level of previous ADC capture)*/
	static uint16 prev_adc_value;

	/* The absolute value of the difference of the current and previous ADC values */
	uint16 abs_val_diff = 0;

	/* Color in RGB space */
	rgb my_rgb;

	/* Color in HSV space */
	hsv my_hsv;

	/* TODO: make rainbow when no volume */
	static int count = 0;

	/* defines the number of ws2812 LEDs */
	static int num_leds = 30;

	/* Changes of volume below the threshold are ignored */
	static int threshold = 20;

	/* Controls the gradual LED color change animation */
	static bool lock = false;

	/* Controls the heatmap of the LEDs */
	static int heatmap = 0;

	/* Buffer that stores the colors of each LED in the strip */
	uint8_t led_out[num_leds * 3];

	const uint16 red_up_num_cycles = 25;
	static bool count_up = true;

	/* Init saturation and value for HSV color */
	my_hsv.s = 1.0;
	my_hsv.v = 0.1;

	/* Read ADC */
	adc_value = system_adc_read();

	/* Calculate the absolute value of the previous and current ADC values */
	abs_val_diff = ( adc_value > prev_adc_value ) ? adc_value - prev_adc_value : prev_adc_value - adc_value;

	/* Save adc_value */
	prev_adc_value = adc_value;

	/* Gradual color change animation taking place?
	 * OR
	 * Is the measured value larger than the last value
	 * that triggered the animation?*/
	if ( lock == false || abs_val_diff > heatmap )
	{
#if 0
		os_printf("\r\nheatmap: %d", heatmap);
#endif

		/* Is the measured value larger than the threshold? */
		if ( abs_val_diff >= threshold )
		{
			/* Trigger detected! Start animation! */
			lock = true;

			/* Save the value that triggered the animation */
			heatmap = abs_val_diff;

			/* Reset important variables used in the animation */
			count_up = true;
			count = 0;
		}
	}
	else
	{
		/* No, then continue with animation */
	}


	/************************************
	 * GRADUAL LED COLOR CHANGE ANIMATION
	 ************************************/

	/* Enter only if an animation was triggered above */
	if ( lock == true )
	{
		/* Are we 'heating up' or 'cooling down'? */
		if ( count_up == true )
		{
			/* Heat up (faster than cooling down) */
			count += 3;

			count_up = (count >= red_up_num_cycles) ? false : true ;
		}
		else
		{
			/* Cool down */
			count--;

			if ( count <= 2 )
			{
				/* Stop gradual color change */
				lock = false;
			}
		}

#if 0
		os_printf("\r\ncount: %d", count);
#endif

		/* Set color for each LED */
		for (i = 0; i < num_leds; i++)
		{
			if ( i == 0 )
			{
				/* The first LED is always set to blue */
				my_hsv.h = 240.0;
			}
			else
			{
				/* Decrease hue depending on the change in volume */
				my_hsv.h = (uint16)my_hsv.h - (uint16)(heatmap * 0.005 * count);
			}


			/* Clip hue if below 0 */
			my_hsv.h = my_hsv.h < 0 ? 0 : my_hsv.h;

			/* Convert color from HSV to RGB space */
			my_rgb = hsv2rgb(my_hsv);

			/* Store RGB color in the output buffer */
			led_out[i * 3 + 0] = my_rgb.g * 255;
			led_out[i * 3 + 1] = my_rgb.r * 255;
			led_out[i * 3 + 2] = my_rgb.b * 255;
		}

		ws2812_push(led_out, sizeof(led_out));

	#if 0
			os_printf("\r\nG R B: %x %x %x", led_out[0], led_out[1],led_out[2]);
	//		os_printf("\r\nhue: %d", (int)my_hsv.h);
	#endif

	#if 0
	//	os_printf("\r\nsystem_adc_read: %x", adc_value);
	//	os_printf("\r\ndiff_abs_val: %x", diff_abs_val);
	#endif
	}

	return;
}

/* ====================================== */
/* HARDWARE TIMER                         */
/* ====================================== */

#define FRC1_ENABLE_TIMER  BIT7
#define FRC1_AUTO_LOAD  BIT6

//TIMER PREDIVED MODE
typedef enum {
    DIVDED_BY_1 = 0,	//timer clock
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

/* this array will contain the raw IR message */
static uint32 	rawIrMsg[100] = {0};
static uint32 	rawIrMsgLen = 0;

/* this variable will contain the decoded IR command */
static uint32 	irCmd = 0;

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
	int logicState = 1;
	int logicStateLen = 0;
	bool repeatCode = false;

	/* stop the HW TIMER */
	RTC_REG_WRITE(FRC1_CTRL_ADDRESS, DIVDED_BY_16 | TM_EDGE_INT);

	//Set GPIO0 to LOW
	gpio_output_set(0, BIT0, BIT0, 0);

	/* load the HW TIMER for next IR message frame */
	uint32 ticks = usToTicks(70000);
	RTC_REG_WRITE(FRC1_LOAD_ADDRESS, ticks);

	/* derive the raw IR message frame */
	for( i = 0 ; i < ( edgeIndex - 1 ) ; i++)
	{
		/* find number of bits in current interval */
		logicStateLen = ( intervalArr[i] / minInterval );

		for( j = 0 ; j < logicStateLen ; j++)
		{
			rawIrMsg[ rawIrMsgLen ] = logicState;
			rawIrMsgLen++;
		}

		/* toggle state */
		logicState ^= 1;

#if 0
		os_printf( "\r\nDuration of interval %d: %d us\r\n", i, intervalArr[i] );
#endif
	}

#if 1
	/* print the received raw IR message frame */
	os_printf( "\r\nRAW IR CODE: ");
	for ( i = 0 ; i < rawIrMsgLen ; i++ )
	{
		os_printf( "%d", rawIrMsg[i] );
	}
#endif

	/**********************************************
	 * DECODE NEC MESSAGE FRAME!
	 * - every message frame contains 32 coded bits
	 **********************************************/

	/* set index to the beginning of the coded bits */

	/* the message frame starts with a burst of 16 logic 1's, skip them all */
	i = 0 ;
	while ( rawIrMsg[i] == 1 ) i++;

	/* the message frame continues with a burst of 8 logic 0's, skip them all */
	j = 0;
	while (rawIrMsg[i] == 0)
	{
		i++;
		j++;
	}

	/* if the number of zeros is 4, then ignore the current message frame since
	 * it corresponds to a "REPEATED CODE".
	 */
	if ( j <= 4 )
	{
#if 1
		os_printf( "\r\nREPEATED CODE");
#endif
		repeatCode = true;
	}

	/* decode raw message only if it is not a repeat code */
	if (repeatCode == false)
	{
		/* at this point 'i' contains the index of the beginning of the encoded bits */

		/* decode raw message
		 * - [1][0][0][0] 	represents a '1'
		 * - [1][0]			represents a '0'
		 */
		irCmd = 0;
		for (j = 0; j < 32; j++)
		{
			if (rawIrMsg[i + 2] == 0)
			{
				/* it is a '1', so left shift a '1' */
				irCmd = (irCmd << 1) | 1;

				/* move to the beginning of the next encoded bit
				 * (increment i until next 1 in raw message frame)
				 */
				do {i++;} while ( rawIrMsg[i] == 0 );
			}
			else {
				/* it is a '0', so left shift a '0' */
				irCmd = irCmd << 1;

				/* move to the beginning of the next encoded bit */
				i += 2;
			}
		}

#if 1
		/* print the received IR cmd */
		os_printf("\r\nIR CMD: %x", irCmd);
#endif
	}

	/**********************************************
	 * END - DECODE NEC MESSAGE FRAME!
	 * - every message frame contains 32 coded bits
	 **********************************************/

	/* reset index */
	edgeIndex = 0;

#if 1
	os_printf("\r\nEnd of IR message frame\r\n");
#endif

	return;
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
		/* yes, and is it the first edge of the IR message frame? */
		if ( edgeIndex == 0 )
		{
			/* yes, then store counter value */
			prevTicks = RTC_REG_READ(FRC1_COUNT_ADDRESS);

			/* and start the HW TIMER */
			RTC_REG_WRITE(FRC1_CTRL_ADDRESS,
					DIVDED_BY_16 | FRC1_ENABLE_TIMER | TM_EDGE_INT);

			/* reset relevant variables */
			minInterval = 0xFFFFFFFF; // initialize to a very big number
			rawIrMsgLen = 0;

#if 1
			os_printf("\n\nBeginning of IR message frame detected");
#endif

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


//*****************************************************************************
//
// Appends a float to a string
// source: http://www.esp8266.com/viewtopic.php?f=6&t=2445
//
//*****************************************************************************

void ICACHE_FLASH_ATTR printFloat(float val, char *buff)
{
	char smallBuff[16];
	int val1 = (int) val;
	unsigned int val2;
	if (val < 0) {
		val2 = (int) (-100.0 * val) % 100;
	} else {
		val2 = (int) (100.0 * val) % 100;
	}
	os_sprintf(smallBuff, "%i.%02u", val1, val2);
	strcat(buff, smallBuff);
} //- See more at: http://www.esp8266.com/viewtopic.php?f=6&t=2445#sthash.O8yVW828.dpuf


/* ====================================== */
/* TCP CONNECTION                    	  */
/* ====================================== */


const char apiKey[] = "MJEMBUXY1DVNMWMY";

void startHttpRequestTimerCallback(void *arg)
{
	/* start TCP connection as client */
	sint8 err = espconn_connect(pTcpConn);
	if (err != 0)
	{
		os_printf("Error connecting to a TCP server: %d\n", err);
	}
}

/* ====================================== */
/* TCP SERVER CALLBACKS					  */
/* ====================================== */

static void ICACHE_FLASH_ATTR
tcpConnCb(void *arg)
{
	int i;
	os_printf("tcpConnCb\n");

	struct espconn* pCon = (struct espconn *)arg;
	espconn_regist_recvcb(pCon, tcpRecvCb);
	espconn_regist_disconcb(pCon, tcpDisconnCb);
	espconn_regist_sentcb(pCon, tcpSentCb);

	/* source: http://www.arduinesp.com/thingspeak */
	int res = 0;

	/* initialize as empty strings */
	uint8_t httpRequest[300] = "";
	uint8_t postStr[300] = "";
	uint8_t postStrLen[20] = "";

	uint8_t * pch = uart_input_buff;

	/* concatenate the message body (bottom part of the HTTP request) */
	os_strcat(postStr, apiKey);

	for ( i = 0 ; i < 8 ; i++ )
	{
		os_strcat(postStr, "&field");
		os_sprintf(postStrLen, "%d=", i + 1);
		os_strcat(postStr, postStrLen);

		/* locate each sample inside UART input buff and append it to the post string */
		/* samples are divided with "\n\r" substrings */
		while ( *pch != 0x0a) // 0x0a -> \n
		{
			/* get length of post string */
			uint16_t postStrLen = os_strlen(postStr);

			/* append character (overwrites the terminating NULL char) */
			postStr[ postStrLen ] = *pch;

			/* move the terminating NULL char 1 cell to the right */
			postStr[ postStrLen + 1 ] = 0;

			/* point to next char in UART input buffer */
			pch++;
		}

		/* point to the char that follow the "\n\r" divisor */
		pch += 1;
	}

	os_strcat(postStr, "\r\n\r\n");

	/* concatenate the request-line and headers (top part of the http request) */
	os_strcat(httpRequest, "POST /update HTTP/1.1\n");
	os_strcat(httpRequest, "Host: api.thingspeak.com\n");
	os_strcat(httpRequest, "Connection: close\n");
	os_strcat(httpRequest, "X-THINGSPEAKAPIKEY: ");
	os_strcat(httpRequest, apiKey);
	os_strcat(httpRequest, "\n");
	os_strcat(httpRequest, "Content-Type: application/x-www-form-urlencoded\n");
	os_strcat(httpRequest, "Content-Length: ");
	os_sprintf(postStrLen, "%i", os_strlen(postStr));
	os_strcat(httpRequest, postStrLen);
	os_strcat(httpRequest, "\n\n");
	os_strcat(httpRequest, postStr);

	/* print the complete http request */
	os_printf("%s\n", httpRequest);


	/* send the http post request */
	res = espconn_send(pCon, (uint8_t *)httpRequest, os_strlen(httpRequest));
	if (res != 0)
	{
		os_printf("espconn_send: error %d\n", res);
	}
	else
	{
		os_printf("espconn_send: OK %d\n", res);
	}
}

static void ICACHE_FLASH_ATTR
tcpRecvCb(void *arg, char *pData, unsigned short len)
{
	int i;

	os_printf("tcpRxCb\n");

	for ( i = 0 ; i < len ; i++)
	{
		os_printf("%c", pData[i]);
	}
}

static void ICACHE_FLASH_ATTR
tcpReconnCb(void *arg, sint8 err)
{
	int i;

	os_printf("tcpReconnCb\n");
	os_printf("%i\n",err);
}

static void ICACHE_FLASH_ATTR
tcpDisconnCb(void *arg)
{
	os_printf("tcpDisconnCb\n");
}

static void ICACHE_FLASH_ATTR
tcpSentCb(void *arg)
{
	os_printf("tcpSentCb\n");
}


/* ====================================== */
/* UART 								  */
/* ====================================== */

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : CheckSyncPattern                                            *
 *                                                                            *
 *  Description : checks weather a desired sequence of characters is received *
 *                by incrementing an index variable on each success. Whenever *
 *                an incorrect character is detected the index variable is    *
 *                reseted to 0. When the complete sequence has been matched   *
 *                the index will equal the size of the sync pattern           *
 *                                                                            *
 *  Input       : byte_read   the next byte to be checked with the patter     *
 *                                                                            *
 *  Output      :                                                             *
 *                                                                            *
 *----------------------------------------------------------------------------*/

static void CheckSyncPattern(uint8_t byte_read)
{
    /* does the incoming byte match the next letter in the pattern? */
    if(byte_read == rx_sync_pattern[sync_idx])
    {
        /* yes, then increase sync_index */
        sync_idx++;
    }
    else
    {
        /* no, but does it at least match the first letter in the pattern? */
        if(byte_read == rx_sync_pattern[0])
        {
           /* yes, then the following check should be done with the second letter */
            sync_idx = 1;
        }
        else
        {
            /* no, then the pattern check has to start over (with the first letter) */
          sync_idx = 0;

#ifdef USE_LP1
          if( byte_read != 0x00 )
          {
            /* reset flag that indicates that the processor was woken up by the UART */
            uartLp2Requested = false;
          }
#endif
        }
    }
    return;
}


//Called from UART.
void ICACHE_FLASH_ATTR charrx( uint8_t byte_read )
{
#if 0
	uart_tx_one_char(1, byte_read);
#endif

	static uint16_t payload_len = 0;

	/* did the sync pattern match already? */
	if ( sync_idx < SYNC_PATTERN_SIZE )
	{
		/* no, so read the incoming byte and continue with the sync pattern check process */
		CheckSyncPattern(byte_read);
	}
	/* yes, but was it thanks to the previous byte read? */
	else if ( payload_len == 0 )
	{
		/* yes, therefore the incoming byte contains the payload length */
		payload_len = byte_read;
		os_printf("payload_len: %d\n\r",payload_len);

		/* reset UART input buffer */
		uart_input_buff[0] = 0;
	}
	/* no, then the byte is part of the cmd. Continue reading the cmd from FIFO */
	else
	{
		/* store next byte */
		uart_input_buff[os_strlen(uart_input_buff)] = byte_read; // read byte from FIFO

		/* did we read all bytes already? */
		if ( payload_len == os_strlen(uart_input_buff) )
		{
			/* yes, then the payload was received completely */

			/* start an HTTP post request by starting a client TCP connection */
			sint8 err = espconn_connect(pTcpConn);
			if (err != 0)
			{
				os_printf("Error connecting to a TCP server: %d\n", err);
			}

			os_printf("String received:\n\r%s\n\r",uart_input_buff);

			/* restart sync pattern check */
			sync_idx = 0;

			/* reset payload lenght */
			payload_len = 0;
		}
	}
}

/* ====================================== */
/* WIFI                         	  	  */
/* ====================================== */

void wifiConnectTimerCb(void *arg)
{
	/* Do we have an IP already? */
	int status = wifi_station_get_connect_status();
	if( status != STATION_GOT_IP)
	{
		/* No, then connect to the WiFi station */
		wifi_station_disconnect();

		os_printf("Trying to connect to %s\n", stconf.ssid );

		/* connect to a WiFi station */
		wifi_station_connect();
	}
	else
	{
		/* yes, then disable the timer WiFi setup timer*/
		os_timer_disarm(&wifi_setup_timer);

		/* enable POST timer in periodic mode */
		os_timer_disarm((ETSTimer*)&post_timer);
		os_timer_setfn((ETSTimer*)&post_timer, (os_timer_func_t *) startHttpRequestTimerCallback, NULL);
		os_timer_arm((ETSTimer*)&post_timer, 10000, 0); // thingspeak needs minimum 15 sec delay between updates
	}
}


/* ====================================== */
/* OS TASK							      */
/* ====================================== */

static void ICACHE_FLASH_ATTR
user_procTask(os_event_t *events)
{
    os_delay_us(10);
    os_printf("OS task\n");

//    system_os_post(user_procTaskPrio, 0, 0 );
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
void user_init()
{
	// Initialize the GPIO subsystem.
	gpio_init();

	/* ====================================== */
	/* UART                                   */
	/* ====================================== */

	// Initialize UART0 and UART1
	/* NOTE: UART1 and I2S share same GPIO. Cannot use simultaneously. */
	uart_init( BIT_RATE_115200, BIT_RATE_115200 );

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

	/* PIN FUNCTION INIT FOR PWM OUTPUT */
	pwm_init(pwm_period, pwm_duty, PWM_CHANNEL, io_info);

	/* set pwm_duty cycle */
	pwm_set_duty (14185, 0);
	pwm_set_duty (22222, 1); // todo: explain why 22222 is the highest possible value

	/* start PWM */
	pwm_start(); // NOTE: PWM causes spikes in other GPIOs
#endif


	/* ====================================== */
	/* GPIO INTERRPUT                         */
	/* ====================================== */

	/* Set GPIO12 in GPIO mode */
	PIN_FUNC_SELECT( PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12 );

	/* Set GPIO12 as input */
	GPIO_DIS_OUTPUT( GPIO_ID_PIN(12) );

	/* Disable all GPIO interrupts */
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

	/* disarm timer */
	os_timer_disarm((ETSTimer*)&some_timer);

	/* set callback */
	os_timer_setfn((ETSTimer*)&some_timer, (os_timer_func_t *) softwareTimerCallback, NULL);

	/* arm the timer -> os_timer_arm(<pointer>, <period in ms>, <fire periodically>) */
	os_timer_arm((ETSTimer*)&some_timer, 10, 1);

	/* ====================================== */
	/* OS TASK                                */
	/* ====================================== */

	/* setup OS task */
//	system_os_task(user_procTask, user_procTaskPrio, user_procTaskQueue, user_procTaskQueueLen);

	/* send a message to OS task (fire task) */
//	system_os_post(user_procTaskPrio, 0, 0 );


	/* ====================================== */
	/* HARDWARE TIMER                         */
	/* ====================================== */

	/* The hardware timer is used to indicate when a complete IR message frame should have
	 * arrived in order to process the received data and calculate the IR command.
	 *
	 * It is configured in "one-shot" mode. It is started when the beginning of an
	 * IR message frame is detected and stopped after the complete message frame has been read.
	 * This means that the duration of the HW timer should be longer than the duration of
	 * the longest message frame. In the NEC IR tranmission protocol all message frames have
	 * a duration of approximately 67.5ms.
	 */

	/* load the HW TIMER */
	uint32 ticks = usToTicks(70000); // 70ms
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

	/* usage:	echo <data> | nc -wl -u <ip address> <port>
	 * example: echo "foo" | nc -w1 -u 192.168.1.187 7777 */

	/* allocate space for server */
	pUdpServer = (struct espconn *) os_zalloc(sizeof(struct espconn));

	/* clear allocated memory */
	ets_memset(pUdpServer, 0, sizeof(struct espconn));

	/* create the server */
	espconn_create(pUdpServer);

	/* set the type of server */
	pUdpServer->type = ESPCONN_UDP;

	/* allocate memory for UDP settings */
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

	wifi_set_opmode(STATION_MODE);

	wifi_station_get_config_default(&stconf);

//	os_strncpy((char*) stconf.ssid, "TP-LINK_2.4GHz_FC2E51", 32);
//	os_strncpy((char*) stconf.password, "tonytony", 64);

	os_strncpy((char*) stconf.ssid, "WLAN-PUB", 32);
	os_strncpy((char*) stconf.password, "", 64);

//	os_strncpy((char*) stconf.ssid, "MAD air", 32);
//	os_strncpy((char*) stconf.password, "glioninlog", 64);

	stconf.bssid_set = 0;
	wifi_station_set_config(&stconf);

//	/* ====================================== */
//	/* WS2812 LED STRIP                	  	  */
//	/* ====================================== */
//
//	/* NOTE: UART1 and I2S share same GPIO. Cannot use simultaneously. */
//	ws2812_init();
//
//	/*						G		R		B			*/
//	uint8_t ledout[] = 	{
//							0xff,	0x00,	0x00, 		//4th
////							0xff,	0x00,	0x00,		//3rd
////							0x00,	0xff,	0x00,		//2nd
////							0x00,	0x00,	0xff, 		//1st
//						};
//
//#if 0
//		os_printf("\r\nB R G: %x %x %x\r\n", ledout[0], ledout[1],ledout[2]);
//#endif
//
//	ws2812_push( ledout, sizeof( ledout ) );

	/* ====================================== */
	/* TCP CONNECTION                    	  */
	/* ====================================== */

	/* allocate space for server */
	pTcpConn = (struct espconn *) os_zalloc(sizeof(struct espconn));

	/* clear allocated memory */
	ets_memset(pTcpConn, 0, sizeof(struct espconn));

	/* set the type of connection */
	pTcpConn->type = ESPCONN_TCP;

	/* set state to NONE */
	pTcpConn->state = ESPCONN_NONE;

	/* allocate memory for TCP settings */
	pTcpConn->proto.tcp = (esp_tcp *) os_zalloc(sizeof(esp_tcp));

	/* set the port that the connection will be listening to */
	pTcpConn->proto.tcp->local_port = espconn_port();

	/* set the remote port and IP address */
	pTcpConn->proto.tcp->remote_port = 80;
	os_memcpy(pTcpConn->proto.tcp->remote_ip, server_ip_address, sizeof(server_ip_address));

	/* register callbacks */
	espconn_regist_connectcb(pTcpConn, tcpConnCb);
	espconn_regist_reconcb(pTcpConn, tcpReconnCb);

	/* disarm timer */
	os_timer_disarm((ETSTimer*)&wifi_setup_timer);

	/* set callback */
	os_timer_setfn((ETSTimer*)&wifi_setup_timer, (os_timer_func_t *) wifiConnectTimerCb, NULL);

	/* arm the timer -> os_timer_arm(<pointer>, <period in ms>, <fire periodically>) */
	os_timer_arm((ETSTimer*)&wifi_setup_timer, 5000, 1);

	return;

}
