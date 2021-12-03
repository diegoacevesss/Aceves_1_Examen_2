/**
 * @file    Practica_2.c
 * @brief   Application entry point.
 * Author: Alejandro Avina & Diego Aceves
 */
#include <stdio.h>
#include "MK64F12.h"
#include "fsl_clock.h"

#define ESC				0x1B
#define ENTER			0x0D
#define SYSTEM_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define DELAY_INIT	1500000U
#define UART_BAUDRATE 115200

//Inicio del menu
uint8_t g_vt100_0[] = "\033[0;30;41m";
/*VT100 command for clearing the screen*/
uint8_t g_vt100_1[] = "\033[2J";
/** VT100 command for positioning the cursor in x and y position*/
uint8_t g_vt100_2[] = "\033[10;10H";
uint8_t g_vt100_3[] = "1) Introducir secuencia de LEDS \r";
/** VT100 command for positioning the cursor in x and y position*/
uint8_t g_vt100_4[] = "\033[11;10H";

//Colors
uint8_t blue = 'Z';
uint8_t red = 'R';
uint8_t green = 'V';

//Counters
uint8_t counter = 0;
uint8_t cont_rec_sec = 0;

typedef struct{
	uint8_t flag; /** Flag to indicate that there is new data*/
	uint8_t mail_box; /** it contains the received data*/
} uart_mail_box_t;

uart_mail_box_t g_mail_box_uart_0 ={0, 0};
void (*arreglo_seq[10])(void);
void (*secuencia_inicio[10])(void) = {Azul_ON, Rojo_ON, Verde_ON};

void Funcion_recorrer_arreglo_sec(void); //Funcion de la sacada practica

void Funcion_recorrer_arreglo_sec(void){
	if(10<contador_seq)
		contador_seq = 10;
	else{
		if(contador_seq == cont_rec_sec)
		{
			cont_rec_sec=0;
		}
		if(contador_seq != cont_rec_sec)
		{
			arreglo_seq[cont_rec_sec]();
			cont_rec_sec++;
		}
	}
}

void Blue_on(); //Funcion para azul
void Red_on(); //Funcion para rojo
void Green_on(); //Funcion para verde

void Blue_on(){
	GPIO_PortClear(GPIOB, 0 << 22);
	GPIO_PortClear(GPIOE, 0 << 26);
	GPIO_PortSet(GPIOB, 0 << 21);
}

void Red_on(){
	GPIO_PortClear(GPIOB, 0 << 21);
	GPIO_PortClear(GPIOE, 0 << 26);
	GPIO_PortSet(GPIOB, 0 << 22);
}

void Green_on(){
	GPIO_PortClear(GPIOB, 0 << 22);
	GPIO_PortClear(GPIOB, 0 << 21);
	GPIO_PortSet(GPIOE, 0 << 26);
}


void UART0_IRQHandler(void)
{
	/* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART0))
    {
    	g_mail_box_uart_0.mail_box = UART_ReadByte(UART0);
    	g_mail_box_uart_0.flag = TRUE;

    }
    SDK_ISR_EXIT_BARRIER;
}

//De la tarea nos traemos esto.
void driver_uart (void){
	uart_config_t config;
	uint32_t uart_clock;

	CLOCK_EnableClock(kCLOCK_PortB);

	PORT_SetPinMux(PORTB, 16U, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTB, 17U, kPORT_MuxAlt3);

	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = UART_BAUDRATE;
	config.enableTx     = TRUE;
	config.enableRx     = TRUE;

	uart_clock = CLOCK_GetFreq(UART0_CLK_SRC);

	UART_Init(UART0, &config, uart_clock);

	/* Enable RX interrupt. */
	UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);

	EnableIRQ(UART0_IRQ);
	//NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_6);
	EnableIRQ(PIT0_IRQn);
	NVIC_global_enable_interrupts;
}


void main(){
	/* Structure of initialize PIT */
	pit_config_t pitConfig;
	 /* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput,
		0,
	};

	CLOCK_SetSimSafeDivs();
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortE);
	driver_uart ();
	/* PORTB22 (pin 68) is configured as PTB22 */
	PORT_SetPinMux(PORTB, 22U, kPORT_MuxAsGpio);
	/* PORTB22 (pin 68) is configured as PTB22 */
	PORT_SetPinMux(PORTB, 21U, kPORT_MuxAsGpio);
	/* PORTB22 (pin 68) is configured as PTB22 */
	PORT_SetPinMux(PORTE, 26U, kPORT_MuxAsGpio);

	 /* Init output LED GPIO. */
	GPIO_PinInit(GPIOB, 22U, &led_config);
	GPIO_PinInit(GPIOB, 21U, &led_config);
	GPIO_PinInit(GPIOE, 26U, &led_config);


	GPIO_PortClear(GPIOB, 0 << 22);
	GPIO_PortClear(GPIOB, 0 << 21);
	GPIO_PortClear(GPIOE, 0 << 26);

	/* Encendemos el PIT */
	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);
	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(DELAY_INIT, SYSTEM_CLOCK));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	UART_WriteBlocking(UART0, g_vt100_0, sizeof(g_vt100_0) / sizeof(g_vt100_0[0]));
	UART_WriteBlocking(UART0, g_vt100_1, sizeof(g_vt100_1) / sizeof(g_vt100_1[0]));
	UART_WriteBlocking(UART0, g_vt100_2, sizeof(g_vt100_2) / sizeof(g_vt100_2[0]));
	UART_WriteBlocking(UART0, g_vt100_3, sizeof(g_vt100_3) / sizeof(g_vt100_3[0]));
	UART_WriteBlocking(UART0, g_vt100_4, sizeof(g_vt100_4) / sizeof(g_vt100_4[0]));
}
