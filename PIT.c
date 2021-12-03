/*
 * PIT.c
 *
 *  Created on: 27 sep 2021
 *  @author: diegoacevess  & Alejandro Covarrubias
 */

#include "MK64F12.h"
#include "Bits.h"
#include "PIT.h"

/**An interruption has ocurred*/
volatile uint32_t dummy_read = 0;
static uint8_t g_pit0_intrrupt_status_flag = 0;
static uint8_t g_pit1_intrrupt_status_flag = 0;
static uint8_t g_pit2_intrrupt_status_flag = 0;
static uint8_t g_pit3_intrrupt_status_flag = 0;

/*Callbacks for each channel*/
static void (*PIT_CH0_callback)(void) = 0;
static void (*PIT_CH1_callback)(void) = 0;
static void (*PIT_CH2_callback)(void) = 0;
static void (*PIT_CH3_callback)(void) = 0;

void PIT_delay(PIT_timer_t pit_timer, My_float_pit_t system_clock , My_float_pit_t delay)
{
		My_float_pit_t PIT_frq = system_clock/2; /**PIT  FRQ*/

		//Value to load into LDVAL trigger//
		uint32_t trigger = (uint32_t)(delay/(1/PIT_frq))-1;
		PIT->CHANNEL[pit_timer].LDVAL = trigger;
		PIT_enable_interrupt(pit_timer);
		PIT->CHANNEL[pit_timer].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void PIT_enable(void)
{
	PIT->MCR &= ~(PIT_MCR_MDIS_MASK); /**Enables Timer*/
	PIT->MCR |= PIT_MCR_FRZ_MASK; /**Stopping the PIT when enter in Debug mode*/
}

void PIT_stop(PIT_timer_t pit_timer)
{

	switch (pit_timer)
	{
		case PIT_0:
		PIT->CHANNEL[0].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		case PIT_1:
		PIT->CHANNEL[1].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		case PIT_2:
		PIT->CHANNEL[2].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		case PIT_3:
		PIT->CHANNEL[3].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		default:
		break;
	}

	return;

} /* PIT_stop() */

void PIT_clock_gating(void)
{
	SIM->SCGC6 = SIM_SCGC6_PIT_MASK;
}

uint8_t PIT_get_interrupt_flag_status(PIT_timer_t pit)
{
	uint8_t flag_status=0;
	//Select flag status of a specific channel
	//!These flags are programmer flags not hardware flags
		switch (pit)
			{
				case PIT_0:
					flag_status = g_pit0_intrrupt_status_flag;
					break;
				case PIT_1:
					flag_status = g_pit1_intrrupt_status_flag;
					break;
				case PIT_2:
					flag_status = g_pit2_intrrupt_status_flag;
					break;
				case PIT_3:
					flag_status = g_pit3_intrrupt_status_flag;
					break;
				default:
					break;
		}
		//Return flag status of selected channel
	return flag_status;
}/* PIT_get_interrupt_flag_status() */

void PIT_clear_interrupt_flag(PIT_timer_t pit){

	uint32_t dummyRead=0;
	//Clear the hardware flag of a channel
		switch (pit)
			{
				case PIT_0:
					PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
					g_pit0_intrrupt_status_flag = 0;
					dummyRead = PIT->CHANNEL[0].TCTRL;//read control register for clear PIT flag, this is silicon bug
					break;
				case PIT_1:
					PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
					g_pit1_intrrupt_status_flag = 0;
					dummyRead = PIT->CHANNEL[1].TCTRL;//read control register for clear PIT flag, this is silicon bug
					break;
				case PIT_2:
					PIT->CHANNEL[2].TFLG |= PIT_TFLG_TIF_MASK;
					g_pit2_intrrupt_status_flag = 0;
					dummyRead = PIT->CHANNEL[2].TCTRL;//read control register for clear PIT flag, this is silicon bug
					break;
				case PIT_3:
					PIT->CHANNEL[3].TFLG |= PIT_TFLG_TIF_MASK;
					g_pit3_intrrupt_status_flag = 0;
					dummyRead = PIT->CHANNEL[3].TCTRL;//read control register for clear PIT flag, this is silicon bug
					break;
				default:
					break;
			}
}/* PIT_clear_interrupt_flag() */


void PIT_enable_interrupt(PIT_timer_t pit)
{
	//Enable interrupt of a specific channel
		switch (pit)
			{
				case PIT_0:
					PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK;
					break;
				case PIT_1:
					PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK;
					break;
				case PIT_2:
					PIT->CHANNEL[2].TCTRL = PIT_TCTRL_TIE_MASK;
					break;
				case PIT_3:
					PIT->CHANNEL[3].TCTRL = PIT_TCTRL_TIE_MASK;
					break;
				default:
					break;
		}
}/* PIT_enable_interrupt() */

void PIT_callback_init(PIT_timer_t pit, void(*handler)(void))
{
	//Set the handler of the specific channel interrupt
	switch (pit)
		{
			case PIT_0:
				PIT_CH0_callback = handler;
				break;
			case PIT_1:
				PIT_CH1_callback = handler;
				break;
			case PIT_2:
				PIT_CH2_callback = handler;
				break;
			case PIT_3:
				PIT_CH3_callback = handler;
				break;
			default:
				break;
		}
}/*PIT_callback_init */

void PIT0_IRQHandler(void)
{
	if(PIT_CH0_callback)
	{
		PIT_CH0_callback();
	}
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	g_pit0_intrrupt_status_flag = TRUE;
	dummy_read = PIT->CHANNEL[0].TCTRL;     //read control register for clear PIT flag, this is silicon bug
}

void PIT_timer_enable(void)
{
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;/**Clock enable in PIT*/
}

