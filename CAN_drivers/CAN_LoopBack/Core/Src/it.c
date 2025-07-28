/*
 * it.c
 *      Albertalooop CANbus loopback mode test
 *      Created on: 05-30-2025
 *      Author: Jaspreet Chhabra
 *      Ref: kiran (FastBit Embedded Brain Academy)
 */

#include "main_app.h"

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}



