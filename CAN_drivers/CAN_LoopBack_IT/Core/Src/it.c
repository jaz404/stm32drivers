/*
 * it.c
 *
 *      Albertalooop CANbus loopback IT test
 *      Created on: 06-25-2025
 *      Author: Jaspreet Chhabra
 *      Ref: kiran
 */

#include "main_app.h"

extern CAN_HandleTypeDef hcan1;

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

// Can get the names for the interrupt handler from the startup code
/**
  * @brief This function handles CAN_TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);

}

/**
  * @brief This function handles CAN_RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief This function handles CAN SCE interrupt.
  */
void CAN1_SCE_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);
}







