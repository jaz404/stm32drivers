/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Aug 4, 2025
 *      Author: jaspr
 */

#include "stm32f466xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}

}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes GPIO peripheral
 *
 * @param[in]         - GPIO pin handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0; //temp. register

	 //enable the peripheral clock

	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// configure the Mode of the pin

	// Handle non interrupt case
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// calculating the correct offset, mul by 2 since each pin occupies 2 bits
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );

		// masking out (clears) the mode bits for pin 2, leaving all other bits untouched.
		// 0x3 = 11 shifting this by the offset
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing

		// use OR to set
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}
	// Handle Interrupt case
	else
	{
		// interrupt cases (FT, RT, RFT)
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR - we do not want both to be configured if one was configured before
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR - we do not want both to be configured if one was configured before
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// configure the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// configure GPIO port selection in SYS_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2);

		// enable EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// similar code
	// configure the Speed of the pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// configure the push / pull setting
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	// configure the output type
	// no need to shift by 2 since only one bit required by each
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		// since AF is divided into AF1 and AF2
		// AF1 is for pins 0-7 and AF2 for pins 8-15
		// temp1 decides 1 or 2
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		// temp2 decides which pin in AFx
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		// mul by 4 since each pin requires 4 bits
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the value from the mentioned GPIO pin
 *
 * @param[in]         - GPIO pin register definition structure
 * @param[in]         -	Pin number
 * @param[in]         -
 *
 * @return            -  returns a uint8_t value
 *
 * @Note              -  none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); // right shift by pin number and then mask
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads the value from the mentioned GPIO port
 *
 * @param[in]         - GPIO pin register definition structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  returns a uint16_t value
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes the value to the mentioned GPIO pin
 *
 * @param[in]         - GPIO pin register definition structure
 * @param[in]         -	pin number
 * @param[in]         - value
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET){
		// set the mentioned pin to 1
		pGPIOx->ODR |= (1 << PinNumber); 	// using OR is necessary to avoid setting other bits
		// 1 << PinNumber shifts 1 to left by PinNumber so 1 << 5 will be 10000 and then OR with ODR
	}
	else {
		// set the mentioned pin to 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes the value to the mentioned GPIO port
 *
 * @param[in]         - GPIO pin register definition structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles mentioned GPIO pin
 *
 * @param[in]         - GPIO pin register definition structure
 * @param[in]         - pin number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber); // 1 ^ 1 = 0, 0 ^ 1 = 1
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Deinitializes the provided port
 *
 * @param[in]         - GPIO pin register definition structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - initializes IRQ on the processor side by setting processor's registers
 * 						Refer to Table 4-2 NVIC register summary, generic guide, Cortex - M4
 * 						registers of interest: NVIC_ISER (Interrupt set-enable registers),
 * 						NVIC_ICER (Interrupt Clear-enable Registers)
 *
 * @param[in]         - IRQ number
 * @param[in]         - Enable or Disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		if (IRQNumber <=31){
			// NVIC_ISER0 register (handles 0-31 IRQ numbers)
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber <64){
			// NVIC_ISER1 register (handles 32-63 IRQ numbers)
			*NVIC_ISER1 |= (1<<(IRQNumber%32));	// mod 32 to restart numbering in reg
		}
		else if (IRQNumber >=64 && IRQNumber <96){
			// NVIC_ISER2 register (handles 64-96 IRQ numbers)
			// we do not have to go beyond ISER2 since we do not have IRQ>81 in F466xx
			*NVIC_ISER2 |= (1<<(IRQNumber%32));
		}
	}
	else {
		// disable
		if (IRQNumber <=31){
			// NVIC_ICER0 register (handles 0-31 IRQ numbers)
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if (IRQNumber >31 && IRQNumber <64){
			// NVIC_ICER1 register (handles 32-63 IRQ numbers)
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
		}
		else if (IRQNumber >=64 && IRQNumber <96){
			// NVIC_ICER2 register (handles 64-96 IRQ numbers)
			// we do not have to go beyond ICER2 since we do not have IRQ>81 in F466xx
			*NVIC_ISER2 |= (1<<(IRQNumber%32));
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Initializes the IRQ priority associated with the IRQ number
 *
 * @param[in]         - IRQNumber
 * @param[in]         - IRQPriority
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	// find the relevant IPR register
	uint8_t temp1 = IRQNumber/4;		// IPRx
	uint8_t temp2 = IRQNumber%4;		// IPRx_section, where in the register
	uint8_t shift_amt = (8*temp2) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (temp1*4)) |= (IRQPriority << shift_amt); //
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - clears the Pending Register Bit corresponding to the pin  number
 *
 * @param[in]         - GPIO pin register definition structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber){
	// clear the EXTI pending register corresponding to the pin number
	if (EXTI->PR && (1<<PinNumber)) {
		//clear
		EXTI->PR |= (1<<PinNumber);
	}
}

