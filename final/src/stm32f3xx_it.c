/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
//#include "stm32f3xx_hal.h"
//#include "stm32f3xx.h"
#include "peripheral.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f3xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

/*
 * @brief EXTI0 interrupt handler (user button)
 */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

#ifdef TIM6_INT_ENABLE
/*
 * TIM6 and DAC IRQHandler
 */
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac);
}
#endif

#ifdef DMA2CH3_INT_ENABLE
void DMA2_Channel3_IRQHandler(void) {
	HAL_DMA_IRQHandler(&hdma_dac);
}
#endif
