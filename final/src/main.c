/**
  ******************************************************************************
  * @file    main.c
  * @author  Tianyu Gu
  * @version V0.1
  * @date    5-April-2017
  * @brief   Default main function.
  ******************************************************************************
*/


#include "peripheral.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "data.h"


void Error_Handler(char*);

osThreadId IdleTaskHandle;
void StartIdleTask(void const*);

uint16_t u16_rawHP;

int main(void)
{
	NVIC_Init();
	RCC_Init();
	GPIO_Init();
	TIM_Init();
	DAC_Init();
	ADC_Init();

	// DAC start
	HAL_TIM_Base_Start(&htim6);  // start TIM6
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) hr_data, 965, DAC_ALIGN_8B_R);  // start DAC in DMA mode
	// ADC start
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&u16_rawHP, 1);  // start ADC in DMA mode


	osThreadDef(idleTask, StartIdleTask, osPriorityIdle, 0, 128);
	IdleTaskHandle = osThreadCreate(osThread(idleTask), NULL);


	/* Start scheduler */
	osKernelStart();


	for(;;);
}


/* StartDefaultTask function */
void StartIdleTask(void const * argument)
{
  for(;;)
  {
	BSP_LED_Toggle(LED5);
    osDelay(500);
  }
}


/*
 * @brief EXTI line detection callback.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (BSP_PB_GetState(BUTTON_USER)==GPIO_PIN_SET) {
		BSP_LED_Toggle(LED3);
	}
}


void Error_Handler(char* msg) {
	while (1) {
	}
}
