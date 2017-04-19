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


void error_handler(char*);

osThreadId IdleTaskHandle;
void StartIdleTask(void const*);


int main(void)
{
	RCC_Init();
	GPIO_Init();


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


void error_handler(char* msg) {
	while (1) {
	}
}
