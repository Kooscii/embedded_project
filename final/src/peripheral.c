#include "peripheral.h"

extern void error_handler(char*);

void RCC_Init(void) {
	RCC_OscInitTypeDef RCC_OscInit;
	RCC_ClkInitTypeDef RCC_ClkInit;

	// some necessary setting before init rcc
//	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	// 4'b pre, 0'b sub

	// init the oscillators
	RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInit.HSEState = RCC_HSE_BYPASS;
	RCC_OscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInit.HSIState = RCC_HSI_ON;
	RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInit.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInit) != HAL_OK) {
		error_handler("Osc Init Fails");
	}

	// init the clocks
	RCC_ClkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_2) != HAL_OK) {
		error_handler("Clk Init Fails");
	}

	// init systick(HAL)
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

}

void GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	// GPIO clock enable
	__HAL_RCC_GPIOE_CLK_ENABLE();

	// configure led pin
	GPIO_InitStruct.Pin =  LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN
						 | LED7_PIN | LED8_PIN | LED9_PIN | LED10_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}
