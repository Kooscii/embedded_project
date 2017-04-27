#include "peripheral.h"

extern void error_handler(char*);

/*
 * RCC Initialization
 */
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
	if (HAL_RCC_OscConfig(&RCC_OscInit) != HAL_OK)
		error_handler("Osc Init Fails");

	// init the clocks
	RCC_ClkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_2) != HAL_OK)
		error_handler("Clk Init Fails");

	// init systick(HAL)
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

}

/*
 * Board GPIO Initialization (Button, LEDs)
 */
void GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_Init;

	// GPIO clock enable
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;

	// Configure led pin
	GPIO_Init.Pin = LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN | LED7_PIN
			| LED8_PIN | LED9_PIN | LED10_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_Init);

	// Configure user button
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

/*
 * TIM6 Initialization
 */
void TIM6_Init(void)
{
	TIM_HandleTypeDef htim6;

  TIM_MasterConfigTypeDef TIM_MasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)

    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/*
 * DAC Initialization
 */
void DAC_Init(DAC_HandleTypeDef * hdac) {
	GPIO_InitTypeDef GPIO_Init;
	DMA_HandleTypeDef hdma;
	DAC_ChannelConfTypeDef DAC_ChannelConf;

	// Configure DAC GPIO
	GPIO_Init.Pin = DAC_OUT1_PIN;
	GPIO_Init.Mode = GPIO_MODE_ANALOG;
	GPIO_Init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DAC_OUT1_GPIO_PORT, &GPIO_Init);

	// Configure DMA2 CH3 for DAC
	hdma.Instance = DMA2_Channel3;
	hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma.Init.MemInc = DMA_MINC_ENABLE;
	hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma.Init.Mode = DMA_CIRCULAR;
	hdma.Init.Priority = DMA_PRIORITY_LOW;
	hdma.Parent = hdac;
	if (HAL_DMA_Init(&hdma) != HAL_OK)
		error_handler("DMA for DAC Init Fails");

	// Configure DAC
	__HAL_RCC_DAC1_CLK_ENABLE();

	hdac->Instance = DAC;
	hdac->DMA_Handle1 = hdma;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
		error_handler("DAC Init Fails");

	DAC_ChannelConf.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	DAC_ChannelConf.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &DAC_ChannelConf, DAC_CHANNEL_1) != HAL_OK)
		error_handler("DAC Init Fails");
}
