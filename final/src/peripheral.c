#include "peripheral.h"

extern void Error_Handler(char*);

/*
 * NVIC Initialization
 */
void NVIC_Init(void) {
	__HAL_RCC_SYSCFG_CLK_ENABLE()
	;

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	// 4'b pre, 0'b sub

	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	/* BusFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	/* UsageFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/*
 * RCC Initialization
 */
void RCC_Init(void) {
	// some necessary setting before init rcc
//	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();

	// init the oscillators
	RCC_OscInitTypeDef RCC_OscInit;

	RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInit.HSEState = RCC_HSE_BYPASS;
	RCC_OscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInit.HSIState = RCC_HSI_ON;
	RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInit.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInit) != HAL_OK) {
		Error_Handler("Osc Init Fails");
	}

	// init the clocks
	RCC_ClkInitTypeDef RCC_ClkInit;

	RCC_ClkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler("Clk Init Fails");
	}

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
void TIM_Init(void) {
	// Enable Timer 6 clock
	__HAL_RCC_TIM6_CLK_ENABLE()
	;

	// Configure Timer 6
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 71;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 9999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler("TIM6 Init Fails");
	}

	// Master mode select to update event for DAC
	TIM_MasterConfigTypeDef TIM_MasterConfig;

	TIM_MasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	TIM_MasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &TIM_MasterConfig)
			!= HAL_OK) {
		Error_Handler("TIM6 Init Fails");
	}

#ifdef TIM6_INT_ENABLE
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
#endif
}

/*
 * DAC Initialization
 */
void DAC_Init(void) {
	// Configure DAC GPIO
	__RCC_DAC_GPIO_CLK_ENABLE()
	;

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = DAC_OUT1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DAC_OUT1_GPIO_PORT, &GPIO_InitStruct);

	// Configure DMA2 CH3 for DAC
	__HAL_RCC_DMA2_CLK_ENABLE()
	;

	hdma_dac.Instance = DMA2_Channel3;
	hdma_dac.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_dac.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_dac.Init.MemInc = DMA_MINC_ENABLE;
	hdma_dac.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_dac.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_dac.Init.Mode = DMA_CIRCULAR;
	hdma_dac.Init.Priority = DMA_PRIORITY_LOW;
	hdma_dac.Parent = &hdac;
	if (HAL_DMA_Init(&hdma_dac) != HAL_OK) {
		Error_Handler("DMA for DAC Init Fails");
	}

#ifdef DMA2CH3_INT_ENABLE
	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
#endif

	// Configure DAC
	__HAL_RCC_DAC1_CLK_ENABLE()
	;

	hdac.Instance = DAC;
	hdac.DMA_Handle1 = &hdma_dac;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler("DAC Init Fails");
	}

	// Configure DAC Channel1
	DAC_ChannelConfTypeDef DAC_ChannelConf;

	DAC_ChannelConf.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	DAC_ChannelConf.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &DAC_ChannelConf, DAC_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler("DAC Init Fails");
	}
}

/*
 * ADC Initialization
 */
void ADC_Init(void) {
	// Configure ADC GPIO
	__RCC_ADC_GPIO_CLK_ENABLE()
	;

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = ADC_IN7_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_IN7_GPIO_PORT, &GPIO_InitStruct);

	// Configure DMA for ADC1 IN7
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	hdma_adc1.Instance = DMA1_Channel1;
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc1.Init.Mode = DMA_CIRCULAR;
	hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
	hdma_adc1.Parent = &hadc1;
	if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
		Error_Handler("DMA for ADC Init Fails");
	}

#ifdef DMA1CH1_INT_ENABLE
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#endif

	/* Peripheral clock enable */
	__HAL_RCC_ADC12_CLK_ENABLE()
	;

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.DMA_Handle = &hdma_adc1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler("ADC1 Init Fails");
	}

	// Configure ADC
	ADC_ChannelConfTypeDef ADC_ChannelConf;

	ADC_ChannelConf.Channel = ADC_CHANNEL_7;
	ADC_ChannelConf.Rank = 1;
	ADC_ChannelConf.SingleDiff = ADC_SINGLE_ENDED;
	ADC_ChannelConf.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	ADC_ChannelConf.OffsetNumber = ADC_OFFSET_NONE;
	ADC_ChannelConf.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &ADC_ChannelConf) != HAL_OK) {
		Error_Handler("ADC1 Init Fails");
	}
}
