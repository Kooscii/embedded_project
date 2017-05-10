 /**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <MEMS.h>
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "data.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FLASH_BASE_ADDR 0x8030000
#define STEP_IDLE (0x01)
#define STEP_UPPER (0x01<<1)
#define STEP_LOWER (0x01<<2)
#define STEP_FIRST (0x01<<7)

#define HR_IDLE (0x01)
#define HR_RISING (0x01<<1)
#define HR_FALLING (0x01<<2)
#define HR_BLOCK (0x01<<3)
#define HR_ESTAB (0x01<<7)

uint32_t msTimCnt = 0;
uint8_t rawHR = 0;
uint8_t rawStep_rms = 0;
uint8_t filtStep_rms = 0;
float32_t f32Step_rms = 0;
uint8_t rawStep_x = 0;
uint8_t rawStep_y = 0;
uint8_t rawStep_z = 0;
uint8_t rawFlash = 0;
uint8_t rawUpdated = 0;

uint8_t outHR = 0;
uint8_t outSR = 0;
uint8_t txbuf[] = {0, 0, 0, 0, 0, 0, 0, '\n'};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
HAL_StatusTypeDef logData2Flash(uint64_t _data) {
	HAL_StatusTypeDef status = HAL_OK;
	static uint32_t _offset=0;

	if (_offset<6000) {
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_BASE_ADDR+_offset, _data);
		_offset += 2;
	}
	else {
		HAL_FLASH_Lock();
		HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, 0);
	}
	return status;
}

/*
 * Interrupt Callback
 */

/* UART Callback */
void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart) {
	UNUSED(huart);
}

/* ADC Convert Cplt Callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	int16_t x,y,z;
	int16_t acc[3];
	float32_t facc[3];
	float32_t f_acc_rms;
	static uint16_t offset = 0;

	/* get heart pulse data */
	HAL_ADC_Stop_IT(hadc);
	rawHR = (uint8_t) HAL_ADC_GetValue(hadc);

	/* get accelero data */
//	z = ACCELERO_MEMS_GetData(ACC_Z);
	BSP_ACCELERO_GetXYZ(acc);
	// calc rms
	facc[0] = (float32_t)acc[0];
	facc[1] = (float32_t)acc[1];
	facc[2] = (float32_t)acc[2];
	arm_rms_f32(facc, 3, &f_acc_rms);
	f32Step_rms = f_acc_rms*1.73205080756888;
	rawStep_rms = (((uint16_t)f_acc_rms+4096)>>5) & 0xff;
	// calc x y z
	rawStep_x = ((acc[0]+4096)>>5) & 0xff;
	rawStep_y = ((acc[1]+4096)>>5) & 0xff;
	rawStep_z = ((acc[2]+4096)>>5) & 0xff;

	/* get logged data in flash */
	rawFlash = *((uint16_t *)(FLASH_BASE_ADDR+offset));
	offset = (offset+2)%6000;

	rawUpdated = 1;
}

/* USER Button Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		UNUSED(GPIO_Pin);
	}
}

/* ADC TimeBase 6,7 Callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	static uint8_t cnt=0;
	if (htim->Instance == TIM7) {
		msTimCnt++;
	} else if (htim->Instance == TIM6) {
		if (cnt==0) {
			HAL_ADC_Start_IT(&hadc1);

			txbuf[0] = (rawHR=='\n')? rawHR+1: rawHR;
			txbuf[1] = (rawStep_x=='\n')? rawStep_x+1: rawStep_x;
			txbuf[2] = (rawStep_y=='\n')? rawStep_y+1: rawStep_y;
			txbuf[3] = (rawStep_z=='\n')? rawStep_z+1: rawStep_z;
			txbuf[4] = (filtStep_rms=='\n')? filtStep_rms + 1 : filtStep_rms;
			txbuf[5] = (outHR=='\n')? outHR + 1 : outHR;
			txbuf[6] = (outSR=='\n')? outSR + 1 : outSR;
			HAL_UART_Transmit_IT(&huart1, (uint8_t*) &txbuf, 8);

	//		txbuf[0] = rawStep_rms;
	////		txbuf[1] = rawStep_y;
	////		txbuf[2] = rawStep_z;
	//		HAL_UART_Transmit_IT(&huart1, (uint8_t*) &txbuf, 1);
		}
		cnt = (cnt+1)%2;

		if (HAL_GPIO_ReadPin(LD9_GPIO_Port, LD9_Pin)==1)
			logData2Flash(rawStep_rms);
		else
			HAL_FLASH_Lock();
	}
}

void u8minmax(uint8_t list[], uint8_t size, uint8_t len, uint8_t start_idx, uint8_t * min, uint8_t * max) {
	uint8_t i, j;

	*max = 0;
	*min = 255;
	for (i=0; i<len; i++) {
		j = start_idx>=i? start_idx-i: size-(i-start_idx);
		if (*max < list[j])
			*max = list[j];
		if (*min > list[j])
			*min = list[j];
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	FLASH_EraseInitTypeDef eri;
	uint32_t err;
	uint8_t i;
	uint8_t j;
	uint8_t k;

	// step part
	float32_t rawStep_stack[100] = {0};
	uint8_t rawStep_stackIdx = 100;
	float32_t rawStep_gval = 0;
	float32_t currStep_rms = 0;
	float32_t prevStep_rms = 0;
	uint8_t step_state = 0;
	uint8_t step_cross0 = 0;
	float32_t step_peak = 0;
	float32_t step_valley = 0;
	float32_t step_baseline = 0;
	float32_t step_threshold = 0.05;
	uint32_t step_mstick = 0;
	uint8_t step_periodidx = 0;
	float32_t step_period[3] = {1000};
	float32_t step_mean = 0;

	// heart part
	uint8_t rawHR_stack[100] = {0};
	uint8_t rawHR_stackIdx = 0;
	uint8_t currHR_val = 0;
	uint8_t prevHR_val = 255;
	uint8_t hr_state = 0;
	uint8_t hr_peak = 0;
	uint8_t hr_baseline = 0;
	uint8_t hr_threshold = 0;
	uint8_t hr_periodcnt = 0;
	float32_t hr_period[3] = {1000};
	uint32_t hr_mstick = 0;
	float32_t hr_std = 0;
	float32_t hr_mean = 0;
	uint8_t hr_datacnt = 0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 1);

  /*
   * check if entering data collecting mode
   * hold User button to enter
   */
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1) {
	HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, 1);
	HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, 1);
	HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, 1);

	/* erase flash */
	HAL_FLASH_Unlock();
	eri.NbPages = 4;
	eri.PageAddress=FLASH_BASE_ADDR;
	eri.TypeErase=FLASH_TYPEERASE_PAGES;
	HAL_FLASHEx_Erase(&eri, &err);

	for (i=0; i<30; i++) {
		HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		HAL_Delay(100);
	}

	HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, 0);
	HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, 1);
  }



	/* Init ACCELERO MEMS */
//  	BSP_ACCELERO_Reset();
	ACCELERO_MEMS_Init();

	/* Using DAC CH1 to simulate the heart pulse wave at PA4 */
	/* DO NOT MODIFY */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) hr_data, 965,
			DAC_ALIGN_8B_R);
//	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) FLASH_BASE_ADDR, 3000,
//				DAC_ALIGN_8B_R);

	// acc init
	rawStep_gval = 0;
	for (rawStep_stackIdx=99; rawStep_stackIdx>49; rawStep_stackIdx--) {
		while (!rawUpdated);
			rawStep_stack[rawStep_stackIdx] = f32Step_rms;
			rawUpdated = 0;
	}
	// Get the mean value of first 100 data. This value equals to 1g.
	arm_mean_f32(rawStep_stack+50, 50, &rawStep_gval);
	// Zero-centering the all raw values. (val / 1g - 1)
	arm_scale_f32(rawStep_stack+50, 1/rawStep_gval, rawStep_stack+50, 50);
	arm_offset_f32(rawStep_stack+50, -1, rawStep_stack+50, 50);

	step_state = STEP_IDLE;
	hr_state = HR_IDLE;

	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		// wait for raw data being updated
		while (!rawUpdated);

		/*
		 * FIR filter
		 */
		// put raw data into stack
		rawStep_stack[rawStep_stackIdx] = f32Step_rms/rawStep_gval-1.;
		// get filtered current step value
		arm_dot_prod_f32((float*) (rawStep_stack+rawStep_stackIdx), (float*) step_filter,
							51, &currStep_rms);
//		arm_mean_f32(rawStep_stack+rawStep_stackIdx, 51, &currStep_rms);
		filtStep_rms = ((( (uint16_t) ( (currStep_rms+1)*rawStep_gval) )+4096)>>5) & 0xff;
		// store raw step data history
		rawStep_stack[rawStep_stackIdx+50] = rawStep_stack[rawStep_stackIdx];
		// rawStep_stackIdx--
		rawStep_stackIdx = (rawStep_stackIdx + 49)%50;

		/*
		 * Steps detecting algorithm
		 */
		if (step_state & STEP_IDLE) {
			if (currStep_rms > step_baseline + step_threshold) {
				step_state = STEP_FIRST | STEP_UPPER;
				step_peak = currStep_rms;
				step_cross0 = 0;
			}
		}
		else if (step_state & STEP_UPPER) {
			if (currStep_rms > step_peak) {
				step_peak = currStep_rms;
				if (!(step_state & STEP_FIRST)) {
					step_baseline = (step_peak + step_valley)/2;
				}
			}
			else if (currStep_rms < step_baseline - step_threshold) {
				if (!(step_state & STEP_FIRST)) {
					step_period[step_periodidx] = (float32_t) (msTimCnt - step_mstick);
					step_periodidx = (step_periodidx+1)%3;
				}
				step_state = STEP_LOWER;
				step_valley = currStep_rms;
				step_baseline = (step_peak + step_valley)/2;
				step_cross0 = 0;
				step_mstick = msTimCnt;
				arm_mean_f32(step_period, 3, &step_mean);
				outSR = (uint8_t) (60000./step_mean);
			}
		}
		else if (step_state & STEP_LOWER) {
			if (currStep_rms < step_valley) {
				step_valley = currStep_rms;
				step_baseline = (step_peak + step_valley)/2;
			}
			else if (currStep_rms > step_baseline + step_threshold) {
				step_state = STEP_UPPER;
				step_peak = currStep_rms;
				step_baseline = (step_peak + step_valley)/2;
				step_cross0 = 0;
			}
		}

		if (!(step_state & STEP_IDLE) && ((currStep_rms * prevStep_rms <0) || (msTimCnt-step_mstick > 3000))) {
			step_cross0++;
			if (step_cross0 > 5) {
				step_state = STEP_IDLE;
				step_baseline = 0;
				step_period[0] = 1000;
				step_period[1] = 1000;
				step_period[2] = 1000;
				outSR = 0;
			}
		}

		prevStep_rms = currStep_rms;

		/*
		 * Heart rate algorithm
		 */
		currHR_val = rawHR;
		rawHR_stack[rawHR_stackIdx] = currHR_val;
		rawHR_stackIdx = (rawHR_stackIdx+1)%100;
		hr_datacnt = hr_datacnt<100? hr_datacnt+1: hr_datacnt;

		if (hr_state & HR_IDLE) {
//			outHR = 0;
			u8minmax(rawHR_stack, 100, hr_datacnt, rawHR_stackIdx, &hr_baseline, &hr_peak);
			hr_threshold = (uint8_t) (hr_peak-hr_baseline)/1.5;
			if ((hr_peak - hr_baseline > 30) && (currHR_val > hr_baseline + hr_threshold)) {
				hr_state = HR_FALLING;
				hr_mstick = msTimCnt;
				hr_periodcnt = 0;
			}
		}
		else if (hr_state & HR_RISING) {
			if ((float32_t) (msTimCnt - hr_mstick) > hr_mean*0.3 && currHR_val > hr_baseline + hr_threshold) {
				hr_state = HR_FALLING | (hr_state & HR_ESTAB);
				hr_period[hr_periodcnt-1] = (float32_t) (msTimCnt-hr_mstick);
				hr_mstick = msTimCnt;
			}
			else if (msTimCnt - hr_mstick > 2000) {
				hr_state = HR_IDLE;
				outHR = 0;
			}
		}
		else if (hr_state & HR_FALLING) {
			if (currHR_val < hr_peak - hr_threshold) {
				if (!(hr_state & HR_ESTAB)) {
					hr_state = HR_RISING;
					if (hr_periodcnt) {
						hr_period[hr_periodcnt-1] += (float32_t) (msTimCnt-hr_mstick);
						if (hr_periodcnt == 3) {
							arm_std_f32(hr_period, 3, &hr_std);
							arm_mean_f32(hr_period, 3, &hr_mean);
							if (hr_mean > 250 && hr_mean < 3000 && hr_std < 500) {
								hr_state = HR_RISING | HR_ESTAB;
							}
							else {
								hr_state = HR_IDLE;
							}
						}
					}
					else {
						u8minmax(rawHR_stack, 100, hr_datacnt, rawHR_stackIdx, &hr_baseline, &hr_peak);
						hr_threshold = (uint8_t) (hr_peak-hr_baseline)/1.5;
					}
					hr_mstick = msTimCnt;
					hr_periodcnt = hr_periodcnt%3 + 1;
				}
				else {
					hr_state = HR_RISING | HR_ESTAB;
					hr_period[hr_periodcnt-1] += (float32_t) (msTimCnt-hr_mstick);
					hr_mstick = msTimCnt;
					u8minmax(rawHR_stack, 100, 50, rawHR_stackIdx, &hr_baseline, &hr_peak);
					hr_threshold = (uint8_t) (hr_peak-hr_baseline)/1.5;
					arm_mean_f32(hr_period, 3, &hr_mean);
					arm_std_f32(hr_period, 3, &hr_std);
					if (hr_mean > 250 && hr_mean < 3000 && hr_std < 500) {
						outHR = (uint8_t) (60000./hr_mean);
						hr_periodcnt = hr_periodcnt%3 + 1;
					}
					else {
						hr_state = HR_IDLE;
						outHR = 0;
					}
				}
			}
			else {
				if (msTimCnt - hr_mstick > 1000) {
					hr_state = HR_IDLE;
					outHR = 0;
				}
			}
		}

		prevHR_val = currHR_val;

		rawUpdated = 0;
	}

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 71;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
     PA11   ------> USB_DM
     PA12   ------> USB_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE6 PE7 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC3 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF9 PF10 PF2 PF4 
                           PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_2|GPIO_PIN_4 
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA8 PA9 
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB4 PB5 PB8 
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
