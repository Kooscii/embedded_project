/**
  ******************************************************************************
  * @file    main.c
  * @author  Tianyu Gu
  * @version V1.1
  * @date    14-May-2017
  ******************************************************************************
*/



#include "peripheral.h"
#include "filter.h"
#include "arm_math.h"


#define STEP_IDLE 		(0x01)
#define STEP_FALLING 	(0x01<<1)
#define STEP_RISING 	(0x01<<2)
#define STEP_FIRST 		(0x01<<7)
#define STEP_THRESH		0.05
#define STEP_THRESH_DYN	0.5
#define STEP_FILTER_LEN 50
#define STEP_PREIOD_LEN 5
#define STEP_WINDOW_LEN 50

#define HP_IDLE 		(0x01)
#define HP_FALLING		(0x01<<1)
#define HP_RISING 		(0x01<<2)
#define HP_FIRST 		(0x01<<7)
#define HP_THRESH		3
#define HP_THRESH_DYN	0.3
#define HP_FILTER_LEN 	50
#define HP_PREIOD_LEN 	5
#define HP_WINDOW_LEN 	50


/* sensor buffer */
uint16_t 	u16rawHP_dma = 0;			// unprocessed data from heart pulse sensor
int16_t 	i16rawAcc[3] = {0,0,0};		// unprocessed data from acceleometer
uint8_t 	u8rawHP = 0;				// preprocessed raw heart pulse data
float32_t 	f32rawStep = 0;				// preprocessed raw step data
uint8_t 	rawUpdated = 0;				// raw data updating flag

/* data struct for uart communication */
struct {
	uint8_t index;
	uint8_t rawHP;
	uint8_t thrsHP;
	uint8_t filtHP;
	uint8_t rawStepx;
	uint8_t rawStepy;
	uint8_t rawStepz;
	uint8_t thrsStep;
	uint8_t filtStep;
	uint8_t outHR;
	uint8_t outSR;
	uint8_t outHC;
	uint8_t outSC;
	uint8_t _end;
} uart = {110,0,0,0,0,0,0,0,0,0,0,0,0,'\n'};


/* To avoid data in uart struct equal to '\n' */
uint8_t uart_assign(uint8_t);
void Error_Handler(char*);

/*
 * main
 */
int main(void)
{
	/*
	 * Local Variables
	 */
	uint8_t i;

	// Step
	float32_t 	step_rawqueue[STEP_FILTER_LEN*2];
	uint8_t 	step_rawqueueIdx = STEP_FILTER_LEN*2;
	float32_t 	step_filtqueue[STEP_WINDOW_LEN];
	uint8_t 	step_filtqueueIdx = STEP_WINDOW_LEN;
	float32_t 	step_rawgval = 0;
	float32_t 	step_currval = 0;
	uint8_t 	step_state = 0;
	float32_t 	step_peak = 0;
	float32_t 	step_valley = 0;
	float32_t 	step_baseline = 0;
	float32_t 	step_threshold = STEP_THRESH;
	uint32_t 	step_timeout = 0;
	uint32_t 	step_mstick = 0;
	float32_t 	step_period[STEP_PREIOD_LEN];
	uint8_t 	step_periodidx = 0;
	float32_t 	step_mean = 0;
//	float32_t 	step_std = 0;
	uint8_t 	outSR = 0;			// stride rate
	uint8_t 	outSC = 0;			// strides count

	// Heart Pulse
	float32_t 	hp_rawqueue[HP_FILTER_LEN*2];
	uint8_t 	hp_rawqueueIdx = HP_FILTER_LEN*2;
	float32_t 	hp_filtqueue[HP_WINDOW_LEN];
	uint8_t 	hp_filtqueueIdx = HP_WINDOW_LEN;
	float32_t 	rawHP_offset = 0;
	float32_t 	hp_currval = 0;
	uint8_t 	hp_state = 0;
	float32_t 	hp_peak = 0;
	float32_t 	hp_valley = 0;
	float32_t 	hp_baseline = 0;
//	float32_t 	hp_baselineoffset = 0;
	float32_t 	hp_threshold = HP_THRESH;
	uint32_t 	hp_timeout = 0;
	uint32_t 	hp_mstick = 0;
	float32_t 	hp_period[HP_PREIOD_LEN];
	uint8_t 	hp_periodidx = 0;
	float32_t 	hp_mean = 0;
	uint8_t 	outHR = 0;			// heart rate
	uint8_t 	outHC = 0;			// beats count


	/*
	 * Peripherial Initialization
	 */
	NVIC_Init();
	RCC_Init();
	GPIO_Init();
	TIM_Init();
	DAC_Init();
	ADC_Init();
	UART_Init();
	I2C_Init();

	/*
	 * Application Initialization
	 */
	HAL_GPIO_WritePin(LED6_GPIO_PORT, LED6_PIN, 1);

	// DAC start
	HAL_TIM_Base_Start_IT(&htim6);  // start TIM6
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) hr_data, 965, DAC_ALIGN_8B_R);  // start DAC in DMA mode

	// ADC start
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&u16rawHP_dma, 1);  // start ADC in DMA mode

	// Accelerometer start
	MEMS_ACCELERO_Init();

	// Step Algorithm Init
	step_rawgval = 0;
	for (step_rawqueueIdx=STEP_FILTER_LEN*2-1; step_rawqueueIdx>STEP_FILTER_LEN-1; step_rawqueueIdx--) {
		while (!rawUpdated);
		step_rawqueue[step_rawqueueIdx] = f32rawStep;
		rawUpdated = 0;
	}
	// 1. Get the mean value of first STEP_FILTER_LEN data. This value equals to 1g.
	arm_mean_f32(step_rawqueue+STEP_FILTER_LEN, STEP_FILTER_LEN, &step_rawgval);
	// 2. Zero-centering the all raw values. (val / 1g - 1)
	arm_scale_f32(step_rawqueue+STEP_FILTER_LEN, 1/step_rawgval, step_rawqueue+STEP_FILTER_LEN, STEP_FILTER_LEN);
	arm_offset_f32(step_rawqueue+STEP_FILTER_LEN, -1, step_rawqueue+STEP_FILTER_LEN, STEP_FILTER_LEN);
	// 3. Init period
	for (i=0; i<STEP_PREIOD_LEN; i++)
		step_period[i] = 2000;

	// Heart Pulse Algorithm Init
	rawHP_offset = 0;
	for (hp_rawqueueIdx=HP_FILTER_LEN*2-1; hp_rawqueueIdx>HP_FILTER_LEN-1; hp_rawqueueIdx--) {
		while (!rawUpdated);
		hp_rawqueue[hp_rawqueueIdx] = (float32_t) u8rawHP;
		rawUpdated = 0;
	}
	// 1. Get the mean value of first HP_FILTER_LEN data, Update baseline
	arm_mean_f32(hp_rawqueue+HP_FILTER_LEN, HP_FILTER_LEN, &rawHP_offset);
	arm_std_f32(hp_rawqueue+HP_FILTER_LEN, HP_FILTER_LEN, &hp_threshold);
	hp_baseline = rawHP_offset;
	// 2. Init period
	for (i=0; i<HP_PREIOD_LEN; i++)
			hp_period[i] = 2000;

	// Init state machine
	step_state = STEP_IDLE;
	hp_state = HP_IDLE;

	HAL_GPIO_WritePin(LED6_GPIO_PORT, LED6_PIN, 0);
	/*
	 * End of Application Initialization
	 */

	for(;;){
		// wait until new raw data is available
		HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, 0);
		while (!rawUpdated);
		rawUpdated = 0;
		HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, 1);

		/*
		 * Step detecting algorithm
		 */
		/* 1. Filter */
		// put new data into raw queue
		step_rawqueue[step_rawqueueIdx] = f32rawStep/step_rawgval-1.;
		// get filtered step value
		arm_dot_prod_f32((float*) (step_rawqueue+step_rawqueueIdx), (float*) step_filter,
									STEP_FILTER_LEN+1, &step_currval);
		// add current filtered value to filtered queue
		step_filtqueue[step_filtqueueIdx] = step_currval;
		step_filtqueueIdx = (step_filtqueueIdx+1)%STEP_WINDOW_LEN;
		// forming a continuous array for 'arm_dot_prod_f32'
		step_rawqueue[step_rawqueueIdx+STEP_FILTER_LEN] = step_rawqueue[step_rawqueueIdx];
		// step_rawqueueIdx--
		step_rawqueueIdx = (step_rawqueueIdx + STEP_FILTER_LEN-1)%STEP_FILTER_LEN;

		/* 2. Update baseline and threshold */
		if (!(step_state & STEP_IDLE)) {
			arm_std_f32(step_filtqueue, STEP_WINDOW_LEN, &step_threshold);
			step_threshold *= STEP_THRESH_DYN;
			if (step_threshold < 0.03)
				step_threshold = 0.03;
		}

		/* 3. Update uart struct (data and threshold)*/
		uart.filtStep = uart_assign((uint8_t) (((uint16_t)((step_currval+1)*step_rawgval))>>5)+128);
		if ((step_state & STEP_IDLE) || (step_state & STEP_RISING))
			uart.thrsStep = uart_assign((uint8_t) (((uint16_t)((step_baseline+step_threshold+1)*step_rawgval))>>5)+128);
		else
			uart.thrsStep = uart_assign((uint8_t) (((uint16_t)((step_baseline-step_threshold+1)*step_rawgval))>>5)+128);

		/* 4. State machine */
		// if IDLE
		if (step_state & STEP_IDLE) {
			if (step_currval > step_baseline + step_threshold) {
				step_state = STEP_FIRST | STEP_FALLING;
				step_peak = step_currval;
				step_timeout = HAL_GetTick();
			}
		}
		// if FALLING
		else if (step_state & STEP_FALLING) {
			// if new peak, actually it's ture when signal is rising
			if (step_currval > step_peak) {
				step_peak = step_currval;
			}
			// when falling, the true threshold is (baseline - threshold),
			// in order to limit the lower bound of the amplitude.
			else if (step_currval < step_baseline - step_threshold) {
//				arm_std_f32(step_period, STEP_PREIOD_LEN, &step_std);
				// if not first step, calc step rate
				if (!(step_state & STEP_FIRST)) {
					// current step rate = mean of previous 5 step rates
					step_period[step_periodidx] = (float32_t) (HAL_GetTick() - step_mstick);
					step_periodidx = (step_periodidx+1)%STEP_PREIOD_LEN;
					arm_mean_f32(step_period, STEP_PREIOD_LEN, &step_mean);
					outSR = (uint8_t) (60000./step_mean);
					outSC = (outSC+1)%100;
				}
				// next state
				step_state = STEP_RISING;
				step_valley = step_currval;
				step_mstick = HAL_GetTick();
				step_timeout = HAL_GetTick();
			}
			// else, update baseline
			else if (!(step_state & STEP_FIRST)) {
				step_baseline = (step_peak + step_valley)/2;
			}
		}
		// if RISING
		else if (step_state & STEP_RISING) {
			// if new valley , actually it's ture when signal is falling
			if (step_currval < step_valley) {
				step_valley = step_currval;
			}
			// when rising, the true threshold is (baseline + threshold),
			// in order to limit the lower bound of the amplitude.
			else if (step_currval > step_baseline + step_threshold) {
				step_state = STEP_FALLING;
				step_peak = step_currval;
				step_timeout = HAL_GetTick();
			}
			// else, update baseline
			else {
				step_baseline = (step_peak + step_valley)/2;
			}
		}

		/* Timeout checking */
		if ( ((step_state & STEP_RISING) && (HAL_GetTick() - step_timeout>3000))
				|| ((step_state & STEP_FALLING) && (HAL_GetTick() - step_timeout>3000)) ) {
			step_state = STEP_IDLE;
			step_baseline = 0;
			step_threshold = STEP_THRESH;
			for (i=0; i<STEP_PREIOD_LEN; i++)
				step_period[i] = 2000;
			outSR = 0;
		}
		/*
		 * End of Step Detecting Algorithm
		 */


		/*
		 * Heart Pulse Detecting Algorithm
		 */
		/* 1. Filter */
		// put new data into raw queue
		hp_rawqueue[hp_rawqueueIdx] = ((float32_t)u8rawHP);
		// get filtered hp value
		arm_dot_prod_f32((float*) (hp_rawqueue+hp_rawqueueIdx), (float*) hp_filter,
									HP_FILTER_LEN+1, &hp_currval);
		// add current filtered value to filtered queue
		hp_filtqueue[hp_filtqueueIdx] = hp_currval;
		hp_filtqueueIdx = (hp_filtqueueIdx+1)%HP_WINDOW_LEN;
		// forming a continuous array for 'arm_dot_prod_f32'
		hp_rawqueue[hp_rawqueueIdx+HP_FILTER_LEN] = hp_rawqueue[hp_rawqueueIdx];
		// hp_rawqueueIdx--
		hp_rawqueueIdx = (hp_rawqueueIdx + HP_FILTER_LEN-1)%HP_FILTER_LEN;

		/* 2. Update baseline and threshold */
		arm_mean_f32(hp_filtqueue+hp_filtqueueIdx, HP_WINDOW_LEN/2, &rawHP_offset);
		arm_std_f32(hp_filtqueue+hp_filtqueueIdx, HP_WINDOW_LEN/5, &hp_threshold);
		hp_threshold *= HP_THRESH_DYN;
//		hp_baseline = rawHP_offset;
		if (hp_threshold < HP_THRESH) {
			hp_threshold = HP_THRESH;
		}

		/* 3. Update uart struct (data and threshold)*/
		uart.filtHP = uart_assign((uint8_t)hp_currval);
		if ((hp_state & HP_IDLE) || (hp_state & HP_RISING))
			uart.thrsHP = uart_assign((uint8_t) (hp_baseline + hp_threshold));
		else
			uart.thrsHP = uart_assign((uint8_t) (hp_baseline - hp_threshold));

		/* 4. State machine */
		// similar to step detecting
		if (hp_state & HP_IDLE) {
			if (hp_currval > hp_baseline + hp_threshold) {
				hp_state = HP_FIRST | HP_FALLING;
				hp_peak = hp_currval;
				hp_timeout = HAL_GetTick();
			}
		}
		else if (hp_state & HP_FALLING) {
			if (hp_currval > hp_peak) {
				hp_peak = hp_currval;
			}
			else if (hp_currval < hp_baseline - hp_threshold) {
				if (!(hp_state & HP_FIRST)) {
					// calc rate
					hp_period[hp_periodidx] = (float32_t) (HAL_GetTick() - hp_mstick);
					hp_periodidx = (hp_periodidx+1)%HP_PREIOD_LEN;
					arm_mean_f32(hp_period, HP_PREIOD_LEN, &hp_mean);
					outHR = (uint8_t) (60000./hp_mean);
					outHC = (outHC+1)%100;
				}
				hp_state = HP_RISING;
				hp_valley = hp_currval;
				hp_mstick = HAL_GetTick();
				hp_timeout = HAL_GetTick();
			}
			else if (!(hp_state & HP_FIRST)) {
				// the baseline of heart pulse is a little higher
				hp_baseline = (hp_peak - hp_valley)*2/3 + hp_valley;
			}
		}
		else if (hp_state & HP_RISING) {
			if (hp_currval < hp_valley) {
				hp_valley = hp_currval;
				hp_baseline = (hp_peak - hp_valley)*2/3 + hp_valley;
			}
			else if (hp_currval > hp_baseline + hp_threshold) {
				hp_state = HP_FALLING;
				hp_peak = hp_currval;
				hp_timeout = HAL_GetTick();
			}
			// Here is the difference. Tricky but works pretty well.
			// The magnitude of heart pulse signal will unexpectedly decrease sometimes
			// So I want the baseline keep going down while the signal rising,
			// and make them meet with each other
			else {
				hp_baseline -= hp_threshold*0.05;
			}
		}

		/* 5. Timeout Checking */
		if ( ((hp_state & HP_RISING) && (HAL_GetTick() - hp_timeout>2000))
			|| ((hp_state & HP_FALLING) && (HAL_GetTick() - hp_timeout>1000)) ) {
			hp_state = HP_IDLE;
			hp_baseline = rawHP_offset;		// heart pulse signal is not zero-centered
			for (i=0; i<HP_PREIOD_LEN; i++)
				hp_period[i] = 2000;
			outHR = 0;
		}
		/*
		 * End of Heart Pulse Detecting
		 */

		/* Update uart struct (rate and cnt output) */
		uart.outSR = uart_assign(outSR);
		uart.outSC = uart_assign(outSC);
		uart.outHR = uart_assign(outHR);
		uart.outHC = uart_assign(outHC);

		/*
		 * Sending new uart struct to pc
		 */
		uart.index = (uart.index-'\n') % 100 + '\n'+1;
		HAL_UART_Transmit_IT(&huart1, &uart.index, 14);
	}
}

/*
 * Timer Interrupt Callback, used for sampling (50Hz)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	if (htim->Instance == TIM6) {
		/* Sampling rate = 50 Hz */
		// read accelerometer
		HAL_I2C_Mem_Read_IT(&hi2c1, 0x32, 0x28|0x80, I2C_MEMADD_SIZE_8BIT, (uint8_t *)i16rawAcc, 6);
		// heart pulse data from ADC is get by DMA
		// stored in u16rawHP_dma
	}
}

/*
 * I2C (Accelerometer) Recv Callback
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	float32_t facc[3];
	float32_t f_acc_rms;;

	if (hi2c->Instance == I2C1) {
		/*
		 * Preprocessing raw data
		 */
		// Get heart pulse raw data
		u8rawHP = (uint8_t) u16rawHP_dma;

		// Get step raw data, and calc rms
		facc[0] = (float32_t)(i16rawAcc[0]>>3);	// return value from acceleo is 12-bit left aligned,
		facc[1] = (float32_t)(i16rawAcc[1]>>3);	// and sensitivity under 4g scale is 2mg/bit, so >>3
		facc[2] = (float32_t)(i16rawAcc[2]>>3);
		arm_rms_f32(facc, 3, &f_acc_rms);
		f32rawStep = f_acc_rms*1.73205080756888;

		/*
		 * Update uart struct
		 */
		uart.rawHP = uart_assign(u8rawHP);
		// >> 8 we get 8-bit resolution
		uart.rawStepx = uart_assign((uint8_t) ((i16rawAcc[0]>>8)+128));
		uart.rawStepy = uart_assign((uint8_t) ((i16rawAcc[1]>>8)+128));
		uart.rawStepz = uart_assign((uint8_t) ((i16rawAcc[2]>>8)+128));

		rawUpdated = 1;
	}
}

/*
 * Button Interrupt Callback
 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (BSP_PB_GetState(BUTTON_USER)==GPIO_PIN_SET) {
//		BSP_LED_Toggle(LED3);
//	}
//}

/*
 * Error Callback
 */
void Error_Handler(char* msg) {
	while (1) {
	}
}

uint8_t uart_assign(uint8_t x) {
	return x<255-'\n'? x+'\n'+1: 255;
}
