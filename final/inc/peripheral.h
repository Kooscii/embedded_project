#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#include "stm32f3xx.h"
#include "stm32f3_discovery.h"


//// Oscillator
//#define OSC_IN_Pin 				GPIO_PIN_0
//#define OSC_IN_GPIO_Port 		GPIOF
//
//// MEMS pins
//#define MEMS_GPIO_Port			GPIOE
//#define MEMS_INT1_Pin 			GPIO_PIN_0
//#define MEMS_INT1_GPIO_Port 	GPIOE
//#define MEMS_INT2_Pin 			GPIO_PIN_1
//#define MEMS_INT2_GPIO_Port 	GPIOE
//#define DRDY_Pin 				GPIO_PIN_2
//#define DRDY_GPIO_Port 			GPIOE
//#define CS_I2C_SPI_Pin 			GPIO_PIN_3
//#define CS_I2C_SPI_GPIO_Port 	GPIOE
//#define MEMS_INT3_Pin 			GPIO_PIN_4
//#define MEMS_INT3_GPIO_Port 	GPIOE
//#define MEMS_INT4_Pin 			GPIO_PIN_5
//#define MEMS_INT4_GPIO_Port 	GPIOE
//// LSM303 accelero
//#define I2C1_GPIO_Port			GPIOB
//#define I2C1_SCL_Pin 			GPIO_PIN_6
//#define I2C1_SCL_GPIO_Port 		GPIOB
//#define I2C1_SDA_Pin 			GPIO_PIN_7
//#define I2C1_SDA_GPIO_Port 		GPIOB
//// L3GD20 gyro
//#define SPI1_GPIO_Port			GPIOA
//#define SPI1_SCK_Pin 			GPIO_PIN_5
//#define SPI1_SCK_GPIO_Port 		GPIOA
//#define SPI1_MISO_Pin 			GPIO_PIN_6
//#define SPI1_MISO_GPIO_Port 	GPIOA
//#define SPI1_MISOA7_Pin 		GPIO_PIN_7
//#define SPI1_MISOA7_GPIO_Port 	GPIOA
//
//// User button
//#define B1_Pin 					GPIO_PIN_0
//#define B1_GPIO_Port 			GPIOA
//
//// Leds
//#define LED_GPIO_Port			GPIOE
//#define LED4_Pin 				GPIO_PIN_8
//#define LED4_GPIO_Port 			GPIOE
//#define LED3_Pin 				GPIO_PIN_9
//#define LED3_GPIO_Port 			GPIOE
//#define LED5_Pin 				GPIO_PIN_10
//#define LED5_GPIO_Port 			GPIOE
//#define LED7_Pin 				GPIO_PIN_11
//#define LED7_GPIO_Port 			GPIOE
//#define LED9_Pin 				GPIO_PIN_12
//#define LED9_GPIO_Port 			GPIOE
//#define LED10_Pin 				GPIO_PIN_13
//#define LED10_GPIO_Port 		GPIOE
//#define LED8_Pin 				GPIO_PIN_14
//#define LED8_GPIO_Port 			GPIOE
//#define LED6_Pin 				GPIO_PIN_15
//#define LED6_GPIO_Port 			GPIOE
//
//// USB
//#define USB_GPIO_Port			GPIOA
//#define DM_Pin 					GPIO_PIN_11
//#define DM_GPIO_Port 			GPIOA
//#define DP_Pin 					GPIO_PIN_12
//#define DP_GPIO_Port 			GPIOA
//
//// SWO debug
//#define SWDIO_Pin 				GPIO_PIN_13
//#define SWDIO_GPIO_Port			GPIOA
//#define SWCLK_Pin 				GPIO_PIN_14
//#define SWCLK_GPIO_Port 		GPIOA
//#define SWO_Pin 				GPIO_PIN_3
//#define SWO_GPIO_Port 			GPIOB

// DAC
#define DAC_OUT1_GPIO_PORT 		GPIOA
#define DAC_OUT1_PIN			GPIO_PIN_4

void RCC_Init(void);
void GPIO_Init(void);
void i2c_init(void);
void spi_init(void);
void TIM_init(void);
void DAC_Init(DAC_HandleTypeDef *);
void uart_init(void);
void adc_init(void);


#endif
