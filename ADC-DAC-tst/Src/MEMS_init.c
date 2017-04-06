#include "MEMS_init.h"

extern void Error_Handler(void);

/**
  * @brief Test ACCELERATOR MEMS Hardware.
  *   The main objective of this test is to check acceleration on 2 axis X and Y
  * @param  None
  * @retval None
  */
void ACCELERO_MEMS_Init(void) {
  /* Init Accelerometer Mems */
  if(BSP_ACCELERO_Init() != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

int16_t ACCELERO_MEMS_GetData(ACC_XYZ xyz) {
	int16_t data[3];

	BSP_ACCELERO_GetXYZ(data);
	switch(xyz) {
	case ACC_X:
		return data[0];
		break;
	case ACC_Y:
		return data[1];
		break;
	case ACC_Z:
		return data[2];
		break;
	}

	return 0;
}
