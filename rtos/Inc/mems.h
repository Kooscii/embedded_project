#ifndef __MEMS_H
#define __MEMS_H


#include "stm32f3_discovery_accelerometer.h"

typedef enum
{
  ACC_X,
  ACC_Y,
  ACC_Z
}ACC_XYZ;

void MEMS_ACCELERO_Init(void);
int16_t MEMS_ACCELERO_GetData(ACC_XYZ xyz);


#endif /* __MEMS_H */
