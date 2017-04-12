################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MEMS.c \
../Src/main.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/system_stm32f3xx.c 

OBJS += \
./Src/MEMS.o \
./Src/main.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/system_stm32f3xx.o 

C_DEPS += \
./Src/MEMS.d \
./Src/main.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F303xC -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Inc" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/STM32F3-Discovery" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/Components/l3gd20" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/Components/Common" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/Components/lsm303dlhc" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Drivers/CMSIS/Include" -I"/Users/koala/Documents/workspace/embedded-project/ADC-DAC-tst/Inc"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


