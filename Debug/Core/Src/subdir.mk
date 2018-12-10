################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/dac.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/dac.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/dac.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F302x8 -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Core/Inc" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/App/Inc" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/STM32F3xx_HAL_Driver/Inc" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


