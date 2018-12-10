################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/sixstep.c \
../App/Src/telemetry.c 

OBJS += \
./App/Src/sixstep.o \
./App/Src/telemetry.o 

C_DEPS += \
./App/Src/sixstep.d \
./App/Src/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o: ../App/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F302x8 -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Core/Inc" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/App/Inc" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/STM32F3xx_HAL_Driver/Inc" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"E:/Projects/MotorControllerTest1/MotorControllerTest1/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


