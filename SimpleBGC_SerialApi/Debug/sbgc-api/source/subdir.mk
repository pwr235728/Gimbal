################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sbgc-api/source/SBGC_Parser.c \
../sbgc-api/source/SBGC_cmd_helpers.c 

OBJS += \
./sbgc-api/source/SBGC_Parser.o \
./sbgc-api/source/SBGC_cmd_helpers.o 

C_DEPS += \
./sbgc-api/source/SBGC_Parser.d \
./sbgc-api/source/SBGC_cmd_helpers.d 


# Each subdirectory must supply rules for building sources it contributes
sbgc-api/source/%.o: ../sbgc-api/source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"C:/Users/Kurat/Documents/git_repos/Gimbal/SimpleBGC_SerialApi/Inc" -I"C:/Users/Kurat/Documents/git_repos/Gimbal/SimpleBGC_SerialApi/sbgc-api" -I"C:/Users/Kurat/Documents/git_repos/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Kurat/Documents/git_repos/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Kurat/Documents/git_repos/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Kurat/Documents/git_repos/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


