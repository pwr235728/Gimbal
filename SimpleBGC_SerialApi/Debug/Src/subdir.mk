################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/proxima/Documents/git/Gimbal/SBGC_Api/src/SBGC_CommandHelpers.c \
/home/proxima/Documents/git/Gimbal/SBGC_Api/src/SBGC_Parser.c \
/home/proxima/Documents/git/Gimbal/SBGC_Api/src/SBGC_SerialCommand.c \
../Src/circ_buf.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/SBGC_CommandHelpers.o \
./Src/SBGC_Parser.o \
./Src/SBGC_SerialCommand.o \
./Src/circ_buf.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/SBGC_CommandHelpers.d \
./Src/SBGC_Parser.d \
./Src/SBGC_SerialCommand.d \
./Src/circ_buf.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/SBGC_CommandHelpers.o: /home/proxima/Documents/git/Gimbal/SBGC_Api/src/SBGC_CommandHelpers.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Include" -I"/home/proxima/Documents/git/Gimbal/SBGC_Api/inc"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/SBGC_Parser.o: /home/proxima/Documents/git/Gimbal/SBGC_Api/src/SBGC_Parser.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Include" -I"/home/proxima/Documents/git/Gimbal/SBGC_Api/inc"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/SBGC_SerialCommand.o: /home/proxima/Documents/git/Gimbal/SBGC_Api/src/SBGC_SerialCommand.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Include" -I"/home/proxima/Documents/git/Gimbal/SBGC_Api/inc"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F446xx -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/proxima/Documents/git/Gimbal/SimpleBGC_SerialApi/Drivers/CMSIS/Include" -I"/home/proxima/Documents/git/Gimbal/SBGC_Api/inc"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


