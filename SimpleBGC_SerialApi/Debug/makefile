################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SimpleBGC_SerialApi.elf

dependents:
	-cd C:/Users/Kurat/Documents/git_repos/Gimbal/SBGC_Api/Debug && $(MAKE) all
	-cd C:/Users/Kurat/workspace_gimbal/share_test/Debug && $(MAKE) all

# Tool invocations
SimpleBGC_SerialApi.elf: $(OBJS) $(USER_OBJS) C:/Users/Kurat/Documents/git_repos/Gimbal/SBGC_Api/Debug/SBGC_Api.elf C:/Users/Kurat/workspace_gimbal/share_test/Debug/libshare_test.so ../STM32F446RETx_FLASH.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -L"C:\Users\Kurat\workspace_gimbal\share_test\Debug" -specs=nosys.specs -specs=nano.specs -T"../STM32F446RETx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "SimpleBGC_SerialApi.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating hex and Printing size information:'
	arm-none-eabi-objcopy -O ihex "SimpleBGC_SerialApi.elf" "SimpleBGC_SerialApi.hex"
	arm-none-eabi-size "SimpleBGC_SerialApi.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
C:/Users/Kurat/Documents/git_repos/Gimbal/SBGC_Api/Debug/SBGC_Api.elf:
C:/Users/Kurat/workspace_gimbal/share_test/Debug/libshare_test.so:

-include ../makefile.targets
