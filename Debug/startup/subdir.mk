################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32l053xx.s 

OBJS += \
./startup/startup_stm32l053xx.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/HAL_Driver/Inc/Legacy" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/Utilities" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/inc" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/CMSIS/device" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/CMSIS/core" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/HAL_Driver/Inc" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


