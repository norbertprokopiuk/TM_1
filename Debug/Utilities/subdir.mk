################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/stm32l0xx_nucleo.c 

OBJS += \
./Utilities/stm32l0xx_nucleo.o 

C_DEPS += \
./Utilities/stm32l0xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/%.o: ../Utilities/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32 -DSTM32L0 -DSTM32L053R8Tx -DNUCLEO_L053R8 -DDEBUG -DSTM32L053xx -DUSE_HAL_DRIVER -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/HAL_Driver/Inc/Legacy" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/Utilities" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/inc" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/CMSIS/device" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/CMSIS/core" -I"C:/Users/norbe/Desktop/studia/SEMESTR V/technika mikroprocesorowa 2/projekt/projekt/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


