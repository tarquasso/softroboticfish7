################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../BCU_Test/MS5837/MS5837.cpp 

OBJS += \
./BCU_Test/MS5837/MS5837.o 

CPP_DEPS += \
./BCU_Test/MS5837/MS5837.d 


# Each subdirectory must supply rules for building sources it contributes
BCU_Test/MS5837/%.o: ../BCU_Test/MS5837/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -D__NEWLIB__ -D__CODE_RED -DCPP_USE_HEAP -D__CORTEX_M3 -DTOOLCHAIN_GCC -DTOOLCHAIN_GCC_CR -DARM_MATH_CM3 -DTARGET_LIKE_CORTEX_M3 -DTARGET_CORTEX_M -DTARGET_LIKE_MBED -DTARGET_LPC176X -DTARGET_NXP -DTARGET_LPC1768 -DMBED_BUILD_TIMESTAMP=1460559729.11 -DTARGET_M3 -DTARGET_MBED_LPC1768 -D__MBED__=1 -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\BCU_Test" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\BCU_Test\MS5837" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\BCU_Test\BNO055" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\BCU_Test\BuoyancyControlUnit" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\mbed" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\mbed\TARGET_LPC1768" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\mbed\TARGET_LPC1768\TARGET_NXP" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\mbed\TARGET_LPC1768\TARGET_NXP\TARGET_LPC176X" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\mbed\TARGET_LPC1768\TARGET_NXP\TARGET_LPC176X\TARGET_MBED_LPC1768" -I"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\BCU_Test\mbed\TARGET_LPC1768\TOOLCHAIN_GCC_CR" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -mcpu=cortex-m3 -mthumb -D__NEWLIB__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


