################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include mbed/TARGET_LPC1768/TOOLCHAIN_GCC_CR/subdir.mk
-include Pump_Test/QEI/subdir.mk
-include Pump_Test/PumpWithValve/subdir.mk
-include Pump_Test/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Pump_Test.axf

# Tool invocations
Pump_Test.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU C++ Linker'
	arm-none-eabi-c++ -nostdlib -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\Pump_Test" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\Pump_Test\PumpWithValve" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\Pump_Test\QEI" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\mbed" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\mbed\TARGET_LPC1768" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\mbed\TARGET_LPC1768\TARGET_NXP" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\mbed\TARGET_LPC1768\TARGET_NXP\TARGET_LPC176X" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\mbed\TARGET_LPC1768\TARGET_NXP\TARGET_LPC176X\TARGET_MBED_LPC1768" -L"C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\mbed\TARGET_LPC1768\TOOLCHAIN_GCC_CR" -Xlinker -Map="Pump_Test.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "C:\Users\Cyndia\softroboticfish6\fish\mbed\LPCExpress\Pump_Test\mbed\TARGET_LPC1768\TOOLCHAIN_GCC_CR\LPC1768.ld" -o "Pump_Test.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(C_UPPER_DEPS)$(CXX_DEPS)$(OBJS)$(CPP_DEPS)$(C_DEPS) Pump_Test.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "Pump_Test.axf"; arm-none-eabi-objcopy -O binary "Pump_Test.axf" "Pump_Test.bin" ; # checksum -p LPC1768 -d "Pump_Test.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
