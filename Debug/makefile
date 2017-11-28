################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include example/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: usb_bridge_LPC11U14.axf

# Tool invocations
usb_bridge_LPC11U14.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/nxp_lpcxpresso_11u14_board_lib/Debug" -L"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/lpc_chip_11uxx_lib/Debug" -L"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/lpc_chip_11uxx_lib/libs" -Xlinker -Map="usb_bridge_LPC11U14.map" -Xlinker --gc-sections -mcpu=cortex-m0 -mthumb -T "usb_bridge_LPC11U14_Debug.ld" -o "usb_bridge_LPC11U14.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) usb_bridge_LPC11U14.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "usb_bridge_LPC11U14.axf"; # arm-none-eabi-objcopy -O binary "usb_bridge_LPC11U14.axf" "usb_bridge_LPC11U14.bin" ; checksum -p LPC11U14_201 -d "usb_bridge_LPC11U14.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
