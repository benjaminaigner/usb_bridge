################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../example/src/cdc_vcom.c \
../example/src/composite_desc.c \
../example/src/composite_main.c \
../example/src/cr_startup_lpc11xx.c \
../example/src/hid_mouse.c \
../example/src/sysinit.c 

OBJS += \
./example/src/cdc_vcom.o \
./example/src/composite_desc.o \
./example/src/composite_main.o \
./example/src/cr_startup_lpc11xx.o \
./example/src/hid_mouse.o \
./example/src/sysinit.o 

C_DEPS += \
./example/src/cdc_vcom.d \
./example/src/composite_desc.d \
./example/src/composite_main.d \
./example/src/cr_startup_lpc11xx.d \
./example/src/hid_mouse.d \
./example/src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
example/src/%.o: ../example/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M0 -I"/home/beni/sync/Projects/FH/usb_bridge/example/inc" -I"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/lpc_chip_11uxx_lib/inc" -I"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/nxp_lpcxpresso_11u14_board_lib/inc" -I"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/lpc_chip_11uxx_lib/inc/usbd" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

example/src/cr_startup_lpc11xx.o: ../example/src/cr_startup_lpc11xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M0 -I"/home/beni/sync/Projects/FH/usb_bridge/example/inc" -I"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/lpc_chip_11uxx_lib/inc" -I"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/nxp_lpcxpresso_11u14_board_lib/inc" -I"/home/beni/Documents/MCUXpresso_10.1.0_589/workspace/lpc_chip_11uxx_lib/inc/usbd" -Os -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"example/src/cr_startup_lpc11xx.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


