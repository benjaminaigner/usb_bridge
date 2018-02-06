################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../example/src/cdc_vcom.c \
../example/src/composite_desc.c \
../example/src/composite_main.c \
../example/src/cr_startup_lpc11xx.c \
../example/src/hid.c \
../example/src/parser.c \
../example/src/sysinit.c 

OBJS += \
./example/src/cdc_vcom.o \
./example/src/composite_desc.o \
./example/src/composite_main.o \
./example/src/cr_startup_lpc11xx.o \
./example/src/hid.o \
./example/src/parser.o \
./example/src/sysinit.o 

C_DEPS += \
./example/src/cdc_vcom.d \
./example/src/composite_desc.d \
./example/src/composite_main.d \
./example/src/cr_startup_lpc11xx.d \
./example/src/hid.d \
./example/src/parser.d \
./example/src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
example/src/%.o: ../example/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M0 -I"F:\usb_bridge\example\inc" -I"C:\Users\AsTeRICS\Documents\MCUXpressoIDE_10.1.1_606\workspace\lpc_chip_11uxx_lib\inc" -I"C:\Users\AsTeRICS\Documents\MCUXpressoIDE_10.1.1_606\workspace\nxp_lpcxpresso_11u14_board_lib\inc" -I"C:\Users\AsTeRICS\Documents\MCUXpressoIDE_10.1.1_606\workspace\lpc_chip_11uxx_lib\inc\usbd" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

example/src/cr_startup_lpc11xx.o: ../example/src/cr_startup_lpc11xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M0 -I"F:\usb_bridge\example\inc" -I"C:\Users\AsTeRICS\Documents\MCUXpressoIDE_10.1.1_606\workspace\lpc_chip_11uxx_lib\inc" -I"C:\Users\AsTeRICS\Documents\MCUXpressoIDE_10.1.1_606\workspace\nxp_lpcxpresso_11u14_board_lib\inc" -I"C:\Users\AsTeRICS\Documents\MCUXpressoIDE_10.1.1_606\workspace\lpc_chip_11uxx_lib\inc\usbd" -Os -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"example/src/cr_startup_lpc11xx.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


