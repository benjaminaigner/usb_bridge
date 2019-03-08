/*
 * @brief This file contains USB HID Mouse example using USB ROM Drivers.
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include <hid.h>
#include "parser.h"
#include "board.h"
#include <stdio.h>
#include <string.h>
#include "app_usbd_cfg.h"
#include "cdc_vcom.h"


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* PIN/PORT: SCL, connected to EXT header and ESP32 */
#define PIN_SCL		4
#define PORT_SCL	0
/* PIN/PORT: SCL, connected to EXT header and ESP32 */
#define PIN_SDA		5
#define PORT_SDA	0
/* PIN/PORT: ESP32 power, turns on/off the ESP32; active high*/
#define PIN_ESP_ON	8
#define PORT_ESP_ON	0
/* PIN/PORT/ADC channel: FSR 1-4*/
#define PIN_FSR1	11
#define PORT_FSR1	0
#define ADC_FSR1	0
#define PIN_FSR2	12
#define PORT_FSR2	0
#define ADC_FSR2	1
#define PIN_FSR3	14
#define PORT_FSR3	0
#define ADC_FSR3	3
#define PIN_FSR4	13
#define PORT_FSR4 	0
#define ADC_FSR4	2
/* PIN/PORT/ADC channel: pressure sensor*/
#define PIN_PRESSURE 	16
#define PORT_PRESSURE 	0
#define ADC_PRESSURE	5
/* PIN/PORT: ESP_SIGNAL, used for sending PPM modulated HID commands */
#define PIN_ESP_SIGNAL	17
#define PORT_ESP_SIGNAL	0
/* PIN/PORT: ESP_RX, used for sending UART data from CDC to ESP32 */
#define PIN_ESP_RX	19
#define PORT_ESP_RX 0
/* PIN/PORT: ESP_TX, used for sending UART data from ESP32 to CDC*/
#define PIN_ESP_TX	18
#define PORT_ESP_TX 0


/* PIN/PORT: ESP_DEBUG_RX, used for sending debug/programming data to ESP32 */
#define PIN_ESP_DEBUG_RX	13
#define PORT_ESP_DEBUG_RX 	1
/* PIN/PORT: ESP_DEBUG_TX, used for sending debug/programming from ESP32 to CDC */
#define PIN_ESP_DEBUG_TX	14
#define PORT_ESP_DEBUG_TX 	1
/* PIN/PORT: ESP_RESET, reset pin of ESP32; high or floating -> boot;  */
#define PIN_ESP_RESET	16
#define PORT_ESP_RESET	1
/* PIN/PORT: ESP_BOOT, boot mode select for ESP32: high or floating -> normal boot; low -> serial flasher*/
#define PIN_ESP_BOOT	22
#define PORT_ESP_BOOT	0

#define UART_CDC			0
#define UART_ESP_DEBUG		1


/* Transmit and receive ring buffers (UART/CDC)*/
RINGBUFF_T txring, rxring;

/** @brief I2C Transer struct */
static I2C_XFER_T i2c_xfer;
/** @brief I2C Data area for reading (from ESP32 -> I2C master)
 * @note We have 5 ADC channels (2B each) */
static uint8_t adc_data[10];
/** @brief I2C RX array.
 * @note We will get HID data here
 */
static uint8_t hid_data[10];

/** @brief Our local I2C adress for slave 0 */
#define I2C_SLAVE_ADDR_0 0x05

//added this const here, because if we do not need anything from board lib, it does not compile.
const uint32_t OscRateIn = 12000000;

/* Transmit and receive ring buffer sizes (UART/CDC)*/
#define UART_TXB_SIZE 1024	/* Send to ESP32 */
#define UART_RXB_SIZE 256	/* Receive from ESP32 */
#define ATCMD_LENGTH  256	/* Receive */

/* Transmit and receive buffers (UART/CDC)*/
static uint8_t rxbuff[UART_RXB_SIZE], txbuff[UART_TXB_SIZE];

/* HID input completed, this variable is set, if the stop edge is received. It contains the received edges timings (excluding stop edge) */
volatile uint32_t hid_arrived = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
static USBD_HANDLE_T g_hUsb;
static uint8_t g_rxBuff[UART_TXB_SIZE];

extern const  USBD_HW_API_T hw_api;
extern const  USBD_CORE_API_T core_api;
extern const  USBD_HID_API_T hid_api;
extern const  USBD_CDC_API_T cdc_api;

/* We use following API structures:
 * * HW API
 * * CORE API
 * * HID API
 * * CDC API
 */
static const  USBD_API_T g_usbApi = {
	&hw_api,
	&core_api,
	0,
	0,
	&hid_api,
	&cdc_api,
	0,
	0x02221101,
};

const  USBD_API_T *g_pUsbApi = &g_usbApi;

//tick count (currently configured to 10Hz)
uint32_t tick_count = 0;
uint32_t tick_count_prev = 0;


/**+++++ auto reset into MSD - related stuff (AN11305) ++++*/

//different RAM region, according to https://community.nxp.com/thread/421162
//0x1000017C - 0x1000025B
typedef void (*IAP)(uint32_t [], uint32_t []);
IAP iap_entry_local = (IAP)0x1fff1ff1;
uint32_t command[5], result[4];
///@todo What value is working? 0x1000017C or 0x1000025B?
//#define init_msdstate() *((uint32_t *)(0x10000054)) = 0x0 //original version from AN11305
#define init_msdstate() *((uint32_t *)(0x10000054)) = 0x0
///@todo Also one note: maybe we don't need to set the MSP pointer to 0? https://community.nxp.com/thread/465231

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize pin and clocks for USB port */
static void usb_pin_clk_init(void)
{
	/* enable USB main clock */
	Chip_Clock_SetUSBClockSource(SYSCTL_USBCLKSRC_PLLOUT, 1);
	/* Enable AHB clock to the USB block and USB RAM. */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USB);
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USBRAM);
	/* power UP USB Phy */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_USBPAD_PD);

	/* Enable IOCON clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
	Chip_IOCON_PinMuxSet(LPC_IOCON,0,6,(IOCON_FUNC1 | IOCON_MODE_INACT));
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief Calling this function resets the LPC chip into bootloader mode.
 * Depending on PIO0.3 (USB_VBUS), either serial or MSC download mode is entered.
 * @note MSC mode is only available on LPC11U24
 * @note This function is implemented according to AN11305, with a change for
 * the memory region according to https://community.nxp.com/thread/421162
 */
void LPC_InvokeBootloader(void)
{
	/* make sure USB clock is turned on before calling ISP */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USB);
	/* make sure 32-bit Timer 1 is turned on before calling ISP */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CT32B1);
	/* make sure GPIO clock is turned on before calling ISP */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_GPIO);
	/* make sure IO configuration clock is turned on before calling ISP */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);

	/* make sure AHB clock divider is 1:1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Send Reinvoke ISP command to ISP entry point*/
	command[0] = 57;

	init_msdstate();					 /* Initialize Storage state machine */
	/* Set stack pointer to ROM value (reset default) This must be the last
	     piece of code executed before calling ISP, because most C expressions
	     and function returns will fail after the stack pointer is changed. */
	__set_MSP(*((uint32_t *)0x00000000));

	/* Enter ISP. We call "iap_entry" to enter ISP because the ISP entry is done
	     through the same command interface as IAP. */
	iap_entry(command, result);
	// Not supposed to come back!
}

/**
 * @brief	Reset the ESP32 chip into bootloader mode.
 * @return 	Nothing
 */
void ESP32_ResetBootloader(void)
{
	uint32_t ticklocal = 0;

	//1.) set GPIO0 to low
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT_ESP_BOOT, PIN_ESP_BOOT);
	Chip_GPIO_SetPinState(LPC_GPIO, PORT_ESP_BOOT, PIN_ESP_BOOT, false);
	//2.) set RST pin low
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET);
	Chip_GPIO_SetPinState(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET, false);
	//3.) wait (in our case 1 tick -> 5ms)
	ticklocal = Chip_TIMER_ReadCount(LPC_TIMER32_0);
	while(Chip_TIMER_ReadCount(LPC_TIMER32_0) <= (ticklocal+512));
	//4.) release RST pin
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET);
	//5.) wait (in our case 1 tick -> 5ms)
	ticklocal = Chip_TIMER_ReadCount(LPC_TIMER32_0);
	while(Chip_TIMER_ReadCount(LPC_TIMER32_0) <= (ticklocal+512));
	//6.) release GPIO0
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT_ESP_BOOT, PIN_ESP_BOOT);
}
/**
 * @brief	Change ESP32 reset pin.
 * @return 	Nothing
 * @param	state If set to true, ESP32 will be in reset. ESP32 will start otherwise.
 */
void ESP32_Reset(bool state)
{
	if(state)
	{
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET);
		Chip_GPIO_SetPinState(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET, false);
	} else {
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET);
	}
}

/**
 * @brief Enable ESP32 by turning on power & release the reset line
 * @return Nothing
 */
static inline void ESP32_Enable(void)
{
	//the power mosfet must be driven high
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT_ESP_ON, PIN_ESP_ON);
	Chip_GPIO_SetPinState(LPC_GPIO, PORT_ESP_ON, PIN_ESP_ON, true);
	//the reset pin should be set to input, otherwise
	//we have to charge the 1uF capacitor via this pin, and loosing the delay
	//function of this capacitor
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET);
}

/**
 * @brief Disable ESP32 by pulling the reset line low and disabling the power
 * @return Nothing
 */
static inline void ESP32_Disable(void)
{
	//pull reset pin to low
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET);
	Chip_GPIO_SetPinState(LPC_GPIO, PORT_ESP_RESET, PIN_ESP_RESET, false);

	//the power mosfet is switched off
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, PORT_ESP_ON, PIN_ESP_ON);
	Chip_GPIO_SetPinState(LPC_GPIO, PORT_ESP_ON, PIN_ESP_ON, false);
}

static inline void ESP32_PinInit(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_RESET, PIN_ESP_RESET, IOCON_FUNC0 | IOCON_MODE_PULLUP);
	Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_ON, PIN_ESP_ON, IOCON_FUNC0 | IOCON_MODE_PULLUP);
	Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_BOOT, PIN_ESP_BOOT, IOCON_FUNC0 | IOCON_MODE_PULLUP);
}

/**
 * @brief Switch between UARTs.
 * @param output If UART_CDC is given, the CDC will send to UART0
 * which is used to transfer e.g. AT commands to the ESP32
 * If UART_ESP_DEBUG is given, the CDC will be used for bridging UART0 from ESP32,
 * which is used for debug outputs & programming.
 * @note UART init is not done here, must be initialized before calling this method.
 * @note This method assumes we started with UART_CDC.
 */
void ESP32_SwitchUART(int32_t output)
{
	//remember last state to avoid unnecessary UART changes
	static int32_t output_prev = UART_CDC;

	//if given parameter is the same as currently active
	//UART, return & do nothing
	if(output_prev == output) return;

	//first of all: disable UART.
	NVIC_DisableIRQ(UART0_IRQn);
	Chip_UART_TXDisable(LPC_USART);

	switch(output)
	{
		case UART_ESP_DEBUG:
			//disable ESP32's UART 1 by using pins as GPIOs.
			//note: the TX pin is used with a pullup.
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_TX, PIN_ESP_TX, IOCON_FUNC0 | IOCON_MODE_INACT);
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_RX, PIN_ESP_RX, IOCON_FUNC0 | IOCON_MODE_PULLUP);

			//enable ESP32's UART 0
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_DEBUG_TX, PIN_ESP_DEBUG_TX, IOCON_FUNC3 | IOCON_MODE_INACT);
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_DEBUG_RX, PIN_ESP_DEBUG_RX, IOCON_FUNC3 | IOCON_MODE_INACT);
			Chip_UART_SetBaud(LPC_USART, 115200);
		break;
		case UART_CDC:
			//disable ESP32's UART 0
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_DEBUG_TX, PIN_ESP_DEBUG_TX, IOCON_FUNC0 | IOCON_MODE_INACT);
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_DEBUG_RX, PIN_ESP_DEBUG_RX, IOCON_FUNC0 | IOCON_MODE_PULLUP);

			//enable ESP32's UART 1
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_TX, PIN_ESP_TX, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_18 used for RXD */
			Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_RX, PIN_ESP_RX, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_19 used for TXD */
			Chip_UART_SetBaud(LPC_USART, 230400);
		break;
		default: break;
	}

	//enable UART again.
	Chip_UART_TXEnable(LPC_USART);
	NVIC_EnableIRQ(UART0_IRQn);
}

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UART_IRQHandler(void)
{
	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */
	Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
}

/**
 * @brief	Handle interrupt from USB0
 * @return	Nothing
 */
void USB_IRQHandler(void)
{
	USBD_API->hw->ISR(g_hUsb);
}

/* Find the address of interface descriptor for given class type.  */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
	USB_COMMON_DESCRIPTOR *pD;
	USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
	uint32_t next_desc_adr;

	pD = (USB_COMMON_DESCRIPTOR *) pDesc;
	next_desc_adr = (uint32_t) pDesc;

	while (pD->bLength) {
		/* is it interface descriptor */
		if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

			pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
			/* did we find the right interface descriptor */
			if (pIntfDesc->bInterfaceClass == intfClass) {
				break;
			}
		}
		pIntfDesc = 0;
		next_desc_adr = (uint32_t) pD + pD->bLength;
		pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
	}

	return pIntfDesc;
}

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
	tick_count++;
}

/** @brief Handler for USB suspend event
 * In our case: switch off external MCU, by setting IO1_16 to low
 * @note ISR context, be fast!
 */
ErrorCode_t onSuspendHandler(USBD_HANDLE_T hUsb)
{
	ESP32_Disable();
	return LPC_OK;
}

/** @brief Handler for USB resume event
 * In our case: switch on external MCU, by setting IO1_16 to high
 * @note ISR context, be fast!
 */
ErrorCode_t onResumeHandler(USBD_HANDLE_T hUsb)
{
	ESP32_Enable();
	return LPC_OK;
}

void ADC_Init(void)
{
	static ADC_CLOCK_SETUP_T ADCSetup;
	//function 2 (IOCON) for channels 0-3 (shared with JTAG)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11, FUNC2);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, FUNC2);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13, FUNC2);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, FUNC2);
	//function 1 (IOCON) for channel 5!
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, FUNC1);
	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH2, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH3, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH5, ENABLE);
	//enable interrupt for channel 5
	//this way, we receive an IRQ when all 5 channels are finished
	Chip_ADC_Int_SetChannelCmd(LPC_ADC,ADC_CH5,ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
	Chip_ADC_SetBurstCmd(LPC_ADC,ENABLE);
}


/**
 * @brief	I2C Interrupt Handler
 * @return	None
 */
void I2C_IRQHandler(void)
{
	if (Chip_I2C_IsMasterActive(I2C0)) {
		Chip_I2C_MasterStateHandler(I2C0);
	}
	else {
		Chip_I2C_SlaveStateHandler(I2C0);
	}
}


/* Slave event handler for simulated EEPROM */
static void i2c_events(I2C_ID_T id, I2C_EVENT_T event)
{
	switch (event) {
	case I2C_EVENT_DONE:
		//we receive HID data
		i2c_xfer.rxBuff = hid_data;
		i2c_xfer.rxSz = sizeof(hid_data);
		//we send ADC data
		i2c_xfer.txBuff = adc_data;
		i2c_xfer.txSz = sizeof(adc_data);

		break;

	case I2C_EVENT_SLAVE_RX:
		//we got HID data -> activate parser (in main)
		hid_arrived = 1;
		break;

	case I2C_EVENT_SLAVE_TX:
		//don't need to do anything here.
		break;
	default: break;
	}
}

void ADC_MapToI2C(void)
{
	uint16_t temp = 0;
	//enter a critical section
	//we don't want to be interrupted
	//on updating data in array.
	//-> inconsistent data would be possible
	NVIC_DisableIRQ(I2C0_IRQn);
	//iterate all 4 FSR channels (up/down/left/right)
	for(uint8_t i = 0; i<=3; i++)
	{
		temp = 0;
		Chip_ADC_ReadValue(LPC_ADC,i,&temp);
		//split up into 8bit chunks
		adc_data[i*2] = temp & 0xFF;
		adc_data[i*2+1] = (temp & 0xFF00)>>8;
	}
	//read again the pressure sensor (Channel 5).
	temp = 0;
	Chip_ADC_ReadValue(LPC_ADC,5,&temp);
	adc_data[8] = temp & 0xFF;
	adc_data[9] = (temp & 0xFF00)>>8;
	//re-enable I2C interrupt
	NVIC_EnableIRQ(I2C0_IRQn);
}


void ADC_IRQHandler(void)
{
	//map the ADC data to the I2C array (disable I2C IRQ in between)
	ADC_MapToI2C();
}

void I2C_Init()
{
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
	//init SDA/SCL with FastPlus Bit
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, IOCON_FUNC1 | 0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, IOCON_FUNC1 | 0);
	/* Initialize I2C */
	Chip_I2C_Init(I2C0);
	Chip_I2C_SetClockRate(I2C0, 400000);
	//Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);
	NVIC_EnableIRQ(I2C0_IRQn);
	memset(adc_data, 0x00, sizeof(adc_data));
	memset(hid_data, 0x00, sizeof(hid_data));
	i2c_xfer.slaveAddr = (I2C_SLAVE_ADDR_0 << 1);
	//we receive HID data
	i2c_xfer.rxBuff = hid_data;
	i2c_xfer.rxSz = sizeof(hid_data);
	//we send ADC data
	i2c_xfer.txBuff = adc_data;
	i2c_xfer.txSz = sizeof(adc_data);
	Chip_I2C_SlaveSetup(I2C0, I2C_SLAVE_0, &i2c_xfer, i2c_events, 0);
}

ErrorCode_t USB_Init()
{
	ErrorCode_t ret = LPC_OK;
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	/* enable clocks and pinmux */
	usb_pin_clk_init();
	memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.USB_Resume_Event = onResumeHandler;
	usb_param.USB_Suspend_Event = onSuspendHandler;
	usb_param.usb_reg_base = LPC_USB0_BASE;
	usb_param.max_num_ep = 5;
	usb_param.mem_base = USB_STACK_MEM_BASE;
	usb_param.mem_size = USB_STACK_MEM_SIZE;

	/* Set the USB descriptors */
	desc.device_desc = (uint8_t *) USB_DeviceDescriptor;
	/* FLipMouse strings are loaded */
	desc.string_desc = (uint8_t *) USB_StringDescriptorFLipMouse;

	/* Note, to pass USBCV test full-speed only devices should have both
	 * descriptor arrays point to same location and device_qualifier set
	 * to 0.
	 */
	desc.high_speed_desc = USB_FsConfigDescriptor;
	desc.full_speed_desc = USB_FsConfigDescriptor;
	desc.device_qualifier = 0;

	/* USB Initialization */
	ret = USBD_API->hw->Init(&g_hUsb, &desc, &usb_param);
	if (ret == LPC_OK) {
		//initialize HID interface
		ret = HID_Init(g_hUsb,
				(USB_INTERFACE_DESCRIPTOR *) find_IntfDesc(desc.full_speed_desc,
						USB_DEVICE_CLASS_HUMAN_INTERFACE),
						&usb_param.mem_base, &usb_param.mem_size);
		if (ret == LPC_OK) {
			// Init CDC interface
			ret = vcom_init(g_hUsb, &desc, &usb_param);
			if (ret == LPC_OK) {
				//if everything is fine, enable USB IRQ
				NVIC_EnableIRQ(USB0_IRQn);
				//and set the USB HW to connect
				//normally this would setup the pull up resistor, not used in this case.
				USBD_API->hw->Connect(g_hUsb, 1);
				return LPC_OK;
			}
		}
	}
	//didn't succed, return fail.
	return ERR_FAILED;
}

void UART_Init()
{
	/*++++ Setup UART for 230400 8N1 - CDC communication ++++*/
	Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_TX, PIN_ESP_TX, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_18 used for RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, PORT_ESP_RX, PIN_ESP_RX, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_19 used for TXD */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 230400);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	//finally: enable
	Chip_UART_TXEnable(LPC_USART);

	//initialize UART RX/TX ringbuffers
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RXB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_TXB_SIZE);

	//  Enable receive data and line status interrupt
	Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(UART0_IRQn, 1);
	NVIC_EnableIRQ(UART0_IRQn);
}


/**
 * @brief Main for USB CDC+HID to serial bridge for FLipMouse/FABI with ESP32
 *
 * The main method initialises:
 * * the USBD ROM stack
 * * UART for receiving CDC data or HID commands (determined by GPIO)
 * * Timer for measuring a timeout before processing HID commands
 * * Handle data from/to CDC and HID
 *
 * @return	Function should not exit.
 */
int main(void)
{

	ErrorCode_t ret = LPC_OK;
	//flag for connection status (0 not connected, != 0 connected)
	//currently read bytes from CDC
	uint32_t prompt = 0, rdCnt = 0;
	uint32_t inBootMode = 0;

	/* Initialize GPIOs and turn off ESP32*/
	Chip_GPIO_Init(LPC_GPIO);
	/* Initialize ESP reset related GPIOs */
	ESP32_PinInit();
	ESP32_Disable();

	/* Initialize ADC & I2C */
	ADC_Init();
	I2C_Init();

	/* Initialize USB */
	ret = USB_Init();

	/* Initialize UART */
	UART_Init();

	/*++++ Init systick ++++*/
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 200);

	/*++++ Boot up the ESP32. ++++*/
	//TODO: just for testing, set automatically to UART0 of ESP32.
	//ESP32_SwitchUART(UART_ESP_DEBUG);
	//ESP32_Reset(true);
	ESP32_Enable();
	//ESP32_ResetBootloader();

	while (1) {
		/* If everything went well with stack init do the tasks or else sleep */
		if (ret == LPC_OK) {
			/* Do HID tasks (mouse, keyboard and joystick*/
			if(hid_arrived != 0) //if a HID command was received
			{
				hid_arrived = 0; //reset flag for next packet
				parseBuffer(hid_data, sizeof(hid_data)); //parse buffer for HID commands
			}
			HID_Tasks(); //perform USB-HID tasks

			//check if connection is still existing
			//if not, set our flag here to false
			if(vcom_connected() == 0) prompt = 0;

			/* Check if host has connected and opened the VCOM port */
			if ((vcom_connected() != 0) && (prompt == 0)) {
				//if yes, flush all buffers
				uint8_t b;
				while(vcom_bread(&b, 1) != 0); //flush VCOM in
				//flush UART RBs
				RingBuffer_Flush(&txring);
				RingBuffer_Flush(&rxring);
				//flush buffers
				memset(g_rxBuff,0,sizeof(g_rxBuff));

				//and set connected flag
				prompt = 1;
			}

			//if connected
			if(prompt)
			{
				/* read incoming bytes from UART & send to CDC if something is available*/
				//rdCnt = Chip_UART_ReadRB(LPC_USART, &rxring, &g_rxBuff[0], UART_TXB_SIZE);
				//according to:
				//https://community.nxp.com/thread/429714
				//we need to limit to 63B
				rdCnt = Chip_UART_ReadRB(LPC_USART, &rxring, &g_rxBuff[0], 63);
				//TODO: test return value (sent bytes)
				if(rdCnt != 0) vcom_write(&g_rxBuff[0], rdCnt);
			}

			//check if we have to switch to programming the ESP32:
			//first we check if 2.56s are passed
			/*if((tick_count % 512) == 0)
			{
				//if 2s are passed, check if we reached the rts/dts trigger threshold
				//but just reset once.
				if((rts_dts_count > 4) && (inBootMode == 0))
				{
					//
					inBootMode = 1;
					//switch to programming the ESP32
					ESP32_SwitchUART(UART_ESP_DEBUG);
					ESP32_ResetBootloader();
				}
				//reset count (otherwise we might switch to ESP
				//programming mode by opening/closing the CDC port too often)
				rts_dts_count = 0;
			}*/
		} else {
			//TODO: do some error recovering if init didn't work.
			//maybe resetting or reset some settings?!?

			//maybe we do this via the watchdog:
			//feed em wrong -> reset.
		}

		/* Sleep until next IRQ happens */
		__WFI();
	}
}
