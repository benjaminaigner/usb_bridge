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

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive ring buffer sizes */
#define UART_TXB_SIZE 256	/* Send */
#define UART_RXB_SIZE 256	/* Receive */
#define ATCMD_LENGTH  256	/* Receive */

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RXB_SIZE], txbuff[UART_TXB_SIZE];


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

static uint8_t cimMode = 0;
static uint8_t uartidleflag = 0;
static USBD_HANDLE_T g_hUsb;
static uint8_t g_rxBuff[UART_TXB_SIZE];
static uint8_t g_txBuff[ATCMD_LENGTH];

extern const  USBD_HW_API_T hw_api;
extern const  USBD_CORE_API_T core_api;
extern const  USBD_HID_API_T hid_api;
extern const  USBD_CDC_API_T cdc_api;
/* Since this example only uses HID class link functions for that class only */
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
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UART_IRQHandler(void)
{
	/* Reset timer 0 */
	Chip_TIMER_Reset(LPC_TIMER32_0);
	uartidleflag = 0;
	Chip_TIMER_Enable(LPC_TIMER32_0);
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

void TIMER32_0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER32_0, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER32_0, 1);
		/* if UART input direction is set to HID, set idle flag */
		uartidleflag = 1;
		Chip_TIMER_Disable(LPC_TIMER32_0);
	}
}

/** @brief Handler for USB suspend event
 * In our case: switch off external MCU, by setting IO1_16 to low
 * @note ISR context, be fast!
 */
ErrorCode_t onSuspendHandler(USBD_HANDLE_T hUsb)
{
	Chip_GPIO_SetPinState(LPC_GPIO, 1, 16, false);
	return LPC_OK;
}

/** @brief Handler for USB resume event
 * In our case: switch on external MCU, by setting IO1_16 to high
 * @note ISR context, be fast!
 */
ErrorCode_t onResumeHandler(USBD_HANDLE_T hUsb)
{
	Chip_GPIO_SetPinState(LPC_GPIO, 1, 16, true);
	return LPC_OK;
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
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret = LPC_OK;
	//flag for connection status (0 not connected, != 0 connected)
	//currently read bytes from CDC
	uint32_t prompt = 0, rdCnt = 0;

	/* enable clocks and pinmux */
	usb_pin_clk_init();

	/* Initialise call back structures */
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
			}
		}
	}


	/* Initialize GPIOs */
	Chip_GPIO_Init(LPC_GPIO);

	/*++++ Input signal pin for CDC/HID directing ++++*/
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 17, IOCON_FUNC0 | IOCON_MODE_PULLUP);/* PIO0_17 used for UART function determination */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 17);	/* set PIO0_17 as input */

	/*++++ ESP32 power switch pin ++++*/
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 16);
	Chip_GPIO_SetPinState(LPC_GPIO, 1, 16, true);


	/*++++ Setup UART for 115200 8N1 ++++*/
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 18, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_18 used for RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 19, IOCON_FUNC1 | IOCON_MODE_INACT);	/* PIO0_19 used for TXD */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
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

	/*++++ Setup timer 0 for UART timeout */
	Chip_TIMER_Init(LPC_TIMER32_0);
	// Timer rate is system clock rate
	uint32_t timerFreq = Chip_Clock_GetSystemClockRate();

	// Timer setup for match and interrupt at TICKRATE_HZ
	Chip_TIMER_Reset(LPC_TIMER32_0);
	Chip_TIMER_MatchEnableInt(LPC_TIMER32_0, 1);
	//wait 2ms (500Hz) for an UART idle signal
	//wait 10ms (100Hz) for an UART idle signal
	Chip_TIMER_SetMatch(LPC_TIMER32_0, 1, (timerFreq / 100));
	NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);
	NVIC_EnableIRQ(TIMER_32_0_IRQn);

	/// @todo do something on HID coutnry code (keyboard), feature request. Hard to find offset in uint8 array...

	while (1) {
		/* If everything went well with stack init do the tasks or else sleep */
		if (ret == LPC_OK) {
			/* Do HID tasks (mouse, keyboard and joystick*/
			HID_Tasks();

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
				memset(g_txBuff,0,sizeof(g_txBuff));

				//and set connected flag
				prompt = 1;
				//reset AT parser & cim mode
				cimMode = 0;
			}


			/* either parse as keyboard/mouse/joystick commands or send to vcom */
			if(Chip_GPIO_GetPinState(LPC_GPIO, 0, 17) == false)	{
				if(uartidleflag == 1) {
					rdCnt = Chip_UART_ReadRB(LPC_USART, &rxring, &g_rxBuff[0], UART_TXB_SIZE);
					parseBuffer(g_rxBuff,rdCnt);
				} else {
					rdCnt = 0;
				}
				if(rdCnt != 0) uartidleflag = 0;
			} else {
				/* read incoming bytes from UART */
				rdCnt = Chip_UART_ReadRB(LPC_USART, &rxring, &g_rxBuff[0], UART_TXB_SIZE);
				//if connected to vcom and bytes were in the ringbuffer, send to vcom
				if(prompt && rdCnt != 0) vcom_write(&g_rxBuff[0], rdCnt);
			}

			//independent from UART direction select:
			//always send received data from CDC to UART
			if(prompt)
			{
				//check if buffer has at least 50% free, and read then
				if(RingBuffer_GetFree(&txring) > (UART_TXB_SIZE / 2))
				{
					//read from USB, send to UART ringbuffer
					rdCnt = vcom_bread(g_txBuff, RingBuffer_GetFree(&txring));
					Chip_UART_SendRB(LPC_USART, &txring, g_txBuff, rdCnt);
				}
			}
		}

		/* Sleep until next IRQ happens */
		__WFI();
	}
}
