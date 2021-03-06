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
#include "board.h"
#include <stdint.h>
#include <string.h>
#include "usbd_rom_api.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define FLAG_MOUSE 		(1<<0)
#define FLAG_KEYBOARD   (1<<1)
#define FLAG_JOYSTICK	(1<<2)

/**
 * @brief Structure to hold mouse data
 */

typedef struct {
	USBD_HANDLE_T hUsb;	/*!< Handle to USB stack. */
	uint8_t reportMouse[MOUSE_REPORT_SIZE];	/*!< Last report data  */
	uint8_t reportKeyboard[KEYBOARD_REPORT_SIZE];	/*!< Last report data  */
	uint8_t reportJoystick[JOYSTICK_REPORT_SIZE];	/*!< Last report data  */
	uint8_t tx_busy;	/*!< Flag indicating whether a report is pending in endpoint queue. */
	uint8_t tx_flags;   //flag signalling which HID endpoint needs sending (1<<0 mouse, 1<<1 keyboard, 1<<2 joystick)
	uint8_t nextsend;	//which report has to be sent next (equal to reportid)
	uint8_t tx_autoclear;	//length of next sent report
} Mouse_Ctrl_T;

/** Singleton instance of mouse control */
static Mouse_Ctrl_T g_mouse;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

extern const uint8_t HID_ReportDescriptor[];
extern const uint16_t HID_ReportDescSize;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

void Keyboard_UpdateReport(uint8_t *reportData, uint8_t autoClear)
{
	//keyboard report structure:
	//[0] reportID
	//[1] modifier
	//[2] reserved
	//[3]-[8] keycodes
	g_mouse.reportKeyboard[0] = HID_REPORTID_KEYBOARD;
	g_mouse.reportKeyboard[1] = reportData[0];
	memcpy(&g_mouse.reportKeyboard[3], &reportData[1], KEYBOARD_REPORT_SIZE-3);
	//set flag to update report (send to host)
	g_mouse.tx_flags |= FLAG_KEYBOARD;
	//if autoclear is set, the HID task clears the report after this one is sent
	if(autoClear) g_mouse.tx_autoclear |= FLAG_KEYBOARD;
	else g_mouse.tx_autoclear &= ~FLAG_KEYBOARD;
	//call HID_Task
	//if USB is active, nothing will be done (update will be sent on next interrupt)
	//if not, a transmission will be started
	HID_Tasks();
}

void Joystick_UpdateReport(uint8_t *reportData)
{
	//joystick report structure:
	//TBA
	g_mouse.reportJoystick[0] = HID_REPORTID_JOYSTICK;
	memcpy(&g_mouse.reportJoystick[1], reportData, JOYSTICK_REPORT_SIZE-1);
	//set flag to update report (send to host)
	g_mouse.tx_flags |= FLAG_JOYSTICK;
	//call HID_Task
	//if USB is active, nothing will be done (update will be sent on next interrupt)
	//if not, a transmission will be started
	HID_Tasks();
}

void UpdateReport(void)
{
	//Update a general report, requested by host...
}

/* Routine to update mouse state report */
void Mouse_UpdateReport(uint8_t *reportData, uint8_t autoClear)
{
	static uint8_t buttons_prev = 0;
	//mouse report structure:
	//[0] reportID
	//[1] buttons -> if this byte is set to 0xFF, previous button mask will be used...
	//[2/3] X/Y (-127/127)
	//[4] mouse scroll wheel (-127/127)
	g_mouse.reportMouse[0] = HID_REPORTID_MOUSE;
	memcpy(&g_mouse.reportMouse[1], reportData, MOUSE_REPORT_SIZE-1);
	//sticky buttons
	if(g_mouse.reportMouse[1] == 0xFF) g_mouse.reportMouse[1] = buttons_prev;
	else buttons_prev = g_mouse.reportMouse[1];

	//set flag to update report (send to host)
	g_mouse.tx_flags |= FLAG_MOUSE;
	//if autoclear is set, the HID task clears the report after this one is sent
	if(autoClear) g_mouse.tx_autoclear |= FLAG_MOUSE;
	else g_mouse.tx_autoclear &= ~FLAG_MOUSE;
	//call HID_Task
	//if USB is active, nothing will be done (update will be sent on next interrupt)
	//if not, a transmission will be started
	HID_Tasks();
}

/* HID Get Report Request Callback. Called automatically on HID Get Report Request */
static ErrorCode_t Mouse_GetReport(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t * *pBuffer, uint16_t *plength)
{
	/* ReportID = SetupPacket.wValue.WB.L; */
	switch (pSetup->wValue.WB.H) {
	case HID_REPORT_INPUT:
		UpdateReport();
		//TODO: does this work?
		switch(pSetup->wValue.WB.L)
		{
			case HID_REPORTID_MOUSE:
				*pBuffer = &g_mouse.reportMouse[0];
				*plength = MOUSE_REPORT_SIZE;
				break;
			case HID_REPORTID_KEYBOARD:
				*pBuffer = &g_mouse.reportKeyboard[0];
				*plength = KEYBOARD_REPORT_SIZE;
				break;
			case HID_REPORTID_JOYSTICK:
				*pBuffer = &g_mouse.reportJoystick[0];
				*plength = JOYSTICK_REPORT_SIZE;
			//unkown next sending...
			default:
				return ERR_FAILED;
		}
		break;

	case HID_REPORT_OUTPUT:				/* Not Supported */
	case HID_REPORT_FEATURE:			/* Not Supported */
		return ERR_USBD_STALL;
	}
	return LPC_OK;
}

/* HID Set Report Request Callback. Called automatically on HID Set Report Request */
static ErrorCode_t Mouse_SetReport(USBD_HANDLE_T hHid, USB_SETUP_PACKET *pSetup, uint8_t * *pBuffer, uint16_t length)
{
	/* we will reuse standard EP0Buf */
	if (length == 0) {
		return LPC_OK;
	}
	/* ReportID = SetupPacket.wValue.WB.L; */
	switch (pSetup->wValue.WB.H) {
	case HID_REPORT_INPUT:				/* Not Supported */
	case HID_REPORT_OUTPUT:				/* Not Supported */
	case HID_REPORT_FEATURE:			/* Not Supported */
		return ERR_USBD_STALL;
	}
	return LPC_OK;
}

/* HID interrupt IN endpoint handler. */
static ErrorCode_t Mouse_EpIN_Hdlr(USBD_HANDLE_T hUsb, void *data, uint32_t event)
{
	switch (event) {
	case USB_EVT_IN:
		/* USB_EVT_IN occurs when HW completes sending IN packet. So clear the
		    busy flag for main loop to queue next packet.
		 */
		g_mouse.tx_busy = 0;
		break;
	}
	return LPC_OK;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Mouse init routine. */
ErrorCode_t HID_Init(USBD_HANDLE_T hUsb,
					   USB_INTERFACE_DESCRIPTOR *pIntfDesc,
					   uint32_t *mem_base,
					   uint32_t *mem_size)
{
	USBD_HID_INIT_PARAM_T hid_param;
	USB_HID_REPORT_T reports_data[1];
	ErrorCode_t ret = LPC_OK;

	/* Do a quick check of if the interface descriptor passed is the right one. */
	if ((pIntfDesc == 0) || (pIntfDesc->bInterfaceClass != USB_DEVICE_CLASS_HUMAN_INTERFACE)) {
		return ERR_FAILED;
	}

	/* Init HID params */
	memset((void *) &hid_param, 0, sizeof(USBD_HID_INIT_PARAM_T));
	hid_param.max_reports = 1;
	hid_param.mem_base = *mem_base;
	hid_param.mem_size = *mem_size;
	hid_param.intf_desc = (uint8_t *) pIntfDesc;
	/* user defined functions */
	hid_param.HID_GetReport = Mouse_GetReport;
	hid_param.HID_SetReport = Mouse_SetReport;
	hid_param.HID_EpIn_Hdlr  = Mouse_EpIN_Hdlr;
	/* Init reports_data */
	reports_data[0].len = HID_ReportDescSize;
	reports_data[0].idle_time = 0;
	reports_data[0].desc = (uint8_t *) &HID_ReportDescriptor[0];
	hid_param.report_data  = reports_data;

	ret = USBD_API->hid->init(hUsb, &hid_param);

	/* update memory variables */
	*mem_base = hid_param.mem_base;
	*mem_size = hid_param.mem_size;
	/* store stack handle for later use. */
	g_mouse.hUsb = hUsb;

	return ret;
}

/* reset the USB device (necessary for re-evaluation of the HID country code */
void USB_resetdevice(void)
{
	mwUSB_ResetCore(g_mouse.hUsb);
}

void HID_waitIdle(void)
{
	while(g_mouse.tx_busy);
}

/* HID tasks routine. */
void HID_Tasks(void)
{
	/* check device is configured before sending report. */

	if ( USB_IsConfigured(g_mouse.hUsb)) {
		if (g_mouse.tx_busy == 0) {
			/* update report based on board state */
			UpdateReport();
			/* send report data */
			if((g_mouse.tx_flags & FLAG_MOUSE) == FLAG_MOUSE)
			{
				//active
				g_mouse.tx_busy = 1;
				//send report
				USBD_API->hw->WriteEP(g_mouse.hUsb, HID_EP_IN, &g_mouse.reportMouse[0], MOUSE_REPORT_SIZE);
				//clear flag
				g_mouse.tx_flags &= ~(FLAG_MOUSE);
				//return, we cannot send 2 reports at once.
				return;
			}
			if((g_mouse.tx_flags & FLAG_KEYBOARD) == FLAG_KEYBOARD)
			{
				//active
				g_mouse.tx_busy = 1;
				//send report
				USBD_API->hw->WriteEP(g_mouse.hUsb, HID_EP_IN, &g_mouse.reportKeyboard[0], KEYBOARD_REPORT_SIZE);
				//clear flag
				g_mouse.tx_flags &= ~(FLAG_KEYBOARD);
				//return, we cannot send 2 reports at once.
				return;
			}
			if((g_mouse.tx_flags & FLAG_JOYSTICK) == FLAG_JOYSTICK)
			{
				//active
				g_mouse.tx_busy = 1;
				//send report
				USBD_API->hw->WriteEP(g_mouse.hUsb, HID_EP_IN, &g_mouse.reportJoystick[0], JOYSTICK_REPORT_SIZE);
				//clear flag
				g_mouse.tx_flags &= ~(FLAG_JOYSTICK);
				//return, we cannot send 2 reports at once.
				return;
			}
		}
	} else {
		/* reset busy flag if we get disconnected. */
		g_mouse.tx_busy = 0;
	}
}
