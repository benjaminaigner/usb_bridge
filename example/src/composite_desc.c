/*
 * @brief This file contains USB composite class example using USBD_lib stack.
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

#include "app_usbd_cfg.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/**
 * HID Report Descriptor, including mouse, keyboard and joystick
 */
const uint8_t HID_ReportDescriptor[] = {
	/*++++ HID mouse ++++*/
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),
	HID_Usage(HID_USAGE_GENERIC_MOUSE),
	HID_Collection(HID_Application),
		HID_ReportID(HID_REPORTID_MOUSE),
		HID_Usage(HID_USAGE_GENERIC_POINTER),
		HID_Collection(HID_Physical),
			HID_UsagePage(HID_USAGE_PAGE_BUTTON),
			HID_UsageMin(1),
			HID_UsageMax(8),
			HID_LogicalMin(0),
			HID_LogicalMax(1),
			HID_ReportCount(8),
			HID_ReportSize(1),
			HID_Input(HID_Data | HID_Variable | HID_Absolute),
			//HID_ReportCount(1),
			//HID_ReportSize(5),
			//HID_Input(HID_Constant),
			HID_UsagePage(HID_USAGE_PAGE_GENERIC),
			HID_Usage(HID_USAGE_GENERIC_X),
			HID_Usage(HID_USAGE_GENERIC_Y),
			HID_Usage(HID_USAGE_GENERIC_WHEEL),
			HID_LogicalMin( (uint8_t) -127),
			HID_LogicalMax(127),
			HID_ReportSize(8),
			HID_ReportCount(3),
			HID_Input(HID_Data | HID_Variable | HID_Relative),
		HID_EndCollection,
	HID_EndCollection,
	/*++++ HID keyboard ++++*/
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),
	HID_Usage(HID_USAGE_GENERIC_KEYBOARD),
	HID_Collection(HID_Application),
		HID_ReportID(HID_REPORTID_KEYBOARD),
		HID_ReportSize(1),
		HID_ReportCount(8),
		HID_UsagePage(HID_USAGE_PAGE_KEYBOARD),
		HID_UsageMin(224),
		HID_UsageMax(231),
		HID_LogicalMin(0),
		HID_LogicalMax(1),
		HID_Input(HID_Data | HID_Variable | HID_Absolute),
		HID_ReportCount(1),
		HID_ReportSize(8),
		HID_Input(0x03), //from usb.c from Teensyduino
		HID_ReportCount(5),
		HID_ReportSize(1),
		HID_UsagePage(HID_USAGE_PAGE_LED),
		HID_UsageMin(1),
		HID_UsageMax(5),
		HID_Output(HID_Data | HID_Variable | HID_Absolute),
		HID_ReportCount(1),
		HID_ReportSize(3),
		HID_Output(0x03),
		HID_ReportCount(6),
		HID_ReportSize(8),
		HID_LogicalMin(0),
		HID_LogicalMax(104),
		HID_UsagePage(HID_USAGE_PAGE_KEYBOARD),
		HID_UsageMin(0),
		HID_UsageMax(104),
		HID_Input(HID_Data | HID_Array),
	HID_EndCollection,
	/*++++ HID joystick ++++*/
	//according to teensy USB_hid -> usb.c
	//Thanks again Paul Stoffregen (PJRC), you are a hero!!!
	HID_UsagePage(HID_USAGE_PAGE_GENERIC),
	HID_Usage(HID_USAGE_GENERIC_JOYSTICK),
	HID_Collection(HID_Application),
		HID_ReportID(HID_REPORTID_JOYSTICK),
		HID_LogicalMin(0),
		HID_LogicalMax(1),
		HID_ReportSize(1),
		HID_ReportCount(32),
		HID_UsagePage(HID_USAGE_PAGE_BUTTON),
		HID_UsageMin(1),
		HID_UsageMax(32),
		HID_Input(HID_Variable | HID_Absolute),
		HID_LogicalMin(0),
		HID_LogicalMax(7),
		HID_PhysicalMin(0),
		HID_PhysicalMaxS(315),
		HID_ReportSize(4),
		HID_ReportCount(1),
		HID_Unit(20),
		HID_UsagePage(HID_USAGE_PAGE_GENERIC),
		HID_Usage(HID_USAGE_GENERIC_HATSWITCH),
		HID_Input(HID_Variable | HID_Absolute | HID_NullState),
		HID_UsagePage(HID_USAGE_PAGE_GENERIC),
		HID_Usage(HID_USAGE_GENERIC_POINTER),
		HID_Collection(0),
		HID_LogicalMin(0),
		HID_LogicalMaxL(1023),
		HID_ReportSize(10),
		HID_ReportCount(4),
		HID_Usage(HID_USAGE_GENERIC_X),
		HID_Usage(HID_USAGE_GENERIC_Y),
		HID_Usage(HID_USAGE_GENERIC_Z),
		HID_Usage(HID_USAGE_GENERIC_RZ),
		HID_Input(HID_Variable | HID_Absolute),
		HID_EndCollection,
		HID_LogicalMin(0),
		HID_LogicalMaxL(1023),
		HID_ReportSize(10),
		HID_ReportCount(2),
		HID_Usage(HID_USAGE_GENERIC_SLIDER),
		HID_Usage(HID_USAGE_GENERIC_SLIDER),
		HID_Input(HID_Variable | HID_Absolute),
	HID_EndCollection,
};
const uint16_t HID_ReportDescSize = sizeof(HID_ReportDescriptor);

/**
 * USB Standard Device Descriptor
 */
ALIGNED(4) const uint8_t USB_DeviceDescriptor[] = {
	USB_DEVICE_DESC_SIZE,			/* bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(0x0200),					/* bcdUSB : 2.00*/
	0x00,							/* bDeviceClass */
	0x00,							/* bDeviceSubClass */
	0x00,							/* bDeviceProtocol */
	USB_MAX_PACKET0,				/* bMaxPacketSize0 */
	WBVAL(0x1FC9),					/* idVendor */
	WBVAL(0x0087),					/* idProduct */
	WBVAL(0x0100),					/* bcdDevice : 1.00 */
	0x01,							/* iManufacturer */
	0x02,							/* iProduct */
	0x03,							/* iSerialNumber */
	0x01							/* bNumConfigurations */
};

/**
 * USB FSConfiguration Descriptor
 * All Descriptors (Configuration, Interface, Endpoint, Class, Vendor)
 * WARNING: if USB_FsConfigDescriptor is changed, the index in parser.c (referencing bCountryCode) MUST be replaced!
 */
ALIGNED(4) uint8_t USB_FsConfigDescriptor[] = {
	/* Configuration 1 */
	USB_CONFIGURATION_DESC_SIZE,			/* bLength */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(									/* wTotalLength */
		USB_CONFIGURATION_DESC_SIZE     +
		/* HID class related descriptors */
		USB_INTERFACE_DESC_SIZE         +
		HID_DESC_SIZE                   +
		USB_ENDPOINT_DESC_SIZE          +
		/* CDC class related descriptors */
		USB_INTERFACE_ASSOC_DESC_SIZE   +	/* interface association descriptor */
		USB_INTERFACE_DESC_SIZE         +	/* communication control interface */
		0x0013                          +	/* CDC functions */
		1 * USB_ENDPOINT_DESC_SIZE      +	/* interrupt endpoint */
		USB_INTERFACE_DESC_SIZE         +	/* communication data interface */
		2 * USB_ENDPOINT_DESC_SIZE      +	/* bulk endpoints */
		0
		),
	0x03,							/* bNumInterfaces */
	0x01,							/* bConfigurationValue */
	0x00,							/* iConfiguration */
	USB_CONFIG_BUS_POWERED,		/* bmAttributes */
	USB_CONFIG_POWER_MA(500),			/* bMaxPower */

	/* Interface 0, Alternate Setting 0, HID Class */
	USB_INTERFACE_DESC_SIZE,		/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,	/* bDescriptorType */
	USB_HID_IF_NUM,					/* bInterfaceNumber */
	0x00,							/* bAlternateSetting */
	0x01,							/* bNumEndpoints */
	USB_DEVICE_CLASS_HUMAN_INTERFACE,	/* bInterfaceClass */
	0x00, /*HID_SUBCLASS_BOOT,*/				/* bInterfaceSubClass */
	0x00, /*HID_PROTOCOL_MOUSE,*/				/* bInterfaceProtocol */
	0x04,							/* iInterface */
	/* HID Class Descriptor */
	/* HID_DESC_OFFSET = 0x0012 */
	HID_DESC_SIZE,					/* bLength */
	HID_HID_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(0x0111),					/* bcdHID : 1.11*/
	//WARNING: if USB_FsConfigDescriptor is changed, the index in parser.c (referencing to this position) MUST be replaced!
	0x00,							/* bCountryCode */
	0x01,							/* bNumDescriptors */
	HID_REPORT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(sizeof(HID_ReportDescriptor)),	/* wDescriptorLength */
	/* Endpoint, HID Interrupt In */
	USB_ENDPOINT_DESC_SIZE,			/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType */
	HID_EP_IN,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_INTERRUPT,	/* bmAttributes */
	//WBVAL(0x0008),					/* wMaxPacketSize */
	WBVAL(16), /* wMaxPacketSize */
	HID_MOUSE_REPORT_INTERVAL,		/* bInterval */

	/* Interface association descriptor IAD*/
	USB_INTERFACE_ASSOC_DESC_SIZE,		/* bLength */
	USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,	/* bDescriptorType */
	USB_CDC_CIF_NUM,					/* bFirstInterface */
	0x02,								/* bInterfaceCount */
	CDC_COMMUNICATION_INTERFACE_CLASS,	/* bFunctionClass */
	CDC_ABSTRACT_CONTROL_MODEL,			/* bFunctionSubClass */
	0x00,								/* bFunctionProtocol */
	0x05,								/* iFunction */

	/* Interface 1, Alternate Setting 0, Communication class interface descriptor */
	USB_INTERFACE_DESC_SIZE,			/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_CIF_NUM,					/* bInterfaceNumber: Number of Interface */
	0x00,								/* bAlternateSetting: Alternate setting */
	0x01,								/* bNumEndpoints: One endpoint used */
	CDC_COMMUNICATION_INTERFACE_CLASS,	/* bInterfaceClass: Communication Interface Class */
	CDC_ABSTRACT_CONTROL_MODEL,			/* bInterfaceSubClass: Abstract Control Model */
	0x00,								/* bInterfaceProtocol: no protocol used */
	0x05,								/* iInterface: */
	/* Header Functional Descriptor*/
	0x05,								/* bLength: CDC header Descriptor size */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_HEADER,							/* bDescriptorSubtype: Header Func Desc */
	WBVAL(CDC_V1_10),					/* bcdCDC 1.10 */
	/* Call Management Functional Descriptor*/
	0x05,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_CALL_MANAGEMENT,				/* bDescriptorSubtype: Call Management Func Desc */
	0x01,								/* bmCapabilities: device handles call management */
	USB_CDC_DIF_NUM,					/* bDataInterface: CDC data IF ID */
	/* Abstract Control Management Functional Descriptor*/
	0x04,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_ABSTRACT_CONTROL_MANAGEMENT,	/* bDescriptorSubtype: Abstract Control Management desc */
	0x02,								/* bmCapabilities: SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported */
	/* Union Functional Descriptor*/
	0x05,								/* bFunctionLength */
	CDC_CS_INTERFACE,					/* bDescriptorType: CS_INTERFACE */
	CDC_UNION,							/* bDescriptorSubtype: Union func desc */
	USB_CDC_CIF_NUM,					/* bMasterInterface: Communication class interface is master */
	USB_CDC_DIF_NUM,					/* bSlaveInterface0: Data class interface is slave 0 */
	/* Endpoint 1 Descriptor*/
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_INT_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_INTERRUPT,		/* bmAttributes */
	WBVAL(0x0010),						/* wMaxPacketSize */
	0x02,			/* 2ms */           /* bInterval */

	/* Interface 2, Alternate Setting 0, Data class interface descriptor*/
	USB_INTERFACE_DESC_SIZE,			/* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_DIF_NUM,					/* bInterfaceNumber: Number of Interface */
	0x00,								/* bAlternateSetting: no alternate setting */
	0x02,								/* bNumEndpoints: two endpoints used */
	CDC_DATA_INTERFACE_CLASS,			/* bInterfaceClass: Data Interface Class */
	0x00,								/* bInterfaceSubClass: no subclass available */
	0x00,								/* bInterfaceProtocol: no protocol used */
	0x05,								/* iInterface: */
	/* Endpoint, EP Bulk Out */
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_OUT_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_BULK,				/* bmAttributes */
	WBVAL(USB_FS_MAX_BULK_PACKET),		/* wMaxPacketSize */
	0x00,								/* bInterval: ignore for Bulk transfer */
	/* Endpoint, EP Bulk In */
	USB_ENDPOINT_DESC_SIZE,				/* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,		/* bDescriptorType */
	USB_CDC_IN_EP,						/* bEndpointAddress */
	USB_ENDPOINT_TYPE_BULK,				/* bmAttributes */
	WBVAL(64),							/* wMaxPacketSize */
	0x00,								/* bInterval: ignore for Bulk transfer */

	/* Terminator */
	0								/* bLength */
};

/**
 * USB String Descriptor for FLipMouse device
 */
const uint8_t USB_StringDescriptorFLipMouse[] = {
	/* Index 0x00: LANGID Codes */
	0x04,							/* bLength */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	WBVAL(0x0409),					/* wLANGID : US English */
	/* Index 0x01: Manufacturer */
	(19 * 2 + 2),					/* bLength (19 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'A', 0,
	's', 0,
	'T', 0,
	'e', 0,
	'R', 0,
	'I', 0,
	'C', 0,
	'S', 0,
	' ', 0,
	'F', 0,
	'o', 0,
	'u', 0,
	'n', 0,
	'd', 0,
	'a', 0,
	't', 0,
	'i', 0,
	'o', 0,
	'n', 0,
	/* Index 0x02: Product */
	(14 * 2 + 2),					/* bLength (14 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'F', 0,
	'L', 0,
	'i', 0,
	'p', 0,
	'M', 0,
	'o', 0,
	'u', 0,
	's', 0,
	'e', 0,
	'/', 0,
	'F', 0,
	'A', 0,
	'B', 0,
	'I', 0,
	/* Index 0x03: Serial Number */
	(4 * 2 + 2),					/* bLength (13 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'v', 0,
	'3', 0,
	'.', 0,
	'0', 0,
	/* Index 0x04: Interface 0, Alternate Setting 0 */
	(23 * 2 + 2),					/* bLength (9 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,		/* bDescriptorType */
	'M', 0,
	'o', 0,
	'u', 0,
	's', 0,
	'e', 0,
	'/', 0,
	'K', 0,
	'e', 0,
	'y', 0,
	'b', 0,
	'o', 0,
	'a', 0,
	'r', 0,
	'd', 0,
	'/', 0,
	'J', 0,
	'o', 0,
	'y', 0,
	's', 0,
	't', 0,
	'i', 0,
	'c', 0,
	'k', 0,
	/* Index 0x05: Interface 1, Alternate Setting 0 */
	( 6 * 2 + 2),						/* bLength (4 Char + Type + lenght) */
	USB_STRING_DESCRIPTOR_TYPE,			/* bDescriptorType */
	'S', 0,
	'e', 0,
	'r', 0,
	'i', 0,
	'a', 0,
	'l', 0,
};
