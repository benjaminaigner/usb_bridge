/*
 * parser.c
 *
 *  Created on: 29 Nov 2017
 *      Author: beni
 */

#include <parser.h>

uint8_t parseBuffer(uint8_t *buf, uint8_t len)
{
	uint8_t testdata[16]; //array for test data
	//we do relatively simple parsing:
	//determine HID function by first character:
	//'K' for a keyboard report, parameter are 7 bytes (1 modifier, 6 keycodes, 0 for unused)
	//'M' for a mouse report: parameter are 4 bytes (buttons, X/Y, wheel)
	//'J' for a joystick report:
		// [Byte]		[Function]
		// [0]			button mask 1 (buttons 0-7)
		// [1]			button mask 2 (buttons 8-15)
		// [2]			button mask 3 (buttons 16-23)
		// [3]			button mask 4 (buttons 24-31)
		// [4]			bit 0-3: hat
		// [4]			bit 4-7: X axis low bits
		// [5]			bit 0-5: X axis high bits
		// [5]			bit 6-7: Y axis low bits
		// [6]			bit 0-7: Y axis high bits
		// [7]			bit 0-7: Z axis low bits
		// [8]			bit 0-1: Z axis high bits
		// [8]			bit 2-7: Z rotate low bits
		// [9]			bit 0-3: Z rotate high bits
		// [9]			bit 4-7: slider left low bits
		// [10]			bit 0-5: slider left high bits
		// [10]			bit 6-7: slider right low bits
		// [11]			bit 0-7: slider right high bits
	//'X' for a simplified keyboard report. format is equal to 'K', but an automatic null report is sent afterwards (no need to send by MCU)
	//'N' for a mouse report, which is automatically cleared (otherwise mouse buttons are pressed as long as no release report is sent)
	//'T' send something of every device, a little bit moving, clicking and keyboardpressing. Joystick will be added.
	if(len<=1) return 1; //too short, cannot proceed

	switch(buf[0])
	{
		case 'K':
			//to short, not enough parameters (-2 because of reportid and reserved byte)
			if(len<(KEYBOARD_REPORT_SIZE-2)) return 1;
			Keyboard_UpdateReport(&buf[1], 0);
			break;
		case 'X':
			//to short, not enough parameters (-2 because of reportid and reserved byte)
			if(len<(KEYBOARD_REPORT_SIZE-2)) return 1;
			Keyboard_UpdateReport(&buf[1], 1);
			break;
		case 'M':
			//to short, not enough parameters (-1 because of reportid)
			if(len<(MOUSE_REPORT_SIZE-1)) return 1;
			Mouse_UpdateReport(&buf[1],0);
			break;
		case 'Y':
			//to short, not enough parameters (-1 because of reportid)
			if(len<(MOUSE_REPORT_SIZE-1)) return 1;
			Mouse_UpdateReport(&buf[1],1);
			break;
		case 'J':
			//to short, not enough parameters (-1 because of reportid)
			if(len<(JOYSTICK_REPORT_SIZE-1)) return 1;
			Joystick_UpdateReport(&buf[1]);
			break;
		case 'T':
			testdata[1] = 27; //press 'x'
			Keyboard_UpdateReport(testdata, 1);
			testdata[0] = (1<<0); //click left
			testdata[1] = (uint8_t) -10; //move
			testdata[2] = (uint8_t) 10; // move
			Mouse_UpdateReport(testdata,1);
			//use mouse test data...
			Joystick_UpdateReport(testdata);
			break;
		case 'C':
			if(len < 2) return 1; //one byte as parameter for country code...
			if(buf[1] <= 35) //according to USB-HID specifications: 36-255 is reserved
			{
				//WARNING: if USB_FsConfigDescriptor is changed, the index MUST be replaced!
				USB_FsConfigDescriptor[22] = buf[1];
			} else return 1;
			//TODO: test if device is reset properly?
			USB_resetdevice(); //reset device, that the host will recognize the country code update
			break;
		default:
			return 2;
			break;
	}
	return 0;
}
