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
	//'M' for a mouse report: parameter are xx bytes
	//'J' for a joystick report: TBD
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
