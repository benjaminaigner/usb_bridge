/*
 * parser.c
 *
 *  Created on: 29 Nov 2017
 *      Author: beni
 */

#include <parser.h>

/** @brief Currently active keyboard report
 * This report is changed and sent on an incoming command
 * 1. byte is the modifier, bytes 2-7 are keycodes
 */
uint8_t keyboard_report[7];


/** @brief Currently active mouse report
 * This report is changed and sent on an incoming command
 * 1. byte is the button map, bytes 2-4 are X/Y/wheel (int8_t)
 */
uint8_t mouse_report[4];


/** @brief Currently active joystick report
 * This report is changed and sent on an incoming command
 * Byte assignment:
 * [0]			button mask 1 (buttons 0-7)
 * [1]			button mask 2 (buttons 8-15)
 * [2]			button mask 3 (buttons 16-23)
 * [3]			button mask 4 (buttons 24-31)
 * [4]			bit 0-3: hat
 * [4]			bit 4-7: X axis low bits
 * [5]			bit 0-5: X axis high bits
 * [5]			bit 6-7: Y axis low bits
 * [6]			bit 0-7: Y axis high bits
 * [7]			bit 0-7: Z axis low bits
 * [8]			bit 0-1: Z axis high bits
 * [8]			bit 2-7: Z rotate low bits
 * [9]			bit 0-3: Z rotate high bits
 * [9]			bit 4-7: slider left low bits
 * [10]			bit 0-5: slider left high bits
 * [10]			bit 6-7: slider right low bits
 * [11]			bit 0-7: slider right high bits
 */
uint8_t joystick_report[12];

/** @brief Remove a keycode from the given HID keycode array.
 *
 * @note The size of the keycode_arr parameter MUST be 6
 * @param keycode Keycode to be removed
 * @param keycode_arr Keycode to remove this keycode from
 * @return 0 if the keycode was removed, 1 if the keycode was not in the array
 * */
uint8_t remove_keycode(uint8_t keycode,uint8_t *keycode_arr)
{
  uint8_t ret = 1;
  if(keycode == 0) return 0;
  //source: Arduino core Keyboard.cpp
  for (uint8_t i=0; i<6; i++) {
		if (keycode_arr[i] == keycode)
    {
      keycode_arr[i] = 0;
      ret = 0;
    }
	}
  return ret;
}

/** @brief Add a keycode to the given HID keycode array.
 *
 * @note The size of the keycode_arr parameter MUST be 6
 * @param keycode Keycode to be added
 * @param keycode_arr Keycode to add this keycode to
 * @return 0 if the keycode was added, 1 if the keycode was already in the array, 2 if there was no space
 * */
uint8_t add_keycode(uint8_t keycode,uint8_t *keycode_arr)
{
  uint8_t i;
  if(keycode == 0) return 0;

  //source: Arduino core Keyboard.cpp
  // Add k to the key array only if it's not already present
	// and if there is an empty slot.
	if (keycode_arr[0] != keycode && keycode_arr[1] != keycode &&
		keycode_arr[2] != keycode && keycode_arr[3] != keycode &&
		keycode_arr[4] != keycode && keycode_arr[5] != keycode) {

		for (i=0; i<6; i++) {
			if (keycode_arr[i] == 0x00) {
				keycode_arr[i] = keycode;
				return 0;
			}
		}
		if (i == 6) {
			return 2;
		}
	}
  return 1;
}

uint8_t parseBuffer(uint8_t *buf, uint8_t len)
{
	//we do relatively simple parsing:
	//1 Byte determines HID interface + command:
	//0x00		Reset all HID reports to zero and send

	//0x1X		Update Mouse
		//0x10	Send X movement (needs 1 additional int8 byte)
		//0x11	Send Y movement (needs 1 additional int8 byte)
		//0x12	Send wheel movement (needs 1 additional int8 byte)
		//0x13	Press & release button LEFT
		//0x14	Press & release button RIGHT
		//0x15	Press & release button MIDDLE
		//0x17	Press button LEFT
		//0x18	Press button RIGHT
		//0x19	Press button MIDDLE
		//0x1B	Release button LEFT
		//0x1C	Release button RIGHT
		//0x1D	Release button MIDDLE
		//0x1F	Reset Mouse report

	//0x2X		Update Keyboard
		//0x20	Press & release a key (needs 1 additional keycode byte)
		//0x21	Press a key (needs 1 additional keycode byte)
		//0x22	Release a key (needs 1 additional keycode byte)
		//0x23	Press & release a modifier (needs 1 additional modifier byte)
		//0x24	Press a modifier (needs 1 additional modifier bytemask)
		//0x25	Release a modifier (needs 1 additional modifier bytemask)
		//0x26-8	TBD: Maybe these will be used for media keys
		//0x2F	Reset Keyboard Report

	//0x3X		Update Joystick
		//0x30	Press & release a button (needs 1 additional button/hat byte)
		//0x31	Press a button (needs 1 additional button/hat byte)
		//0x32	Release a button (needs 1 additional button/hat byte)
			/* the additional button/hat byte determines the used button.
			 * If bit 7 is 0, the remaining 7 bits determine the button to be set/released
			 * If bit 7 is 1, the remaining 7 bits are used as hat position (15 is idle, 0-7 are valid positions)
			 */
		//0x34	Update X axis (+2Bytes int16 -> LSB first)
		//0x35	Update Y axis (+2Bytes int16 -> LSB first)
		//0x36	Update Z axis (+2Bytes int16 -> LSB first)
		//0x37	Update Z-rotate axis (+2Bytes int16 -> LSB first)
		//0x38	Update slider left (+2Bytes int16 -> LSB first)
		//0x39	Update slider right (+2Bytes int16 -> LSB first)
		//0x3F	Reset Joystick report
	if(len<=1) return 1; //too short, cannot proceed
	//before parsing, wait for an idle EP
	HID_waitIdle();
	//start parsing, now USB is free.
	switch(buf[0] & 0xF0)
	{
		//reset all reports
		case 0x00:
			memset(mouse_report,0,sizeof(mouse_report));
			memset(keyboard_report,0,sizeof(keyboard_report));
			memset(joystick_report,0,sizeof(joystick_report));
			Keyboard_UpdateReport(keyboard_report,0);
			Mouse_UpdateReport(mouse_report, 0);
			Joystick_UpdateReport(joystick_report);
			break;
		//mouse handling
		case 0x10:
			switch(buf[0] & 0x0F)
			{
				case 0: //move X
					mouse_report[1] = buf[1];
					break;
				case 1: //move Y
					mouse_report[2] = buf[1];
					break;
				case 2: //move wheel
					mouse_report[3] = buf[1];
					break;
				/* Press & release */
				case 3: //left
					mouse_report[0] |= (1<<0);
					//send press report, wait for free EP (sending release is done after switch)
					Mouse_UpdateReport(mouse_report, 0);
					mouse_report[0] &= ~(1<<0);
					HID_waitIdle();
					break;
				case 4: //right
					mouse_report[0] |= (1<<1);
					//send press report, wait for free EP (sending release is done after switch)
					Mouse_UpdateReport(mouse_report, 0);
					mouse_report[0] &= ~(1<<1);
					HID_waitIdle();
					break;
				case 5: //middle
					mouse_report[0] |= (1<<2);
					//send press report, wait for free EP (sending release is done after switch)
					Mouse_UpdateReport(mouse_report, 0);
					mouse_report[0] &= ~(1<<2);
					HID_waitIdle();
					break;
				/* Press */
				case 7: //left
					mouse_report[0] |= (1<<0);
					break;
				case 8: //right
					mouse_report[0] |= (1<<1);
					break;
				case 9: //middle
					mouse_report[0] |= (1<<2);
					break;
				/* Release */
				case 11: //left
					mouse_report[0] &= ~(1<<0);
					break;
				case 12: //right
					mouse_report[0] &= ~(1<<1);
					break;
				case 13: //middle
					mouse_report[0] &= ~(1<<2);
					break;
				case 15: //reset mouse
					memset(mouse_report,0,sizeof(mouse_report));
					break;
			}
			Mouse_UpdateReport(mouse_report, 0);
			break;
		//Keyboard handling
		case 0x20:
			switch(buf[0] & 0x0F)
			{
				case 0: //Press & release a key
					//press key & send
					add_keycode(buf[1], &keyboard_report[1]);
					Keyboard_UpdateReport(keyboard_report, 0);
					//remove keycode & wait for Idle EP,
					//sending the second report is done after this switch
					remove_keycode(buf[1], &keyboard_report[1]);
					HID_waitIdle();
					break;
				case 1: //Press a key
					add_keycode(buf[1], &keyboard_report[1]);
					break;
				case 2: //Release a key
					remove_keycode(buf[1], &keyboard_report[1]);
					break;
				case 3: //Press & release a modifier (mask!)
					keyboard_report[0] |= buf[1];
					Keyboard_UpdateReport(keyboard_report, 0);
					//remove modifier & wait for Idle EP,
					//sending the second report is done after this switch
					keyboard_report[0] &= ~buf[1];
					HID_waitIdle();
					break;
				case 4: //Press a modifier (mask!)
					keyboard_report[0] |= buf[1];
					break;
				case 5: //Release a modifier (mask!)
					keyboard_report[0] &= ~buf[1];
					break;
				case 15:
					memset(keyboard_report,0,sizeof(keyboard_report));
					break;
			}
			Keyboard_UpdateReport(keyboard_report, 0);
			break;
		case 0x30:
			switch(buf[0] & 0x0F)
			{
				case 0: //Press & release button/hat
					//test if it is buttons or hat?
					if((buf[1] & (1<<7)) == 0)
					{
						//buttons, map to corresponding bits in 4 bytes
						if(buf[1] <= 7) joystick_report[0] |= (1<<buf[1]);
						else if(buf[1] <= 15) joystick_report[1] |= (1<<(buf[1]-8));
						else if(buf[1] <= 24) joystick_report[2] |= (1<<(buf[1]-16));
						else if(buf[1] <= 31) joystick_report[3] |= (1<<(buf[1]-24));
					} else {
						//hat, remove bit 7 and set to report (don't touch 4 bits of X)
						joystick_report[4] = (joystick_report[4] & 0xF0) | (buf[1] & 0x0F);
					}
					//send press action
					Joystick_UpdateReport(joystick_report);
					//wait for sent report
					HID_waitIdle();
					//release button/hat
					//test if it is buttons or hat?
					if((buf[1] & (1<<7)) == 0)
					{
						//buttons, map to corresponding bits in 4 bytes
						if(buf[1] <= 7) joystick_report[0] &= ~(1<<buf[1]);
						else if(buf[1] <= 15) joystick_report[1] &= ~(1<<(buf[1]-8));
						else if(buf[1] <= 24) joystick_report[2] &= ~(1<<(buf[1]-16));
						else if(buf[1] <= 31) joystick_report[3] &= ~(1<<(buf[1]-24));
					} else {
						//hat release means always 15.
						joystick_report[4] = (joystick_report[4] & 0xF0) | 0x0F;
					}
					break;
				case 1: //Press button/hat
					//test if it is buttons or hat?
					if((buf[1] & (1<<7)) == 0)
					{
						//buttons, map to corresponding bits in 4 bytes
						if(buf[1] <= 7) joystick_report[0] |= (1<<buf[1]);
						else if(buf[1] <= 15) joystick_report[1] |= (1<<(buf[1]-8));
						else if(buf[1] <= 24) joystick_report[2] |= (1<<(buf[1]-16));
						else if(buf[1] <= 31) joystick_report[3] |= (1<<(buf[1]-24));
					} else {
						//hat, remove bit 7 and set to report (don't touch 4 bits of X)
						joystick_report[4] = (joystick_report[4] & 0xF0) | (buf[1] & 0x0F);
					}
					break;
				case 2: //Release button/hat
					//test if it is buttons or hat?
					if((buf[1] & (1<<7)) == 0)
					{
						//buttons, map to corresponding bits in 4 bytes
						if(buf[1] <= 7) joystick_report[0] &= ~(1<<buf[1]);
						else if(buf[1] <= 15) joystick_report[1] &= ~(1<<(buf[1]-8));
						else if(buf[1] <= 24) joystick_report[2] &= ~(1<<(buf[1]-16));
						else if(buf[1] <= 31) joystick_report[3] &= ~(1<<(buf[1]-24));
					} else {
						//hat release means always 15.
						joystick_report[4] = (joystick_report[4] & 0xF0) | 0x0F;
					}
					break;
				case 4: //X Axis
					//preserve 4 bits of hat
					joystick_report[4] = (joystick_report[4] & 0x0F) | ((buf[1] & 0x0F) << 4);
					//preserve 2 bits of Y
					joystick_report[5] = (joystick_report[5] & 0xC0) | ((buf[1] & 0xF0) >> 4) | ((buf[2] & 0x03) << 4);
					break;
				case 5: //Y Axis
					//preserve 6 bits of X
					joystick_report[5] = (joystick_report[5] & 0x3F) | ((buf[1] & 0x03) << 6);
					//save remaining Y
					joystick_report[6] = ((buf[1] & 0xFC) >> 2) | ((buf[2] & 0x03) << 6);
					break;
				case 6: //Z Axis
					joystick_report[7] = buf[1];
					joystick_report[8] = (joystick_report[8] & 0xFC) | (buf[2] & 0x03);
					break;
				case 7: //Z-rotate
					//preserve 2 bits of Z-axis
					joystick_report[8] = (joystick_report[8] & 0x03) | ((buf[1] & 0x3F) << 2);
					//preserve slider left & combine 2 bits of LSB & MSB to one nibble
					joystick_report[9] = (joystick_report[9] & 0xF0) | ((buf[1] & 0xC0) >> 6) | ((buf[2] & 0x03) << 2);
					break;
				case 8: //slider left
					//preserve 4 bits of Z-rotate, add low nibble of first byte
					joystick_report[9] = (joystick_report[9] & 0x0F) | ((buf[1] & 0x0F) << 4);
					//preserve 2 bits of slider right, add high nibble of first byte and second byte
					joystick_report[10] = (joystick_report[10] & 0xC0) | ((buf[1] & 0xF0) >> 4) | ((buf[2] & 0x03) << 4);
					break;
				case 9: //slider right
					//preserve 6 bits of slider left, add 2 bits for slider right
					joystick_report[10] = (joystick_report[10] & 0x3F) | ((buf[1] & 0x03) << 6);
					//save remaining slider right
					joystick_report[11] = ((buf[1] & 0xFC) >> 2) | ((buf[2] & 0x03) << 6);
					break;
				case 15: //reset
					memset(joystick_report,0,sizeof(joystick_report));
					break;
			}
			Joystick_UpdateReport(joystick_report);
			break;
		default:
			return 2;
			break;
	}
	return 0;
}
