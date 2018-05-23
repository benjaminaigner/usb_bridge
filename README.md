# usb_bridge

USB bridge via LPC11U14 chip for next revision of FABI and FLipMouse (or other devices)


# Pinning of the LPC11U14

| Pin     | I/O | Function    |
|---------|-----|-------------|
| PIO0_18 | I   | RXD of UART |
| PIO0_19 | O   | TXD of UART |
| PIO0_17 | I   | RXD for HID |
| PIO0_16 | O   | power control for external HW (USB deep sleep) |

The LPC chip is wired via UART ( _*230400, 8N1*_ ) to an external MCU (in case of FABI/FLipMouse an ESP32 in the next revision).
This LPC acts as a composite USB device with following features:

* HID mouse 
* HID keyboard
* HID joystick
* CDC virtual serial port

The HID functions are controlled via pulse width modulated commands, received on pin IO0_17:

* LSB is sent first
* 0 pulse width: 10us
* 1 pulse width: 20us
* stop bit width: 40us
* Maximum length of one packet is 16 Bytes.

If you want to send data via the USB-serial bridge, it is sent via the UART 0 interface.

To fulfill USB specifications, a device needs to support the USB sleep command with very little power consumption. On our boards, this is done via a N-channel MOSFET which switches off the remaining HW (which is not necessary in sleep mode).
This MOSFET can be connected to PIO0_16. Output is low in sleep mode, output is high if in active mode.


# Supported HID commands

| Command | Parameter Length | Description |
|---------|------------------|-------------|
|'K' | 7bytes | send keyboard report, 1. parameter byte is the modifier byte, all others are max. 6 keycodes |
|'X' | 7bytes | same as 'K', but an empty report is sent immediately afterwards (releases all keys) |
|'M' | 4bytes | send mouse report, 1. parameter byte are mouse buttons, 2. is X, 3. is Y and 4th is mouse wheel |
|'Y' | 4bytes | same as 'M', but an empty report (with cleared mouse buttons) is immediately sent afterwards |
|'J' | 12bytes| too long to be documented here, please have a look at parser.c:17 for format. |
|'T' | -- | Send test data via USB, one key is pressed, the mouse is moved and some buttons on the joystick are pressed. |
|'C' | 1byte  | Change bCountryCode for HID, see USB-HID specification. An USB reset will be performed afterwards. |
