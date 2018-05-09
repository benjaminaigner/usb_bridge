# usb_bridge

USB bridge via LPC11U14 chip for next revision of FABI and FLipMouse (or other devices)


# Pinning of the LPC11U14

| Pin     | I/O | Function    |
|---------|-----|-------------|
| PIO0_18 | I   | RXD of UART |
| PIO0_19 | O   | TXD of UART |
| PIO0_17 | I   | UART direction select |
| PIO0_16 | O   | power control for external HW (USB deep sleep) |

The LPC chip is wired via UART ( _*115200, 8N1*_ ) to an external MCU (in case of FABI/FLipMouse an ESP32 in the next revision).
This LPC acts as a composite USB device with following features:

* HID mouse 
* HID keyboard
* HID joystick
* CDC virtual serial port

The HID functions are controlled via UART commands, which are similar to some Bluetooth HID modules.
If you want to send data via the USB-serial bridge, it is also sent via the same UART interface.
Therefor, it should be determined via the external MCU if the UART input data should be processed as HID commands or sent
to the USB serial. This is done via the UART direction select pin (PIO0_17), if it is tied to VCC (+3,3V!) all received UART data is sent transparently via the USB-CDC interface. If this is set to GND, input data is parsed as HID commands.

To fulfill USB specifications, a device needs to support the USB sleep command with very little power consumption. On our boards, this is done via a N-channel MOSFET which switches off the remaining HW (which is not necessary in sleep mode).
This MOSFET can be connected to PIO0_16. Output is low in sleep mode, output is high if in active mode.


# Serial API (via UART, NOT USB)

| Command | Parameter Length | Description |
|---------|------------------|-------------|
|'K' | 7bytes | send keyboard report, 1. parameter byte is the modifier byte, all others are max. 6 keycodes |
|'X' | 7bytes | same as 'K', but an empty report is sent immediately afterwards (releases all keys) |
|'M' | 4bytes | send mouse report, 1. parameter byte are mouse buttons, 2. is X, 3. is Y and 4th is mouse wheel |
|'Y' | 4bytes | same as 'M', but an empty report (with cleared mouse buttons) is immediately sent afterwards |
|'J' | 12bytes| too long to be documented here, please have a look at parser.c:17 for format. |
|'T' | -- | Send test data via USB, one key is pressed, the mouse is moved and some buttons on the joystick are pressed. |
|'C' | 1byte  | Change bCountryCode for HID, see USB-HID specification. An USB reset will be performed afterwards. |
