# usb_bridge
USB bridge via LPC11U14 chip for next revision of FABI and FLipMouse (or other devices)


# Pinning of the LPC11U14

| Pin     | I/O | Function    |
|---------|-----|-------------|
| PIO0_18 | I   | RXD of UART |
| PIO0_19 | O   | TXD of UART |
| PIO0_17 | I   | UART direction select |
| PIO0_16 | I   | USB device name selection |

The LPC chip is wired via UART (115200, 8N1) to an external MCU (in case of FABI/FLipMouse an ESP32 in the next revision).
This LPC acts as a composite USB device with following features:

* HID mouse 
* HID keyboard
* (HID joystick, currently not implemented yet)
* CDC virtual serial port

The HID functions are controlled via UART commands, which are similar to some Bluetooth HID modules.
If you want to send data via the USB-serial bridge, it is also sent via the same UART interface.
Therefor, it should be determined via the external MCU if the UART input data should be processed as HID commands or sent
to the USB serial. This is done via the UART direction select pin (PIO0_17), if it is tied to VCC (+3,3V!) all received UART data is sent transparently via the USB-CDC interface. If this is set to GND, input data is parsed as HID commands.

This USB bridge can be used either with FABI or FLipMouse devices. Basically, there is no difference. But for clarification for the end-user, the USB descriptor strings are changed depending on the device. This is done via the input pin PIO0_16. This pin is configured as input with pullup. If this pin is tied to GND on boot (is done via the PCB design), FLipMouse descriptors are loaded. If it is left floating or pulled up, FABI descriptors are loaded. This is done only on power-up.
