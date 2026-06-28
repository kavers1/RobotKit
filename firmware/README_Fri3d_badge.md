# Fri3d Badge 2026 IO expander firmware

This firmware runs on the [CH32X035](https://www.wch-ic.com/products/CH32X035.html) companian MCU of the [Fri3d Camp 2026 badge](https://github.com/Fri3dCamp/badge_2026).

> If you are looking for the firmware that runs on the main MCU (ESP32S3) of the Fri3d Badge 2026 badge, please look at [this repository](https://github.com/Fri3dCamp/badge_firmware_MicroPythonOS).

The firmware handles different functions of the badge and acts as a I2C IO expander to the main ESP32-S3 MCU of the badge:
 * Battery and charger monitoring
 * USB power monitoring
 * Joystick and button handling
 * LCD brightness and reset control
 * debug LED control
 * LORA module reset control
 * Digital multimeter monitoring

Please check the [hardware schematics](https://github.com/Fri3dCamp/badge_2026_hw) to get more details about how these peripherals of the badge are connected to the CH32X035 MCU.

### I2C

The expander has I2C address `0x50` and uses the following registers to interface/control with its connected peripherals:

| Address (hex) | Name | Access | Bytes | description |
|-|-|-|-|-|
| 0x00 | Version number | R | 3 | Reports the firmware version number |
| 0x04 | Button states | R | 2 | Reports the button states (see below) |
| 0x08 | Digital multimeter AIN0 | R | 2 | Digital multimeter AIN0 analog value (0-4096) |
| 0x0A | Battery monitor | R | 2 | Battery voltage analog value (0-4096) (1) |
| 0x0C | USB voltage | R | 2 | USB voltage (0-4096) (1) |
| 0x0E | Joystick Y | R | 2 | Joystick Y-axis analog value (0-4096) |
| 0x10 | Joystick X | R | 2 | Joystick X-axis analog value (0-4096) |
| 0x12 | LCD brightness | R/W | 2 | LCD brightness value (0-100) |
| 0x14 | Debug LED | R/W | 2 | debug LED brightness (0-100) |
| 0x16 | Digital outputs | R/W | 1 | digital output state (see blow) |
 1. be aware that the full range of 0-4095 will not be used since there is a voltage divider used.

The button states are a 2-byte value with the following encoding:

| Bit | Name |
|-|-|
| \[15:12\] | reserved |
| 11 | USB plugged state |
| 10 | Joystick: Right state  |
| 9 | Joystick: Left state  |
| 8 | Joystick: Down state  |
| 7 | Joystick: Up state |
| 6 | Button Menu state |
| 5 | Button B state |
| 4 | Button A state |
| 3 | Button Y state |
| 2 | Button X state |
| 1 | Charger: standby state |
| 0 | Charger: charging state |

The output states are a 1-byte value with the following encoding:

| Bit | Name |
|-|-|
| \[7:5\] | reserved |
| 4 | Lora reset (1 = release, 0 = held in reset) |
| 3 | trigger remap of I2C to SWD for reflashing |
| 2 | trigger reboot |
| 1 | LCD Reset state |
| 0 | AUX 3v3 state |

## Building

Use [platformio](https://platformio.org) to build this project. You should install the [ch32v platform package](https://github.com/Community-PIO-CH32V/platform-ch32v) as well. If you use the command line, build using:

```
pio run -e debug
```

## Flashing

MicropythonOS holds the latest stable version of this firmware and automatically tries to flash it to the companion MCU when it needs to at boot. So there is nothing a user should do.

For manual reflashing, please use the ESP32-S3 firmware provided together with the release of the firmware `flash_ch32_coprocessor_<version>_on_esp32s3_for_fri3d_2026.bin` and flash it onto the ESP32-S3 MCU of the Fri3d Camp badge 2026. After that, monitor the USB serial output and perform a full power-off/power-on sequence. When the firmware is succesfully flashed, the debug LED on the backside of the badge should be powered on.

## Development

### Micropython

In Micropython on the ESP32-S3, use the `readfrom_mem()` and `writeto_mem()` I2C APIs to interface with the expander, e.g.:

```
from machine import I2C, Pin
import struct
import time

ADDRESS = 0x50

def callback(p):
    # debounce
    time.sleep(.01)
    # read the button states
    print("button state:", expander_i2c.readfrom_mem(ADDRESS, 4, 2).hex())

pin_interrupt = Pin(3, Pin.IN)
pin_interrupt.irq(trigger=Pin.IRQ_RISING, handler=callback)

expander_i2c = I2C(sda=Pin(39), scl=Pin(42), freq=400000)
# read the version:
print("version:", expander_i2c.readfrom_mem(ADDRESS, 0x00, 3).hex())

# read the analog state
print("analog channels:", struct.unpack("<HHHHH", expander_i2c.readfrom_mem(ADDRESS, 0x08, 10)))

# read the debug led state
print("debug LED PWM:", struct.unpack("<H", expander_i2c.readfrom_mem(ADDRESS, 0x14, 2)))
print("LCD backlight PWM:", struct.unpack("<H", expander_i2c.readfrom_mem(ADDRESS, 0x12, 2)))

# set the LCD brightness to 50%
expander_i2c.writeto_mem(ADDRESS, 0x12, struct.pack("<H", 50))

# read the LCD brightness
print("LCD backlight PWM:", struct.unpack("<H", expander_i2c.readfrom_mem(ADDRESS, 0x12, 2)))

# fade the debug LED up and down
for i in range (100):
    expander_i2c.writeto_mem(ADDRESS, 0x14, struct.pack("<H", i))
    time.sleep(.1)
for i in range (100, 0, -1):
    expander_i2c.writeto_mem(ADDRESS, 0x14, struct.pack("<H", i))
    time.sleep(.1)

# turn off 3v3 aux
expander_i2c.writeto_mem(ADDRESS, 0x16, b'\x00')

# turn on 3v3 aux
expander_i2c.writeto_mem(ADDRESS, 0x16, b'\x01')

# trigger a reboot to bootloader
expander_i2c.writeto_mem(ADDRESS, 0x16, b'\x04')

# trigger a remap of the i2c pins
expander_i2c.writeto_mem(ADDRESS, 0x16, b'\x10')

# release the reset pin of the LORA chip
expander_i2c.writeto_mem(ADDRESS, 0x16, b'\x20')

```

A [driver](https://github.com/MicroPythonOS/MicroPythonOS/blob/main/internal_filesystem/lib/drivers/fri3d/expander.py) for this companion MCU is available in MicropythonOS, the OS that runs on the main MCU of the Fri3d Camp Badge 2026.

### Arduino

```
TODO
```
