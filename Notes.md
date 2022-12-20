Design Notes
============

## Features

 * Speed control. Perhaps H-bridge motor drivers to allow reversing.
 * Width control. Needs a stepper driver. External module?
 * Ready-in / Ready-out connections.
 * Sensors at both ends to detect incoming / outgoing boards.
 * Ethernet to receive configuration commands.
 * Numeric keypad to set width / speed.
 * Display to show width setting.
 * Homing switch.
 * Calibration of homing switch offset.
 * Configure width using gcode commands.
 * Maybe configure speed using gcode too? Like spindle speed / direction.

I/O: Use MCP23017, and expose the extra pins on a spare header?

## General notes

Example conveyor:

https://www.alibaba.com/product-detail/KAYO-300-Automatic-PCB-Output-Connected_62495647850.html

Specs say that is 600-2200mm/min.

N20 motor 100RPM:

 * 12V = 3620mm/min
 * 4V = 1130mm/min
 * 3V = 780mm/min
 
Height from floor: SMEMA spec says 37" to 38" (940mm to 965mm)

## Software
The controller can be combined as a single device that provides the UI and also controls
the hardware, using custom firmware, OR it can be split so there's an I/O controller as
part of the chassis and a separate UI device.

The second approach as the advantage that the I/O controller can run GRBL:

 * Only enable the Y axis.
 * Use the spindle output to control the conveyor motor:
   * Spindle PWM speed control.
   * Spindle direction control.

With a separate I/O controller, it could be controlled by plugging in a USB cable from
a laptop and not having a UI controller at all.

https://github.com/gnea/grbl

However, how would it do ready-in/ready-out, and board detection? Custom firmware may
be better for those things.

### User Interface
https://www.bitsanddroids.com/creating-our-first-esp32-flight-display-graphics/

## Lighting
Should there be LED strips along the inside faces of the conveyor arms?

## Connections for controller

 * Ethernet
 * USB
 * 12V (PoE option)
 * Ready-in
 * Ready-out
 * Motor 1 out
 * Motor 2 out
 * Left-side sensor
 * Right-side sensor

## Controller
http://www.wireless-tag.com/portfolio/wt32-sc01/

### LCD driver
https://github.com/moononournation/Arduino_GFX

https://github.com/moononournation/Arduino_GFX/wiki/Dev-Device-Declaration


#include <Arduino_GFX_Library.h>
#define TFT_BL 23
Arduino_DataBus *bus = new Arduino_ESP32SPI(21 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, -1 /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7796(bus, 22 /* RST */, 3 /* rotation */);

https://haswitchplate.github.io/openHASP-docs/0.6/devices/wt32-sc01/

https://www.alibaba.com/product-detail/WT32-SC01-ESP32-Dev-kitC-with_1600120762835.html

### Touch screen driver

https://stackoverflow.com/questions/58472138/find-ft6336-touch-screen-library-for-esp32

https://github.com/crystalfontz/CFAF240400C0-030SC/blob/master/CFAF240400C0030T/CFAF240400C0030SC.ino

https://github.com/strange-v/FT6X36

## Upstream / downstream connections

Needs a way to read from the adjacent device, and also to signal to
that device.

Open-collector input, with GND connection, a pullup, and a data in.

Dry-contact output. Perhaps a reed relay?


## Movement logic

### Incoming board

When there is no board ready to be sent, upstreams RO will not be
asserted.

When we're not ready for a board, our RI will not be asserted.

When upstream is ready to send a board, its RO will be asserted.

When we're ready to receive a board, our RI will be asserted.

When both RO and RI on upstream are asserted, start the belt.





## Board sensor

Sensor type: VL53L0X

VDD  Regulated 2.8 V output. Almost 150 mA is available to power
  external components. (If you want to bypass the internal regulator,
  you can instead use this pin as a 2.8 V input with VIN disconnected.)
 
VIN  This is the main 2.6 V to 5.5 V power supply connection. The SCL
  and SDA level shifters pull the I2C lines high to this level.
 
GND  The ground (0 V) connection for your power supply. Your I2C
  control source must also share a common ground with this board.
 
SDA  Level-shifted I2C data line: HIGH is VIN, LOW is 0 V
 
SCL  Level-shifted I2C clock line: HIGH is VIN, LOW is 0 V
 
XSHUT  This pin is an active-low shutdown input; the board pulls it up
  to VDD to enable the sensor by default. Driving this pin low puts
  the sensor into hardware standby. This input is not level-shifted. 

I2C address is 0x29 and can't be changed in hardware. The driver can
change it on startup. Use XSHUT to hold all others in shutdown while
that's being done.
