Design Notes
============

== Features

 * Speed control. Perhaps H-bridge motor drivers to allow reversing.
 * Width control. Needs a stepper driver. External module?
 * Ready-in / Ready-out connections.
 * Sensors at both ends to detect incoming / outgoing boards.
 * Ethernet to receive configuration commands.
 * Numeric keypad to set width / speed.
 * Display to show width setting.
 * Homing switch.
 * Calibration of homing switch offset.
 * Configure width using gcode commands? Like a Y-axis motion system?
 * Maybe configure speed using gcode too, like an X-axis.


== General notes

Example conveyor:

https://www.alibaba.com/product-detail/KAYO-300-Automatic-PCB-Output-Connected_62495647850.html

Specs say that is 600-2200mm/min.


== Connections for controller

 * Ethernet
 * USB
 * 12V (PoE option)
 * Ready-in
 * Ready-out
 * Motor 1 out
 * Motor 2 out
 * Left-side sensor
 * Right-side sensor


== Upstream / downstream connections

Needs a way to read from the adjacent device, and also to signal to
that device.

Open-collector input, with GND connection, a pullup, and a data in.

Dry-contact output. Perhaps a reed relay?


== Movement logic

=== Incoming board

When there is no board ready to be sent, upstreams RO will not be
asserted.

When we're not ready for a board, our RI will not be asserted.

When upstream is ready to send a board, its RO will be asserted.

When we're ready to receive a board, our RI will be asserted.

When both RO and RI on upstream are asserted, start the belt.





== Board sensor

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
