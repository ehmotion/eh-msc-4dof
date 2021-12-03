# eh-msc-4dof

4 channel servo controller code for Arduino Mega 2560 for AASD-15A and similar motor driver

Arduino sketch for generating pulses needed to control servo drivers such as the AASD-15A used in the SFX100 project and the pt-actuators.<br>
The controller is meant for the SFX100 actuator and the likes but it could easily be adapted to other configurations.

The controller will start detecting your actuator limits as soon as it is powered on so make sure you have all required connections ready prior to power on.<br>
The following connections are required:
```
AASD-15A
CN2 DB25 pin / Arduino pin
Motor 1
 3              12
 4              A0
 6              A4
 23             18 (via 2.2kohm resistor and 0.1uF capacitor to GND)
 9              5Vcc
 5,10,14        GND

Motor 2
 3              2
 4              A1
 6              A5
 23             19 (via 2.2kohm resistor and 0.1uF capacitor to GND)
 9              5Vcc
 5,10,14        GND

Motor 3
 3              7
 4              A2
 6              A6
 23             20 (via 2.2kohm resistor and 0.1uF capacitor to GND)
 9              5Vcc
 5,10,14        GND

Motor 4
 3              45
 4              A3
 6              A7
 23             21 (via 2.2kohm resistor and 0.1uF capacitor to GND)
 9              5Vcc
 5,10,14        GND
```
!!!WARNING: failure to make the proper connections may result in broken actuators.<br>
<br>
!!!If this is your first time, try one actuator at a time in each port since it won't mind if any of them is not connected. Watch the serial console/monitor in Arduino IDE to follow what happens during limits detection (the LED flashes slowly).

-the torque reach signal requires the following connection:
```
CN2 DB25 pin 23 -- 2.2kohm resistor -- Arduino pin
                \-- 0.1uF capacitor -- GND<br>
(resistor between DB25 pin 23 and Arduino; capacitor between DB25 pin 23 and GND)
```
The following parameters are required for the AASD 15A controller:<br>
(push MOD until you see Pn000 to enter the parameter setup mode)
```
Pn2 = 2
Pn8 = 300
Pn9 = -300
Pn51 = 3000 (motor top speed/rpm, could be run slower if wanted)
Pn98 = 10
Pn109 = 1
Pn110 = 30
Pn113 = 20
Pn114 = 10
Pn115 = 100
Extra parameters needed in the AASD-15A drivers used for limits detection:
Pn24 = 100 (reach predetermined torque > 100%)
Pn52 = 1
Pn60 = 2 (not used for now)
Pn61 = 6

AASD-15A CN2 DB25 control signal connector explained:
 p3 - pwm signal
 p4 - direction
 p5 - GND
 p6 - enable (motor ON)
 p9 - 5Vcc
p10 - GND
p11 - ready (motor present and ready - not used for now)
p14 - GND
p23 - torque reached
```
Message format used for position control in FlyPTMover (starts with 'P4', ends with '!!'):
```
P4<Axis1a><Axis2a><Axis3a><Axis4a>!!
```
Serial port configuration: 115200,8N1<br>
Command speed frequency: 2ms<br>
Position data: 2 bytes binary<br>
Position encoding: 16bits<br>
Check the screenshots for details on FlyPTMover settings.

Position control range: 0 to 65535 - it gets mapped to actuator limits as detected on power on.<br>
To avoid limit detection for each serial port access, use a 10uF capacitor between Arduino pins RESET and GND.<br>
Note that it is no longer possible to upload the sketch on the Arduino while the capacitor is connected.<br>

Thanks to cubexvr! based on https://github.com/cubexvr/servoController

ENJOY at your own risk!
