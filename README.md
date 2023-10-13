# WaterControl
Arduino code for controlling cooling water systems

Intended for P1AM-100 PLC style arduino compatible CPU:

https://www.automationdirect.com/adc/shopping/catalog/programmable_controllers/productivity_open_(arduino-compatible)/controllers_-a-_shields/p1am-100

by default only 7 interrupt-capable inputs are available, and we need 8. To enable the extra interrupt on pin 3, copy the variant.cpp files in the /corepatch folder over the one included with the P1AM-100 board package.

On my windows install this is located at:
C:\Users\'user'\AppData\Local\Arduino15\packages\P1AM-100\hardware\samd\1.6.21\variants\P1AM-100

The other possible location depending on IDE version seems to be:

C:\Users\'user'\Documents\ArduinoData\packages\P1AM-100\hardware\samd\1.6.21\variants\P1AM-100