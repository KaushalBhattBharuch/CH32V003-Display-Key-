Low Voltage Disconnect LVD using CH32v003f4p6

DC Voltage maximum upto 60V converted to Maximum up to 5V using voltage devider.
This 5V scaned and averaged using ADC and DMA.
One Mosfet triggerd on the basis of ADC reading to connect/disconnect battery
Voltage Displayed on 7 Segment 4 digit display.
Cut off voltage can be set using 4 keys.

Display and keys interfaced using two 74hc595 Shift Registers cascaded interfaced through SPI Master Transmit only mode.
Keys ware scanned using one GPIO pin.
