## Supported cases

------

Currently only yts gcc test with YLOOP enable is supported. Run "aos make yts@lpcxpresso54018 test=certificate" to build and generate binary.

## Board Settings

------------------------
No special settings are required.

## Run the case
------------------------
1. Connect a micro USB cable between the PC host and the Debug Link port(J8 on the board).

2. Open a serial terminal with the following settings:
   - 115200 baud rate
   - 8 data bits
   - No parity
   - One stop bit
   - No flow control

3. Program the generated binary to external flash address 0x10000000 run the case.
