## Steps to download and run the binary

1. Download the [flash loader](https://www.nxp.com/webapp/Download?colCode=Flashloader_RT1020_1.0_GA&appType=license)
   and unzip it.
2. Unzip the flash\_loader\_patch.zip in folder _board/evkmimxrt1020_ and copy
   it to flash loader folder to override the files.
3. Connect USB port J23 to PC, this is the debug console port. Open terminal
   and set the format:
   - 115200 baud rate
   - 8 data bits
   - No parity
   - One stop bit
   - No flow control
4. Connect USB port J9 to PC, this is the port for image download.
5. Build the AliOS binary, rename it to AliOS.bin and copy it to the folder
   _&lt;flash\_loader\_folder&gt;/Tools/mfgtools-rel/Profiles/MXRT102X/OS Firmware_.
6. Set SW8 (back of the board) to 0001, open the _&lt;flash\_loader\_folder&gt;/Tools/mfgtools-rel/MfgTool2.exe_,
   then press SW5, then from the MfgTool2, you can see the device is connected.
7. Click 'Start' in MfgTool2 to download image. Click 'Stop' when finished. The
   flash erase time might be long. With current setting, only 1MB flash is erased,
   if the binary is large, you can modify the file
   _&lt;flash\_loader\_folder&gt;/Tools/mfgtools-rel/Profiles/MXRT102X/OS Firmware/ucl2.xml
   line 50
8. Set SW8 to 0010 and press SW5, the board boots from flash and run. You can see
   the output from terminal.

## Introduction to mini bootloader

Mini bootloader is a seperate application to support reliable OTA, it is started
after ROM bootloader and before AliOS binary. It provides such functions:

1. Configure the flash and SDRAM, so the AliOS application does not need to
   configure them again.
2. It maintains the information about the new valid AliOS binary. If there is
   new valid AliOS binary, the mini bootloader copies it to override the old one.
3. AliOS binary is built to run in SDRAM, mini bootloader copies the AliOS binary
   to SDRAM and jump to run AliOS binary.

There is a pre-built mini bootloader binary (mini\_bl.bin), it is copied to the folder
_&lt;flash\_loader\_folder&gt;/Tools/mfgtools-rel/Profiles/MXRT102X/OS Firmware_.
When MfgTool2 downloading the AliOS binary, it is downloaded to flash too.
Generally you don't need to modify the mini\_bl.bin. If you have new requirement,
open the project _projects/IAR/evkmimxrt1020/mini\_bl/mini\_bl.eww_, make your changes,
build the new binary and copy it to
_&lt;flash\_loader\_folder&gt;/Tools/mfgtools-rel/Profiles/MXRT102X/OS Firmware_.
