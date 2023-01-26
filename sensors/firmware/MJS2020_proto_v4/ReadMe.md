# SensorischLandschap - Sensors - MJS2020_proto_v4

The firmware used in this project was adapted from the original firmware by Meet Je Stad, release v5.
See the repository. https://github.com/meetjestad/mjs_firmware. 

This firmware uses the same dependencies as the original firmware (v5).Additionally the board can be programmed similarly, using the software provided in the mjs_boorstrap repository:
 https://github.com/meetjestad/mjs_bootstrap

Some changes were made with respect to the original firmware:
- GPS packages are sent seperate of other sensor data, to limit the number of bytes transfered. 
- Some system settings are send along in the GPS packages, including reset statistics and firmware version
- Firmware for reading a number of sensors is added, including analog soil moisture measurements, soil temperature and CO2 (Sensirion SDC30)
- Firmware for resetting the external watchdog timer is added
- Firmware for setting up serial over USB is removed from the main .ino file and imported from "usb_serial.h"
- All measurements are called in a seperate measure function
- The user can skip joining to the LoRa network for debugging purposes- 
- The bitstream packet manager was removed for a more simplistic definition of the package
- Some minor lay-out changes were made

## firmware_A
This firmware is used for connecting two analog soil moisture sensors and two ntcs for soil temperature measurements

## firmware_F
This firmware is used for connecting the Sensirion SDC30 CO2 sensor, two analog soil moisture sensors and two ntcs for soil temperature measurements
