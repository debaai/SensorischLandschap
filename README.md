# Sensorisch Landschap
Public github repository for the project Sensorisch Landschap. In this project we desinged and build measurement stations for measuring soil moisture, soil temperature, humidity, temperature and atmospheric CO2 on grasslands. This repository contains all firmware and hardware designs developed during this project. More information about the project can be found on www.sensorischlandschap.nl

## sensors
The 'sensors' folder contains firmware and hardware developed in this project. 
 The main components used for the measurement station are:
- Meet Je Stad PCB Proto v4 
- Pinotech Soilwatch 10
- Si7021 temperature and humidity sensor
- Sensirion SDC30 CO2 sensor

The Meet Je Stad PCB is responsible for reading the sensors and transmitting the data using the LoRaWAN protocol. The firmware provided in the sensors folder is forked from the original Meet Je Stad firmware (https://github.com/meetjestad/mjs_firmware), with various (major) alterations in the code. Additionally a custom wakeup timer was designed to prevent the PCB to jam. This folder contains both the electronic schematics as well as the firmware for this add-on device. 
