EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Microchip_ATtiny:ATtiny24A-SSU U1
U 1 1 62A99CFD
P 5200 2250
F 0 "U1" H 4670 2296 50  0000 R CNN
F 1 "ATtiny24A-SSU" H 4670 2205 50  0000 R CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 5200 2250 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc8183.pdf" H 5200 2250 50  0001 C CNN
	1    5200 2250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J1
U 1 1 62A9B406
P 6850 1900
F 0 "J1" H 6930 1892 50  0000 L CNN
F 1 "Conn_01x06" H 6930 1801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 6850 1900 50  0001 C CNN
F 3 "~" H 6850 1900 50  0001 C CNN
	1    6850 1900
	1    0    0    -1  
$EndComp
Text GLabel 6650 1700 0    50   Input ~ 0
3V
Text GLabel 6650 1800 0    50   Input ~ 0
SWCLK
Text GLabel 6650 2000 0    50   Input ~ 0
SWDIO
Text GLabel 6650 2100 0    50   Input ~ 0
NRST
Text GLabel 5200 1350 1    50   Input ~ 0
3V
Text GLabel 5800 2850 2    50   Input ~ 0
SWDIO
Text GLabel 5800 2050 2    50   Input ~ 0
SWCLK
Text GLabel 5800 2150 2    50   Input ~ 0
MISO
Text GLabel 5800 2250 2    50   Input ~ 0
MOSI
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 62A9EB46
P 6850 2400
F 0 "J2" H 6930 2392 50  0000 L CNN
F 1 "Conn_01x02" H 6930 2301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6850 2400 50  0001 C CNN
F 3 "~" H 6850 2400 50  0001 C CNN
	1    6850 2400
	1    0    0    -1  
$EndComp
Text GLabel 6650 2500 0    50   Input ~ 0
SWDIO
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 62AA2142
P 6850 2700
F 0 "J3" H 6930 2692 50  0000 L CNN
F 1 "Conn_01x02" H 6930 2601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6850 2700 50  0001 C CNN
F 3 "~" H 6850 2700 50  0001 C CNN
	1    6850 2700
	1    0    0    -1  
$EndComp
Text GLabel 6650 2200 0    50   Input ~ 0
MISO
Text GLabel 6650 2700 0    50   Input ~ 0
MOSI
Text GLabel 5800 1750 2    50   Input ~ 0
NRST
$Comp
L power:GND #PWR0101
U 1 1 62AA9EFB
P 6650 2400
F 0 "#PWR0101" H 6650 2150 50  0001 C CNN
F 1 "GND" V 6655 2272 50  0000 R CNN
F 2 "" H 6650 2400 50  0001 C CNN
F 3 "" H 6650 2400 50  0001 C CNN
	1    6650 2400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 62AAA4DC
P 6650 2800
F 0 "#PWR0102" H 6650 2550 50  0001 C CNN
F 1 "GND" V 6655 2672 50  0000 R CNN
F 2 "" H 6650 2800 50  0001 C CNN
F 3 "" H 6650 2800 50  0001 C CNN
	1    6650 2800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 62AABE96
P 6650 1900
F 0 "#PWR0103" H 6650 1650 50  0001 C CNN
F 1 "GND" V 6655 1772 50  0000 R CNN
F 2 "" H 6650 1900 50  0001 C CNN
F 3 "" H 6650 1900 50  0001 C CNN
	1    6650 1900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 62AAC478
P 5200 3150
F 0 "#PWR0104" H 5200 2900 50  0001 C CNN
F 1 "GND" H 5205 2977 50  0000 C CNN
F 2 "" H 5200 3150 50  0001 C CNN
F 3 "" H 5200 3150 50  0001 C CNN
	1    5200 3150
	1    0    0    -1  
$EndComp
$EndSCHEMATC
