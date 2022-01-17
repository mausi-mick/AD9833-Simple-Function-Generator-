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
L Analog_DAC:MCP4822 U1
U 1 1 61DB90F9
P 4000 2950
F 0 "U1" H 4000 3531 50  0000 C CNN
F 1 "MCP4822" H 4000 3440 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 4800 2650 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20002249B.pdf" H 4800 2650 50  0001 C CNN
	1    4000 2950
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSP89 Q2
U 1 1 61DBBA2B
P 6950 2950
F 0 "Q2" H 7155 2996 50  0000 L CNN
F 1 "BSP89" H 7155 2905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 7150 2875 50  0001 L CIN
F 3 "https://www.infineon.com/dgdl/Infineon-BSP89-DS-v02_02-en.pdf?fileId=db3a30433b47825b013b4b8a07f90d55" H 6950 2950 50  0001 L CNN
	1    6950 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 61DBCC34
P 7050 3650
F 0 "R4" H 7109 3696 50  0000 L CNN
F 1 "1024" H 7109 3605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" H 7050 3650 50  0001 C CNN
F 3 "~" H 7050 3650 50  0001 C CNN
	1    7050 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 61DBD372
P 6500 2950
F 0 "R3" V 6696 2950 50  0000 C CNN
F 1 "1k" V 6605 2950 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 6500 2950 50  0001 C CNN
F 3 "~" H 6500 2950 50  0001 C CNN
	1    6500 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 61DBE253
P 6200 3250
F 0 "R2" V 6004 3250 50  0000 C CNN
F 1 "10k" V 6095 3250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 6200 3250 50  0001 C CNN
F 3 "~" H 6200 3250 50  0001 C CNN
	1    6200 3250
	0    1    1    0   
$EndComp
$Comp
L power:+28V #PWR0101
U 1 1 61DBEF0A
P 6100 1100
F 0 "#PWR0101" H 6100 950 50  0001 C CNN
F 1 "+28V" H 6115 1273 50  0000 C CNN
F 2 "" H 6350 1150 50  0001 C CNN
F 3 "" H 6350 1150 50  0001 C CNN
	1    6100 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 61DBF7FD
P 7050 4100
F 0 "#PWR0102" H 7050 3850 50  0001 C CNN
F 1 "GND" H 7055 3927 50  0000 C CNN
F 2 "" H 7050 4100 50  0001 C CNN
F 3 "" H 7050 4100 50  0001 C CNN
	1    7050 4100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J4
U 1 1 61DDD2F2
P 3000 1000
F 0 "J4" H 2972 932 50  0000 R CNN
F 1 "Conn_01x03_Male" H 2972 1023 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 3000 1000 50  0001 C CNN
F 3 "~" H 3000 1000 50  0001 C CNN
	1    3000 1000
	1    0    0    1   
$EndComp
$Comp
L Connector:Conn_01x06_Male J2
U 1 1 61DDEA0D
P 7750 2050
F 0 "J2" V 7812 1662 50  0000 R CNN
F 1 "Conn_01x06_Male" V 7903 1662 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Horizontal" H 7750 2050 50  0001 C CNN
F 3 "~" H 7750 2050 50  0001 C CNN
	1    7750 2050
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J3
U 1 1 61DE076D
P 5850 3600
F 0 "J3" H 5800 3800 50  0000 R CNN
F 1 "Conn_01x02_Male" H 6150 3700 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 5850 3600 50  0001 C CNN
F 3 "~" H 5850 3600 50  0001 C CNN
	1    5850 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2950 6750 2950
Wire Wire Line
	4000 3950 4000 3450
Connection ~ 7050 3950
Wire Wire Line
	7050 3950 7050 4100
Wire Wire Line
	3500 2950 3400 2950
Connection ~ 3400 3950
Wire Wire Line
	3400 3950 4000 3950
Connection ~ 4000 3950
$Comp
L Device:R_Small R1
U 1 1 61DF3BB3
P 5450 2850
F 0 "R1" V 5254 2850 50  0000 C CNN
F 1 "10k" V 5345 2850 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 5450 2850 50  0001 C CNN
F 3 "~" H 5450 2850 50  0001 C CNN
	1    5450 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R9
U 1 1 61DF9727
P 4450 1800
F 0 "R9" V 4254 1800 50  0000 C CNN
F 1 "100k" V 4345 1800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 4450 1800 50  0001 C CNN
F 3 "~" H 4450 1800 50  0001 C CNN
	1    4450 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R8
U 1 1 61DF9DFF
P 3900 2050
F 0 "R8" H 3959 2096 50  0000 L CNN
F 1 "12.1k" H 3959 2005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 3900 2050 50  0001 C CNN
F 3 "~" H 3900 2050 50  0001 C CNN
	1    3900 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R7
U 1 1 61DFAB49
P 3750 1300
F 0 "R7" V 3554 1300 50  0000 C CNN
F 1 "10k" V 3645 1300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 3750 1300 50  0001 C CNN
F 3 "~" H 3750 1300 50  0001 C CNN
	1    3750 1300
	0    1    1    0   
$EndComp
Connection ~ 3400 2950
Wire Wire Line
	3850 1300 4100 1300
Wire Wire Line
	4800 900  4800 1400
Wire Wire Line
	4800 1400 4700 1400
Wire Wire Line
	4100 1500 3900 1500
Connection ~ 4800 1400
Wire Wire Line
	3200 900  4800 900 
Wire Wire Line
	3300 1100 3300 1300
Wire Wire Line
	3300 1300 3650 1300
Wire Wire Line
	3200 1100 3300 1100
Wire Wire Line
	3900 1500 3900 1800
Wire Wire Line
	4350 1800 3900 1800
Connection ~ 3900 1800
Wire Wire Line
	3900 1800 3900 1950
Wire Wire Line
	4550 1800 4800 1800
Wire Wire Line
	4800 1400 4800 1800
Wire Wire Line
	3400 2250 3900 2250
Wire Wire Line
	3900 2250 3900 2150
$Comp
L Device:R_Small R6
U 1 1 61E2A6F9
P 5550 1400
F 0 "R6" H 5609 1446 50  0000 L CNN
F 1 "33k" H 5609 1355 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 5550 1400 50  0001 C CNN
F 3 "~" H 5550 1400 50  0001 C CNN
	1    5550 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 1100 6100 1200
Wire Wire Line
	6100 1200 5550 1200
Wire Wire Line
	5550 1200 5550 1300
Wire Wire Line
	6100 1850 7400 1850
Wire Wire Line
	6100 1200 6100 1850
Connection ~ 6100 1200
Wire Wire Line
	7400 1850 7400 1950
Wire Wire Line
	7400 1950 7550 1950
Connection ~ 7400 1850
Wire Wire Line
	7400 1850 7550 1850
Wire Wire Line
	7550 2050 7400 2050
Wire Wire Line
	7050 2050 7050 2750
Wire Wire Line
	7400 2050 7400 2150
Wire Wire Line
	7400 2150 7550 2150
Connection ~ 7400 2050
Wire Wire Line
	7400 2050 7050 2050
Wire Wire Line
	7550 2250 7400 2250
Wire Wire Line
	7400 2250 7400 2350
Wire Wire Line
	7400 3950 7050 3950
Wire Wire Line
	7550 2350 7400 2350
Connection ~ 7400 2350
Wire Wire Line
	7400 2350 7400 3950
Wire Wire Line
	5550 2850 5650 2850
Wire Wire Line
	4000 2550 4650 2550
Wire Wire Line
	4000 3950 4650 3950
Wire Wire Line
	4650 2550 4650 2950
Wire Wire Line
	4650 3550 4650 3950
Connection ~ 4650 3950
Connection ~ 4650 2550
$Comp
L Transistor_FET:BSS123 Q1
U 1 1 61E569E2
P 6400 3600
F 0 "Q1" H 6604 3646 50  0000 L CNN
F 1 "BSS123" H 6604 3555 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6600 3525 50  0001 L CIN
F 3 "http://www.diodes.com/assets/Datasheets/ds30366.pdf" H 6400 3600 50  0001 L CNN
	1    6400 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 61E6D861
P 6800 3400
F 0 "R5" V 6900 3400 50  0000 C CNN
F 1 "113" V 6900 3550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 6800 3400 50  0001 C CNN
F 3 "~" H 6800 3400 50  0001 C CNN
	1    6800 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 3900 7050 3950
Wire Wire Line
	7050 3750 7050 3950
Wire Wire Line
	7050 3150 7050 3250
Wire Wire Line
	5650 3050 5650 3250
Wire Wire Line
	5650 3250 6100 3250
Wire Wire Line
	6300 3250 7050 3250
Connection ~ 7050 3250
Wire Wire Line
	6250 2950 6400 2950
Wire Wire Line
	4500 2850 5350 2850
$Comp
L Amplifier_Operational:MCP6002-xP U2
U 1 1 61EC1BFD
P 5950 2950
F 0 "U2" H 5950 3317 50  0000 C CNN
F 1 "MCP6002-xP" H 5950 3226 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 5950 2950 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 5950 2950 50  0001 C CNN
	1    5950 2950
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP6002-xP U2
U 2 1 61EC7317
P 4400 1400
F 0 "U2" H 4400 1767 50  0000 C CNN
F 1 "MCP6002-xP" H 4400 1676 50  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 4400 1400 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 4400 1400 50  0001 C CNN
	2    4400 1400
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP6002-xP U2
U 3 1 61ECA83E
P 4750 3250
F 0 "U2" V 4500 2950 50  0000 L CNN
F 1 "MCP6002-xP" V 4800 3000 50  0000 L CNN
F 2 "Package_DIP:DIP-8_W7.62mm_LongPads" H 4750 3250 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 4750 3250 50  0001 C CNN
	3    4750 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1500 5550 2550
Wire Wire Line
	3400 2250 3400 1000
Wire Wire Line
	3400 1000 3200 1000
Connection ~ 3400 2250
Wire Wire Line
	3150 3950 3400 3950
Wire Wire Line
	3400 2950 3400 3950
Wire Wire Line
	7050 3250 7050 3400
Wire Wire Line
	6500 3400 6700 3400
Wire Wire Line
	6900 3400 7050 3400
Connection ~ 7050 3400
Wire Wire Line
	7050 3400 7050 3550
Wire Wire Line
	6500 3800 6500 3950
Connection ~ 6500 3950
Wire Wire Line
	6500 3950 7050 3950
$Comp
L Device:R_Small R10
U 1 1 62056F12
P 6200 3750
F 0 "R10" H 6141 3704 50  0000 R CNN
F 1 "10k" H 6141 3795 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 6200 3750 50  0001 C CNN
F 3 "~" H 6200 3750 50  0001 C CNN
	1    6200 3750
	-1   0    0    1   
$EndComp
Wire Wire Line
	6200 3600 6200 3650
Wire Wire Line
	6200 3850 6200 3950
Connection ~ 6200 3950
Wire Wire Line
	6200 3950 6500 3950
Wire Wire Line
	6050 3600 6200 3600
Connection ~ 6200 3600
Connection ~ 6050 3950
Wire Wire Line
	6050 3950 6200 3950
Wire Wire Line
	6050 3700 6050 3950
Wire Wire Line
	4650 2550 5150 2550
Wire Wire Line
	4650 3950 5150 3950
Wire Wire Line
	5150 2550 5150 3050
Connection ~ 5150 2550
Wire Wire Line
	5150 2550 5550 2550
Wire Wire Line
	5150 3650 5150 3950
Connection ~ 5150 3950
Wire Wire Line
	5150 3950 6050 3950
$Comp
L kalle:ZRB500_5 U3
U 1 1 61DD4BC9
P 4850 2950
F 0 "U3" H 5378 2596 50  0000 L CNN
F 1 "ZRB500_5" H 5378 2505 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5200 1950 50  0001 C CNN
F 3 "C:\\Users\\kalle\\Documents\\KiCad\\DataSheet\\ZRB500_5V.pdf" H 4850 3000 50  0001 C CNN
	1    4850 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2250 3400 2950
Wire Wire Line
	3100 3050 3150 3050
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 61F183F4
P 2900 3150
F 0 "J1" V 2950 3400 50  0000 R CNN
F 1 "Conn_01x04_Male" V 3050 3950 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 2900 3150 50  0001 C CNN
F 3 "~" H 2900 3150 50  0001 C CNN
	1    2900 3150
	1    0    0    1   
$EndComp
Wire Wire Line
	3100 2950 3400 2950
Wire Wire Line
	3150 3050 3150 2850
Wire Wire Line
	3150 2850 3500 2850
Wire Wire Line
	3100 3150 3250 3150
Wire Wire Line
	3250 3150 3250 3050
Wire Wire Line
	3250 3050 3500 3050
Wire Wire Line
	3100 3250 3300 3250
Wire Wire Line
	3300 3250 3300 3150
Wire Wire Line
	3300 3150 3500 3150
$EndSCHEMATC
