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
L power:GND #PWR0101
U 1 1 61C6AC45
P 6200 5100
F 0 "#PWR0101" H 6200 4850 50  0001 C CNN
F 1 "GND" H 6205 4927 50  0000 C CNN
F 2 "" H 6200 5100 50  0001 C CNN
F 3 "" H 6200 5100 50  0001 C CNN
	1    6200 5100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0102
U 1 1 61C6B02A
P 6300 2250
F 0 "#PWR0102" H 6300 2100 50  0001 C CNN
F 1 "+5V" H 6315 2423 50  0000 C CNN
F 2 "" H 6300 2250 50  0001 C CNN
F 3 "" H 6300 2250 50  0001 C CNN
	1    6300 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW1
U 1 1 61C6B667
P 4100 3150
F 0 "SW1" H 4100 2783 50  0000 C CNN
F 1 "Rotary_Encoder_Switch" H 4100 2874 50  0000 C CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC12E-Switch_Vertical_H20mm" H 3950 3310 50  0001 C CNN
F 3 "~" H 4100 3410 50  0001 C CNN
	1    4100 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 3150 4400 3150
Wire Wire Line
	3800 3050 3650 3050
Wire Wire Line
	3650 3050 3650 2600
Wire Wire Line
	3650 2600 4500 2600
Wire Wire Line
	4500 2600 4500 2850
Connection ~ 4500 3150
Wire Wire Line
	4650 3250 4650 3150
Wire Wire Line
	3800 3250 3650 3250
Wire Wire Line
	6600 2950 6700 2950
$Comp
L Connector:Conn_01x05_Male J2
U 1 1 61C88552
P 4150 4050
F 0 "J2" H 4258 4431 50  0000 C CNN
F 1 "Conn_to_AD9833" H 4258 4340 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Horizontal" H 4150 4050 50  0001 C CNN
F 3 "~" H 4150 4050 50  0001 C CNN
	1    4150 4050
	1    0    0    1   
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 61C89D36
P 4300 2300
F 0 "J1" H 4408 2481 50  0000 C CNN
F 1 "Conn_Power_7_12V" H 4408 2390 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 4300 2300 50  0001 C CNN
F 3 "~" H 4300 2300 50  0001 C CNN
	1    4300 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2400 4500 2600
Connection ~ 4500 2600
$Comp
L Device:C_Small C1
U 1 1 61C7388D
P 3650 3150
F 0 "C1" H 3742 3196 50  0000 L CNN
F 1 "10nF" H 3742 3105 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3650 3150 50  0001 C CNN
F 3 "~" H 3650 3150 50  0001 C CNN
	1    3650 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 61C73F93
P 4650 2950
F 0 "C2" H 4742 2996 50  0000 L CNN
F 1 "10nF" H 4742 2905 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 4650 2950 50  0001 C CNN
F 3 "~" H 4650 2950 50  0001 C CNN
	1    4650 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 61C749E5
P 4650 3350
F 0 "C3" H 4742 3396 50  0000 L CNN
F 1 "10nF" H 4742 3305 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 4650 3350 50  0001 C CNN
F 3 "~" H 4650 3350 50  0001 C CNN
	1    4650 3350
	1    0    0    -1  
$EndComp
Connection ~ 3650 3050
Connection ~ 4500 2850
Wire Wire Line
	4500 2850 4500 3150
Wire Wire Line
	3650 3600 3650 3250
Connection ~ 3650 3250
Wire Wire Line
	4500 3450 4500 3150
Wire Wire Line
	3900 3050 4350 3050
Wire Wire Line
	6100 4450 6200 4450
$Comp
L Connector:Conn_01x05_Male J3
U 1 1 61C9F2A5
P 5400 3550
F 0 "J3" H 5508 3931 50  0000 C CNN
F 1 "Conn_open" H 5050 3700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Horizontal" H 5400 3550 50  0001 C CNN
F 3 "~" H 5400 3550 50  0001 C CNN
	1    5400 3550
	1    0    0    1   
$EndComp
Wire Wire Line
	5150 4250 5150 3850
Wire Wire Line
	5150 3850 5600 3850
Wire Wire Line
	5600 3550 5700 3550
Wire Wire Line
	4500 3450 4500 3950
Connection ~ 4500 3450
Connection ~ 4650 3250
Wire Wire Line
	4400 3250 4650 3250
Connection ~ 4650 3050
Wire Wire Line
	4400 3050 4650 3050
Wire Wire Line
	4500 3450 4650 3450
Wire Wire Line
	4500 2850 4650 2850
$Comp
L power:+9V #PWR01
U 1 1 61CD9EF2
P 5250 2250
F 0 "#PWR01" H 5250 2100 50  0001 C CNN
F 1 "+9V" H 5265 2423 50  0000 C CNN
F 2 "" H 5250 2250 50  0001 C CNN
F 3 "" H 5250 2250 50  0001 C CNN
	1    5250 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2250 5250 2300
$Comp
L power:GND #PWR02
U 1 1 61CDC47F
P 4650 2550
F 0 "#PWR02" H 4650 2300 50  0001 C CNN
F 1 "GND" H 4655 2377 50  0000 C CNN
F 2 "" H 4650 2550 50  0001 C CNN
F 3 "" H 4650 2550 50  0001 C CNN
	1    4650 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2400 4650 2550
$Comp
L Device:Fuse_Small F1
U 1 1 61CE3FA3
P 5750 2300
F 0 "F1" H 5750 2485 50  0000 C CNN
F 1 "Fuse_Small 500mA" H 5750 2394 50  0000 C CNN
F 2 "Fuse:Fuse_2010_5025Metric_Pad1.52x2.65mm_HandSolder" H 5750 2300 50  0001 C CNN
F 3 "~" H 5750 2300 50  0001 C CNN
	1    5750 2300
	1    0    0    -1  
$EndComp
Connection ~ 5250 2300
Wire Wire Line
	5250 2300 5650 2300
Wire Wire Line
	5850 2300 6000 2300
Wire Wire Line
	6000 2300 6000 2450
Wire Wire Line
	6600 3850 7550 3850
Wire Wire Line
	7300 2950 7300 3650
Wire Wire Line
	7300 3650 7550 3650
Connection ~ 7300 3650
Wire Wire Line
	6300 2400 7450 2400
Wire Wire Line
	7450 3750 7550 3750
$Comp
L Connector:Conn_01x07_Male J6
U 1 1 61D0EB55
P 7100 3450
F 0 "J6" H 7072 3382 50  0000 R CNN
F 1 "Conn_ADC" H 7072 3473 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Horizontal" H 7100 3450 50  0001 C CNN
F 3 "~" H 7100 3450 50  0001 C CNN
	1    7100 3450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6600 3750 6900 3750
Wire Wire Line
	6600 3650 6900 3650
Wire Wire Line
	6600 3550 6900 3550
Wire Wire Line
	6600 3450 6900 3450
Wire Wire Line
	6600 3250 6600 3350
Wire Wire Line
	6600 3350 6900 3350
Wire Wire Line
	6700 3250 6900 3250
Wire Wire Line
	7300 3650 7300 4950
Wire Wire Line
	6200 4450 6200 4950
Wire Wire Line
	6850 4700 6850 3150
Wire Wire Line
	6850 3150 6900 3150
Wire Wire Line
	5450 4050 5450 4600
Wire Wire Line
	5550 4150 5550 4700
Connection ~ 5550 4150
Wire Wire Line
	5550 4150 5600 4150
Connection ~ 5550 4700
Wire Wire Line
	5550 4700 6850 4700
Connection ~ 6200 4450
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 61C5FDFD
P 6100 3450
F 0 "A1" H 6100 2361 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 6100 2270 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6100 3450 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 6100 3450 50  0001 C CNN
	1    6100 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 4050 5600 4050
Wire Wire Line
	5350 3950 5600 3950
$Comp
L Connector:Conn_01x04_Male J4
U 1 1 61C8B2B3
P 7750 3850
F 0 "J4" H 7722 3732 50  0000 R CNN
F 1 "Conn_to_I2C_LCD" H 7722 3823 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 7750 3850 50  0001 C CNN
F 3 "~" H 7750 3850 50  0001 C CNN
	1    7750 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	6600 3950 7550 3950
$Comp
L Connector:Conn_01x04_Male J5
U 1 1 61D70537
P 4850 4500
F 0 "J5" H 4822 4382 50  0000 R CNN
F 1 "Conn_SPI" H 4822 4473 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 4850 4500 50  0001 C CNN
F 3 "~" H 4850 4500 50  0001 C CNN
	1    4850 4500
	1    0    0    -1  
$EndComp
Connection ~ 6200 4950
Wire Wire Line
	6200 4950 7300 4950
Wire Wire Line
	6200 4950 6200 5100
Wire Wire Line
	4500 2300 5250 2300
Wire Wire Line
	4650 3050 5600 3050
Wire Wire Line
	4650 3150 5600 3150
Wire Wire Line
	5250 3600 5250 3250
Wire Wire Line
	5250 3250 5600 3250
Wire Wire Line
	3650 3600 5250 3600
Connection ~ 5350 4050
Wire Wire Line
	5350 4050 5350 3950
Wire Wire Line
	4350 3950 4500 3950
Wire Wire Line
	4500 3950 4500 4950
Connection ~ 4500 3950
Wire Wire Line
	4500 2400 4650 2400
Connection ~ 4500 2400
Connection ~ 6300 2400
Wire Wire Line
	6300 2400 6300 2450
Wire Wire Line
	4350 3850 5000 3850
Wire Wire Line
	5000 3850 5000 2400
Wire Wire Line
	5000 2400 6300 2400
Wire Wire Line
	4500 4950 6200 4950
Wire Wire Line
	4350 4050 5350 4050
Wire Wire Line
	4350 4150 5550 4150
Wire Wire Line
	4350 4250 5150 4250
Wire Wire Line
	5050 4700 5550 4700
Wire Wire Line
	5050 4600 5450 4600
Wire Wire Line
	5050 4500 5350 4500
Wire Wire Line
	5350 4050 5350 4500
Wire Wire Line
	5150 4250 5150 4400
Wire Wire Line
	5150 4400 5050 4400
Connection ~ 5150 4250
Wire Wire Line
	7450 2400 7450 3750
Wire Wire Line
	6300 2250 6300 2400
$Comp
L Switch:SW_DIP_x01 SW2
U 1 1 61C736A0
P 7000 2950
F 0 "SW2" H 7000 3217 50  0000 C CNN
F 1 "SW_DIP_x01" H 7000 3126 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H7.3mm" H 7000 2950 50  0001 C CNN
F 3 "~" H 7000 2950 50  0001 C CNN
	1    7000 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2450 6700 3250
Wire Wire Line
	6200 2450 6700 2450
$EndSCHEMATC
