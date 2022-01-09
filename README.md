# AD9833-Simple-Function-Generator

The project based on the project "Signal Generator AD9833" from Peter Balch  :  https://www.instructables.com/Signal-Generator-AD9833/   

where the input commands like Curve-Type (Sine, Triangle, Square) , Sweep or Frequency are send from a PC via USB to Arduino-Nano and from there ro the AD9833 modul.

I changed the program, that it works standalone without connection to the PC.

For input the commands I use a rotary encoder with integrated push-button-switch.

An HD44780 display (LCD1602) shows all information (Curve-Type, Frequency, seep ...) 

On the Encoder and button-switch I use Pin-Change-Interrupt (PCI) for a quick change of the commnds and frequency.


A small Video is here: https://www.youtube.com/watch?v=DLXJh2jmxDk


I have in the new version expand the program with new functions inside the AD9833-Signal-Generator and with a digital ConstantCurrentSource (CCS)
working from about 1µA ( starts at 10 µA ) to 4095 µA (step 1 µA) or from 10 µA to 40.95 mA (step 10 µA) . The error over hole area (> 10 µA) is about 1 %, I think
most of them from the DAC , further from the Mosfet (leakage current IDss).

It needs only one further Pin (CS) of the Arduino-Nano.
I have made some changes in the program: For the encoder I use the ext. Interrupt-Pins(2,3 Nano) and only for the push-button PinChangeInterrupt (PCI).

More information (current-source) in the next days.
