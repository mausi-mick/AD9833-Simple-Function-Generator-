# AD9833-Simple-Function-Generator

The project based on the projet "Signal Generator AD9833" from Peter Balch  :  https://www.instructables.com/Signal-Generator-AD9833/   

where the input commands like Curve-Type (Sine, Triangle, Square) , Sweep or Frequency are send from a PC via USB to Arduino-Nano and from there ro the AD9833 modul.

I changed the program, that it works standalone without connection to the PC.

For input the commands I use a rotary encoder with integrated button-switch.

An HD44780 display (LCD1602) shows all information (Curve-Type, Frequency, seep ...) 

On the Encoder and button-switch I use Pin-Change-Interrupt (PCI) for a quick change of the commnds and frequency.

