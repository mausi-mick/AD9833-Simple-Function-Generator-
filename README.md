
AD9833-Simple-Function-Generator

The project based on the project "Signal Generator AD9833" from Peter Balch :
    •  https://www.instructables.com/Signal-Generator-AD9833/
    • where the input commands like Curve-Type (Sine, Triangle, Square) , Sweep or Frequency are send from a PC via USB to Arduino-Nano and from there to the AD9833 Signalgenerator-Modul.
    • I changed the program  so that it works standalone without connection to the PC.
    • For input the commands, I use only a rotary encoder with integrated push-button-switch.
    • An HD44780 display (LCD1602) shows all information (Curve-Type, Frequency, sweep ...)
    • On the encoder I use external interrupts (Pin2 / Pin3 Arduino NANO) , on push-button-switch I use Pin-Change-Interrupt (PCI) for a direct/quick response.
    • A small Video is here:   https://www.youtube.com/watch?v=DLXJh2jmxDk
    • In the new version I have expand the program with new functions inside the AD9833-Signal-Generator and outside this function with a digital ConstantCurrentSource (CCS) working from about 1µA ( starts at 10 µA ) to 4095 µA (step 1 µA) or from 10 µA to 40.95 mA (step 10 µA) .

So the device  includes now this functions:

    • AD9833 Signalgenerator for Sinus, Triangle and Squarewave up to 12.5 Mhz in 1 Hz-Steps 
    • Low-Frequency Function-Generator for Sinus, Triangle StepUp , StepDown and 
    • Squarewave for low frequencies (mHz to some kHz) 
    •  video  („TRIANGLE“) :   https://www.youtube.com/watch?v=seZB-FUj7es
    • digital ConstantCurrentSource/Sink from 10 µA to 4.095 mA (Stepwidth : 1 µA) or (10 µA to 40.95 mA with stepwidth 10 µA) 
For small budget I use only a few parts with price over 1 $/€:

    • AD9833 board
    • HD44780 with I2C 
    • Arduino NANO (clone?) 
    • DAC MCP4822 
    • OPV MCP6002 
    • n-Mosfet BSP89 
    • Rotary Encoder with push-button 
    • n-Mosfet for switching Rp ( from 1025 Ohm to 10.25 Ohm) with Low R-on (< 100 mOhm) 
    • 5V Zener/Ref-Diode  (or other stabilization for MCP4822/MCP6002) 
    • 2x MT3608 Step up (5V/ for NANO/LCD..., 28V for CurrentSource) 
    • Battery-holder for 18650 Battery 
    • USB  Lion-loader board 
    • 18650 Lion-Battery (one piece) 
    • 
More information in the next days.
