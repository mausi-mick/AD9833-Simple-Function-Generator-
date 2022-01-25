//
//--------------------------------------------------------------------------------------- 
//  Thanks:
//
// http://www.vwlowen.co.uk/arduino/AD9850-waveform-generator/AD9850-waveform-generator.htm
//
//---------------------------------------------------------------------------------------
// subject to the GNU General Public License
//---------------------------------------------------------------------------------------
// Developped with Arduino IDE 1.8.4 / 1.8.13
//---------------------------------------------------------------------------------------
//
/* Based on AD9851 code from Andrew Smallbone - modified for AD9850
   http://www.rocketnumbernine.com/2011/10/25/programming-the-ad9851-dds-synthesizer 
 */

#include <Adafruit_GFX.h>       // Core graphics library https://github.com/adafruit/Adafruit-GFX-Library
//#include <Adafruit_ST7735.h>   // Hardware-specific library https://github.com/adafruit/Adafruit-ILI9341-Library
#include <Adafruit_ILI9341.h>    // Hardware-specific library https://github.com/adafruit/Adafruit-ILI9341-Library
#include <SPI.h>

//#include <Rotary.h>            //  Rotary encoder: https://github.com/brianlow/Rotary 
#include <Encoder.h>
Encoder myEnc(2,3);
//int TFT_LED = 9;
#define TFT_SCLK 13             // 1.8" TFT Display.
#define TFT_MOSI 11             //
#define TFT_CS   10
#define TFT_RST  -1  
#define TFT_DC    9

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS,  TFT_DC, TFT_RST);

#define AD9850_CLOCK 125000000         // Module crystal frequency. Tweak here for accuracy.

#define W_CLK 7 //4                        // 8 AD9850 Module pins.    
#define FQ_UD 6 //7       
#define DATA  5 //6       
#define RESET 4 //5     

#define ENC_A0  A0                    // Set 'Step' rotary encoder pins
#define ENC_A1  A1
// PCINT10 : Pin A2
int ENC_BUT  = A2;                     // 'Step' rotary encoder's push button - Set 1 Hz steps.
// PCINT11 : Pin A3
//int force_kHz = A3;                     // Interrupt-driven encoder's push button - force 1kHz freq.
/*
Rotary i = Rotary(ENC_A0, ENC_A1); // Rotart encoder for setting increment.
Rotary r = Rotary(2, 3);               // Rotary encoder for frequency connects to interrupt pins
*/
volatile long unsigned int freq_s = 1000,freq=1000;         // Set initial frequency.
volatile long unsigned int freqOld = freq;

long oldPos = -999,newPos=0;
long int timer;

const char* stepText[] = {"   1 Hz", "   2 Hz", "   5 Hz", "  10 Hz", "  20 Hz", "  50 Hz", " 100 Hz", " 200 Hz", " 500 Hz",
                            "  1 kHz", "  2 kHz", "  5 kHz", " 10 kHz", " 20 kHz", " 50 kHz", "100 kHz", "200 kHz", "500 kHz"};
const long f_st[] = { 0,10,100,1000,10000,100000,1000000};
volatile uint8_t s_Hz=0,s_dirty = 0,s_step=0,s_upd = 0;
volatile int8_t ist = 0;
volatile int stepPointer = 0; 
volatile unsigned long  incr = 0;
String units = stepText[stepPointer];

#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }


 // transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data) {
  for (int i = 0; i < 8; i++, data >>= 1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

void sendFrequency(double frequency) {
  int32_t freq1 = frequency * 4294967295/AD9850_CLOCK;  // note 125 MHz clock on 9850
  for (int b = 0; b < 4; b++, freq1 >>= 8) {
    tfr_byte(freq1 & 0xFF);
  }
  tfr_byte(0x000);                     // Final control byte, all 0 for 9850 chip
  pulseHigh(FQ_UD);                    // Done!  Should see output
}


void setup() {
  
  pinMode(2, INPUT_PULLUP);            // Pins for interrupt-driven rotary encoder and push buttons
  pinMode(3, INPUT_PULLUP);

  pinMode(ENC_A0,    INPUT_PULLUP);     // A0 PCINT8  Pins for step rotary encoder on analogue pins A2, A3
  pinMode(ENC_A1,    INPUT_PULLUP);     // A1 PCINT9
  pinMode(ENC_BUT,  INPUT_PULLUP);     // A2 PCINT10    
 // pinMode(force_kHz, INPUT_PULLUP);     // A3 PCINT11
  
  pinMode(FQ_UD, OUTPUT);              // Configure pins for output to AD9850 module.
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);
  
 //pinMode(TFT_RST, OUTPUT);            // Configure pins for output to TFT display.
  pinMode(TFT_DC, OUTPUT);
  //pinMode(TFT_LED, OUTPUT);
 
  //analogWrite(TFT_LED, 255);           // Adjust backlight brightness.
 cli();
 // Configure interrupt and enable for rotary encoder.
 // PCICR |= (1 << PCIE2);                      // Port: D
 // PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);  // D2, D3
  PCICR =  0b00000010;   // PCIE2: Pin Change Interrupt Enable 1
  PCMSK1 = 0b00000111;   // Enable Pin Change Interrupt for A0,A1,A2 // - A3
  sei();
  

 // tft.initR(INITR_BLACKTAB);           // initialize a ILI9341S chip, black tab
  tft.begin();
  tft.setRotation(3); 
  tft.setTextWrap(false);              // Allow text to run off right edge
  tft.fillScreen(ILI9341_BLACK);
   
  tft.setCursor(15, tft.height() -20);
  tft.setTextSize(1);
  tft.drawFastHLine(0, tft.height() - 23, tft.width()-10, ILI9341_BLUE);
  tft.setTextColor(ILI9341_BLUE);
  tft.println("AD9850 1 Hz to 5 MHz ");
  tft.print("   sinewave generator");
  
  // Initialise the AD9850 module. 
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);    // this pulse enables serial mode - Datasheet page 12 figure 10  
  
  updateDisplay();       // Update the TFT display.
}

void getStep() {
  switch(stepPointer) {
    case  0:  incr =      1; break;
    case  1:  incr =      2; break;
    case  2:  incr =      5; break;
    case  3:  incr =     10; break;
    case  4:  incr =     20; break;
    case  5:  incr =     50; break;
    case  6:  incr =    100; break;
    case  7:  incr =    500; break;
    case  9:  incr =   1000; break;
    case 10:  incr =   2000; break;
    case 11:  incr =   5000; break;
    case 12:  incr =  10000; break;
    case 13:  incr =  20000; break;
    case 14:  incr =  50000; break;
    case 15:  incr = 100000; break;
    case 16:  incr = 200000; break;
    case 17:  incr = 500000; break;
  } 
}


ISR (PCINT1_vect) {  // Port A

  static uint16_t seqA, seqB;

  // If interrupt is triggered by the button
  if      (!digitalRead(ENC_BUT) ) { // A2 / PCIN10
   // stepPointer = 0;
    s_Hz = 1;
  //  s_step = 1;
    if      (s_step == 1) s_step = 0;
    else if (s_step == 0) s_step = 1;
    s_dirty = 1;
  //  s_upd = 1;
  }
  else
  {
    // interrupt generated by encoder dial

    // Read A and B signals
    boolean A_val = digitalRead(ENC_A0);
    boolean B_val = digitalRead(ENC_A1);

    // Record the A and B signals in seperate sequences
    seqA <<= 1;
    seqA |= A_val;

    seqB <<= 1;
    seqB |= B_val;

    // Mask the MSB four bits
    seqA &= 0b00001111;
    seqB &= 0b00001111;


    if      (seqA == 0b00001001 && seqB == 0b00000011) {
      if ((freq - incr) >= 10) freq -= incr;
    }
    else if (seqA == 0b00000011 && seqB == 0b00001001) {
      if ((freq + incr) <= 10000000) freq += incr;
    }
    if (freq <= 10)       freq = 10;
    if (freq >= 10000000) freq = 10000000;
    
    s_dirty = 1;  
  }
} 

/*
ISR(PCINT1_vect) {
  unsigned char result = i.process();
  if (result) {
    if (result == DIR_CW) {
      if ((freq + incr) <= 10000000) freq += incr;
    } else {
      if ((freq - incr) >= 10) freq -= incr;
    }
    if (freq <= 10)  freq = 10;
    if (freq >=10000000) freq = 10000000;
  }
}
*/
void updateDisplay() {
  getStep();                          // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  units = stepText[stepPointer];
  tft.setTextSize(2);
 
  tft.setTextColor(ILI9341_ORANGE);
  
  tft.setCursor( 10, 20);  tft.print("Start: ");
 
  printFreq(80,20,freq_s);  // start-frequency  
  if (s_step == 0) tft.fillRect(70,50,160,16, ILI9341_DARKGREY);
  else tft.fillRect(70,50,160,16, ILI9341_BLACK);
   
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor( 10, 50);  tft.print("Step: ");
  tft.setCursor(170, 50);  tft.print(units);

  tft.setTextColor(ILI9341_GREEN);
  printFreq(80,80,freq);
  
}
void printFreq(int xp,int yp, long frx) {
  
   tft.fillRect(xp,yp,180, 20, ILI9341_DARKGREY);
   if (frx < 1000) {
    tft.setCursor(xp+42,yp);
    if (frx < 1000) tft.print(" ");
    if (frx <  100) tft.print(" ");
    tft.print(frx); 
    tft.setCursor(xp+127, yp);  tft.print(" Hz");
  } else
  if (frx < 1000000) {
   tft.setCursor(xp+22, yp);
   if (frx < 10000) tft.print(" ");
   tft.print((float)frx/1000, 3); 
   tft.setCursor(xp+127, yp);   tft.print(" kHz");
  }  else {
   tft.setCursor(xp, yp);  // ??? 
   format(frx);
   tft.setCursor(xp+127, yp);
   tft.print(" MHz");
  }
}


void format(long value) {
  int M = (value/1000000);
  int T100 = ((value/100000)%10);
  int T10 = ((value/10000)%10);
  int T1 = ((value/1000)%10);
  int U100 = ((value/100)%10);
  int U10 = ((value/10)%10);
  int U1 = ((value/1)%10);
//  tft.setCursor(25, 50);
  tft.print(M);tft.print(".");tft.print(T100);tft.print(T10);tft.print(T1);
  tft.print(",");tft.print(U100);tft.print(U10);tft.print(U1);
} 

void loop() {
  newPos = myEnc.read();    // Pin2 / Pin3
  if (newPos != oldPos) {
    if (newPos > oldPos) {
      if (s_step == 0) {
        if (stepPointer < 17) stepPointer++;
      }  
      else {
        if (ist < 5) {
          ist++;
          freq_s = f_st[ist];
          freq  = f_st[ist];
        }
      }  
    }
    else {
      if (s_step == 0) {
        if (stepPointer > 0) stepPointer--; 
      }  
      else         {
        if (ist > 0) {
          ist--;
          freq_s = f_st[ist];
          freq   = f_st[ist];
        }
      } 
    }
    s_upd = 1;
    oldPos = newPos;
 //   Serial.println(newPosition);
  }
 
  if (s_upd == 1 or freqOld != freq) {
    s_upd = 0;
    sendFrequency(freq);
    updateDisplay();
    freqOld = freq;
  }
}


