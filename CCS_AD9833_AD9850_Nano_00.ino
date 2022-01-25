//
//--------------------------------------------------------------------------------------- 
//  Thanks:
//
// Peter Balch  :  https://www.instructables.com/Signal-Generator-AD9833/               : AD9833 Source, flow-control, sweep, ...
// fhdm-devels  :  https://www.instructables.com/Simple-AD9833-Based-Signal-Generator/  : PinChangeInterrupt
// GreatScottLab:  https://www.instructables.com/DIY-FunctionWaveform-Generator/        : HD44780 display
//                 https://omerk.github.io/lcdchargen/                                  : Pixel-Generator HD44780 LCD Modules
//                 Interrupt-based (switches (PCI) by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt, Steve Spence
//---------------------------------------------------------------------------------------
// subject to the GNU General Public License
//---------------------------------------------------------------------------------------
// Developped with Arduino IDE 1.8.4 / 1.8.13
//---------------------------------------------------------------------------------------
//
// CCS_AD9833_Nano_00: 2022_01_05: copy from  CCS_NanO_08 and AD9833_PCI_Nano_HD44780_02
// CCS_AD9833_Nano_01: 2022_01_08: copy from  00. Test freq-Lo with Steps like current-steps
// CCS_AD9833_Nano_02: 2022_01_22: copy from  01. Test current-source
// CCS_AD9833_Nano_03: 2022_01_25: copy from  02 and AD9850_Nano_PCI_01: Integration Sine-Gen. AD9850
// CCS_AD9833_AD9850_Nano_00: 2022_01_25: copy from CCS_AD9833_Nano_03: 
//
#include <SPI.h>                     // for DAC and AD9833
#include <AD9833.h>                  // test
#include <MCP4822.h>                 // DAC 12 bit
#include <Encoder.h>
#include <Wire.h>                    // Wire for I2C
#include <LiquidCrystal_I2C.h>       // HD55780 LCD1602

//------------------------  Encoder with ext. Interrupt: 
const int ENC_CW   =  2; // ext.Int. Pin D2 <<-- change with D3 wrong direction ???
const int ENC_CCW  =  3; // ext.int. Pin D3 <<-- change with D2 wrong direction ???
//------------------------  switches with Pin-Change-Interrupt (PCI): 

const int RESET    =  4; // AD9850 Modul
const int DATA     =  5; // AD9850 Modul
const int FQ_UD    =  6; // AD9850 Modul
const int W_CLK    =  7; // AD9850 Modul   

const int SWITCH_R =  8; // switch a resistor parallel to Rm for increasing current ##################
const int DAC_CS   =  9; // MCP4822 12-bit Dac

//------------------------  AD9833 SPI-Pins
const int SG_FSYNC = 10; // AD9833 SPI-CS
const int SG_DATA  = 11; // AD9833 SPI-MOSI
const int SG_CLK   = 13; // AD9833 SPI-SCLK

const int ENC_A0   = A0; // PCINT8  : Pin A0    // rotary encoder steps
const int ENC_A1   = A1; // PCINT9  : Pin A1    // rotary encoder steps

const int ENC_BUT  = A2; // PCINT10 : Pin A2

#define AD9850_CLOCK 125000000      // Module AD9850 crystal frequency. 


AD9833 ad9(SG_FSYNC);               // SCK and MOSI must CLK and DAT pins on the AD9833 for SPI   

LiquidCrystal_I2C lcd(0x27, 16, 2); // HEX-Address 0x27 (or 0x3F), 16 rows , 2 lines_I2C lcd(0x27, 20, 4)" 


MCP4822 dac= MCP4822(DAC_CS);       // CS-Pin 9, AD9833 on Pin 10  

Encoder myEnc(ENC_CW,ENC_CCW);

volatile int EncPos    = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte enc_dir  = 'r';
volatile uint8_t but_p = 0,but_c=0;
volatile unsigned long  incr = 0;
volatile long unsigned int freq_s = 1000,freq=1000;         // Set initial frequency.
volatile long unsigned int freqOld = freq;
volatile uint8_t s_Hz=0,s_step=0,s_upd = 1;
volatile int stepPointer = 0; 

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
const byte numberOfDigits = 8;  // number of digits in the frequency

const uint16_t wft_s = 0x2000; // sine
const uint16_t wft_t = 0x2002; // triangle
const uint16_t wft_q = 0x2028; // square
const uint16_t wft_h = 0x2020; // halfsquare

//const int wSine     = 0b0000000000000000;
//const int wTriangle = 0b0000000000000010;
//const int wSquare   = 0b0000000000101000;
const int step_t[]    = {1,2,3,5,6,9,10,15,30};                           // step-sine  0...8          
const uint16_t sweep_t[] = {1000,2000,5000,10000,20000,50000};            // sweep-pos 
const uint16_t delay_t[] = {100,200,500,1,2,5,10,20,50,100,200,500,1000}; // delay
const long f_st[] = { 0,10,100,1000,10000,100000,1000000};                // strt frequency
const char* stepText[] = {"   1 Hz", "   2 Hz", "   5 Hz", "  10 Hz", "  20 Hz", "  50 Hz", " 100 Hz", " 200 Hz", " 500 Hz",
                            "  1 kHz", "  2 kHz", "  5 kHz", " 10 kHz", " 20 kHz", " 50 kHz", "100 kHz", "200 kHz", "500 kHz"};
String units = stepText[stepPointer];
//                     0   1   2   3   4   5   6   7   8   9  10  11  12
const char com_t[] = {'S','T','Q','L','C','H','M','P','D','G','R','X',' '};   // Menu-Commands:
// 0  'S': start Sine 
// 1  'T': start Triangle 
// 2  'Q': start Square
// 3  'L': set Freq.-Low 
// 4  'C': set Freq.-Low scroll contin. or steps 
// 5  'H': set Freq.-High
// 6  'M': swap Freq. Low  <--> High
// 7  'P': set sweep param. up/down,  step-nr
// 8  'D': set / change Delay sweep
// 9  'G': sweep 
//10  'R': Reset AD9833
//11  'X': return 

const char *str_com[]= {"Sine            ","Triangle        ","Square          ",
                        "set Low freq.   ","scroll Stepw >=1","set High freq.  ",
                        "swap Lo/Hi freq.","set sweep param.","set delay sweep ",
                        "sweep           ","Reset AD9833    ","return          "}; 

// HD44780 special-signs

const byte C_C_1_Arrow_D[8] = {  // from: https://omerk.github.io/lcdchargen/
  0b01110, 0b01010, 0b01010, 0b01010, 0b01010, 0b11011, 0b01110, 0b00100};

const byte C_C_2_Arrow_U[8] = {
  0b00100, 0b01010, 0b11011, 0b01010, 0b01010, 0b01010, 0b01010, 0b01110};

const byte C_C_3_Arrow_L[8] = {
  0b00010, 0b00100, 0b01111, 0b11000, 0b01111, 0b00100, 0b00010, 0b00000};

const byte C_C_4_Arrow_R[8] = { 
  0b01000, 0b00100, 0b11110, 0b00001, 0b11110, 0b00100, 0b01000, 0b00000};
  
const byte C_C_5_Tilde[8]  = { 
  0b00000, 0b00000, 0b00000, 0b01000, 0b10101, 0b00010, 0b00000, 0b00000};
/*
const byte C_C_5_Line_D[8] = {
  0b00000, 0b00000, 0b11111, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000};

const byte C_C_6_Sine_1[8] = { //  Sinus oben
  0b01110, 0b11011, 0b10001, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};


const byte C_C_7_Sine_2[8] = {// Sinus oben links nach unten
  0b00000, 0b00000, 0b10000, 0b11000, 0b01110, 0b00011, 0b00001, 0b00000};


const byte C_C_8_Sine_3[8] = {// Sinus unten
  0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b10001, 0b11011, 0b01110};


const byte C_C_9_Sine_4[8] = {// Sinus von unten links nach oben rechts
  0b00000, 0b00000, 0b00011, 0b01111, 0b01100, 0b11000, 0b10000, 0b00000};

*/

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

uint8_t  freqSave[numberOfDigits] = {0,0,1,0,0,0,0,0}; //   100 Hz 
uint8_t  freqSGLo[numberOfDigits] = {0,0,1,0,0,0,0,0}; //   100 Hz 
uint8_t  freqSGHi[numberOfDigits] = {0,0,0,0,2,0,0,0}; // 20000 Hz 

uint8_t  func_t[] = {0,1,2,3,4};
uint8_t  digi_t[] = {0,0,0,0};

uint32_t time_t[] = {0,1,2,5,10,20,50,100,200,500};    // µs 0...9
uint32_t time_m[] = {1,2,5,10,20,50,100,200,500,800};  // ms 0...9 // 10 ... 19
uint32_t time_s[] = {1,2,5,10,20,30,50,60,100,120};    // s  0...9 // 20 ... 29

int32_t  fact_t[] = {1,2,5,10,20,50,100,200,500,1000};
long     dig4_0_t[] = {0,0,0,0,0};

long     ilm=0,ilt=0,ifu=9,istep=0,zconst,fox=100,fstart=0,f_lowo=100,gainy=10,oldPos=-999,newPos=0;
int8_t   pos_i=15,pos_o=15,dig_o=0,dig_n=0,s_sel,ila=2,ido=10,ioo=0,igo=10,line_d=0,sel_c=0,hz_nr=0,sweep_r=0;;
int16_t  iso=10,ist=0,zwave_t = wft_s;        // sine
int32_t  time_del=10,delay_s=0,off_s=0,off_r=0;
unsigned long time_f,time_l,time_d,time_n;
long     fact=1,fact_o=1,iconst=0,fac_i=0,offs=0;
uint8_t  s_first=0,digi,s_ms=0,s_del=0,s_stepa=0,s_par=0,s_end=0,s_kHz=0,s_sweep=0;
uint8_t  s_low=0,sel_o=0,dmax=0,freq_zw=0,tab_nr=0,s_sine=0;
//int  waveType = wSine;

char c = ' ',com_o = 'S'; // old command

int i_del=3,i_sw=2,grad,step_s=1,stap=0,endp=1,iao,ieo;    // 4 ?
uint8_t posc_t[] = {7, 5};
int delay_micros = 1000;  // 1ms
int sweep_nr     = 5000;  

int SG_iSweep,SG_nSweep;

int16_t sinx = 0,ils; 

float wert=0.0,gainf=1.0;


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



//-----------------------------------------------------------------------------
// Main routines
// The setup function
//-----------------------------------------------------------------------------
void setup() {

// Serial.begin(115200);  //  Open serial port

  pinMode(ENC_CW,  INPUT_PULLUP);
  pinMode(ENC_CCW, INPUT_PULLUP);

  pinMode(ENC_A0,  INPUT_PULLUP);     // A0 PCINT8
  pinMode(ENC_A1,  INPUT_PULLUP);     // A1 PCINT9
  pinMode(ENC_BUT, INPUT_PULLUP);     // A2 PCINT10  
 
  cli();
 // Configure interrupt and enable for rotary encoder.
 // PCICR |= (1 << PCIE2);                      // Port: D
 // PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);  // D2, D3
  PCICR =  0b00000010;   // Port: C / PCIE1: Pin Change Interrupt Enable 1
  PCMSK1 = 0b00000111;   // Enable Pin Change Interrupt for A0,A1,A2 // - A3
  sei();
 /*  
   PCICR  = 0b00000100;   // PCIE2: Pin Change Interrupt Enable 2 // Port D (D0...D7)
 //PMMSK2 = 0b00011100;    // Enable Pin Change Interrupt for D2 - D4
 //PMMSK2 = 0b00001100;    // Enable Pin Change Interrupt for D2 - D3
  PCMSK2 = 0b00010000;   // Enable Pin Change Interrupt for D4
*/
  pinMode(FQ_UD, OUTPUT);              // Configure pins for output to AD9850 module.
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);
  
  pinMode(SWITCH_R, OUTPUT);
  digitalWrite(SWITCH_R, LOW);  // sets the digital pin off

  pinMode(DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH);  // sets the digital pin HIGH inactiv

  lcd.begin();                  //  HD44780

  // HD44780 special signs
  lcd.createChar(1, C_C_1_Arrow_D);  //########### Down
  lcd.createChar(2, C_C_2_Arrow_U);  //########### Up
  lcd.createChar(3, C_C_3_Arrow_L);  //########### Left
  lcd.createChar(4, C_C_4_Arrow_R);  //########### Right
  lcd.createChar(5, C_C_5_Tilde);    //########### Tilde 
  //lcd.createChar(5, C_C_5_Line_D);   //########### Line
  //lcd.createChar(6, C_C_6_Sine_1);   //########### Sine Top
  //lcd.createChar(7, C_C_7_Sine_2);   //########### Sine Down
  //lcd.createChar(1, C_C_8_Sine_3);   //########### Sine Bottom
  //lcd.createChar(2, C_C_9_Sine_4);   //########### Sine Up
  
  lcd.backlight(); //switch-on backlight,  lcd.noBacklight(); switch-off backlight 
  int8_t ilx=0,ily=1,ilz;
  for (uint8_t ill = 0;ill<5;ill++) {
    lcd.setCursor(0,ilx); lcd.print("Const.Cur.Source");  
    lcd.setCursor(0,ily); lcd.print("AD9833 Func.Gen.");
    ilz = ily;
    ily = ilx;
    ilx = ilz;
    delay(500);
  }  

// SPI.beginTransaction (SPISettings (8000000, MSBFIRST, SPI_MODE0));  // for AD9833 max 12.5 MHz, MCP4822: 20 MHz
// SPI.begin();

  delay(200);

  sel_c = 11;// "Sine " at start-display
  oldEncPos = 999;
  fillBlank(0,0,16); 
  fillBlank(0,1,16); 
}

void startDAC() {
 
  SPI.beginTransaction (SPISettings (20000000, MSBFIRST, SPI_MODE0));  // for AD9833 max 12.5 MHz, MCP4822: 20 MHz
  SPI.begin();
  
  dac.begin();
   // Configure analog outputs
  dac.off_B();
  dac.setGain2X_A();  // _AB();

  delay(200);
  
}



ISR (PCINT1_vect) {  // Port C  A0...A5 ??
  
  // If interrupt is triggered by the button
  if (!digitalRead(ENC_BUT)) but_p = 1;    // Pin A2: Push-Button pressed

}

/*
ISR (PCINT2_vect) {
  
  // If interrupt is triggered by the button
  if (!digitalRead(ENC_BUT)) but_p = 1;;    // Pin4: Push-Button pressed

}
*/
void fillBlank(uint8_t col, uint8_t line, uint8_t nr) {
  if (nr == 0) return;
  lcd.setCursor(col,line);
  for (uint8_t l=0;l<nr;l++)  {
    lcd.print(" ");
  }
}

//-----------------------------------------------------------------------------
//returns 10^y
//-----------------------------------------------------------------------------
unsigned long Power(int y) {
  unsigned long t = 1;
  for (byte i = 0; i < y; i++)
    t = t * 10;
  return t;
}

//-----------------------------------------------------------------------------
//calculate the frequency from the array.
//-----------------------------------------------------------------------------
unsigned long calcFreq(byte* freqSG) {
  unsigned long i = 0;
  for (byte x = 0; x < numberOfDigits; x++)
    i = i + freqSG[x] * Power(x);
  return i;
}

//-----------------------------------------------------------------------------
// calculate freq-array from the frequency (8-digits)
//-----------------------------------------------------------------------------
void printFreq(long freq) {
  long fo=freq,pot;
  int8_t zw=0,tab_l=numberOfDigits-1;// 8 tab-length - 1: 7... 0
  
  dmax = 0;
 
  for (int8_t ilc=tab_l;ilc>=0;ilc--) {  //4...0
 //   freqSGLo[ilc] = 0;
    pot = 1;
    if (ilc > 0) pot = Power(ilc);
    zw = fo/pot;
    if (zw > 0)  {
      if (dmax == 0) dmax = ilc;
      fo = fo - zw*pot;  
 //   freqSGLo[ilc] = zw;
      lcd.print(zw); 
    }  
    else {
      if (dmax == 0)  lcd.print(" ");
      else            lcd.print(zw); 
    }
  }

}



//-----------------------------------------------------------------------------
// calculate iconst-array from the iconst (5-digits)
//-----------------------------------------------------------------------------
void printIconst(long ic) {
  long fo=ic,pot;
  int8_t zw=0,tab_l=5-1;   // 5=tab-length - 1: 4 ... 0
  
  dmax = 0;
 
  for (int8_t ilc=tab_l;ilc>=0;ilc--) {  //4...0
    dig4_0_t[ilc] = 0;
    pot = 1;
    if (ilc > 0) pot = Power(ilc);
    zw = fo/pot;
    if (zw > 0)  {
      if (dmax == 0) dmax = ilc;
      fo = fo - zw*pot;  
      dig4_0_t[ilc] = zw;
      lcd.print(zw); 
    }  
    else {
      if (dmax == 0)  lcd.print(" ");
      else            lcd.print(zw); 
    }
  }

}


//-----------------------------------------------------------------------------
// calculate iconst-array from the iconst (5-digits)
//-----------------------------------------------------------------------------
void printIstep(long ist) {
  if      (ist >= 1000)  
    lcd.print(ist);
  else if (ist >=  100)  {
    lcd.print(" ");
    lcd.print(ist);
  }
  else if (ist >=  10)   {
    lcd.print("  ");
    lcd.print(ist);
  }
  else                   {
    lcd.print("   ");
    lcd.print(ist);
  }
}

/*
//-----------------------------------------------------------------------------
// SG_WriteRegister
//-----------------------------------------------------------------------------
void SG_WriteRegister(word dat) {
  digitalWrite(SG_CLK, LOW);
  digitalWrite(SG_CLK, HIGH);

  digitalWrite(SG_FSYNC, LOW);
  for (byte i = 0; i < 16; i++) {
    if (dat & 0x8000)
      digitalWrite(SG_DATA, HIGH);
    else
      digitalWrite(SG_DATA, LOW);
    dat = dat << 1;
    digitalWrite(SG_CLK, HIGH);
    digitalWrite(SG_CLK, LOW);
  }
  digitalWrite(SG_CLK, LOW);   //#HIGH
  digitalWrite(SG_FSYNC, HIGH);
  digitalWrite(SG_FSYNC, HIGH);
}

//-----------------------------------------------------------------------------
// SG_Reset
//-----------------------------------------------------------------------------
void SG_Reset() {
  delay(100);
  SG_WriteRegister(0x100);
  delay(100);
}

//-----------------------------------------------------------------------------
// SG_freqReset
//    reset the SG regs then set the frequency and wave type
//-----------------------------------------------------------------------------
void SG_freqReset(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2100);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
  SG_WriteRegister(0xC000);
  SG_WriteRegister(wave);
  waveType = wave;
}

//-----------------------------------------------------------------------------
// SG_freqSet
//    set the SG frequency regs 
//-----------------------------------------------------------------------------
void SG_freqSet(long frequency, int wave) {
  long fl = frequency * (0x10000000 / 25000000.0);
  SG_WriteRegister(0x2000 | wave);
  SG_WriteRegister((int)(fl & 0x3FFF) | 0x4000);
  SG_WriteRegister((int)((fl & 0xFFFC000) >> 14) | 0x4000);
}
*/


//-----------------------------------------------------------------------------
// SG_StepSweep
//    increment the FG frequency 
//-----------------------------------------------------------------------------
void SG_StepSweep(void) {
  if (SG_iSweep > SG_nSweep) SG_iSweep = 0;
  long f = exp((log(calcFreq(freqSGHi)) - log(calcFreq(freqSGLo)))*SG_iSweep/SG_nSweep + log(calcFreq(freqSGLo))) +0.5;
  if (s_sine == 0) { 
    ad9.ApplySignal(zwave_t,REG0,f); // 
    ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF
  }
  else sendFrequency(f); // AD9850
  
 // SG_freqSet(f, waveType);
  SG_iSweep++;
}

//-----------------------------------------------------------------------------
// Sweep
//   sweeps siggen freq continuously
//   takes n mS for whole sweep
//   SDC regs are saved and restored
//   stops when push-button is pressed
//-----------------------------------------------------------------------------
void Sweep(int n) {
  int fmin,fmax;
  long f;
  delay(100);
  but_p = 0;
  fmin = calcFreq(freqSGLo);
  fmax = calcFreq(freqSGHi);
  int i= n-1; 
  if (s_sine == 0) ad9.EnableOutput(true);     //
  if (sweep_r == 0)        {  // up
    while (but_p == 0)      {
      for ( i=n-1;i>=0;i--)  {   
        f=exp((log(fmax) - log(fmin)) * i/(n-1) + log(fmin)) + 0.5;  //??
        if (s_sine == 0)  ad9.ApplySignal(zwave_t,REG0,f);
        else  sendFrequency(f);  // AD9850
        delayMicroseconds(delay_micros); //delay(1);
         if (but_p == 1) goto end_sw;
      }
    }
  }
  else if (sweep_r == 1)  { // down   
    while (but_p == 0)     {
      for ( i=0;i<=n-1;i++) {   
        f=exp((log(fmax) - log(fmin)) * i/(n-1) + log(fmin)) + 0.5;   //??
        if (s_sine == 0)  ad9.ApplySignal(zwave_t,REG0,f);
        else  sendFrequency(f); // AD9850
        delayMicroseconds(delay_micros); //delay(1);
        if (but_p == 1) goto end_sw;
      }
    }
  }                           
  else if (sweep_r == 2)  { // up  and down
    i = 0;
    do {
      f = exp((log(fmax) - log(fmin))*i/(n-1) + log(fmin)) + 0.5;  //
      if (s_sine == 0)  ad9.ApplySignal(zwave_t,REG0,f); //
      else sendFrequency(f); // AD9850
      delayMicroseconds(delay_micros); //delay(1);
      i++;
      if (i >= n) i = 0;
      if (but_p == 1) goto end_sw;
    } while (but_p == 0) ;//!Serial.available());
  }  
 end_sw: 
  if (s_sine == 0)   {
    ad9.ApplySignal(zwave_t,REG0,fmin); // T
    ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF
  }
  else sendFrequency(fmin);  // AD9850
  
  //SG_freqSet(calcFreq(freqSGLo), waveType);
}

//-----------------------------------------------------------------------------
//
// 
//-----------------------------------------------------------------------------
void controlAD9833() {
 
  s_end = 0;
  lcd.setCursor(0,1);lcd.print("              ");
  sel_c = 11;  // start with sine  ???
  while(s_end == 0) {
    but_p   = 0;
    s_first = 0;
  
    while(but_p == 0) {   // not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (oldEncPos < EncPos) sel_c++;
          else                    sel_c--;
          if      (sel_c <  0) sel_c = 11;
          else if (sel_c > 11) sel_c =  0;
          fillBlank(0,1,5); //lcd.setCursor(0,1);lcd.print("     ");
          lcd.setCursor(0,1);lcd.print(str_com[sel_c]); 
        }  
        oldEncPos = EncPos;
      }  
    }  // while(but_p
  
    TabCommand(com_t[sel_c]);  // execute commands ####################################

   // if (s_end == 1) return;
  }
}

//-----------------------------------------------------------------------------
// TabCommand
//   if a byte is available in teh serial input buffer
//   execute it as a command
//-----------------------------------------------------------------------------
void TabCommand(char com_x) {
  int8_t itt;
  c = com_x;  // globale variable
  
  switch_s:
     
    switch (com_x) {
      case 'S':  // 0  run sine
        lcd.setCursor(0,1); lcd.print(str_com[0]);
        
        freq_Lo_to_Display(); // Hz, MHz;
        if (s_sine == 0) { // AD9833
          ad9.ApplySignal(wft_s,REG0,fox);  // SINE_WAVE
          ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF
        }
        else sendFrequency(fox);  // AD9850
        
        lcd.setCursor(0,1); lcd.print("Sine  start     ");
        zwave_t = wft_s;
        delay(50);
        com_o = 'S';
        sel_o = 0;
       break;    
      case 'T':  // 1 run triangle
        if (s_sine == 1) break;
        lcd.setCursor(0,1); lcd.print(str_com[1]);
        freq_Lo_to_Display();// Hz, MHz 
        ad9.ApplySignal(wft_t,REG0,fox); // TRIANGLE_WAVE
        ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF
             
        lcd.setCursor(0,1); lcd.print("Triangle start  ");
        zwave_t = wft_t;
        delay(50);
        com_o = 'T';
        sel_o = 1;
       break;    
      case 'Q':  // 2 run Square
        if (s_sine == 1) break;
        lcd.setCursor(0,1);lcd.print(str_com[2]);
        freq_Lo_to_Display();// Hz, MHz 
        ad9.ApplySignal(wft_q,REG0,fox);  // SQUARE_WAVE
        ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF 
           
        lcd.setCursor(0,1); lcd.print("Sqare start     ");
        zwave_t = wft_q;
        delay(50);
        com_o = 'Q';
        sel_o = 2;
       break;    
      case 'L':  // 3 set Fequency Low
      //  lcd.setCursor(0,1);//lcd.print(str_com[3]); 
        lcd.setCursor(10,0);lcd.print(" Hz    ");
        s_low = 0;
        changeFreqLoHi();
        f_lowo = fox;
        for (itt=0;itt<numberOfDigits;itt++) {  // 0 ... 7
          freqSave[itt] = freqSGLo[itt];
        }
        com_x = com_o;
        sel_c = sel_o;
        goto switch_s;                           //####################
       break;   
      case 'C':  // 4 set Fequency Low continually or steps
      //  lcd.setCursor(0,1);//lcd.print(str_com[3]); 
        lcd.setCursor(10,0);lcd.print(" Hz    ");
        istep = 0;
        for(uint8_t ilg = 0;ilg<=3;ilg++) {
          digi_t[ilg] = 0;
        }
        s_low = 3;   
       
        changeFreqLoContin();  //###################################
        fox = f_lowo;
        for (itt=0;itt<numberOfDigits;itt++) {  // 0 ... 7
          freqSGLo[itt] = freqSave[itt];
        }
        com_x = com_o;
        sel_c = sel_o;
        goto switch_s;                           //####################
       break;      
      case 'H':  // 5 set Fequency High
      //  lcd.setCursor(0,1);//lcd.print(str_com[4]); 
        lcd.setCursor(10,0);lcd.print(" Hz    ");
        s_low = 1;
        
        changeFreqLoHi();      //##########################
        com_x = com_o;
        sel_c = sel_o;
        goto switch_s;                           //####################
       break;   
   /*   case 'F':  // 5 set Fequency Low and High
      //  lcd.setCursor(0,1);//lcd.print(str_com[3]); 
        lcd.setCursor(10,0);lcd.print(" Hz    ");
        s_low = 2;
        changeFreqLoHi();
        com_x = com_o;
        sel_c = sel_o;
        goto switch_s;                           //####################
       break;      
      */ 
      case 'M':  // 6 swap frequencys high and low 
        lcd.setCursor(0,1);lcd.print("swap High ");lcd.write(3);lcd.write(4);lcd.print(" Low"); 
        lcd.setCursor(0,0);lcd.print("  ");
        for (int i=7; i>=0; i--) {
          freq_zw     = freqSGHi[i];  // save old High
          freqSGHi[i] = freqSGLo[i];
          freqSGLo[i] = freq_zw; 
          lcd.print(freq_zw);
       //   Serial.println(freq_zw);
        }  
        freq_zw   = posc_t[0];  // cursor-position Low/High
        posc_t[0] = posc_t[1];
        posc_t[1] = freq_zw;
        lcd.print(" Hz   ");
        if (s_sine == 0) {
          ad9.ApplySignal(zwave_t,REG0,fox);
          ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF 
        }
        else sendFrequency(fox);  // AD9850
       
        freq_Lo_to_Display(); // frequency in Hz, MHz
        lcd.setCursor(14,0);lcd.print("ok");
        delay(500);
        lcd.setCursor(14,0);lcd.print("  ");
        if (com_o == ' ' ) com_o = 'S';
        com_x = com_o;
        goto switch_s;                          //####################
       break;   
      case 'P':  // 7 set param. sweep up/down and step-nr
        lcd.setCursor(0,1);lcd.print(str_com[7]);// lcd.print("sweep");
        changeSweepNr();
        lcd.setCursor(14,1);lcd.print("ok");
        delay(1000);
        fillBlank(10,0,7); 
        fillBlank(10,1,6);
     //   if (com_o == ' ' ) com_o = 'S';
        com_x = 'G';
        goto switch_s;                          //####################
       break;    
        
      case 'D':  // 8 set delay 
        lcd.setCursor(0,1);lcd.print(str_com[8]); // lcd.print("delay(ms)"); //\344s/ms)"); // \344 = µ
        lcd.setCursor(10,1);lcd.print("                 ");
        changeDelay();
        lcd.setCursor(14,1);lcd.print("ok");
        delay(1000);
        lcd.setCursor(10,0);lcd.print("       ");
        lcd.setCursor(14,1);lcd.print("  ");
      //  if (com_o == ' ' ) com_o = 'S';
        com_x = 'G'; //com_o;
        goto switch_s;                          //####################
       break; 
     case 'G':  // 9 sweep  up/down, steps
        lcd.setCursor(0,1);lcd.print(str_com[9]);
        lcd.setCursor(6,1);
        if      (sweep_r == 0) lcd.write(2);   // up 
        else if (sweep_r == 1) lcd.write(1);   // down
        else                   lcd.write(5);   // Tilde// direction: up,down, Tilde  
        
        Sweep(sweep_nr); 
        lcd.setCursor(14,0);lcd.print("  ");
        if (com_o == ' ' ) com_o = 'S';
        if (com_o == 'G' ) com_o = 'S';
        com_x = com_o;
        goto switch_s;                          //####################
       break;  
      
      case 'R':  // 10 Reset AD9833
        if (s_sine == 1) break;
        lcd.setCursor(0,1);lcd.print(str_com[10]); // "Reset  ");
        fillBlank(0,0,16); 
        ad9.Reset(); 
       break;    // SigGen reset 
      case 'X':  // 11 Return main
        lcd.setCursor(0,1);lcd.print(str_com[11]); // "Return  ");
        fillBlank(0,0,16);
        delay(200);
        s_end = 1; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<######################################
        fillBlank(0,1,16); 
        return; 
       break;    // SigGen reset  
      default:  // return;
       break;
    }
  
}


/*

//-----------------------------------------------------------------------------
// InitSigGen
//-----------------------------------------------------------------------------
void InitSigGen(void) {
  
  pinMode(SG_DATA,  OUTPUT);
  pinMode(SG_CLK,   OUTPUT);
  pinMode(SG_FSYNC, OUTPUT);
  digitalWrite(SG_FSYNC, HIGH); 
  digitalWrite(SG_CLK, LOW);  //#HIGH
  
  SG_Reset();
  SG_freqReset(calcFreq(freqSGLo), waveType);
  
}
*/


void changeDelay()  {
  int z_micros = 0;
  oldEncPos = 999;
  EncPos = 0;
  but_p  = 0;
  lcd.setCursor(0,0); lcd.blink();  
  lcd.setCursor(0,0); lcd.print("delay:        ");
  lcd.setCursor(7,0); lcd.print(delay_t[i_del]); 
  if (i_del < 3) lcd.print(" \344s");
  else           lcd.print(" ms");
 // while(1==1)  {
     
    while (but_p == 0)  {    // button-switch not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (oldEncPos < EncPos) enc_dir = 'r';
        else                    enc_dir = 'l';
                 
        if (enc_dir == 'r') if (i_del < 12) i_del++;
        if (enc_dir == 'l') if (i_del >  0) i_del--;
        
        lcd.setCursor(7,0);lcd.print("        ");
        lcd.setCursor(7,0);lcd.print(delay_t[i_del]); 
        if (i_del < 3) lcd.print(" \344s");
        else           lcd.print(" ms");
        if      (i_del <=  3) z_micros = delay_t[i_del];            // µs 
        else if (i_del <= 11) z_micros = delay_t[i_del] * 1000;     // ms 
        else                  z_micros = delay_t[i_del] * 1000000;  //  s 
        
        oldEncPos = EncPos;  
        lcd.setCursor(7,0);
       
      }  
      
    }  // while
    delay_micros = z_micros;
  
    return;
 // }  // while
   
}

void changeSweepNr()  {
    int z_number = 0;
    uint8_t s_switch=0;
    oldEncPos = 999;
    EncPos  = 0;
    but_p   = 0;
    sweep_r = 2;
    lcd.setCursor(0,1); lcd.print(" ...steps:      "); 
    lcd.setCursor(0,0); lcd.print("sweep-up/down: ");
    lcd.setCursor(15,0); lcd.blink();
    lcd.setCursor(15,0);
    if      (sweep_r == 0) lcd.write(2);   // up 
    else if (sweep_r == 1) lcd.write(1);   // down
    else                   lcd.write(5);   // Tilde
    while (but_p == 0)  {    // button-switch not pressed
   
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_switch == 0) s_switch = 1;
        else {
          s_switch = 0;
          lcd.setCursor(15,0);
          if (enc_dir == 'r') sweep_r++;
          if (enc_dir == 'l') sweep_r--;
          if (sweep_r < 0) sweep_r = 2;
          if (sweep_r > 2) sweep_r = 0;
          if      (sweep_r == 0) lcd.write(2);   // up 
          else if (sweep_r == 1) lcd.write(1);   // down
          else                   lcd.write(5);   // Tilde
        }  
        oldEncPos = EncPos;
      }  
    }
    oldEncPos = 999;
   // lcd.print(" nr");
//  while(1==1)  {
    but_p = 0; 
    lcd.setCursor(11,1);
    s_switch = 0;
    while (but_p == 0)  {    // button-switch not pressed
    
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_switch == 0) s_switch = 1;
        else {
          s_switch = 0;
          if (oldEncPos < EncPos) enc_dir = 'r';
          else                    enc_dir = 'l';           
          if (enc_dir == 'r') if (i_sw <  5) i_sw++;
          if (enc_dir == 'l') if (i_sw >  0) i_sw--;
        
          fillBlank(11,1,5); //lcd.setCursor(7,0);lcd.print("         ");
          if (sweep_t[i_sw] >= 10000) lcd.setCursor(11,1);
          else                        lcd.setCursor(12,1);
          lcd.print(sweep_t[i_sw]); 
        
          z_number = sweep_t[i_sw];
        }        
        oldEncPos = EncPos;  
        lcd.setCursor(11,1);
      }  
    }  // while
    sweep_nr = z_number;
    
    return;
 // }  // while
   
}


void changeFreqLoHi()  {
  uint8_t il; 
 
  fillBlank(0,0,16); //lcd.setCursor(0,0);lcd.print("                ");
  fillBlank(10,1,6); //lcd.setCursor(10,1);lcd.print("      "); 
  lcd.setCursor(0,0); lcd.print("Lo");
  for (il=0;il<numberOfDigits;il++) {
    lcd.print(freqSGLo[numberOfDigits-1-il]);
  }  
  if (s_low != 3) {
    lcd.print(" Hz   ");
    lcd.setCursor(0,1); lcd.print("Hi");
    for (il=0;il<numberOfDigits;il++) {
      lcd.print(freqSGHi[numberOfDigits-1-il]);
    }  
    lcd.print(" Hz   ");
  }
  //else fillBlank(0,1,9);                 // Lo 
  line_d = 0;
  if (s_low == 3) {                        // continually rotation or Steps  ################
     fox = f_lowo;
     return;
  }   
  if (s_low == 0 or s_low == 2) scanPos(); // Low- Frequency #############################
  if (s_low == 0) return;
  line_d = 1;
  scanPos();                               // High-Frequency #############################  
  s_low = 0;  // ?? 
   
}


void scanPos()  {  // decimal-positions of Low-frequency
  
  
  uint8_t ilp;
  long ild;
  oldEncPos = 999;
  EncPos    = 999;//0;
 
  pos_i = posc_t[line_d];  //?
 
  while(1==1)  {
    but_p   = 0;
    s_first = 0; 
    lcd.setCursor( pos_i,line_d);  lcd.blink();  
    while (but_p == 0)  {    // button-switch not pressed
      
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1; // first  switch / teeth
        else {
          s_first = 0;    
          if (oldEncPos < EncPos) {  // 'r'
            if (pos_i < 9) pos_i++;  // 
            else {
              pos_i = 2;
              lcd.setCursor(2,line_d); // lcd.blink();  
            }
          }
          else pos_i--;              // 'l'           
               
          lcd.setCursor( pos_i,line_d); // lcd.blink();  
          if (pos_i <  2)     {
             // lcd.blink_off();
            if (pos_i == 1) pos_i = 1; 
            if (pos_i <= 0) {  // == 0 is ok
              but_p = 0;
              while (but_p == 0) { } // button not pressed 
              but_p  = 0;
              goto pos_end;
            }
          }
        }
        oldEncPos = EncPos;
     
      }  
       
      lcd.setCursor(pos_i,line_d); // lcd.blink();   //#################
     
    }  // while but_p == 0)
     // button-switch pressed
    
   
    if (pos_i >= 2)  changeDigit_AD();          // ######################################
   
  }  // while 1
 pos_end:
  
   
   if (line_d == 0) {
     fox = 0;
     ild = 0;
     for (ilp = 0;ilp<=7;ilp++) {
       if (ild == 0) ild = 1;
       else ild = ild*10;
       fox = fox + freqSGLo[ilp] * ild;
     }
     fillBlank(0,0,16); //lcd.setCursor(0,0); lcd.print("  ");
   }  
   else {
     fillBlank(0,0,16); //lcd.setCursor(0,0); lcd.print("                ");  //line_d ???
     lcd.setCursor(14,line_d);lcd.print("ok");
     fillBlank(0,0,16); //lcd.setCursor(0,0); lcd.print("                ");
   }
  
}


void freq_Lo_to_Display() {
     uint8_t ilx,il0=0;
     if (fox < 1000000) {
       lcd.setCursor(7,1);//lcd.print(freqSGLo[0]);lcd.print(freqSGLo[1]);lcd.print(".");
       for(ilx=0;ilx<=5;ilx++) { 
         if (il0 == 0) {
           if (freqSGLo[5-ilx] == 0) lcd.print(" ");
           else {
              lcd.print(freqSGLo[5-ilx]); 
              il0 = 1;
           }
         }
         
         else lcd.print(freqSGLo[5-ilx]); 
       };  // last 6 digits
       lcd.print(" Hz");
    
     }
     else {  // >= 1000000
      
       if      (fox > 9999999) { // 10-12 MHz
         lcd.setCursor(6,1);lcd.print(freqSGLo[7]);lcd.print(freqSGLo[6]);lcd.print(".");
       }  
       
       else if (fox > 999999) { // 1-9 MHz
         lcd.setCursor(7,1);lcd.print(" ");lcd.print(freqSGLo[6]);lcd.print(".");
       }  
       for(ilx=2;ilx<=6;ilx++) { lcd.print(freqSGLo[7-ilx]); };
       lcd.setCursor(12,1);lcd.print(" MHz");
       
     }
  
}

void changeDigit_AD() {
   int8_t dig_x;
   dig_x = pos_i - 2;  // 0...7
   dig_x = 7 - dig_x;  // 7...0 
 
   if (dig_x >= 0 and dig_x < 8) {  // > 0  8//  9 ?
     if (line_d == 0) dig_o = freqSGLo[dig_x];  //digi_t[dig_x];
     else             dig_o = freqSGHi[dig_x];  //digi_t[dig_x];
     dig_n = dig_o;
   }
   else return;
      
   oldEncPos = dig_o;   //######################
   EncPos    = dig_o;   //######################
   lcd.noBlink();      //############################# ??  ##############################
   but_p = 0;
   s_first = 0;
   while (but_p == 0)  {         // button-switch not pressed ?
     EncPos = myEnc.read();     //###################################
     if (oldEncPos != EncPos) {
       if (s_first == 0) s_first = 1; // first  switch / teeth
       else {
         s_first = 0;    
         if (oldEncPos < EncPos) dig_n--;// enc_dir = 'r';
         else                    dig_n++;//enc_dir = 'l'; 
         if (dig_n > 9) dig_n = 0;
         if (dig_n < 0) dig_n = 9;  
         if (pos_i == 2 ) {    // 10Mhz pos
           if (dig_n > 1 and freqSGLo[6] <= 2) dig_n = 1; // example 12.4 MHz 
           else dig_n = 0;                                // example: 9.3 Mhz 
         }
       
         if (line_d == 0) freqSGLo[dig_x] = dig_n;  // save in tab  Low_frequeny   ##########################
         else             freqSGHi[dig_x] = dig_n;  // save in tab  High-frequency ##########################
         
       //  Serial.print(" dx: "); Serial.print(dig_x);Serial.print(" digit: "); Serial.println(digi_t[dig_x]);
       //  Serial.print("s_low "); Serial.print(s_low);Serial.print(" line: "); Serial.print(line_d);Serial.print(" dig_n: "); Serial.println(dig_n);
         
         lcd.setCursor(pos_i,line_d);lcd.print(dig_n);
        
         if (line_d == 0) {         
           if (s_low != 3) fox = 0;
           else            fox = fstart;
           long ilf = 0;
           for (uint8_t ilq = 0;ilq<=7;ilq++) {
              if (ilf == 0) ilf = 1;
              else ilf = ilf*10;
              fox = fox + freqSGLo[ilq] * ilf;
           }  
           if (s_sine == 0) {
             ad9.ApplySignal(zwave_t,REG0,fox);  //ChangeDigit_AD
        //     ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF
           }
           else sendFrequency(fox);  // AD9850
         }  
 //      delay(50);  // 100
 //      lcd.blink_off();
       } 
       oldEncPos = EncPos;
       
     } 
//     lcd.setCursor(pos_i,line_d);  lcd.blink();   //#################
     posc_t[line_d] = pos_i;
    
   }  // while but_p == 0
 //  delay(50);   // 50
  
   but_p = 0;
  
}



void  setGainy() {
    
    int8_t ilt=igo;
    oldEncPos = 999;
    s_first=0;
    but_p = 0; 
    gainf = 1.0;
    while(but_p == 0) {          // push-button not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (oldEncPos < EncPos) ilt--; 
          else                    ilt++; 
          if (ilt >  25) ilt = 25;
          if (ilt <   0) ilt =  0; 
          gainy = ilt;
          gainf = (float) gainy;
          gainf = gainf/10.0; 
          s_ms = 0;  // µs
          lcd.setCursor(0,1);  lcd.print("   ");
          if (gainy ==  0) gainy = 1;
          if (gainy >= 10) {
            lcd.setCursor(0,1);lcd.print(gainy);
            lcd.setCursor(1,1);lcd.print(".");
            if (gainy > 19) gainy = gainy - 20;
            else            gainy = gainy - 10;
            lcd.setCursor(2,1);lcd.print(gainy);   
          }
          else {
            lcd.setCursor(1,1);lcd.print(".");
            lcd.setCursor(2,1);lcd.print(gainy);
          }
         
        }             
        oldEncPos = EncPos;
        delay(100);
      }   // if
    }    // while
    igo = ilt;
}


void  setOffset() {
    int8_t ilt=ioo;
    long zoff;
    off_s = 0;
    oldEncPos = 999;
    but_p = 0;
    s_first = 0;
    while(but_p == 0) {          // push-button not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (oldEncPos < EncPos) ilt--; // enc_dir = 'r';
          else                    ilt++; // enc_dir = 'l';
          if (ilt >  25) ilt =  25;
          if (ilt < -25) ilt = -25; 
          zoff  = ilt;
          off_s = ilt*100;   // mV
          lcd.setCursor(3,1);
          if (zoff < 0) 
            lcd.print("-");
          else 
            lcd.print(" ");
         
          if (zoff >= 10 or zoff <= -10) { // on screen 1.0
            if (zoff < 0 ) zoff = -zoff;
            lcd.print(zoff);
            lcd.setCursor(5,1);lcd.print(".");
            if      (zoff >  19) zoff = zoff - 20;  // 0...5
            else if (zoff >   9) zoff = zoff - 10;  // 0...9
            lcd.print(zoff);   
          }
          else {
            lcd.print("0.");
            if (zoff < 0) zoff  = -zoff; 
            lcd.print(zoff);
          }
        //  lcd.print("V"); 
        }     
        oldEncPos = EncPos;
        delay(70);
      }  // if (old
    }  // while
    but_p = 0;
   ioo = ilt;
   
  
}


void setDelay() {
    
    oldEncPos = 999;
    s_first = 0;
    s_sweep = 0;
    but_p   = 0; 
    ilt     = ido;
    s_sweep = 0;
    //  lcd.setCursor(12, 0);  lcd.print("\133\344s\135\ ");   // HD47880 Sonderzeichen: \133: [ ,\135: ] ,\344: µ,\364: Omega (für z.B. Ohm 
    while(but_p == 0) {          // push-button not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (oldEncPos < EncPos) ilt--; // enc_dir = 'r';
          else                    ilt++; // enc_dir = 'l';
                      
          if (ilt >  29) ilt = 29;
          if (ilt <   0) ilt =  0; 
          if      (ilt <=  9) {      // 0...500 µs  //  0... 9
            s_ms = 0;  // µs
            lcd.setCursor( 11, 1);  lcd.print("\344s"); //del 14
            time_del = time_t[ilt];  // µs
            s_del = ilt;
            delay_s = time_del;      // µs 
          }
          else if (ilt <= 19) {      // 1...800 ms  // 10...19
            s_ms = 1; // ms
            lcd.setCursor(11, 1);  lcd.print("ms"); //del 14
            ilm = ilt - 10;
            s_del = ilm;
            time_del = time_m[ilm];  // ms
            delay_s = time_del;      // ms
          }
          else if (ilt <= 29) {      // 1...120 s   // 20...29
            s_ms = 2; //  s
            lcd.setCursor(11, 1);  lcd.print(" s");  // del 14
            ilm = ilt - 20;
            time_del = time_s[ilm];  // s
            s_del = ilm;
            delay_s = time_del*1000; // ms
          }
          lcd.setCursor( 7, 1);                               lcd.print("    ");  // del 10
          if      (time_del <      10) {lcd.setCursor( 10, 1); lcd.print(time_del);}
          else if (time_del <     100) {lcd.setCursor(  9, 1); lcd.print(time_del);}
          else if (time_del <    1000) {lcd.setCursor(  8, 1); lcd.print(time_del);}
          else                         {lcd.setCursor(  7, 1); lcd.print(time_del);}     
          if (delay_s == 0) s_sweep = 1;
        }
        oldEncPos = EncPos;
        delay(100);
      }   // if
    }    // while
    ido = ilt;
}


void  setStep() {
    
    int16_t isi = 1,ilt,ilu=0;
    oldEncPos = 999;
    s_first=0;
    but_p = 0; 
    gainf = 1.0;
    ist = iso;
    
    ilt = 0;
    while(but_p == 0) {          // push-button not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (oldEncPos < EncPos) ilt = -1; 
          else                    ilt =  1; 
          if (ifu > 0) {  // Triangle, StepUp, StepDown
            if      (ist >= 100) isi = 20;
            else if (ist >=  50) isi = 10;
            else if (ist >=  20) isi =  5; 
            else if (ist >=  10) isi =  2; 
          }
          else {         // Sine  
            ilu= ilu + ilt;
            if      (ilu > 8) ilu = 0;
            else if (ilu < 0) ilu = 8; 
           
          }
          
          ist = ist + ilt * isi;  
   /*       if (ilt < 2) {
            if (ilt <= 0)  ist =  0;  // 360°
            else           ist =  3; 
          }*/
          if (ist >= 200) ist = 200;
          if (ifu == 0) {
            ist = step_t[ilu];
            step_s = ist;
          }
          lcd.setCursor(13,1);  lcd.print("   ");
          if      (ist <  10)   lcd.setCursor(15,1);
          else if (ist < 100)   lcd.setCursor(14,1);
          else                  lcd.setCursor(13,1);
          lcd.print(ist);
         
        }             
        oldEncPos = EncPos;
        delay(100);
      }   // if
    }    // while
    if (ist <= 0) ist = 1;
    iso = ist;
}

void  setStart() {
    
    int8_t ilt=iao;
    oldEncPos = 999;
    s_first=0;
    but_p = 0; 
    gainf = 1.0;
    while(but_p == 0) {          // push-button not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (oldEncPos < EncPos) ilt--; 
          else                    ilt++;
          if (ifu > 1) { 
            if (ilt >  step_s) ilt = step_s;
          }
          else if (ifu == 0) { 
            if (ilt >  4 * step_s) ilt = 4*step_s-1;
          }   
          else if (ifu == 1) { 
            if (ilt >  2 * step_s) ilt = 2*step_s-1;
          }   
          if (ilt <       0) ilt =  0; 
          stap = ilt;
          lcd.setCursor(0,0);  lcd.print("Sta"); 
          lcd.setCursor(0,1);  lcd.print("   ");
          if      (stap > 99)     lcd.setCursor(0,1);
          else if (stap >  9)     lcd.setCursor(1,1);
          else                    lcd.setCursor(2,1);
          lcd.print(stap);
        }            
        oldEncPos = EncPos;
        delay(100);
      }   // if
    }    // while
    iao = ilt;
}


void  setEnd() {
    
    int8_t ilt=ieo;
    oldEncPos = 999;
    s_first=0;
    but_p = 0; 
    gainf = 1.0;
    while(but_p == 0) {          // push-button not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (oldEncPos < EncPos) ilt--; 
          else                    ilt++; 
          if (ilt <  stap+1) ilt = stap+1; 
          if ( ifu > 1) { 
            if(ilt >= step_s) ilt = step_s;
          }    
          else if (ifu == 0) {
            if (ilt >= 4 * step_s) ilt = 4 * step_s; 
          }  
          else if (ifu == 1) {
            if (ilt >= 2 * step_s) ilt = 2 * step_s; 
          }  
          endp = ilt;
          lcd.setCursor(4,0);  lcd.print("End"); 
          lcd.setCursor(4,1);  lcd.print("   ");
          if      (endp > 99)     lcd.setCursor(4,1);
          else if (endp >  9)     lcd.setCursor(5,1);
          else                    lcd.setCursor(6,1);
          lcd.print(endp);
        }             
        oldEncPos = EncPos;
        delay(100);
      }   // if
    }    // while
    ieo = ilt;
}

void newDacVal(float vald) {
   if (ifu != 0) vald = vald*1000.00;     // not for sine 
   if (gainf != 1.0)  vald = vald * gainf;
   if (vald > 4095.0) vald = 4095.0;
   if (off_s != 0)    vald = vald + (float)off_s;
   if (vald > 4095.0) vald = 4095.0;
   if (vald <    0.0) vald =    0.0;
 
   dac.setValue_A((int)vald);
  
   if (delay_s > 0) {
     if (s_ms == 0) delayMicroseconds(delay_s);  // µs
     else           delay(delay_s);              // ms
   } 
  
}


void runSine()  {  // Sine
   int32_t delay_m;
   uint16_t sin_t[90];
   delay_m = 200;
   stap=0;
   endp=100;
   for(grad=0;grad<90;grad+=step_s) {        // quarter-Sinewave in Tab sin_t[90]
     const float grad_bog = 0.01745329;  // pi / 180 
     wert = sin(grad_bog*(float)grad); // Bogenmaß Wertebereich -1 ... +1
     wert = wert+1.00; // nur positive Zahlen 0 ... +2
     wert = wert*1000.00;
     sinx = (int)(wert+0.5);
     if (sinx <    0) sinx = 0;
     if (sinx > 4095) sinx = 4095;
     sin_t[grad] = sinx;
   }  //for
   oldEncPos = EncPos;
   but_p = 0;
   while(but_p == 0) {   // run Sine-wave
 
     for(grad=0;grad<=89;grad+=step_s) {  // first 0...90°
  //    if(grad <= stap or grad >= endp)                  wert = 0.0;
       wert = (float)sin_t[grad];
       newDacVal(wert);
       if (s_ms > 0 and but_p == 1) return;
     }  //for 1... 180
     for(grad=89;grad>=0;grad-=step_s) {  // second 90...0°
 //       if((grad + 90) <= stap or (grad + 90) >= endp)   wert = 0.0; 
       wert = (float)sin_t[grad];
       newDacVal(wert);
       if (s_ms > 0 and but_p == 1) return;
     }  //for 1... 180
     for(grad=0;grad<=89;grad+=step_s) { // third 0...90° ##### look 2000-sin_t[grad] ##############
  //     if((grad + 180) <= stap or (grad + 180) >= endp) wert = 0.0; 
       wert = (float)(2000-sin_t[grad]);
       newDacVal(wert);
       if (s_ms > 0 and but_p == 1) return;
     }  //for
     for(grad=89;grad>=0;grad-=step_s) { // fourth 0...90° ##### look 2000-sin_t[grad] ##############
 //      if((grad + 270) <= stap or (grad + 270) >= endp) wert = 0.0; 
       wert = (float)(2000-sin_t[grad]);
       newDacVal(wert);
       if (s_ms > 0 and but_p == 1) return;
     }  //for
     if (s_sweep == 1) {    // delay_s = 0;
       EncPos = myEnc.read();           //###################################
       if (oldEncPos != EncPos) { 
         if (oldEncPos > EncPos) delay_s = delay_s - 5;
         else                    delay_s = delay_s + 5;
         if (delay_s <= 0)       delay_s = 0;
         if (delay_s >= 5000)    delay_s = 5000; 
         oldEncPos = EncPos;
       }  
    }  
  }  // while ... 
   
}


void runTriangle()  {  // Triangle
   float grad_tri = 0.01111111;  // 2  / 180
   int16_t imx = 180;
   stap = 0;
   endp=500;
   if ( ist > 0) { 
    grad_tri = 2.0 / (float) ist;
    imx = ist;
   }
   oldEncPos = EncPos;
  
   but_p = 0;
   while(but_p == 0) {
     for(grad=0;grad<imx;grad++) { //
    //   if(grad  <= stap or grad >= endp) wert = 0.0; 
       wert = grad_tri*(float)grad; // Triangle Wertebereich 0 ... +2
       newDacVal(wert);
       if (but_p == 1) return;
     }  //for
     for(grad=0;grad<imx;grad++) { //
       // if (grad+180) <= stap or (grad+180) >= endp) wert = 0.0; 
       wert = 2.00 - grad_tri*(float)grad; // Triangle Wertebereich +2 ... 0
       newDacVal(wert);
       if (but_p == 1) return;
     }  //for
     if (s_sweep == 1) {    // delay_s = 0;
       EncPos = myEnc.read();           //###################################
       if (oldEncPos != EncPos) { 
         if (oldEncPos > EncPos) delay_s = delay_s - 5;
         else                    delay_s = delay_s + 5;
         if (delay_s <= 0)       delay_s = 0;
         if (delay_s >= 5000)    delay_s = 5000; 
         oldEncPos = EncPos;
       }
     }       
   }  // while
}


void runStepUp()  {   // StepUp
   float grad_upd = 0.00555556;  // 2  / 360
   int16_t imx = 360;
   if ( ist > 0) { 
     grad_upd = 2.0 / (float) ist;
     imx = ist;
   }
   oldEncPos = EncPos;
   but_p = 0;
   while(but_p == 0)  {
     for(grad=0;grad<imx;grad++) { //
       wert = grad_upd*(float)grad; // Triangle Wertebereich 0 ... +2
       newDacVal(wert);
       if (but_p == 1) return;   
     }  //for
     if (s_sweep == 1) {    // delay_s = 0;
       EncPos = myEnc.read();           //###################################
       if (oldEncPos != EncPos) { 
         if (oldEncPos > EncPos) delay_s = delay_s - 5;
         else                    delay_s = delay_s + 5;
         if (delay_s <= 0)       delay_s = 0;
         if (delay_s >= 5000)    delay_s = 5000; 
         oldEncPos = EncPos;
       }   
     }  
   } // while
 
}


void runStepDown()  {   // StepDown
   float grad_upd = 0.00555556;  // 2  / 360
   int16_t imx = 360;
   if ( ist > 0) { 
     grad_upd = 2.0 / (float) ist;
     imx = ist;
   }
   oldEncPos = EncPos;
   but_p = 0;
   while(but_p == 0)  {
     for(grad=0;grad<imx;grad++) { //
       wert = 2.00 - grad_upd*(float)grad; // Rampe Wertebereich 0 ... +2
       newDacVal(wert);
       if (but_p == 1) return;      
     }  //for
     if (s_sweep == 1) {    // delay_s = 0;
       EncPos = myEnc.read();           //###################################
       if (oldEncPos != EncPos) { 
         if (oldEncPos > EncPos) delay_s = delay_s - 5;
         else                    delay_s = delay_s + 5;
         if (delay_s <= 0)       delay_s = 0;
         if (delay_s >= 5000)    delay_s = 5000; 
         oldEncPos = EncPos;
       }  
     }  
   } // while
   
}


void runSquareWave()  {  // SqareWave
   uint8_t ihl;  
   but_p = 0;      
   ihl = 0;    
   while(but_p == 0)  {
     if (ihl == 0) {
        wert = 0.00; // SquareWavee Wertebereich 0 or +2
        ihl = 1;
     }   
     else {
       wert = 2.00; // SquareWavee Wertebereich +2 or 0
       ihl = 0;
     }
     newDacVal(wert);
     if (s_sweep == 1) {    // delay_s = 0;
       EncPos = myEnc.read();           //###################################
       if (oldEncPos != EncPos) { 
         if (oldEncPos > EncPos) delay_s = delay_s - 5;
         else                    delay_s = delay_s + 5;
         if (delay_s <= 0)       delay_s = 0;
         if (delay_s >= 5000)    delay_s = 5000; 
         oldEncPos = EncPos;
       }   
     }  
   } // while (but_p ...
  
}



void  ConstCurGraph() {
     char *str_fu = "Triangle";          
  sel_f:

    lcd.setCursor(0,0);     lcd.print("select Sig.Type:");
    fillBlank(0,1,16);   
     
    ifu = 4;
    but_p = 0;
    while (but_p == 0) { 
      ifu++;
      if  (ifu > 4) ifu = 0;
      lcd.setCursor(4, 1);
      if      (ifu == 0)  str_fu = "  Sine  ";
      else if (ifu == 1)  str_fu = "Triangle";
      else if (ifu == 2)  str_fu = " StepUp ";
      else if (ifu == 3)  str_fu = "StepDown";
      else if (ifu == 4)  str_fu = " Square ";
      lcd.print(str_fu);
      delay(1500);  
    }  // while (but_p == 
  upd_f:
                        // lcd.print("0123456789012345");
 //   lcd.setCursor(0, 0);   lcd.print("Gain Offs. Delay");
    lcd.setCursor(0, 0);   lcd.print("Gn. Off Delay St");
 // lcd.setCursor(0, 0);   lcd.print("0,6-0.4 120  120");
    fillBlank(0,1,16); //lcd.setCursor(0, 1);   lcd.print("                "); 
    
    setGainy();      // ## 0.1,...2.0           ########
    setOffset();     // ## 0mv, 100mV, ... 2.0V ########
    setDelay();      // ## 0 µs ...120 s        ######## 
    if (ifu < 4)  {  // ## not for squarewave   ########  
      setStep();     // ## 0 ... 500 , sine: 1,2,3,5,6,9,15,30 
    //  setStart();   // test
    //  setEnd();     // test
      
    }
    
    lcd.setCursor(0,0); lcd.print(str_fu);lcd.print(" started");
    if      (ifu == 0)   runSine();       // Sine
    else if (ifu == 1)   runTriangle();   // Triangle
    else if (ifu == 2)   runStepUp();     // StepUp
    else if (ifu == 3)   runStepDown();   // StepDown       
    else if (ifu == 4)   runSquareWave(); // SqareWave   
    delay(100);
    fillBlank(0,0,16);  
    lcd.setCursor(0,1); lcd.print(str_fu);lcd.print(" stopped");
    but_p = 0;     
    oldEncPos = EncPos;
    s_first = 0;
    while (1==1) {
      EncPos = myEnc.read();           //###################################
      if (oldEncPos != EncPos) { 
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if   (oldEncPos < EncPos) {   // 'r'
            fillBlank(0,1,16); 
            return;                    //#### leave Const current Grah
          }   
          else goto upd_f;             //##############################
        }   
        oldEncPos = EncPos;
      }
      delay(100);
      
     
    } 
  
}

void changeDigit() {
   uint8_t dig_x;
   int32_t cnt_r;
   dig_x = 15-pos_i;  // 15 - 15,14,13,12 : 0...3
   dig_o = digi_t[dig_x];
   dig_n = dig_o;
 
   s_upd = 0;
    
   lcd.blink_off(); //bl??
   fillBlank(0,0,9); //lcd.setCursor(0,0);lcd.print("         ");
 //  lcd.setCursor(6,0);lcd.print("ChaDi");

   dig_o = digi_t[dig_x];
   oldEncPos = dig_o;   //######################
   EncPos    = dig_o;   //######################
//   Serial.print("dig_x: ");Serial.print(dig_x);Serial.print(" EncPos: ");Serial.print(EncPos);Serial.print(" enc_dir: ");Serial.println(enc_dir);
   enc_dir == 'm';
   but_p = 0;
   s_first = 0;               // encoder with 2 switch / teeth
   while(but_p == 0) {                // while button-switch not pressed
     EncPos = myEnc.read();           //###################################
     if (oldEncPos != EncPos) { 
       if (s_first == 0) s_first = 1; // first  switch / teeth  :: do nothing
       else {                         // second switch / teeth
         s_first = 0;   
         if (oldEncPos < EncPos) enc_dir = 'r';
         else                    enc_dir = 'l';
         if (enc_dir == 'l') dig_n++;
         if (enc_dir == 'r') dig_n--;
         if (dig_n > 9) dig_n = 0;
         if (dig_n < 0) dig_n = 9;   //## ?? ####################################
         if (dig_n != dig_o) {
           digi_t[dig_x] = dig_n;
           s_upd = 1;
         }  
  //     Serial.print(" EncPos: ");Serial.print(EncPos);Serial.print(" enc_dir: ");Serial.print(enc_dir);Serial.print(" dig_x: ");Serial.print(dig_x);Serial.print(" dig_n: ");Serial.println(dig_n);
     
         lcd.setCursor(pos_i,1);  lcd.print(dig_n);
   
       } 
       oldEncPos = EncPos;
       
     } 
    
     lcd.setCursor(pos_i,1); 
     //lcd.blink();   //bli?#################
     lcd.blink_off();
   
   }  // while but_p == 0) 
   but_p = 0;
   delay(20);
   
   zconst =  1000*digi_t[3] + 100* digi_t[2] + 10 * digi_t[1] + digi_t[0];

   lcd.setCursor(12,1);
   printIstep(zconst);
     
   delay(20);
         
   if(s_upd == 1) {      //
     
     dig_n = digi_t[15-pos_i];
     lcd.setCursor(pos_i,1); lcd.print(dig_n);
     if ((s_par == 0 and zconst >  4095) or
         (s_par == 1 and zconst > 40950))  {
       digi_t[dig_x] = dig_o; // repair to old value
       lcd.setCursor(pos_i,1); lcd.print(dig_o);
       s_upd = 0;
     }
     delay(60);
    
     iconst = zconst;   
    
    
   }               // if (s_upd == 1 
  
   istep = zconst;
   
   lcd.setCursor(12,1); 
   printIstep(istep);
 //  Serial.print("istep ");Serial.println(istep);
   
}



void inc_dec_Iconst() {
    lcd.setCursor(12,1);
    printIstep(istep);  // 
    
    long zconst_o,cnt_r;
    s_first = 0;
  //  Serial.print("523 ic_dec istep: ");Serial.println(istep);
  
    if (s_low != 3) zconst = iconst;
    else            zconst = fox;
    oldEncPos = 999;   //###################### 

    but_p = 0; 
    zconst_o = 0;  //??
    zconst   = zconst_o;
    while (but_p == 0) {                    // button-switch not pressed  
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1; // first  switch / teeth  : do nothing
        else {                         // second switch / teeth
          s_first = 0;
          if (oldEncPos < EncPos) cnt_r = -1; 
          else                    cnt_r =  1;
         
        
          if (s_low != 3)   zconst = zconst + cnt_r * istep;
          else {
            if (s_kHz == 0) zconst = zconst + cnt_r * istep;
            else            zconst = zconst + cnt_r * istep * 1000;
          }
          if (zconst < 0) zconst = 0;
          if (s_low != 3) {
      //  Serial.print("541 ic_dec istep: ");Serial.print(istep);Serial.print(" zconst: ");Serial.println(zconst);
            if (( s_par == 0 and zconst >  4095) or
                ( s_par == 1 and zconst > 40950))    { 
                           
              if (s_par == 0) zconst = zconst_o;
              if (s_par == 1) zconst = zconst_o;
            }
        
            else {   // z_const in range
             
              iconst = zconst;
   //          Serial.print("1751 in-range  istep: ");Serial.print(istep);Serial.print("z_const: ");Serial.println(zconst);
              s_upd = 1;
          
              displayIconst();            //#########################################
              if (s_par == 0) dac.setValue_A(iconst);   
              else            dac.setValue_A(iconst/10);
        
              s_upd = 0;
            }   
          }  
          else { // s_low == 3) 
             //  Serial.print("541 ic_dec istep: ");Serial.print(istep);Serial.print(" zconst: ");Serial.println(zconst);
            if (zconst > 12500000)  {  // max. 12.5 MHz
              zconst = zconst_o;
            }
            else {
              iconst = zconst;
              s_upd = 1;
           //   Serial.println(iconst);
              displayIconst();            //#########################################
            }
          } 
        } 
        zconst_o = zconst; // ??
        oldEncPos = EncPos; 
      }  
    }  // while but_p == 0
    but_p = 0;
}



void set_Iconst() {
  uint8_t c_step=0,s_ret=0; 
  int8_t pos_f = 11;
  if (s_low != 3) {  // normal mA...
    set_Min_Step_Iconst(); // set min. current-Step to 1 µA or 10 µA  
  
    fillBlank(0,0,16); //lcd.setCursor(0,0); lcd.print("                      "); 
    lcd.setCursor(0,1);  
     
    lcd.print("Ic\133\344A\135\Step ");   // HD47880 Sonderzeichen: \133: [ ,\135: ] ,\344: µ,\364: Omega (für z.B. Ohm 
  }
  else {  // == 3: contin
    fillBlank(0,1,10);
    set_Step_Hz_kHz();  //# 3 continous frequency-change  ad9 ##############
    lcd.setCursor(1,15);
  }
  oldEncPos = 999;
  EncPos    = 0;
  s_first   = 0;
  pos_i = 15;
  
   
 // if (s_low != 3) fillBlank(10,0,6);
  lcd.setCursor(pos_i,0); lcd.write(1);//print("!");  //#################################
    
  while(1==1)  {
    s_upd = 0;
   
    but_p = 0;  // 0=not pressed
    while (but_p == 0)  {    // button-switch not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1; // first  switch / teeth  :: do nothing
        else {                         // second switch / teeth
          s_first = 0;
          if (oldEncPos < EncPos) enc_dir = 'r';
          else                    enc_dir = 'l';
   
          s_upd = 1;
        
          pos_i = pos_o;
          if (enc_dir == 'l' and pos_o == 12) {
            c_step++;
            if (c_step > 2) {
              c_step=0; 
            }
            else {
              pos_i++;
              delay(200);
            }
            
          }
          if (enc_dir == 'r') pos_i++;
          if (enc_dir == 'l') pos_i--;
          if (pos_i > 15)   {
             s_ret++;                                            // ##### for return next right rotations
             if (s_ret > 6) {
               fillBlank(0,0,16); //cd.setCursor(0,0); lcd.print("                ");
               fillBlank(0,1,16); //lcd.setCursor(0,1); lcd.print("                ");
               return;                                           // ######### return ########################
             }
             pos_i = 15;
          }
          else {
           
            s_ret = 0;
          }
          if (pos_i < pos_f)  pos_i = pos_f;
      //   Serial.print(" o: ");Serial.print(pos_o);Serial.print(" i: ");Serial.println(pos_i);
          lcd.setCursor(pos_o,0); lcd.print(" ");
          if ( pos_i > 11) {    // 12,13,14,15  current-values
            lcd.setCursor(pos_i,0); lcd.write(1);//print("!");  //########################
          }      
          else  {                         // 11 start step-width incr./decr. with encoder or return
           
            lcd.setCursor(pos_i,0); lcd.write(3);//print("<");  //########################  <<<<<  ######################
            s_stepa = 1;
            pos_i = pos_f + 1;
            but_p = 1;                                          // ######## to leave while ...
            lcd.setCursor(0,0); lcd.print(" ");
              
          }
        }  
        pos_o = pos_i;
        oldEncPos = EncPos;
        // delay(50);
        
      }  
       
      lcd.setCursor(pos_i,1);  lcd.blink();   //#################
      if (s_stepa == 1) but_p = 1;    // ###leave while ( ... 
    }     // while (but_p == 0 
    but_p = 0;
    if (s_stepa == 1) {
   
      lcd.noBlink(); //#################################################
      inc_dec_Iconst();  //########################################################
      if(s_low != 3) lcd.setCursor(11,0); lcd.print(" ");
      pos_i = 15;                                         //############# 
      lcd.setCursor(pos_i,0); lcd.write(1);//print("!");  //####################### 1
      lcd.setCursor(12,1); lcd.print("    ");
      lcd.setCursor(12,1);
      
      uint8_t s_blank = 0;   // blanks eliminieren
      for ( uint8_t ilx=0;ilx<=3;ilx++) {  
        
      //  Serial.print("ilx ");Serial.print(ilx);Serial.print(" t[ilx] ");Serial.println(digi_t[3-ilx]); 
        lcd.setCursor(12+ilx,1);   // 12,13,14,15
        if (digi_t[3-ilx] > 0) {
          lcd.print(digi_t[3-ilx]); 
          s_blank = 1;                                   
        }
        else 
        if (s_blank == 1)   lcd.print("0"); //lcd.print(digi_t[3-ilx]); 
        else                lcd.print(" ");
           
      }
      lcd.setCursor(15,1);
      s_stepa = 0;
      pos_i = 15; 
      pos_o = 15;
  
    }
              
    changeDigit();            // ######################################
    //            changeDigit_AD();         // AD??#####################################
    if (s_upd == 1) {
     
      if      (EncPos <  0) EncPos = 0;  // ??
       
      displayIconst(); //###########################################  
      if (s_low != 3) {
        if (s_par == 0) dac.setValue_A(iconst);  
        else            dac.setValue_A(iconst/10);  // 100 Ohm
     }  
    // else fox = zconst;  //AD??
      s_upd = 0;
    }
  }
 delay(50);
   
}


void set_Step_Hz_kHz() { // 1 Hz ... 9999 Hz or 1 kHz ... 9999 kHz stepwidth
    
    int zEncPos;
    s_kHz   = 0;
    s_first = 0; 
    lcd.setCursor(0,1);lcd.print("f-Step");
    lcd.setCursor(6,1);
    if (s_kHz == 0) lcd.print("\133Hz\135? ");  // HD47880 special-sign:  \133: "["   \135 "]"
    else            lcd.print("\133kHz\135? "); // HD47880 special-sign:  \133: "["   \135 "]"
     
    fstart  = fox;
    but_p = 0;  // 0=not pressed
    oldEncPos = 999;
    while (but_p == 0)  {        // button-switch not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1; // first  switch / teeth
        else {
          s_first = 0;                 // second switch / teeth
          lcd.setCursor(6,1); 
          if (s_kHz == 1) {
            lcd.print("\133Hz\135: ");  // HD47880 special-sign:  \133: "["   \135 "]"  // Hz
            s_kHz = 0;
          
          }
          else            {
            lcd.print("\133kHz\135: "); // HD47880 special-sign:  \133: "["   \135 "]"  kHz
            s_kHz = 1;
           
          }
         
        } 
        oldEncPos = EncPos;
      }   
    }   // while (but_p   
    lcd.setCursor(15,1);
}

void set_Min_Step_Iconst() { // 1 µA or 10 µA step: max 4.095 mA or 40 mA
  
    uint8_t s_paro = s_par;  // old state of SWITCH_R
    int zEncPos;
    s_first = 0;
    but_p = 0;  // 0=not pressed
    oldEncPos = 999;
    while (but_p == 0)  {        // button-switch not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {
        if (s_first == 0) s_first = 1; // first  switch / teeth
        else {
          s_first = 0;                 // second switch / teeth
          if (s_par == 1) {
            lcd.setCursor(0,0);lcd.print(" 1 \344A ...  4 mA ");  // \344 = µ
            lcd.setCursor(0,1);lcd.print("   Rm = 1000 Ohm");
            s_par = 0;
          }
          else            {
            lcd.setCursor(0,0);lcd.print("10 \344A ... 40 mA ");  // \344 = µ
            lcd.setCursor(0,1);lcd.print("   Rm =  100 Ohm");
            s_par = 1;
          }
        } 
        oldEncPos = EncPos;
      }   
    }   // while (bat_p  
    fillBlank(11,1,5); //lcd.setCursor(11,1);lcd.print("     ");
    if (s_par == s_paro) return;
    if (s_par == 0)  switchOff();
    if (s_par == 1)  switchOn();
             
}

void switchOn( ) {
  digitalWrite(SWITCH_R, HIGH);  // set RM to  100 Ohm
  s_par = 1;
}

void switchOff( ) {
  digitalWrite(SWITCH_R, LOW);   // set RM to 1000 Ohm
  s_par = 0;
}



void displayIconst()  {
     int8_t ilc,ilq;
     long zf;

     fillBlank(5,0,4); //lcd.setCursor(5, 0);           lcd.print("    ");  //??
     lcd.setCursor(0, 1); 
     if (s_low == 3) {                            // AD9833-frequency  contin. 
      //  Serial.print("_flowo ");Serial.print(f_lowo);Serial.print(" fox ");Serial.print(fox);Serial.print(" iconst ");Serial.print(iconst);Serial.print(" istep ");Serial.println(istep);                              // 
       if (s_stepa == 0) return; 
       fox = f_lowo + iconst;
       if (fox < 0) fox = 0;
   
   //    Serial.print("fox ");Serial.print(fox);Serial.print(" iconst ");Serial.print(iconst);Serial.print(" istep ");Serial.println(istep);
       if (s_sine == 0) ad9.ApplySignal(zwave_t,REG0,fox); //  DisplayIconst ##############################################################
       else sendFrequency(fox);  // AD9850
     
       lcd.setCursor(2,0);       // fill upper Lin
       printFreq(fox);           // tab freqSGLo
   
       fox = f_lowo;
     
       return;
     }
     else {                                    // iconst s_low = 0/1/2
       lcd.print("Ic\133\344A\135\ ");         // HD47880 Sonderzeichen: \133: [ ,\135: ] ,\344: µ,\364: Omega (für z.B. Ohm  
       lcd.print("Step:");   
     
       if (iconst <= 1000) { // µA
 //        Serial.print(" 2040 iconst ");Serial.print(iconst);Serial.print(" digi ");Serial.print(digi);Serial.print(" istep ");Serial.println(istep);
         lcd.setCursor(0,1);lcd.print("Ic\133\344A\135\ ");         // HD47880 Sonderzeichen: \133: [ ,\135: ] ,\344: µ,\364: Omega (für z.B. Ohm  
         fillBlank(0,0,5); //lcd.setCursor(0,0);      lcd.print("     ");
         lcd.setCursor(5,0); // 5-digi,0)        lcd.print(iconst);
         printIconst(iconst);                    // ######################## new
       }
       else {               // mA
 //         Serial.print("2047 4 ");Serial.print(dig4_0_t[4]); Serial.print(" 3 ");Serial.print(dig4_0_t[3]); Serial.print(" 2 ");Serial.print(dig4_0_t[2]);
 //        Serial.print(" 1 ");Serial.print(dig4_0_t[1]);Serial.print(" 0 ");Serial.println(dig4_0_t[0]);    
         lcd.setCursor(0,1);lcd.print("Ic\133mA\135\ ");            // HD47880 Sonderzeichen: \133: [ ,\135: ] ,\344: µ,\364: Omega (für z.B. Ohm  
         fillBlank(0,0,10); //0,0,5
         lcd.setCursor(0,0);
         printIconst(iconst);                    // ######### here not for print only first 2 bytea, beacause of . 
             
         lcd.setCursor(2,0); lcd.print(".");
                
         lcd.setCursor(3,0); lcd.print(dig4_0_t[2]);
         lcd.setCursor(4,0); lcd.print(dig4_0_t[1]);
         lcd.setCursor(5,0); lcd.print(dig4_0_t[0]);
     
       }
     } 
     delay(60);
  
}
  

void changeFreqLoContin()  {
  s_low = 3;    
  changeFreqLoHi();    //# here: show lofrequency on display######################
  set_Iconst(); 
  
  delay(40);
}

void loop(){
    int8_t s_first = 0; 
    s_low   = 0;
    but_p   = 0;
    sel_c   = 0;    // ??
    EncPos    = 999;
    oldEncPos = 999;
    lcd.setCursor(0,0); lcd.print("Select Function:");
    while (but_p == 0)  {        // button-switch not pressed
      EncPos = myEnc.read();     //###################################
      if (oldEncPos != EncPos) {  
        if (s_first == 0) s_first = 1;
        else {
          s_first = 0;
          if (EncPos > oldEncPos) ila++;
          else                    ila--;
          if (ila > 3) ila = 0;   
          if (ila < 0) ila = 3;
          lcd.setCursor(0,1); //lcd.print(ila);
          if      (ila == 0) lcd.print("AD9833-Func-Gen.");
          else if (ila == 1) lcd.print("Const.Cur.Source");
          else if (ila == 2) lcd.print("LowFreq-Func-Gen");
          else if (ila == 3) lcd.print("AD9850-Sine-Gen.");
          delay(100);
       } // if
       oldEncPos = EncPos;
     } // if  
   } // while     
   but_p = 0;  
   if      (ila == 0) {  // AD9833-Func-Gen.
     fillBlank(0,1,16);
     lcd.setCursor(0,0);lcd.print("AD9833-Func-Gen."); 
     delay(100);

     SPI.beginTransaction (SPISettings (12500000, MSBFIRST, SPI_MODE0));  // for AD9833 max 12.5 MHz, MCP4822: 20 MHz
     SPI.begin();
     
     delay(200);  
     ad9.Begin();
     ad9.ApplySignal(SINE_WAVE,REG0,1111);
     ad9.EnableOutput(true);   // Turn ON the output - it defaults to OFF
    
     controlAD9833();  //########################################
     digitalWrite(SG_FSYNC,HIGH);    // disable AD9833  ?? dual-generator
   }
   else if (ila == 1) {  // Const.Cur.Source
     fillBlank(0,1,16);
     lcd.setCursor(0,0);lcd.print("Const.Cur.Source"); 
     delay(1000);
     startDAC();
     istep = 0;
     set_Iconst();
     dac.off_A();
     digitalWrite(DAC_CS,HIGH);  
   }  
   else if (ila == 2) {  // Const.Cur.Graph
     fillBlank(0,1,16);
     lcd.setCursor(0,0);lcd.print("LowFreq-Func-Gen"); 
     delay(1000);
     startDAC(); 
     ConstCurGraph();
     dac.off_A();
     digitalWrite(DAC_CS,HIGH);
   }
   else if (ila == 3) {  // AD9850-Sine-Gen.
     s_sine = 1;
     fillBlank(0,1,16);
     lcd.setCursor(0,0);lcd.print("AD9850-Sine-Gen."); 
     delay(100);
      // Initialise the AD9850 module. 
     pulseHigh(RESET);
     pulseHigh(W_CLK);
     pulseHigh(FQ_UD);    // this pulse enables serial mode - Datasheet page 12 figure 10  
       
     controlAD9833();     // modified for AD9850  ########
  
   }
   but_p = 0;  
   delay(100);
}
