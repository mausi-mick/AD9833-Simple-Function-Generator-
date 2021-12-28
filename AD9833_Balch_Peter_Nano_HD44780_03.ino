//--------------------------------------------------------------------------------------- 
//  Thanks:
//
// Peter Balch  :  https://www.instructables.com/Signal-Generator-AD9833/               : AD9833 Source, flow-control, sweep, ...
// fhdm-devels  :  https://www.instructables.com/Simple-AD9833-Based-Signal-Generator/  : PinChangeInterrupt
// GreatScottLab:  https://www.instructables.com/DIY-FunctionWaveform-Generator/        : HD44780 display
//                 https://omerk.github.io/lcdchargen/                                  : Pixel-Generator HD44780 LCD Modules
//---------------------------------------------------------------------------------------
// subject to the GNU General Public License
//---------------------------------------------------------------------------------------
// Developped with Arduino IDE 1.8.4
//---------------------------------------------------------------------------------------
// AD9833_Balch_Peter_Nano_HD44780_01 2021_12_20 
// AD9833_Balch_Peter_Nano_HD44780_02 2021_12_23 Menu changed : 'L' change f-Low, 'H' change f-High 
// AD9833_Balch_Peter_Nano_HD44780_03 2021_12_27 Menu changed : '0-'9' commands deactivated,quicker update AD9833

#include <Wire.h>              // I2C - Lib
#include <LiquidCrystal_I2C.h> // HD44780 LCD1602 

 LiquidCrystal_I2C lcd(0x27, 16, 2);   // standard-I2C address PCF8574
 //LiquidCrystal_I2C lcd(0x3F, 16, 2); // alternative  adress by some boards

// SDA ------------> A4  // I2C PCF8574 HD44780
// SCL ------------> A5  // I2C PCF8574 HD44780

//------------------------  Encoder/Switches with Pin-Change-Interrupt: 
const int ENC_CW   =  2; // PCINT18 = Pin D2
const int ENC_CCW  =  3; // PCINT19 = Pin D3
const int ENC_BUT  =  4; // PCINT20 = Pin D4
//------------------------  AD9833 SPI-Pins
const int SG_FSYNC = 10; // AD9833 SPI-Cs
const int SG_DATA  = 11; // AD9833 SPI-Mosi
const int SG_CLK   = 13; // AD9833 SPI-Clk



//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

const long BAUDRATE = 115200;  // Baud rate of UART in bps
const byte numberOfDigits = 8;  // number of digits in the frequency

const int wSine     = 0b0000000000000000;
const int wTriangle = 0b0000000000000010;
const int wSquare   = 0b0000000000101000;

const uint16_t sweep_t[] = {1000,2000,5000,10000,20000,50000};            // sweep-pos 
const uint16_t delay_t[] = {100,200,500,1,2,5,10,20,50,100,200,500,1000}; // delay

const char com_t[] = {'S','T','Q','L','H','F','G','M','D','W','R',' '};   // Menu-Commnds:

// 0  'S': start Sine 
// 1  'T': start Triangle 
// 2  'Q': start Square
// 3  'L': set Freq.-Low 
// 4  'H': set Freq.-High
// 5  'F': set Freq.-Low and Freq-High 
// 6  'G': start Sweep
// 7  'M': swap Freq. Low  <--> High
// 8  'D': set / change Delay sweep
// 9  'W': set / change Nr./Pos. sweep
//10  'R': Reset AD9833

const char *str_com[]= {"Sine            ","Trian.          ","Square          ",
                        "set Low freq.   ","set High freq.  ","set Lo/Hi freq. ",
                        "Sweep           ","swap Lo/Hi freq.","set Delay       ",
                        "set Pos-Nr      ","Reset AD9833     "}; 

// HD44780 special-signs
const byte C_C_1_Arrow_D[8] = {  // from: https://omerk.github.io/lcdchargen/
  0b01110, 0b01010, 0b01010, 0b01010, 0b01010, 0b11011, 0b01110, 0b00100};

const byte C_C_2_Arrow_U[8] = {
  0b00100, 0b01010, 0b11011, 0b01010, 0b01010, 0b01010, 0b01010, 0b01110};

const byte C_C_3_Arrow_L[8] = {
  0b00010, 0b00100, 0b01111, 0b11000, 0b01111, 0b00100, 0b00010, 0b00000};

const byte C_C_4_Arrow_R[8] = {
  0b01000, 0b00100, 0b11110, 0b00001, 0b11110, 0b00100, 0b01000, 0b00000};

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
//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------


uint8_t freqSGLo[numberOfDigits] = {0,0,1,0,0,0,0,0}; //   100 Hz 
uint8_t freqSGHi[numberOfDigits] = {0,0,0,0,2,0,0,0}; // 20000 Hz 
//######## potencies basis 10:      0,1,2,3,4,5,6,7   #####

uint8_t freq_zw = 0;
uint8_t s_low=0,sel_o=0;
int waveType = wSine;
long fox = 100;
char com_o = 'S'; // old command

volatile int EncPos    = 0; //this variable stores current value of encoder position. 
volatile int oldEncPos = 0; //this variable stores the value of old encoder position.
volatile uint8_t but_p = 0;
volatile char enc_dir  = 'r';

volatile bool dirty = false;   // True if something was changed in an interrupt handler

int8_t  pos_i=5,pos_o=5,dig_o=0,dig_n=0,line_d=0,sel_c=0,hz_nr=0;

char c = ' ';

// Currently selected digit
// 0 is the leftmost, 6 the rightmost, -1 none selected

int digits[numberOfDigits] = {0, 0, 1, 0, 0, 0, 0, 0};
//int digits[8] = {0, 0, 0, 1, 0, 0, 0, 0};
int selected_digit = 3,i_del=3,i_sw=2;    // 4 ?
uint8_t posc_t[] = {7, 5};
int delay_micros = 1000;  // 1ms
int sweep_nr     = 5000;  

int SG_iSweep,SG_nSweep;



ISR (PCINT2_vect) {

  static uint16_t seqA, seqB;

  // If interrupt is triggered by the button
  if    (!digitalRead(ENC_BUT)) {    // Pin4
    but_p = 1;                       // pressed
    delay(60);                       // 20 not allowed in "normal" ISR
   
    return;
  }
 
  else
  {
    // interrupt generated by encoder dial

    // Read A and B signals
    boolean A_val = digitalRead(ENC_CW);      // Pin2
    boolean B_val = digitalRead(ENC_CCW);     // Pin3

    // Record the A and B signals in seperate sequences
    seqA <<= 1;
    seqA |= A_val;

    seqB <<= 1;
    seqB |= B_val;

    // Mask the MSB four bits
    seqA &= 0b00001111;
    seqB &= 0b00001111;

    // Compare the recorded sequence with the expected sequence
    int selected_digit_value = digits[selected_digit];
  //  lcd.setCursor(0,1);lcd.print(selected_digit_value);
    if (seqA == 0b00001001 && seqB == 0b00000011) {
      if (selected_digit_value == 0) {
        selected_digit_value = 9;
      }
      else {
        selected_digit_value--;
        EncPos--;
        enc_dir = 'l';
      }
      dirty = true;
    }
    else if (seqA == 0b00000011 && seqB == 0b00001001) {
      if (selected_digit_value == 9) {
        
        selected_digit_value = 0;
        
      }
      else {
        selected_digit_value++;
        EncPos++;
        enc_dir = 'r';
      }
      dirty = true;
    }
    digits[selected_digit] = selected_digit_value;
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
  digitalWrite(SG_CLK, HIGH);
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

//-----------------------------------------------------------------------------
// SG_StepSweep
//    increment the FG frequency 
//-----------------------------------------------------------------------------
void SG_StepSweep(void) {
  if (SG_iSweep > SG_nSweep) SG_iSweep = 0;
  long f = exp((log(calcFreq(freqSGHi)) - log(calcFreq(freqSGLo)))*SG_iSweep/SG_nSweep + log(calcFreq(freqSGLo))) +0.5;
  SG_freqSet(f, waveType);
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
  delay(100);
  but_p = 0;
  fmin = calcFreq(freqSGLo);
  fmax = calcFreq(freqSGHi);
  int i=0; 
  do {
    long f = exp((log(fmax) - log(fmin))*i/(n-1) + log(fmin)) +0.5;
    SG_freqSet(f, waveType);
    delayMicroseconds(delay_micros); //delay(1);
    i++;
    if (i >= n) i = 0;
  } while (but_p == 0) ;//!Serial.available());
  
  SG_freqSet(calcFreq(freqSGLo), waveType);
}

//-----------------------------------------------------------------------------
// SerialCommand
//   if a byte is available in teh serial input buffer
//   execute it as a command
//-----------------------------------------------------------------------------
void SerialCommand(void) {
  if ( Serial.available() > 0 ) {
     c = Serial.read();

    if ((c >= '0') && (c <= '9')) {
      for (int i=5; i>0; i--) freqSGLo[i] = freqSGLo[i-1];
      freqSGLo[0] = c - '0';
    } else {
      switch (c) {
        case 'S': waveType = wSine; SG_freqReset(calcFreq(freqSGLo), waveType); break;     // SigGen wave is sine
        case 'T': waveType = wTriangle; SG_freqReset(calcFreq(freqSGLo), waveType); break; // SigGen wave is triangle
        case 'Q': waveType = wSquare; SG_freqReset(calcFreq(freqSGLo), waveType); break;   // SigGen wave is square
        case 'R': SG_Reset();  break;   // SigGen reset
        case 'M': for (int i=0; i<=5; i++) freqSGHi[i] = freqSGLo[i]; break;   // move freq to high array
        case 'G': Sweep(1000);  break;  // sweep SigGen
        case 'H': Sweep(5000);  break;  // sweep SigGen
        case 'I': Sweep(20000);  break; // sweep SigGen

        default: return;
      }
    }
  }
}

//-----------------------------------------------------------------------------
// TabCommand
//   if a byte is available in teh serial input buffer
//   execute it as a command
//-----------------------------------------------------------------------------
void TabCommand(char com_x) {
  c = com_x;  // globale variable
  
  switch_s:
     
    switch (com_x) {
      case 'S':  // 0  run sine
        lcd.setCursor(0,1); lcd.print(str_com[0]);
        
        freq_Lo_to_Display(); // Hz, MHz;
        waveType = wSine; SG_freqReset(calcFreq(freqSGLo), waveType); 
        lcd.setCursor(0,1); lcd.print("Sine  started   ");
        delay(500);
        com_o = 'S';
        sel_o = 0;
       break;    
      case 'T':  // 1 run triangle
        lcd.setCursor(0,1); lcd.print(str_com[1]);
        freq_Lo_to_Display();// Hz, MHz 
        waveType = wTriangle; SG_freqReset(calcFreq(freqSGLo), waveType);
        lcd.setCursor(0,1); lcd.print("Triangle started");
        delay(500);
        com_o = 'S';
        sel_o = 1;
       break;    
      case 'Q':  // 2 run Square
        lcd.setCursor(0,1);lcd.print(str_com[2]);
        freq_Lo_to_Display();// Hz, MHz 
        waveType = wSquare; SG_freqReset(calcFreq(freqSGLo), waveType);
        lcd.setCursor(0,1); lcd.print("Sqare start     ");
        delay(500);
        com_o = 'S';
        sel_o = 2;
       break;    
      case 'L':  // 3 set Fequency Low
      //  lcd.setCursor(0,1);//lcd.print(str_com[3]); 
        lcd.setCursor(10,0);lcd.print(" Hz    ");
        s_low = 0;
        changeFreqLoHi();
        com_x = com_o;
        sel_c = sel_o;
        goto switch_s;                           //####################
       break;    
      case 'H':  // 4 set Fequency High
      //  lcd.setCursor(0,1);//lcd.print(str_com[3]); 
        lcd.setCursor(10,0);lcd.print(" Hz    ");
        s_low = 1;
        changeFreqLoHi();
        com_x = com_o;
        sel_c = sel_o;
        goto switch_s;                           //####################
       break;   
      case 'F':  // 5 set Fequency Low and High
      //  lcd.setCursor(0,1);//lcd.print(str_com[3]); 
        lcd.setCursor(10,0);lcd.print(" Hz    ");
        s_low = 2;
        changeFreqLoHi();
        com_x = com_o;
        sel_c = sel_o;
        goto switch_s;                           //####################
       break;      
      case 'G':  // 6 5weep-nr
        lcd.setCursor(0,1);lcd.print(str_com[5]); 
        lcd.setCursor(14,0);lcd.print("ok");
        Sweep(sweep_nr); 
        lcd.setCursor(14,0);lcd.print("  ");
        if (com_o == ' ' ) com_o = 'S';
        if (com_o == 'G' ) com_o = 'S';
        com_x = com_o;
        goto switch_s;                          //####################
       break;  
      case 'M':  // 7 swap frequencys high and low 
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
        freq_Lo_to_Display(); // frequency in Hz, MHz
        lcd.setCursor(14,0);lcd.print("ok");
        delay(500);
        lcd.setCursor(14,0);lcd.print("  ");
        if (com_o == ' ' ) com_o = 'S';
        com_x = com_o;
        goto switch_s;                          //####################
       break;    
      case 'D':  // 8 set delay 
        lcd.setCursor(0,1);lcd.print(str_com[7]); // lcd.print("delay(ms)"); //\344s/ms)"); // \344 = µ
        lcd.setCursor(10,1);lcd.print("                 ");
        changeDelay();
        lcd.setCursor(14,1);lcd.print("ok");
        delay(1000);
        lcd.setCursor(10,0);lcd.print("       ");
        lcd.setCursor(14,1);lcd.print("  ");
        if (com_o == ' ' ) com_o = 'S';
        com_x = com_o;
        goto switch_s;                          //####################
       break; 
      case 'W':  // 9 sweep-nr
        lcd.setCursor(0,1);lcd.print(str_com[8]); // lcd.print("sweep nr");
        changeSweepNr();
        lcd.setCursor(14,1);lcd.print("ok");
        delay(1000);
        lcd.setCursor(10,0);lcd.print("       ");
        lcd.setCursor(14,1);lcd.print("  ");
        if (com_o == ' ' ) com_o = 'S';
        com_x = com_o;
        goto switch_s;                          //####################
       break; 
      case 'R':  // 10 Reset AD9833
        lcd.setCursor(0,1);lcd.print(str_com[4]); // "Reset  ");
        lcd.setCursor(0,0);lcd.print("                ");
        SG_Reset(); 
       break;    // SigGen reset 
      default: return;
    }
  
}

//-----------------------------------------------------------------------------
// InitSigGen
//-----------------------------------------------------------------------------
void InitSigGen(void) {
  
  pinMode(SG_DATA,  OUTPUT);
  pinMode(SG_CLK,   OUTPUT);
  pinMode(SG_FSYNC, OUTPUT);
  digitalWrite(SG_FSYNC, HIGH);
  digitalWrite(SG_CLK, HIGH);
  SG_Reset();
  SG_freqReset(calcFreq(freqSGLo), waveType);
}


// Draw the freqency on the Screen with the selected digit

void draw_frequency() {

  lcd.setCursor(2,0); 
  for (uint8_t i = 0; i < numberOfDigits; i++) {
    lcd.print(digits[i]);
  }

}


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
  while(1==1)  {
     
    while (but_p == 0)  {    // button-switch not pressed
      if(oldEncPos != EncPos) {  // encoder rotated
                 
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
  }  // while
   
}

void changeSweepNr()  {
  int z_number = 0;
  oldEncPos = 999;
  EncPos = 0;
  but_p  = 0;
  lcd.setCursor(0,0); lcd.blink();  
  lcd.setCursor(0,0); lcd.print("sweep:        ");
  lcd.setCursor(7,0); lcd.print(sweep_t[i_sw]); 
  
  lcd.print(" nr");
  while(1==1)  {
     
    while (but_p == 0)  {    // button-switch not pressed
      if(oldEncPos != EncPos) {  // encoder rotated
                 
        if (enc_dir == 'r') if (i_sw <  5) i_sw++;
        if (enc_dir == 'l') if (i_sw >  0) i_sw--;
        
        lcd.setCursor(7,0);lcd.print("         ");
        lcd.setCursor(7,0);lcd.print(sweep_t[i_sw]); 
        lcd.print(" nr");
        z_number = sweep_t[i_sw];      
        oldEncPos = EncPos;  
        lcd.setCursor(7,0);
      }  
      
    }  // while
    sweep_nr = z_number;
    return;
  }  // while
   
}



void changeFreqLoHi()  {
  uint8_t il; 
 
  lcd.setCursor(0,0);lcd.print("                ");
  lcd.setCursor(10,1);lcd.print("      "); 
  lcd.setCursor(0,0); lcd.print("Lo");
  for (il=0;il<numberOfDigits;il++) {
    lcd.print(freqSGLo[numberOfDigits-1-il]);
  }  
  lcd.print(" Hz   ");
  lcd.setCursor(0,1); lcd.print("Hi");
  for (il=0;il<numberOfDigits;il++) {
    lcd.print(freqSGHi[numberOfDigits-1-il]);
  }  
  lcd.print(" Hz   ");
 
  line_d = 0;
  if (s_low == 0 or s_low == 2) scanPos(); // Low- Frequency #############################
  if (s_low == 0) return;
  line_d = 1;
  scanPos();                               // High-Frequency #############################  
  s_low = 0;  // ?? 
   
}



void scanPos()  {
  

  uint8_t ilp;
  long ild;
  oldEncPos = 999;
  EncPos = 0;
  
  pos_i = posc_t[line_d];
 
  lcd.setCursor( pos_i,line_d);  lcd.blink();  

  while(1==1)  {
     
    while (but_p == 0)  {    // button-switch not pressed
      if(oldEncPos != EncPos) {  // encoder rotated
                 
        if (enc_dir == 'r') {
          if (pos_i < 9) pos_i++;
          else {
            pos_i = 2;
            lcd.setCursor(2,line_d);
          }
         
        }
        if (enc_dir == 'l') pos_i--;
         
        if (pos_i <  2)     {
       
         // lcd.blink_off();
          if (pos_i == 1); 
          if (pos_i <= 0) {  // == 0 is ok
             lcd.setCursor(pos_i,line_d);  lcd.blink();   //#################
             while (but_p == 0) { } // button not pressed 
            
             goto pos_end;
          }
        }
   /*     
        if (line_d == 0) {         // #################### test_03
          fox = 0;
          ild = 0;
          for (ilp = 0;ilp<=7;ilp++) {
            if (ild == 0) ild = 1;
            else ild = ild*10;
            fox = fox + freqSGLo[ilp] * ild;
          }  
          SG_freqReset(calcFreq(freqSGLo), waveType);  // #################### test_03
        }  
        */
        oldEncPos = EncPos;
      }  
       
      lcd.setCursor(pos_i,line_d);  lcd.blink();   //#################
     
    }  
     // button-switch pressed
    delay(50);   
       
    if (pos_i >= 2)  changeDigit();          // ######################################
   
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
   }  
   else {
     lcd.setCursor(0,0); lcd.print("                ");
     lcd.setCursor(14,line_d);lcd.print("ok");
     lcd.setCursor(0,0); lcd.print("                ");
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

void changeDigit() {
  int8_t dig_x;
   dig_x = pos_i - 2;  // 0...7
   dig_x = 7 - dig_x;  // 7...0 
 
   if (dig_x > 0 and dig_x < 8) {  // 8//  9 ?
     if (line_d == 0) dig_o = freqSGLo[dig_x];  //digi_t[dig_x];
     else             dig_o = freqSGHi[dig_x];  //digi_t[dig_x];
     dig_n = dig_o;
   }
  
   but_p = 0;
     
   delay(60);
   if (line_d == 0) dig_o = freqSGLo[dig_x];  //dig_o = digi_t[dig_x];
   else             dig_o = freqSGHi[dig_x];  //digi_t[dig_x];
   oldEncPos = dig_o;   //######################
   EncPos    = dig_o;   //######################
    
   lcd.setCursor(pos_i,line_d);  lcd.blink();   //#################
   while (but_p == 0)  {         // button-switch not pressed ?
     if(oldEncPos != EncPos) {  // encoder rotated
       if (enc_dir == 'l') dig_n++;
       if (enc_dir == 'r') dig_n--;
       if (dig_n > 9) dig_n = 0;
       if (dig_n < 0) dig_n = 9;  
       if (pos_i == 2 ) {    // 10Mhz pos
         if (dig_n > 1 and freqSGLo[6] <= 2) dig_n = 1; // example 12.4 MHz 
         else dig_n = 0;                                // example: 9.3 Mhz 
       }
       
       if (line_d == 0) freqSGLo[dig_x] = dig_n;  // save in tab
       else             freqSGHi[dig_x] = dig_n;  // save in tab
       //  Serial.print(" dx: "); Serial.print(dig_x);Serial.print(" digit: "); Serial.println(digi_t[dig_x]);
       lcd.setCursor(pos_i,line_d);  lcd.print(dig_n);
       if (line_d == 0) {         
         fox = 0;
         long ilf = 0;
         for (uint8_t ilq = 0;ilq<=7;ilq++) {
            if (ilf == 0) ilf = 1;
            else ilf = ilf*10;
            fox = fox + freqSGLo[ilq] * ilf;
         }  
         SG_freqReset(fox/*calcFreq(freqSGLo)*/, waveType); 
       }  
       delay(50);  // 100
       lcd.blink_off();
        
       oldEncPos = EncPos;
     } 
     lcd.setCursor(pos_i,line_d);  lcd.blink();   //#################
     posc_t[line_d] = pos_i;
    
   }  // while but_p == 0
   lcd.blink_off();
   delay(200);   // 500
   but_p = 0;
  
}



//-----------------------------------------------------------------------------
// Main routines
// The setup function
//-----------------------------------------------------------------------------
void setup (void) {
//  Open serial port with a baud rate of BAUDRATE b/s
  Serial.begin(BAUDRATE);

//  Serial.println("SigGen " __DATE__); // compilation date
//  Serial.println("OK");

  
  pinMode(ENC_CW,  INPUT_PULLUP);
  pinMode(ENC_CCW, INPUT_PULLUP);
  pinMode(ENC_BUT, INPUT_PULLUP);
 
 // pinMode(LED_BUILTIN, OUTPUT);

  PCICR  = 0b00000100;   // PCIE2: Pin Change Interrupt Enable 2
  PCMSK2 = 0b00011100;   // Enable Pin Change Interrupt for D2 - D4

  lcd.begin(); // init()Im Setup wird der LCD gestartet 

  // HD44780 special signs
  //  lcd.createChar(1, C_C_1_Arrow_D);  //########### Down
  //  lcd.createChar(2, C_C_2_Arrow_U);  //########### Up
  lcd.createChar(3, C_C_3_Arrow_L);  //########### Left
  lcd.createChar(4, C_C_4_Arrow_R);  //########### Right
  lcd.createChar(5, C_C_5_Line_D);   //########### Line
  lcd.createChar(6, C_C_6_Sine_1);   //########### Sine Top
  lcd.createChar(7, C_C_7_Sine_2);   //########### Sine Down
  lcd.createChar(1, C_C_8_Sine_3);   //########### Sine Bottom
  lcd.createChar(2, C_C_9_Sine_4);   //########### Sine Up
  
  lcd.backlight(); //switch-on backlight,  lcd.noBacklight(); switch-off backlight 

  lcd.setCursor(0,0);lcd.print("AD9833 Func.Gen.");  
  lcd.setCursor(0,1);lcd.write(2);lcd.write(6);lcd.write(7);lcd.write(1);lcd.write(2);
  delay(2000);
  
  InitSigGen();
  sel_c = 0;  // 9
  oldEncPos = 999;
  lcd.setCursor(0,0);lcd.print("                ");  
}

//-----------------------------------------------------------------------------
// Main routines
// loop
//-----------------------------------------------------------------------------
void loop (void) {
//  lcd.setCursor(0,0);lcd.print("  ");
  lcd.setCursor(0,1);lcd.print("              ");

  but_p = 0;
  
  while(but_p == 0) {   // not pressed
 
    if(oldEncPos != EncPos) {  // encoder rotated
      
      if (enc_dir == 'r')  sel_c++;
      if (enc_dir == 'l')  sel_c--;
      if      (sel_c <  0) sel_c = 10;
      else if (sel_c > 10) sel_c =  0;
      
      lcd.setCursor(0,1);lcd.print("     ");
      lcd.setCursor(0,1);lcd.print(str_com[sel_c]); 
      
      if (sel_c <= 2)  {   // Sine, Triangle, Square
         freq_Lo_to_Display();  // Hz, MHz
      }
      else if (sel_c == 6) { // sweep
        lcd.setCursor(10,1);lcd.print("       ");
        lcd.setCursor(6,1);lcd.print("(");
        lcd.print(sweep_nr); 
        lcd.print(")");
      }
    
      else if (sel_c == 7) {
        lcd.setCursor(0,1);lcd.print("swap Low ");lcd.write(3);lcd.write(4);lcd.print(" High");   
      }
        
      oldEncPos = EncPos;
      delay(100);
    }  
  
  }
 // lcd.setCursor(10,1);lcd.print("run !!"); 
 
  TabCommand(com_t[sel_c]);
}





