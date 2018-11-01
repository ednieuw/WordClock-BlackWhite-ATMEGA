// =============================================================================================================================
/* 
This Arduino code controls the ATMEGA328 ot ARMEGA1284 chip on the PCB board that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05
- DCF77 module DCF-2
- FM Stereo Radio Module RDA5807M RRD-102V2.0  
- Red_MAX7219_8-Digit_LED_Display
- I2C LCD display
A 74HC595 ULN2803APG combination regulates the LEDs by shifting in bits into the 74HC595 LED are turn On or Off
A FT232RL 5.5V FTDI USB to TTL Serial Module can be attached to program te ATMEGA and read the serial port
The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module
The FM-module can be used to read the RDS-time from radio station or to play FM-radio station from within the clock

 Author .: Ed Nieuwenhuys
 Changes.: 0.27-->0.40 read clock every second instead of every hour"
 Changes.: 0.27-->0.40 Encoder turn writes time directly to clock"
 Changes.: 0.42 toggle HetWasIs"
 Changes.: 0.43 Added PWM on Pin5 to control dimming "
 Changes.: 0.48 Minor changes"
 Changes.: 0.49 rotary encoder improved"
 Changes.: 0.50 Stable and organised coding"
 Changes.: 0.51 (SQRT (long)analog read *  63.5) for less lineair dimming"
 Changes.: 0.52 Changed rotary pins"
 Changes.: 0.54 Coding voor Klok 3 en 5"
 Changes.: 0.60 Nieuwe display format. Hetiswas na 10 sec uit "
 Changes.: 0.61 Pinchanges"
 Changes.: 0.63 Ebben klok No13"
 Changes.: 0.64 Light reducer added"
 Changes.: 0.65 Programma aangepast voor standaard front en KY-040 rotary-besturing ingebouwd"
 Changes.: 0.66 Random tijd when no RTC signal"
 Changes.: 0.67 Source code cleaning"
 Changes.: 0.69 Bluetooth added"
 Changes.: 0.70 Temperature added"
 Changes.: 0.76 Rotary delay 200 ---> 0 ms"
 Changes.: 0.78 BT Lightintensity"
 Changes.: 0.79 FM-radio time and serial/bluetooth input commands added"
 Changes.: 0.81 DCF77 receiver included"
 Changes.: 0.84 #ifdef included statements in source file, optimisation of code"
 Changes.: 0.86 English display added"
 Changes.: 0.87 French display added ---> Failed text does not fit in 11 x 11 character matrix. 
 Changes.: 0.90 Dutch-English version"    myEnc.write(0); at end of rotary function
 Changes.: 0.91 Added support for ATMEGA 1284P
                https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json  add this in preferences
 Changes.: 0.92 Added in Arduino\libraries\Encoder\interrupt_pins.h to facilitate the ATMighty 1284
                // Arduino Mighty
                #elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
                #define CORE_NUM_INTERRUPT 3
                #define CORE_INT0_PIN   10
                #define CORE_INT1_PIN   11
                #define CORE_INT2_PIN   2
  Changes.: 0.93 Added LCD support
  Changes.: 0.94 Added toggle lights on EDSOFT at minute 0 for several seconds
  Changes.: 0.95 Changed because of compile errors #include Time.h"in #include TimeLib.h"
  Changes.: 0.96 Added Red_MAX7219_8-Digit_LED_Display  
  Changes.: 0.97 Added temperature sensor Dallas DS1820. rotary bounces at start of program
  Changes.: 0.98 Fixed Bluetooth not receiving commands bug
  Changes.: 0.99 Franse klok verbeterd. Nog niet perfect. 
                 sqrt( (float) 63.5 * constrain(LDR_read,1,1023))); ...LDR_read,10,1023  --> LDR_read,1,1023 
  Changes.: V100 Show DCF-reception in LEDs of the HET display. Menu entries stored with PROGMEM. Cleaned up:  time print functions. #defines HET IS etc
  Changes.: V101 PROGMEM for menu items to conserve global variable space
  Changes.: V102 PROGMEM Time texts to conserve global variabele space, Added in setup "... enabled"
  Changes.: V103 Purperhartklok, Changed menu entries 
  Changes.: V104 Menu optimised, BT en serial optimised
  Changes.: V105 LCD optimised, DCF signal 0-100
  Changes.: V106 Selftest in menu. Last stable version
 */
// ===============================================================================================================================

//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
//#define FMRADIOMOD       // in development time retrieval works. Needs automatic optimal sender search function
#define BLUETOOTHMOD
#define DCFMOD
#define ROTARYMOD
//#define LCDMOD
//#define MAX7219_8DIGIT   //Only with 1284
//#define DS1820           //Only with 1284
//--------------------------------------------
// ARDUINO Definition of installed language word clock
//--------------------------------------------
 #define NL
// #define UK
// #define DE
// #define FR        // in development
//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include <Wire.h>
#include <avr/pgmspace.h>
                    #ifdef LCDMOD
#include <LiquidCrystal_I2C.h>
                    #endif LCDMOD
#include <RTClib.h>
#include <EEPROM.h>
//                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>                 // for Bluetooth communication
//                     #endif BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>
#ifdef defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
  #define CORE_NUM_INTERRUPT 3
  #define CORE_INT0_PIN   10
  #define CORE_INT1_PIN   11
  #define CORE_INT2_PIN   2
 #endif
                     #endif ROTARYMOD
                     #ifdef DCFMOD
#include <DCF77.h>
#include <TimeLib.h>

                     #endif DCFMOD
                     #ifdef MAX7219_8DIGIT
#include <LedControl.h>
                     #endif MAX7219_8DIGIT
                     #ifdef DS1820
#include <DallasTemperature.h>
                     #endif DS1820
                     
#ifdef NL
const char TimeText[][9] PROGMEM = {"Het ","is ","was ","vijf ","tien ","kwart ","voor ","over ","precies ","half ","elf ","vijf ",
                     "twee ","een ","vier ","tien ","twaalf ","drie ","negen ","acht ","zes ","zeven ","uur ","Edsoft " };
#endif NL
#define HET     ShiftInTime(0 , 1,Toggle_HetWasIs,0);
#define IS      ShiftInTime(1 , 1,Toggle_HetWasIs,1);
#define WAS     ShiftInTime(2 , 1,Toggle_HetWasIs,2);
#define MVIJF   ShiftInTime(3 , 1,              1,3);
#define MTIEN   ShiftInTime(4 , 1,              1,4);
#define KWART   ShiftInTime(5 , 1,              1,5);
#define VOOR    ShiftInTime(6 , 1,              1,6);
#define OVER    ShiftInTime(7 , 1,              1,7);

#define PRECIES ShiftInTime(8 , 2,              1,0);
#define HALF    ShiftInTime(9 , 2,              1,1);
#define ELF     ShiftInTime(10, 2,              1,2);
#define VIJF    ShiftInTime(11, 2,              1,3);
#define TWEE    ShiftInTime(12, 2,              1,4);
#define EEN     ShiftInTime(13, 2,              1,5);
#define VIER    ShiftInTime(14, 2,              1,6);
#define TIEN    ShiftInTime(15, 2,              1,7);

#define TWAALF  ShiftInTime(16, 3,              1,0);
#define DRIE    ShiftInTime(17, 3,              1,1);
#define NEGEN   ShiftInTime(18, 3,              1,2);
#define ACHT    ShiftInTime(19, 3,              1,3);
#define ZES     ShiftInTime(20, 3,              1,4);
#define ZEVEN   ShiftInTime(21, 3,              1,5);
#define UUR     ShiftInTime(22, 3,              1,6);
#define EDSOFT  ShiftInTime(23, 3, ToggleEdsoft  ,7);

#ifdef UK
char TimeText[][10] = {"It ","is ","was ","exact ","half ","twenty ","five ","quarter ","ten ","past ","six ","two ",
                     "five ","twelve ","ten ","eleven ","four ","nine ","three ","eight ","seven ","one ","O'clock ","to " };
#define HET     ShiftInTime(0 , 1,Toggle_HetWasIs,0);
#define IS      ShiftInTime(1 , 1,Toggle_HetWasIs,0);
#define WAS     ShiftInTime(2 , 1,Toggle_HetWasIs,0);
#define PRECIES ShiftInTime(3 , 1,              1,3);
#define HALF    ShiftInTime(4 , 1,              1,4);
#define TWINTIG ShiftInTime(5 , 1,              1,5);
#define MVIJF   ShiftInTime(6 , 1,              1,6);
#define KWART   ShiftInTime(7 , 1,              1,7);

#define MTIEN   ShiftInTime(8 , 2,              1,0);
#define OVER    ShiftInTime(9 , 2,              1,1);
#define ZES     ShiftInTime(10, 2,              1,2);
#define TWEE    ShiftInTime(11, 2,              1,3);
#define VIJF    ShiftInTime(12, 2,              1,4);
#define TWAALF  ShiftInTime(13, 2,              1,5);
#define TIEN    ShiftInTime(14, 2,              1,6);
#define ELF     ShiftInTime(15, 2,              1,7);

#define VIER    ShiftInTime(16, 3,              1,0);
#define NEGEN   ShiftInTime(17, 3,              1,1);
#define DRIE    ShiftInTime(18, 3,              1,2);
#define ACHT    ShiftInTime(19, 3,              1,3);
#define ZEVEN   ShiftInTime(20, 3,              1,4);
#define EEN     ShiftInTime(21, 3,              1,5);
#define UUR     ShiftInTime(22, 3,              1,6);
#define VOOR    ShiftInTime(23, 3, ToggleEdsoft  ,7);
#endif UK
#ifdef FR
char TimeText[][10] = {"Il ","est ","etait ","onze ","cinq ","deux ","trois ","six ","quatre ","minuit ","dix ","neuf ",
                     "midi ","huit ","sept ","une ","heures ","et ","moins ","le ","quart ","dix ","vingt ","cinq ","demie" };
#define HET     ShiftInTime(0 , 1,Toggle_HetWasIs,0);
#define IS      ShiftInTime(1 , 1,Toggle_HetWasIs,1);
#define WAS     ShiftInTime(2 , 1,Toggle_HetWasIs,2);
#define ELF     ShiftInTime(3 , 1,              1,3);
#define VIJF    ShiftInTime(4 , 1,              1,4); 
#define TWEE    ShiftInTime(5 , 1,              1,5);
#define DRIE    ShiftInTime(6 , 1,              1,6);
#define ZES     ShiftInTime(7 , 1,              1,7);

#define VIER    ShiftInTime(8 , 2,              1,0);
#define NTWAALF ShiftInTime(9 , 2,              1,1);
#define TIEN    ShiftInTime(10, 2,              1,2);
#define NEGEN   ShiftInTime(11, 2,              1,3);
#define TWAALF  ShiftInTime(12, 2,              1,4);
#define ACHT    ShiftInTime(13, 2,              1,5);
#define ZEVEN   ShiftInTime(14, 2,              1,6);
#define EEN     ShiftInTime(15, 2,              1,7);

#define UUR     ShiftInTime(16, 3,              1,0);
#define OVER    ShiftInTime(17, 3,              1,1);
#define VOOR    ShiftInTime(18, 3,              1,2);
#define EN      ShiftInTime(19, 3,              1,3);
#define KWART   ShiftInTime(20, 3,              1,4);
#define MTIEN   ShiftInTime(21, 3,              1,5);
#define TWINTIG ShiftInTime(22, 3,              1,6);
#define MVIJF   ShiftInTime(23, 3,              1,7);

#define HALF    ShiftInTime(24, 4,              1,0); //  TIEN; TWINTIG;
#endif FR

//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 
#if defined(__AVR_ATmega328P__) 
// Digital hardware constants ATMEGA 328 ----
enum DigitalPinAssignments {
  DCF_PIN     =  2,               // DCFPulse on interrupt  pin
  encoderPinA  = 3,               // right (labeled DT on decoder)on interrupt  pin
  clearButton  = 4,               // switch (labeled SW on decoder)
  PWMpin       = 5,               // Pin that controle PWM signal on BC327 transistor to dim light
  BT_RX        = 6,               // Bluetooth RX connect to TXD op de BT module
  BT_TX        = 7,               // Bluetooth TX connect to RXD op de BT module
  encoderPinB  = 8,               // left (labeled CLK on decoder)no interrupt pin  
  DCF_LED_Pin  = 9,               // define LED pin voor DCF pulse
  LEDDataPin   = 10,              // blauw HC595
  LEDStrobePin = 11,              // groen HC595
  LEDClockPin  = 12,              // geel  HC595
  secondsPin   = 13,
  HeartbeatLED = 13};
                                  // Analogue hardware constants ----
enum AnaloguePinAssignments {
 PhotoCellPin  = 2,               // LDR pin
 EmptyA3       = 3,               //
 SDA_pin       = 4,               // SDA pin
 SCL_pin       = 5};              // SCL pin
# endif


//--------------------------------------------//-------------------------------------------- 
#if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
                                  // Digital hardware constants ATMEGA 1284P ----
enum DigitalPinAssignments {
  BT_RX        = 0,               // Bluetooth RX
  BT_TX        = 1,               // Bluetooth TX
  DCF_PIN      = 2,               // DCFPulse on interrupt pin
  PWMpin       = 3,               // Pin that controle PWM signal on BC327 transistor to dim light
  MAX7219_DIN  = 4,               // MAX7219 DIN gr     PB4 PWM
  MAX7219_CS   = 5,               // MAX7219 CS  ge     PB5 digital
  MAX7219_CLK  = 6,               // MAX7219 CLK or     PB6 PWM  
  DS1820Temp   = 7,               // DS1820 temperature PB7 PWM 
  PIN08        = 8,               // RX1                PD0 digital
  PIN09        = 9,               // TX1                PD1 digital
  LED10        = 10,              // LED10              PD2 digital
  encoderPinB  = 11,              // left (labeled CLK on decoder) no interrupt pin  PD3 digital
  encoderPinA  = 12,              // right (labeled DT on decoder) no interrupt pin  PD4 PWM
  clearButton  = 13,              // switch (labeled SW on decoder)                  PD5 PWM
  DCF_LED_Pin  = 14,              // define pin voor DCF signal Led                  PD6 PWM
  HeartbeatLED = 15,              // LED15                                           PD7 PWM
  SCL_pin      = 16,              // SCL pin       PC0 interrupt
  SDA_pin      = 17,              // SDA pin       PC1 interrupt
  PIN18        = 18,              // Empty         PC2 digital
  LED19        = 19,              // LED19         PC3 digital
  LEDDataPin   = 20,              // blauw HC595   PC4 digital
  LEDStrobePin = 21,              // groen HC595   PC5 digital
  LEDClockPin  = 22,              // geel  HC595   PC6 digital
  secondsPin   = 23};             //               PC7 digital
                                  // Analogue hardware constants ----
enum AnaloguePinAssignments {
  EmptyA0      = 24,              // Empty
  EmptyA1      = 25,              // Empty
  PhotoCellPin = 26,              // LDR pin
  EmptyA3      = 27,              // Empty
  EmptyA4      = 28,              // Empty
  EmptyA5      = 29,              // Empty
  EmptyA6      = 30};             // Empty
 # endif 
//--------------------------------------------
// LED
//--------------------------------------------
byte BrightnessCalcFromLDR = 200; // BRIGHTNESS 0 - 255
int  ToggleEdsoft = 1;

//--------------------------------------------
// KY-040 ROTARY
//-------------------------------------------- 
                          #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);              // Use digital pin  for encoder
                          #endif ROTARYMOD      
long          Looptime = 0;
unsigned long RotaryPressTimer = 0;
//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
float LightReducer    = 0.80 ;     // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
byte  LowerBrightness = 10;        // Lower limit of Brightness ( 0 - 255)
int   OutPhotocell;                // stores reading of photocell;
int   MinPhotocell    = 1024;      // stores minimum reading of photocell;
int   MaxPhotocell    = 1;         // stores maximum reading of photocell;

//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 50
static unsigned long msTick;        // the number of millisecond ticks since we last incremented the second counter
int    count; 
byte   Isecond, Iminute, Ihour , Iday, Imonth, Iyear; 
byte   Display1   = 0  , Display2 = 0, Display3 = 0;
byte   lastminute = 0  , lasthour = 0, sayhour  = 0;
int    Delaytime          = 200;
byte   Toggle_HetWasIs    = 1;     // Turn On/Off HetIsWas lights
int    Toggle_HetWasIsUit = 0;     // Turn off HetIsWas after 10 sec
byte   SecPulse           = 0;     // give a pulse to the Isecond led
byte   Demo               = false;
byte   Zelftest           = false;
byte   hbval              = 128;
byte   hbdelta            = 8;
String SerialString;

const char menu[][MAXTEXT] PROGMEM =  {
 "Woordklok No31 Woordklok ",
 "Enter time as:hhmm (1321) or hhmmss (132145)",
 "Enter A for normal display",
 "Enter B to suspress Het Is Was in display",
 "Enter C to suspress Het Is Was after 10 seconds",
 "Enter D D15122017 for date 15 December 2017",
 "Enter G for DCF-signalinfo on display",
 "Enter Mnn (M90)Max light intensity (1% - 250%)",
 "Enter Lnn (L5) Min light intensity ( 1 - 255)",
 "Enter I for info",
 "Enter X for Demo mode",
 "Enter Z for Self test",
 "Ed Nieuwenhuys         V106 Feb-2018" };

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
RTC_DS3231 RTC;                   //RTC_DS1307 RTC;  
DateTime Inow;

//--------------------------------------------
// BLUETOOTH
//--------------------------------------------                                     
#ifdef BLUETOOTHMOD               // Bluetooth ---------------------
SoftwareSerial Bluetooth(BT_RX, BT_TX);     // RX <=> TXD on BT module, TX <=> RXD on BT module
String BluetoothString;
#endif BLUETOOTHMOD  
                              
//--------------------------------------------
// RDA5807 FM-RADIO
//-------------------------------------------- 
#ifdef FMRADIOMOD                           // FM radio -----------------------
byte  RadioUur;                             // reading from  RDS FM-radio 
byte  RadioMinuut;                          // reading from  RDS FM-radio                          
float fini = 103.50; //91.60; // 103.50;    //98.10;               // Start frequency
int   ftun;                                 // Selected frequency 
float Freq_lower_bandwith = 87.00;          // lower Band limit 
float Freq_tuned;                           //
int   RDA5807_adrs = 0x10;                  // I2C-Address RDA Chip for sequential  Access
int   RDA5807_adrr = 0x11;                  // I2C-Address RDA Chip for random      Access
int   RDA5807_adrt = 0x60;                  // I2C-Address RDA Chip for TEA5767like Access
int   sidx = 0;                             // Counter of frequency array
int   vol  = 0;                             // Volume
int   rssi = 0;                             // Signal-Level
unsigned int auRDS[32];
unsigned int auRDA5807_Reg[32];
unsigned int aui_RDA5807_Reg[32];
unsigned int aui_buf[8];
unsigned int auRDA5807_Regdef[10] ={
                                    0x0758,  // 00 defaultid
                                    0x0000,  // 01 not used
                                    0xD009,  // 02 DHIZ,DMUTE,BASS, POWERUPENABLE,RDS
                                    0x0000,  // 03
                                    0x1400,  // 04 DE ? SOFTMUTE  
                                    0x84D0,  // 05 INT_MODE, SEEKTH=0110,????, Volume=0
                                    0x4000,  // 06 OPENMODE=01
                                    0x0000,  // 07 unused ?
                                    0x0000,  // 08 unused ?
                                    0x0000   // 09 unused ?
                                  };
#endif FMRADIOMOD                // END FM radio ------------------------
//--------------------------------------------
// DCF-2 DCF77 MODULE
//--------------------------------------------
byte DCF_signal = 50;                        // is a proper time received?
bool SeeDCFsignalInDisplay = false;         // if ON then the display line HET IS WAS will show the DCF77-signal received
                    #ifdef DCFMOD           // DCF77 ------------------------------

#if defined(__AVR_ATmega328P__) 
#define DCF_INTERRUPT 0                     // DCF Interrupt number associated with DCF_PIN
#endif
#if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
#define DCF_INTERRUPT 2                     // DCF Interrupt number associated with DCF_PIN
#endif
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,LOW);
                    #endif DCFMOD 
                    #ifdef LCDMOD
//--------------------------------------------
// LCD Module
//--------------------------------------------
//LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);  // 0x27 is the I2C bus address for an unmodified backpack
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);// 0x27 or 0x3F is the I2C bus address for an unmodified backpack
                    #endif  LCDMOD 
                    #ifdef MAX7219_8DIGIT
//----------------------------------------
// MAX7219_8DIGIT  
//----------------------------------------
// MAX7219_DIN is connected to the DataIn, MAX7219_CLK is connected to the CLK, MAX7219_CS is connected to LOAD  We have only a single MAX72XX.
LedControl lc = LedControl( MAX7219_DIN, MAX7219_CLK, MAX7219_CS, 1 );
                    #endif MAX7219_8DIGIT         
//----------------------------------------
// Temperature sensor DS1820 
//----------------------------------------
                     #ifdef DS1820
#define           TEMPERATURE_PRECISION 9   // a DS18B20 takes from 94ms (9-bit resolution) to 750ms (12-bit resolution) to convert temperature 
OneWire           oneWire(DS1820Temp);      // Setup a oneWire instance to communicate with any OneWire devices  DS1820Temp
DallasTemperature Tempsensors(&oneWire);    // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress     tempDeviceAddress;        // We'll use this variable to store a found device address
int               numberOfDevices;          // Number of temperature devices found
                     #endif DS1820
//----------------------------------------
// Common
//----------------------------------------
char sptext[MAXTEXT+2];                     // for common print use                                      
                                          
                    // End Definitions  ---------------------------------------------------------
                               
//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
SerialCheck();

if(Demo)  Demomode();
else if (Zelftest) Selftest(); 
else
 { 
                              #if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
  heartbeat();                // only heartbeat with ATMEGA1284
                              #endif
  EverySecondCheck();
  
  EveryMinuteUpdate();
                              #ifdef FMRADIOMOD     
  FMradioCheck();  
                              #endif FMRADIOMOD
                              #ifdef BLUETOOTHMOD   
  BluetoothCheck(); 
                              #endif BLUETOOTHMOD
                              #ifdef DCFMOD         
  DCF77Check();
                              #endif DCFMOD

                              #ifdef ROTARYMOD      
  RotaryEncoderCheck(); 
                              #endif ROTARYMOD 
 }
}  
//--------------------------------------------
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                                    // initialise the hardware // initialize the appropriate pins as outputs:

 Serial.begin(9600);                                 // setup the serial port to 9600 baud 
 Wire.begin();                                       // start the wire communication I2C
 RTC.begin();                                        // start the RTC-module
 pinMode(LEDClockPin,  OUTPUT); 
 pinMode(LEDDataPin,   OUTPUT); 
 pinMode(LEDStrobePin, OUTPUT); 
 pinMode(PWMpin,       OUTPUT);
 pinMode(secondsPin,   OUTPUT );
                                  #ifdef ROTARYMOD   
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Tekstprintln("Rotary enabled"); 
                                  #endif ROTARYMOD  
                                  #if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
 pinMode(LED10,        OUTPUT );
 pinMode(HeartbeatLED, OUTPUT );
 pinMode(LED19,        OUTPUT );
                                  #endif

                                  #ifdef BLUETOOTHMOD   
 Bluetooth.begin(9600);                              // setup the Bluetooth port to 9600 baud 
 Tekstprintln("Bluetooth enabled");
                                  #endif BLUETOOTHMOD
                                  #ifdef FMRADIOMOD     
 Setup_FMradio();                                    // start the FM-radio
 Tekstprintln("FM-radio enabled");
                                  #endif FMRADIOMOD 
                                  #ifdef DS1820         
 setupDS1820();
                                  #endif DS1820
                                  #ifdef DCFMOD         
 pinMode(DCF_LED_Pin,  OUTPUT);
 pinMode(DCF_PIN,      INPUT_PULLUP); 
 DCF.Start();                                        // start the DCF-module
 Tekstprintln("DCF enabled");
                                  #endif DCFMOD
                                  #ifdef LCDMOD         
 lcd.begin (16,2);                                   // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH); 
 Tekstprintln("LCD enabled");
                                  #endif LCDMOD 
                                  #ifdef MAX7219_8DIGIT 
 lc.shutdown(0,false);
 lc.setIntensity(0,2);                               // Set the brightness to a medium values 
 lc.clearDisplay(0);                                 // and clear the display
  Tekstprintln("MAX7219 enabled");
                                  #endif MAX7219_8DIGIT
 analogWrite(PWMpin, BrightnessCalcFromLDR);         // the duty cycle: between 0 (lights off) and 255 (light full on).    
  GetTijd(1);                                         // Get the time and print it to serial                      
 DateTime now = RTC.now();                           // Get the time from the RTC
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
   {
    Tekstprintln("RTC is older than compile time! Updating");                                    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__))); 
   } 
  if (EEPROM.read(0) <3 || EEPROM.read(0) > 200)    EEPROM.write(0,(int)(LightReducer * 100));    // default intensity for this clock
  if (EEPROM.read(1) <1 || EEPROM.read(1) > 100)    EEPROM.write(1, LowerBrightness);             // default Lower Brightness for this clock
 LightReducer    = ((float)EEPROM.read(0) / 100);    // store it is the work variable
 LowerBrightness = EEPROM.read(1);                   // store it is the work variable
 Looptime        = millis();                         // Used in KY-040 rotary
 msTick          = Looptime; 
 SWversion();                                        // Display the version number of the software
// Selftest();                                         // Play the selftest
 GetTijd(1);                                         // Get the time and print it to serial
                     #ifdef ROTARYMOD
 myEnc.write(0);                                     // Clear Rotary encode buffer
                     #endif ROTARYMOD

}
// --------------------------- END SETUP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 for (int n=0; n<52; n++) {Serial.print(F("_"));} Serial.println();
 for (int i = 0; i < 13; i++)   {strcpy_P(sptext, menu[i]);   Tekstprintln(sptext);  }
 for (int n=0; n<52; n++) {Serial.print(F("_"));} Serial.println();
 sprintf(sptext,"Max brightness: %3ld%%   ",(long)(LightReducer*100)) ;
 Tekstprintln(sptext);
 sprintf(sptext,"Min brightness: %3ld bits   ",(long)LowerBrightness) ;
 Tekstprintln(sptext);
 for (int n = 0; n < 52; n++) {Serial.print(F("_"));} Serial.println();
}

//--------------------------------------------
// CLOCK Demo mode
//--------------------------------------------
void Demomode(void)
{
  if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);       // Turn OFF the second on pin 13
  if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                               // second++; 
   digitalWrite(secondsPin,HIGH);                                   // turn ON the second on pin 13
   ++SecPulse;                                                      // second routine in function DimLeds
   if( ++Iminute >59) { Iminute = 0; Isecond = 0; Ihour++;}
    if(    Ihour >24)   Ihour = 0;
   DimLeds(false);
   Displaytime();
   Tekstprintln("");
  }
}

//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char tekst[])
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   
 Bluetooth.print(tekst);  
                          #endif BLUETOOTHMOD
}
void Tekstprintln(char tekst[])
{
 Serial.println(tekst);    
                          #ifdef BLUETOOTHMOD
 Bluetooth.println(tekst);
                          #endif BLUETOOTHMOD
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
  if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);       // Turn OFF the second on pin 13
  if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                               // second++; 
   digitalWrite(secondsPin,HIGH);                                   // turn ON the second on pin 13
   ++SecPulse;                                                      // second routine in function DimLeds
   GetTijd(0);                                                      // synchronize time with RTC clock
                       #ifdef LCDMOD
   Print_tijd_LCD();
                       #endif LCDMOD
   if ((Toggle_HetWasIsUit == 2) && (Isecond > 10)) Toggle_HetWasIs = 0; 
    else Toggle_HetWasIs = 1;                                       // HET IS WAS is On
   if(Isecond % 30 == 0) DimLeds(true);                             //Led Intensity Control + seconds tick print every 30 seconds   
    else                 DimLeds(false);
  if ((Toggle_HetWasIsUit == 2) && (Isecond == 11))Displaytime();   // turn Leds OFF on second == 11

  if(Iminute == 0 && Isecond <9)
   { 
    ToggleEdsoft = Isecond % 2;         // ToggleEdsoft becomes 0 or 1 and turn on and off the first seconds at minute 0 the Edsoft light on pin 24
    Serial.println(ToggleEdsoft);
    Displaytime();
   }
                     #ifdef DS1820
   DS1820read();
                     #endif DS1820  
  }
 }
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
 if (Iminute != lastminute)                                         //show time every minute
  { 
   lastminute = Iminute;
   Displaytime();
   Print_RTC_tijd();
   DCF_signal--;
   DCF_signal = constrain( DCF_signal,1,99);
  } 
 if (Ihour != lasthour) {lasthour = Ihour;}
 }
                            #ifdef BLUETOOTHMOD
//--------------------------------------------
// CLOCK check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 long looptimeBT = millis();  //avoid an hangup in this loop  
 while (Bluetooth.available() and (millis() - looptimeBT < 10000) )
  {
   delay(3); 
   char c = Bluetooth.read();
   Serial.print(c);
   if (c>31 && c<128) BluetoothString += c;
   else c = 0;     // delete a CR
  }
 if (BluetoothString.length()>0)  
      ReworkInputString(BluetoothString);            // Rework ReworkInputString();
 BluetoothString = "";
}
                           #endif BLUETOOTHMOD
                           
                           #ifdef DCFMOD
//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
 time_t DCFtime = DCF.getTime();                      // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
   Tekstprint("DCF: Time is updated ----->  ");
   DCF_signal+=2;
   setTime(DCFtime); 
   RTC.adjust(DCFtime);
//   digitalClockDisplay(); *************************
  }
 bool LHbit = digitalRead(DCF_PIN);
 digitalWrite(DCF_LED_Pin, 1 - LHbit );               // write inverted DCF pulse to LED on board 
 if (SeeDCFsignalInDisplay == true)
  {
   Toggle_HetWasIs = LHbit;
   Display1 |= (Toggle_HetWasIs<<0);                  // Turn off the  HET IS WAS LEDs
   Displaytime();
  }
  DCF_signal = constrain(DCF_signal,0,99);            // DCF_signal <100

} 
                           #endif DCFMOD 
//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 while (Serial.available())
  {
   delay(3);  
   char c = Serial.read();
   if (c>31 && c<128) SerialString += c;                            // allow input from Space - Del
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString();
 SerialString = "";
}

//--------------------------------------------
// CLOCK heartbeat
//--------------------------------------------
void heartbeat() 
{
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40)    return;
  last_time = now;
  if (hbval > 230 || hbval < 20 ) hbdelta = -hbdelta; 
  hbval += hbdelta;
  analogWrite(HeartbeatLED, hbval);
}

//------------------------ KY-040 rotary encoder ------------------------- 
//--------------------------------------------
// KY-040 ROTARY check if the rotary is moving
//--------------------------------------------
                           #ifdef ROTARYMOD
void RotaryEncoderCheck(void)
{
// If button pressed, 60 sec after start ATMEGA, then there are 60 seconds to adjust the light intensity. 
// RotaryPressTimer is the time in millisec after start ATMEGA 
 long encoderPos = myEnc.read();
 if ( (encoderPos) && ( (millis() - Looptime) >200))                  // if rotary turned debounce 0.2 sec
  {   
   Serial.print(F("--------> Index:"));   Serial.println(encoderPos);
   if  (encoderPos >0)                                                // increase the MINUTES
    {
     if ( millis() > 60000 && (millis() - RotaryPressTimer) < 60000)
         { WriteLightReducer(0.05); }                                 // If time < 60 sec then adjust light intensity factor
     else 
     {
      if( ++Iminute >59) { Iminute = 0; Isecond = 0;  }
      SetRTCTime();  
     }     
     myEnc.write(0);
     Looptime = millis();                                             // Set encoder pos back to 0
    }
   if  (encoderPos <0)                                                // increase the HOURS
    {
     if (millis() > 60000 &&  (millis() - RotaryPressTimer) < 60000) 
         { WriteLightReducer(-0.05); }    // If time < 60 sec then adjust light intensity factor
     else
      { 
      if( ++Ihour >23) { Ihour = 0; }
      SetRTCTime();   
      }
     myEnc.write(0);
     Looptime = millis();                                             // Set encoder pos back to 0      
    }                                                
  }
 if (digitalRead(clearButton) == LOW )                                // set the time by pressing rotary button
  { 
    delay(200);
    RotaryPressTimer =  millis();                                     // If time < 60 sec then adjust light intensity factor
    Toggle_HetWasIsUit++;
                                    #ifdef DCFMOD    
    if (Toggle_HetWasIsUit == 3)
       {
        Toggle_HetWasIsUit = -1;
        SeeDCFsignalInDisplay = true;
       }
                                    #endif DCFMOD
    if (Toggle_HetWasIsUit >= 3)  { Toggle_HetWasIsUit = 0; SeeDCFsignalInDisplay = false;}
    if (Toggle_HetWasIsUit == 0)  { Toggle_HetWasIs = 1;    SeeDCFsignalInDisplay = false;} // HET IS WAS On 
    if (Toggle_HetWasIsUit == 1)  { Toggle_HetWasIs = 0;}                                   // HET IS WAS Off
    if (Toggle_HetWasIsUit == 2)  { Toggle_HetWasIs = 0; Play_Lights(); } // Off after 10 sec
    
    Serial.print(F("Toggle_HetWasIsUit: "));   Serial.println(Toggle_HetWasIsUit);
    Serial.print(F("Toggle_HetWasIs: "));      Serial.println(Toggle_HetWasIs);    
    Displaytime();
    myEnc.write(0);
    Looptime = millis(); 
   }
  myEnc.write(0);   
 }
                           #endif ROTARYMOD
//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{
  GetTijd(1);             //Prints time in Serial monitor
  LedsOff(); 
#ifdef NL 
  HET;     Laatzien();  IS;      Laatzien();  WAS;     Laatzien();
  MVIJF;   Laatzien();  MTIEN;   Laatzien();
  KWART;   Laatzien();  VOOR;    Laatzien();
  OVER;    Laatzien();  PRECIES; Laatzien(); 
  HALF;    Laatzien();  ELF;     Laatzien();  
  VIJF;    Laatzien();  TWEE;    Laatzien();  
  EEN;     Laatzien();  VIER;    Laatzien();
  TIEN;    Laatzien();  TWAALF;  Laatzien();
  DRIE;    Laatzien();  NEGEN;   Laatzien(); 
  ACHT;    Laatzien();  ZES;     Laatzien(); 
  ZEVEN;   Laatzien();  UUR;     Laatzien();
  EDSOFT;  Laatzien();
#endif NL
#ifdef UK
  HET;     Laatzien();  IS;      Laatzien();  WAS;     Laatzien();
  PRECIES; Laatzien();  HALF;    Laatzien();
  TWINTIG; Laatzien();  MVIJF;   Laatzien();
  KWART;   Laatzien();  MTIEN;   Laatzien();
  OVER;    Laatzien();  VOOR;    Laatzien();
  ZES;     Laatzien();  TWEE;    Laatzien(); 
  VIJF;    Laatzien();  TWAALF;  Laatzien();
  TIEN;    Laatzien();  ELF;     Laatzien(); 
  VIER;    Laatzien();  NEGEN;   Laatzien(); 
  DRIE;    Laatzien();  ACHT;    Laatzien();
  ZEVEN;   Laatzien();  EEN;     Laatzien();
  UUR;     Laatzien();
  #endif UK  

  for(int i=0; i<2; i++)
  {
   Display1=255;   Display2=255;   Display3=255;  Laatzien();
   Display1=0;     Display2=0;     Display3=0;    Laatzien();
  }  
  Play_Lights();     
  Displaytime();
}
// -------------------------- END Selftest   
//--------------------------- Time functions --------------------------

//--------------------------------------------
// CLOCK set the LED's for displaying
//--------------------------------------------
void Displaytime(void)
{
 LedsOff();                                  // start by clearing the display to a known state
 HET;                                        // HET light is always on
 switch (Iminute)
 {
#ifdef NL
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
  case  4: 
  case  5: 
  case  6: IS;  MVIJF; OVER; break;
  case  7: 
  case  8: WAS; MVIJF; OVER; break;
  case  9: 
  case 10: 
  case 11: IS;  MTIEN; OVER; break;
  case 12: 
  case 13: WAS; MTIEN; OVER; break;
  case 14: 
  case 15: 
  case 16: IS;  KWART; OVER; break;
  case 17: 
  case 18: WAS; KWART; OVER; break;
  case 19: 
  case 20: 
  case 21: IS;  MTIEN; VOOR; HALF; break;
  case 22: 
  case 23: WAS; MTIEN; VOOR; HALF; break;
  case 24: 
  case 25: 
  case 26: IS;  MVIJF; VOOR; HALF; break;
  case 27: 
  case 28: WAS; MVIJF; VOOR; HALF; break;
  case 29: IS;  HALF; break;
  case 30: IS;  PRECIES; HALF; break;
  case 31: IS;  HALF; break;
  case 32: 
  case 33: WAS; HALF; break;
  case 34: 
  case 35: 
  case 36: IS;  MVIJF; OVER; HALF; break;
  case 37: 
  case 38: WAS; MVIJF; OVER; HALF; break;
  case 39: 
  case 40: 
  case 41: IS;  MTIEN; OVER; HALF; break;
  case 42: 
  case 43: WAS; MTIEN; OVER; HALF; break;
  case 44: 
  case 45: 
  case 46: IS;  KWART; VOOR; break;
  case 47: 
  case 48: WAS; KWART; VOOR; break;
  case 49: 
  case 50: 
  case 51: IS;  MTIEN; VOOR;  break;
  case 52: 
  case 53: WAS; MTIEN; VOOR;  break;
  case 54: 
  case 55: 
  case 56: IS;  MVIJF; VOOR; break;
  case 57: 
  case 58: WAS; MVIJF; VOOR; break;
  case 59: IS;  break;
#endif NL

#ifdef UK
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
  case  4: 
  case  5: 
  case  6: IS;  MVIJF; OVER; break;
  case  7: 
  case  8: WAS; MVIJF; OVER; break;
  case  9: 
  case 10: 
  case 11: IS;  MTIEN; OVER; break;
  case 12: 
  case 13: WAS; MTIEN; OVER; break;
  case 14: 
  case 15: 
  case 16: IS;  KWART; OVER; break;
  case 17: 
  case 18: WAS; KWART; OVER; break;
  case 19: 
  case 20: 
  case 21: IS;  TWINTIG; OVER;  break;
  case 22: 
  case 23: WAS; TWINTIG; OVER; break;
  case 24: 
  case 25: 
  case 26: IS;  TWINTIG; MVIJF; OVER; break;
  case 27: 
  case 28: WAS; TWINTIG; MVIJF; OVER; break;
  case 29: IS;  HALF; OVER; break;
  case 30: IS;  PRECIES; HALF; OVER; break;
  case 31: IS;  HALF; OVER; break;
  case 32: 
  case 33: WAS; HALF; OVER; break;
  case 34: 
  case 35: 
  case 36: IS;  TWINTIG; MVIJF; VOOR; break;
  case 37: 
  case 38: WAS; TWINTIG; MVIJF; VOOR; break;
  case 39: 
  case 40: 
  case 41: IS;  TWINTIG; VOOR;  break;
  case 42: 
  case 43: WAS; TWINTIG; VOOR;  break;
  case 44: 
  case 45: 
  case 46: IS;  KWART; VOOR; break;
  case 47: 
  case 48: WAS; KWART; VOOR; break;
  case 49: 
  case 50: 
  case 51: IS;  MTIEN; VOOR;  break;
  case 52: 
  case 53: WAS; MTIEN; VOOR;  break;
  case 54: 
  case 55: 
  case 56: IS;  MVIJF; VOOR; break;
  case 57: 
  case 58: WAS; MVIJF; VOOR; break;
  case 59: IS;  break;
#endif UK
#ifdef FR
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
  case  4: 
  case  5: 
  case  6: IS;  MVIJF; OVER; break;
  case  7: 
  case  8: WAS; MVIJF; OVER; break;
  case  9: 
  case 10: 
  case 11: IS;  MTIEN; OVER; break;
  case 12: 
  case 13: WAS; MTIEN; OVER; break;
  case 14: 
  case 15: 
  case 16: IS;  KWART; OVER; break;
  case 17: 
  case 18: WAS; KWART; OVER; break;
  case 19: 
  case 20: 
  case 21: IS;  TWINTIG; OVER;  break;
  case 22: 
  case 23: WAS; TWINTIG; OVER; break;
  case 24: 
  case 25: 
  case 26: IS;  TWINTIG; MVIJF; OVER; break;
  case 27: 
  case 28: WAS; TWINTIG; MVIJF; OVER; break;
  case 29: IS;  HALF; OVER; break;
  case 30: IS;  PRECIES; HALF; OVER; break;
  case 31: IS;  HALF; OVER; break;
  case 32: 
  case 33: WAS; HALF; OVER; break;
  case 34: 
  case 35: 
  case 36: IS;  TWINTIG; MVIJF; VOOR; break;
  case 37: 
  case 38: WAS; TWINTIG; MVIJF; VOOR; break;
  case 39: 
  case 40: 
  case 41: IS;  TWINTIG; VOOR;  break;
  case 42: 
  case 43: WAS; TWINTIG; VOOR;  break;
  case 44: 
  case 45: 
  case 46: IS;  KWART; VOOR; break;
  case 47: 
  case 48: WAS; KWART; VOOR; break;
  case 49: 
  case 50: 
  case 51: IS;  MTIEN; VOOR;  break;
  case 52: 
  case 53: WAS; MTIEN; VOOR;  break;
  case 54: 
  case 55: 
  case 56: IS;  MVIJF; VOOR; break;
  case 57: 
  case 58: WAS; MVIJF; VOOR; break;
  case 59: IS;  break;
#endif FR
 
}
// if (Ihour >=0 && Ihour <12) digitalWrite(DCF_LED_Pin,0); else digitalWrite(DCF_LED_Pin,1);
                                        #ifdef NL
 sayhour = Ihour;
 if (Iminute > 18 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif NL
                                        #ifdef UK
 sayhour = Ihour;
 if (Iminute > 33 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif UK
                                        #ifdef FR
 sayhour = Ihour;                                   // French saying still not perfect
 if (Iminute > 33 )  sayhour = Ihour+1;
 if (sayhour == 24) sayhour = 0;
                                        #endif FR

 switch (sayhour)
 {
  case 13:
  case 1: EEN; break;
  case 14:
  case 2: TWEE; break;
  case 15:
  case 3: DRIE; break;
  case 16:
  case 4: VIER; break;
  case 17:
  case 5: VIJF; break;
  case 18:
  case 6: ZES; break;
  case 19:
  case 7: ZEVEN; break;
  case 20:
  case 8: ACHT; break;
  case 21:
  case 9: NEGEN; break;
  case 22:
  case 10: TIEN; break;
  case 23:
  case 11: ELF; break;
  case 0:
  case 12: TWAALF; break;
 } 
 switch (Iminute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UUR;  break; 
 }

 if(Iminute == 0 && Isecond <9) 
 { 
  ToggleEdsoft = Isecond % 2;         // ToggleEdsoft bocomes 0 or 1 and turn on and off the first 8 seconds at minute 0 the Edsoft light on pin 24
  EDSOFT;
 } 
// Tekstprintln("");
 WriteLEDs();
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    RTC.now();
 Ihour =   Inow.hour();
 Iminute = Inow.minute();
 Isecond = Inow.second();
// if (Ihour > 24) { Ihour = random(12)+1; Iminute = random(60)+1; Isecond = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}

//--------------------------------------------
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
                     #ifdef LCDMOD
//--------------------------------------------
// CLOCK Print time to LCD display
//--------------------------------------------
void Print_tijd_LCD(void)
{
 lcd.home (); // set cursor to 0,0
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Inow.hour(),Inow.minute(),Inow.second());   lcd.print(sptext);
 sprintf(sptext," LDR%d   ",analogRead(PhotoCellPin));                          lcd.print(sptext);
 lcd.setCursor (0,1);        // go to start of 2nd line
 sprintf(sptext,"%0.2d-%0.2d-%0.4d",Inow.day(),Inow.month(),Inow.year());       lcd.print(sptext);
 sprintf(sptext," DCF%d   ",DCF_signal);                                        lcd.print(sptext);
}
                      #endif LCDMOD
//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Ihour,Iminute,Isecond);
 Tekstprintln(sptext);
}

//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = constrain(Ihour  , 0,24);
 Iminute = constrain(Iminute, 0,59); 
 Isecond = constrain(Isecond, 0,59); 
 RTC.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                               // synchronize time with RTC clock
 Displaytime();
 Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from DS3231 Time module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    //temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          //2's complement int portion
    tLSB = Wire.read();                          //fraction portion 
    temp3231 = (tMSB & B01111111);               //do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    //only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else {  temp3231 = -273; }   
  return (temp3231);
}

// ------------------- End  Time functions 

// --------------------Light functions -----------------------------------
//--------------------------------------------
//  LED load the shiftbits in the LED display buffer
//--------------------------------------------
void ShiftInTime(byte num , byte Displaynr, byte Shiftbit, byte ShiftIn)
{  
 strcpy_P(sptext, TimeText[num]);
 if (SeeDCFsignalInDisplay == false)  Tekstprint(sptext);  
 switch (Displaynr)
  {
 case 1:
        Display1 |= Shiftbit << ShiftIn;
        break;
 case 2:
        Display2 |= Shiftbit << ShiftIn;
        break;
 case 3:
        Display3 |= Shiftbit << ShiftIn;
        break; 
 }     
}

//--------------------------------------------
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void){  Display1=0;  Display2=0;  Display3=0; }

//--------------------------------------------
//  LED Turn On the LED's
//  Write the actual values to the hardware 
//--------------------------------------------
void WriteLEDs(void) 
{                                                                                
 digitalWrite(LEDStrobePin,LOW);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display3);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display2);
 shiftOut(LEDDataPin, LEDClockPin, MSBFIRST, Display1);
 digitalWrite(LEDStrobePin,HIGH);
 delay(2);
}
//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Laatzien()
{ 
 WriteLEDs();  
 delay(Delaytime);  
 LedsOff();
}
//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------
void DimLeds(byte print) 
{
 int Temp;                                                                                                 
 if (SecPulse) 
 {
  int LDR_read = analogRead(PhotoCellPin);      // Read lightsensor
  int BrCalc, Temp;
  OutPhotocell = (int) (LightReducer * sqrt( (float) 63.5 * (float) constrain(LDR_read,1,1023))); // Linear --> hyperbolic with sqrt
  MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
  BrightnessCalcFromLDR = constrain(OutPhotocell, LowerBrightness , 255);                   // filter out of strange results
  BrCalc = (int) (BrightnessCalcFromLDR/2.55);
  if(print)
  {
   Temp = get3231Temp()-2; 
   sprintf(sptext,"LDR:%d (%d-%d)->%d=%d%% T=%dC ",LDR_read, MinPhotocell, MaxPhotocell, OutPhotocell,BrCalc,Temp);
   Tekstprint(sptext);
   Print_tijd();
  }
  analogWrite(PWMpin, BrightnessCalcFromLDR);  // write PWM 
 }
 SecPulse = 0;
}
//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
 Display1=255;   Display2=255;   Display3=255; Laatzien();
 for (int n=255 ; n>=0; n--) { analogWrite(PWMpin, n); delay(2);}    // the duty cycle: between 0 (lights off) and 255 (light full on).
 for (int n=0 ; n<=255; n++) { analogWrite(PWMpin, n); delay(2);}  
 LedsOff();
}

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(float amount)
{
 LightReducer += amount; 
 WriteLightReducerEeprom(LightReducer);
}
//--------------------------------------------
//  LED Write light intensity to EEPROM
//--------------------------------------------
void WriteLightReducerEeprom(float value)
{
 LightReducer = value;
 if (LightReducer < 0.01 ) LightReducer = 0.01;
 if (LightReducer > 2.50 ) LightReducer = 2.50;                      // May not be larger than 2.55 (value*100 = stored as byte 
 EEPROM.write(0, (int) (LightReducer * 100));                        // Store the value (0-250) in permanent EEPROM memory at address 0
 sprintf(sptext,"Max brightness: %3ld%%",(long)(LightReducer*100)) ;
 Tekstprintln(sptext);
// Serial.print(millis() - RotaryPressTimer); Serial.print(" msec ------- ");
// Serial.print(F("LightReducer: ")); Serial.print(LightReducer * 100); Serial.println("%");
}

//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(int LowerBrightness)
{
 if (LowerBrightness < 1 ) LowerBrightness =  1;                     // Range between 1 and 100
 if (LowerBrightness > 255) LowerBrightness = 255;
 EEPROM.write(1, LowerBrightness);                                   // Default Lower Brightness for this clock
 sprintf(sptext,"Lower brightness: %3ld bits",(long) LowerBrightness);
 Tekstprintln(sptext);
}

// --------------------End Light functions 

//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 String temp;
 float ff;
 int Jaar;
 InputString.toCharArray(sptext, MAXTEXT-1);

 if ( InputString[0] > 64 )                                         // Does the string start with a letter?
  {
  int val = InputString[0];
  int FMfreq;
  
  Tekstprintln(sptext);
  switch (val)
   {
    case 'A':
    case 'a':   
             Toggle_HetWasIsUit = 0; Toggle_HetWasIs = 1;             // All tekst displayed  
             Tekstprintln("All tekst displayed");
             break;
    case 'B':
    case 'b':    
             Toggle_HetWasIsUit = 1; Toggle_HetWasIs = 0;             // Het Is Was turned off
             Tekstprintln("Het Is Was turned off");
             break;
    case 'C':
    case 'c':    
            Toggle_HetWasIsUit = 2; Toggle_HetWasIs = 0;              // Het Is Was Off after 10 sec
            Play_Lights();                     
            Tekstprintln("Het Is Was Off after 10 sec");                          
            break;
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              temp   = InputString.substring(1,3);     Iday = (byte) temp.toInt(); 
              temp   = InputString.substring(3,5);   Imonth = (byte) temp.toInt(); 
              temp   = InputString.substring(5,9);     Jaar =  temp.toInt(); 
              Iday   = constrain(Iday  , 0, 31);
              Imonth = constrain(Imonth, 0, 12); 
              Jaar  = constrain(Jaar , 1000, 9999); 
              RTC.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
              Tekstprintln(sptext);
             }
             else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");

            break;
    case 'E':
    case 'e':
            Tekstprintln("Entered an E");
            break;       
                     #ifdef FMRADIO                        
    case 'F':
    case 'f':
            //set FM frequency
             temp = InputString.substring(1);
             FMfreq = temp.toInt();
             if (FMfreq < 8750 ) FMfreq = 8750;
             if (FMfreq > 10800) FMfreq = 10800;   
             RDA5807_setFreq((float) FMfreq/100);           
             break;
                     #endif FMRADIO

    case 'G':                                                         // Toggle DCF Signal on Display
    case 'g':
             SeeDCFsignalInDisplay = 1 - SeeDCFsignalInDisplay;
             sprintf(sptext,"SeeDCFsignal: %d",SeeDCFsignalInDisplay);
             Tekstprintln(sptext);
             break;
    case 'L':                                                         // Lowest value for Brightness
    case 'l':    
             temp = InputString.substring(1);
             LowerBrightness = temp.toInt();
             WriteLowerBrightness(LowerBrightness);
             break;
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':    
             temp = InputString.substring(1);
             ff = (float)(temp.toInt()) / 100;
             WriteLightReducerEeprom(ff);
             break;
    case 'I':
    case 'i':   
            SWversion();
            Display1=255;   Display2=255;   Display3=255;  Laatzien();
            Display1=0;     Display2=0;     Display3=0;    Laatzien();
            break;
                     #ifdef FMRADIO
    case 'R':
    case 'r':
            RDA5807_Report();
            break;
    case 'S':
    case 's':
            RDA5807_ReadStatus();            
            break;
    case 'T':
    case 't':    
            RDA5807_RDS_Dump();                                          
            break;             
                     #endif FMRADIO 
    case 'X':
    case 'x':    
            Demo = 1 - Demo;                                          // toggle Demo mode
            Play_Lights();
            GetTijd(0);  
            Displaytime();
            sprintf(sptext,"Demo mode: %d",Demo);
            Tekstprintln(sptext);
            break; 
    case 'Z':
    case 'z':
            Zelftest = 1 - Zelftest;                                       
            break;          
    default:
            break;
   }
   Displaytime();
   InputString = "";
 }
 else if (InputString.length() > 3 && InputString.length() <7 )
 {
  temp = InputString.substring(0,2);   
  Ihour = temp.toInt(); 
  if (InputString.length() > 3) { temp = InputString.substring(2,4); Iminute = temp.toInt(); }
  if (InputString.length() > 5) { temp = InputString.substring(4,6); Isecond = temp.toInt(); }
  
  SetRTCTime();
 }
 InputString = "";
 temp = "";
}
                       #ifdef FMRADIOMOD
//----------------------------------------- FM radio -------------------------------------------- 
//--------------------------------------------
// RDA5807 Check if RDS data are available
//--------------------------------------------
void FMradioCheck(void)                               
{
 int uur, minuut, ofs;
 if (!Serial.available())
  {
   RDA5807_ReadW(4);                              // Read RDS-Data as 4 Word to Array
   if ((auRDS[1] & 0xF000) == 0x4000)
    {
      uur    = (16 * (auRDS[2] & 0x0001) + ((auRDS[3] & 0xF000)>>12));
      minuut = (auRDS[3] & 0x0FC0)>>6;
      ofs    = (auRDS[3] & 0x003F);
      uur   += (ofs / 2);
 //Serial.print(F("<"));     
    }
  if (uur<24 && RadioUur != uur && RadioMinuut != minuut) // to avoid a 100 ms delay. Otherwise same time is retrieved many times
    { 

    sprintf(sptext,"%0.2d:%0.2d",uur,minuut);
    Tekstprintln(sptext);
      
//     if (uur < 10)       { Serial.print(F(" ")); } 
//     Serial.print(uur);    Serial.print(F("u:"));
//     if (minuut < 10)    { Serial.print(F("0")); } 
//     Serial.print(minuut); Serial.println(F("m"));
     RadioUur = uur;
     RadioMinuut = minuut;
//     delay(80); 
    }
   }
}

//--------------------------------------------
// RDA5807 Setup_FMradio
//--------------------------------------------
void  Setup_FMradio(void)
 {
  RDA5807_PowerOn();
  RDA5807_Reset();
  RDA5807_setFreq(fini);
 }  

//--------------------------------------------
// RDA5807 Reset Chip to Default Configuration
//--------------------------------------------
int RDA5807_Reset()
{
  Serial.println(F("RESET RDA5807"));
  for(int i = 0;i < 7; i++) {auRDA5807_Reg[i] = auRDA5807_Regdef[i];}
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x0002;   // Enable SoftReset
  int ret = RDA5807_Write();
  auRDA5807_Reg[2] = auRDA5807_Reg[2] & 0xFFFB;   // Disable SoftReset
  return ret;
}

//----------------------------------------
// RDA5807 Power Off
//----------------------------------------
int RDA5807_PowerOff()
{
  RDA5807_setVol(0);
  Serial.println(F("Power OFF RDA5807"));
  aui_RDA5807_Reg[2]=0x0001;   // all bits off
  return RDA5807_Write();
  auRDA5807_Reg[2] =auRDA5807_Regdef[2];       // Reset to Default Value
}

//----------------------------------------
// RDA5807 Power On
//----------------------------------------
int RDA5807_PowerOn()
{
  Serial.println(F("Power ON RDA5807"));
  auRDA5807_Reg[3] = auRDA5807_Reg[3] | 0x010;   // Enable Tuning
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x001;   // Enable PowerOn
  int ret = RDA5807_Write();
  auRDA5807_Reg[3] = auRDA5807_Reg[3] & 0xFFEF;  // Disable Tuning
  return ret;
}

//----------------------------------------
// RDA5807 Seek up  to next Station
//----------------------------------------
int RDA5807_SeekUp()
{
  Serial.println(F("SeekUp"));
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x0300;   // Enable Seekup
  RDA5807_Write();
  auRDA5807_Reg[2] = auRDA5807_Reg[2] & 0xFCFF;   // Disable Seekup
  return 0;
}

//----------------------------------------
// RDA5807 Seek down  to next Station
//----------------------------------------
int RDA5807_SeekDown()
{

  Serial.println(F("SeekDown"));
  auRDA5807_Reg[2] = auRDA5807_Reg[2] | 0x0100;   // Enable SeekDown(default)
  RDA5807_Write();
  auRDA5807_Reg[2] = auRDA5807_Reg[2] & 0xFCFF;   // Disable Seek 
  return 0;
}

//----------------------------------------
// RDA5807 Tune Radio to defined Frequency
//----------------------------------------
int RDA5807_setFreq(float mhz)
{
  ftun = mhz * 100.0; 
  Freq_tuned = mhz;
  int Chnumber = (int)(( 0.01 + mhz - Freq_lower_bandwith ) / 0.1);
  Serial.print(F("Frequency: "));
  Serial.print(ftun);
  Serial.print(F(" Channel: "));
  Serial.println(Chnumber);
  Chnumber = Chnumber & 0x03FF;
  auRDA5807_Reg[3] = Chnumber * 64 + 0x10;     // Channel + TUNE-Bit + Band=00(87-108) + Space=00(100kHz)
  Wire.beginTransmission(RDA5807_adrs);
  Wire_write16(0xD009);
  Wire_write16(auRDA5807_Reg[3]);
  Wire.endTransmission(); 
  return 0;
}

//----------------------------------------
// RDA5807 Set Volume
//----------------------------------------
int RDA5807_setVol(int setvol)
{
  vol = setvol;
  if (vol > 15) {vol = 15; Serial.println(F("Vol already maximal")); return 1; }
  if (vol < 0)  {vol = 0;  Serial.println(F("Vol already minimal")); return 1; }
  Serial.print(F("Volume="));     Serial.println(vol);
  auRDA5807_Reg[5] = (auRDA5807_Reg[5] & 0xFFF0)| vol;   // Set New Volume
  RDA5807_WriteReg(5);
  return 0;
}

//----------------------------------------
// Write 16Bit To I2C / Two Wire Interface
//----------------------------------------
void Wire_write16(unsigned int val)
{
 // if (b_debug) { Serial_print16h(val);}
  Wire.write(val >> 8); Wire.write(val & 0xFF);
}

//------------------------------------------
// Serial Print 16Bit Number in HEX as hh:ll
//------------------------------------------
void Serial_print16h(unsigned int uval)
{
  byte b_high,b_low;
  b_high = uval >> 8; b_low = uval & 0xFF;
  if (b_high < 0x10){ Serial.write('0');} Serial.print(b_high,HEX); Serial.write(':');
  if (b_low  < 0x10){ Serial.write('0');} Serial.print(b_low ,HEX); 
}

//----------------------------------------
// RDA5807 Set all Configuration Registers
//----------------------------------------
int RDA5807_Write()
{
  Wire.beginTransmission(RDA5807_adrs);
  for ( int i = 2; i < 7; i++) { Wire_write16(auRDA5807_Reg[i]);}
  return Wire.endTransmission();
}
//----------------------------------------
// RDA5807 Set one Configuration Registers
//----------------------------------------
int RDA5807_WriteReg(int reg)
{
  Wire.beginTransmission(RDA5807_adrr);
  Wire.write(reg); 
  Wire_write16(auRDA5807_Reg[reg]);
  return Wire.endTransmission();
}

//---------------------------------------------
// RDA5807 Read Special Data Registers as Word
//---------------------------------------------
void RDA5807_ReadW(int cnt)
{
   Wire.beginTransmission(RDA5807_adrr);            // Device 0x11 for random access
   Wire.write(0x0C);                                // Start at Register 0x0C
   Wire.endTransmission(0);                         // restart condition
   Wire.requestFrom(RDA5807_adrr,2*cnt, 1);         // Retransmit device address with READ, followed by 8 bytes
   for (int i = 0; i < cnt; i++)                    // Loop for Read data    
   {auRDS[i] = 256 * Wire.read() + Wire.read();}    // Read Data into Array of Unsigned Ints
   Wire.endTransmission();                  
} 

//----------------------------------------
// RDA5807 Read and Show all Status Registers
//----------------------------------------
int RDA5807_ReadStatus()
{
  int Chnumber = -1;
  unsigned int aubuf[8];
  memset (aubuf, 0, 8);
  Serial.println(F("Info Status RDA5807:"));
  Serial.println(F("Reg | 0Ah | 0Bh | 0Ch | 0Dh | 0Eh | 0Fh |"));
  Serial.print(F("    |"));
  Wire.requestFrom(RDA5807_adrs, 12); 
  for (int i = 0; i < 6; i++)  { aubuf[i] = 256 * Wire.read () + Wire.read(); }
  Wire.endTransmission();
  for (int i = 0; i < 6; i++)  { Serial_print16h(aubuf[i]); Serial.print("|"); }
  Serial.println();
  Chnumber = (aubuf[0] & 0x03FF); 
  Freq_tuned = Freq_lower_bandwith + Chnumber * 0.10;
  rssi = aubuf[1] >> 10;
  Serial.print(F("RDS Data:    ")); if ((aubuf[0] & 0x8000)==0){ Serial.println(F("NO"));}           else {Serial.println(F("NEW data"));}
  Serial.print(F("SEEK Ready:  ")); if ((aubuf[0] & 0x4000)==0){ Serial.println(F("no"));}           else {Serial.println(F("OK"));}
  Serial.print(F("SEEK Fail:   ")); if ((aubuf[0] & 0x2000)==0){ Serial.println(F("no, Succces!"));} else {Serial.println(F("FAILED"));}
  Serial.print(F("RDS Sync:    ")); if ((aubuf[0] & 0x1000)==0){ Serial.println(F("no"));}           else {Serial.println(F("OK"));}
  Serial.print(F("RDS Block:   ")); if ((aubuf[0] & 0x0800)==0){ Serial.println(F("no"));}           else {Serial.println(F("Block E"));}
  Serial.print(F("Stationmode: ")); if ((aubuf[0] & 0x0400)==0){ Serial.println(F("Mono  "));}       else {Serial.println(F("Stereo"));} 
  Serial.print(F("Channel Nr:  ")); Serial.print(Chnumber); Serial.print(F(" = "));
  Serial.print(Freq_tuned);         Serial.println(F(" MHz"));
  Serial.print(F("SignalLevel: ")); Serial.println(rssi);
  return 0;
}

//----------------------------------------
// RDA5807 Report all available Stations
//----------------------------------------
int RDA5807_Report()
{
  Freq_tuned = Freq_lower_bandwith;
  int cnt_stations = 0;
  int cnt_stereo = 0;
  int cnt_rds = 0;
  int Rssi = 0;
//auRDA5807_Reg[3] =  0x10;  //Set channelnumber 0
//RDA5807_setFreq(87.50);
  Serial.println(F("Sender Report:"));
   for(int Chnumber = 0; Chnumber <= 210; Chnumber++)
  {
    auRDA5807_Reg[3] = 64 * Chnumber + 0x10; 
    Wire.beginTransmission(RDA5807_adrs);
    Wire_write16(0xD009);
    Wire_write16(auRDA5807_Reg[3]);
    Wire.endTransmission();
    delay(300);                           //give de radio some time to settle
    RDA5807_Status();
  }
}

//----------------------------------------
// RDA5807 Show Status
//----------------------------------------
void RDA5807_Status(void)
{
  int Chnumber;
  Wire.requestFrom (RDA5807_adrs, 16); 
  for (int i = 0; i < 8; i++) { auRDA5807_Reg[0x0A + i] = 256 * Wire.read () + Wire.read(); }
  Wire.endTransmission();
  Chnumber = auRDA5807_Reg[0x0A] & 0x03FF;
  rssi = auRDA5807_Reg[0x0B] >> 10;
  Freq_tuned = Freq_lower_bandwith + (Chnumber ) * 0.1;
//  if ( (auRDA5807_Reg[0x0A] & 0x8000) && (auRDA5807_Reg[0x0A] & 0x0400)        )  // if RDS and stereo in station
 if ((auRDA5807_Reg[0x0A] & 0x0400) )                    // if Stereo in station
  {
   if (Freq_tuned <= 99.99){Serial.print(" ");}
   Serial.print(Freq_tuned);
   Serial.print(F(" MHz"));
   Serial.print(F(" Ch=")); if (Chnumber < 10){Serial.print(F(" "));} if (Chnumber < 100) { Serial.print(F(" ")); } Serial.print(Chnumber);
   Serial.print(F(" PI=")); Serial_printuih(auRDA5807_Reg[0x0C]);             // RDS Block A contains Station ID
   if ((auRDA5807_Reg[0x0A] & 0x0400) == 0)    { Serial.print(F(" Mono  "));} else { Serial.print(F(" Stereo"));}
   if ((auRDA5807_Reg[0x0A] & 0x8000) == 0)    { Serial.print(F(" ---"));   } else { Serial.print(F(" RDS"));   }
   Serial.print(F(" Sig= "));   if (rssi < 10) { Serial.print(F(" "));      } else  Serial.print(rssi);  Serial.print(F(" "));
   for(int i = 0; i < rssi - 5; i++) { Serial.print(F("*")); }
   Serial.println();
  }
}

//----------------------------------------
// RDA5807 Show Status
//----------------------------------------
void RDA5807_Get_RSSI()
{
  Wire.requestFrom (RDA5807_adrs, 16); 
  for (int i = 0; i < 8; i++) { auRDA5807_Reg[0x0A + i] = 256 * Wire.read () + Wire.read(); }
  Wire.endTransmission();
  rssi = auRDA5807_Reg[0x0B] >> 10;
}

//----------------------------------------
// SerialPrint 16Bit Number in HEX as hhll
//----------------------------------------
void Serial_printuih(unsigned int val)
{
  if (val < 0xF)   Serial.print(F("0"));                 // if less 2 Digit
  if (val < 0xFF)  Serial.print(F("0"));                 // if less 3 Digit
  if (val < 0xFFF) Serial.print(F("0"));                 // if less 4 Digit
  Serial.print(val,HEX);
  Serial.print(F(" "));
}

//----------------------------------------
// RDA5807 Radio Data System Dump Infos
//----------------------------------------
int RDA5807_RDS_Dump()
{
  Serial.println(" PI |GTxx|Asci");
  while(Serial.available()==0)
  {
    RDA5807_ReadW(4);                           // Read RDS-Data as 4 Word to Array               
    if((auRDS[1] & 0xF000)==0x2000)
    { 
//      Serial_printuih(auRDS[0]);                 // Block A  PI
//      Serial_printuih(auRDS[1]);                 // Block B  GT(5Bit)T(1Bit) PTY(5Bit)POS(5)Bit
//      Serial_printuih(auRDS[2]);
//      Serial_printuih(auRDS[3]);
//      int x = 16 + 4*(auRDS[1] & 0x000F);        
      for (int i=2;i<4;i++)  
      { 
        Serial.write(auRDS[i]>>8);               // Block C/D Ascii Code
        Serial.write(auRDS[i]&0xFF);             // 2 * 2 Byte
      }
    }
    if ((auRDS[1] & 0xF000)==0x4000)
    {
      int i_hh =(16*(auRDS[2] & 0x0001)+((auRDS[3] & 0xF000)>>12));
      int i_mm =(auRDS[3] & 0x0FC0)>>6;
      int i_ofs=(auRDS[3] & 0x003F);
      i_hh=i_hh+(i_ofs/2);
      if (i_hh <10){Serial.write(' ');} Serial.print(i_hh);  Serial.write(':');
      if (i_mm <10){Serial.write('0');} Serial.print(i_mm);  Serial.write(' ');
    }
   if ((auRDS[1]& 0xF000)==0x400)
   { 
    Serial.print(F("RDS CT: ")); for (int i=0;i<4;i++){ Serial_print16h(auRDS[i]); Serial.write(' | ');}  Serial.println();
    }
    delay(80);
    Serial.println();
  }
  return  0;
}

//----------------------------------------
// RDA5807 Radio Data System Dump Infos
//----------------------------------------
int RDA5807_RDS_DumpCT()
{
  int          i_gt,i_gab,i_pty,i_t,i_pos,i_hh,i_mm,i_ofs;
  RDA5807_Status();
  Serial.println(F(" PI |GTxx|Asci      GT  T PTY POS HH:mm Offset"));
  while(Serial.available()==0)
  {
    RDA5807_ReadW(4);                              // Read RDS-Data as 4 Word to Array
    i_gt = auRDS[1] >>12;
    if ((auRDS[1] & 0x0800)==0){i_gab='A';} else {i_gab='B';}
    i_t  =(auRDS[1] & 0x0400)>10;
    i_pty=(auRDS[1] & 0x03FF)>>5;
    i_pos=(auRDS[1] & 0x000F);
    i_hh =(16*(auRDS[2] & 0x0001)+((auRDS[3] & 0xF000)>>12));
    i_mm =(auRDS[3] & 0x0FC0)>>6;
    i_ofs=(auRDS[3] & 0x003F);
    i_hh=i_hh+(i_ofs/2);
    if (i_gt==4)
    {
    Serial_printuih(auRDS[0]);       // Block A  PI
    Serial_printuih(auRDS[1]);       // Block B  GT(4Bit) A/B(1Bit) T(1Bit) PTY(5Bit)POS(5)Bit
    Serial_printuih(auRDS[2]);
    Serial_printuih(auRDS[3]);
    if (i_gt <10){Serial.write(' ');} Serial.print(i_gt);  Serial.write(i_gab); Serial.write(' ');
    if (i_t  <10){Serial.write(' ');} Serial.print(i_t);   Serial.write(' ');
    if (i_pty<10){Serial.write(' ');} Serial.print(i_pty); Serial.print("  ");
    if (i_pos<10){Serial.write(' ');} Serial.print(i_pos); Serial.write(" ");
    if (i_hh <10){Serial.write(' ');} Serial.print(i_hh);  Serial.write(':');
    if (i_mm <10){Serial.write('0');} Serial.print(i_mm);  Serial.write(' ');
    Serial.print(i_ofs);
    Serial.println();
    }
    delay(80);
  }
  return  0;
}
//                                ------------------ End FM-radio
                          #endif FMRADIOMOD


                         #ifdef MAX7219_8DIGIT
// The setChar(addr,digit,value,dp)-function accepts a value of type char for the 
// in the range of a 7-bit ASCII encoding. Since the recognizable patterns are limited, 
// most of the defined characters will print the <SPACE>-char. 
// But there are quite a few characters that make sense on a 7-segment display.
// Display a character on a 7-Segment display.
// Params:
// *   addr  address of the display (0 - 7)
// *   digit the position of the character on the display (0..7)
// *   value the character to be displayed.
// *   dp    sets the decimal point.
                    
//----------------------------------------
// MAX7219_8DIGIT  
//----------------------------------------
 void PrintToMAX7219_8digit(char text[],byte Posdot)
{
 text[8] = 0;                                            // Terminate a too long string
 for(byte n=0 ; n<8; n++)  
   lc.setChar(0,7-n, text[n], (n==(7-Posdot)?1:0));     // No of Posdot decimal positions
}
                         #endif MAX7219_8DIGIT

                         #ifdef DS1820 
//----------------------------------------
// Temperature sensor DS1820 
//----------------------------------------
                      
void setupDS1820(void) 
{
  Tempsensors.begin();                                   // Start up the library
  numberOfDevices = Tempsensors.getDeviceCount();        // Grab a count of devices on the wire  
  Serial.print(F("Found "));   Serial.print(numberOfDevices, DEC);  Serial.println(F(" devices."));
  Serial.print(F("Parasite power is: "));                   // report parasite power requirements
  if (Tempsensors.isParasitePowerMode()) Serial.println("ON");
  else                                   Serial.println("OFF");
  for(int i=0;i<numberOfDevices; i++)                    // Loop through each device, print out address
  {
   if(Tempsensors.getAddress(tempDeviceAddress, i))      // Search the wire for address
    {
    Serial.print(F("Found device ")); Serial.print(i, DEC); Serial.print(F(" with address: ")); printAddress(tempDeviceAddress);  Serial.println();    
    Serial.print(F("Setting resolution to "));              Serial.println(TEMPERATURE_PRECISION, DEC);
    Tempsensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);   
    delay(100);      
    Serial.print(F("Resolution actually set to: "));        Serial.print(Tempsensors.getResolution(tempDeviceAddress), DEC);     Serial.println();
    }
   else 
    {
      Serial.print(F("Found ghost device at "));            Serial.print(i, DEC);    
      Serial.println(F(" but could not detect address. Check power and cabling"));
    }
  }
}

//----------------------------------------
// DS1820 Temperature sensor DS1820 read
//----------------------------------------
 void DS1820read()
{
  char text[8];
  Tempsensors.requestTemperatures();                     // Send the command to get temperatures   
  for(int i=0;i<numberOfDevices; i++)                    // Loop through each device, print out temperature data
  { 
    if(Tempsensors.getAddress(tempDeviceAddress, i))     // Search the wire for address
    {
    Serial.print(F("Temperature for device: "));   Serial.print(i,DEC);           // Output the device ID
    Serial.print(F("  Temp C: "));                 Serial.println(Tempsensors.getTempC(tempDeviceAddress));
 //   text[2] = 'c'; text[3] = 32; 
                         #ifdef MAX7219_8DIGIT  
    sprintf(text,"%7ldC", (long) 100000 * get3231Temp() + (long) (10 * Tempsensors.getTempC(tempDeviceAddress)) );
    PrintToMAX7219_8digit(text,2);
                         #endif MAX7219_8DIGIT
    }                                                    //else ghost device! Check your power requirements and cabling 
  }
}


//----------------------------------------
// DS1820 Temperature sensor DS1820 printAddress
//----------------------------------------
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

                                       #endif DS1820 

                     
//********************************************************************************************

