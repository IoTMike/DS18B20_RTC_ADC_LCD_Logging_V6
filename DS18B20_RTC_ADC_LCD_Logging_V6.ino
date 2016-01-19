/* 03/18/2014  ML  Combined DS1307 RTC routine & LCD to test
   Works with both the green and red 1602 LCD modules, different addresses/pins
   between the different I2C modules (I think), also one's Positive (green module: 
   bright background) and the other Negative (Sparkfun: white on black background).
   
   10/18/2014  ML  Used this to show the Temp and Time, and setup for the SD Card logging.
    Output on LCD:
      F:77.56 C:25.31
      20141019 175256
    - Haven't implemented the SD card logging
    - Would like to find a better way to handle the data...converting to string or
      rounding the float.
    - Unable to drive off the battery pack through the 5v port, but works through the
      USB port...regulator causing problems?
   
   10/19/2014  ML  Forking to V2 to use the RTClib library instead of calling I2C functions.
   10/20/2014  ML  Got SD to work...at times!
   02/18/2015  ML  Got SD working!  Pins were not correct. Several formatting adds.
   03/12/2015  ML  Loaded into Nano v3.0 board and works great!
   01/16/2016  ML  V5 Added ADC reading
   01/16/2016  ML  Moved from proto to a single breadboard.  Added LM4040 to Nano.
   01/18/2016  ML  V6 Moved DS18B20 Address lookup to setup()
                   Added error routine, didn't test yet
                   Running short on variable memory
                    Sketch uses 23,464 bytes (76%)
                    Global variables use 1,702 bytes (83%) of dynamic memory, 
                    leaving 346 bytes for local variables. Maximum is 2,048 bytes.
                    Low memory available, stability problems may occur.
                   The SD Card needs local memory, so if insufficient, won't write to it.
   */

/*  TO DO LIST:
 *   - Add ULED display, remove LCD (or update LCD library)
 *     - Move LCD/ULED to function
 *   - Convert to new Dallas Temp Sensor library
 *     - Consider fixed (addr) since only one sensor will be used;
 *   - Move SD to function
 *   - Change SD filename to date as filename
 *   - Add external time delay input
 */

/* Pinout
 * DS18B20 
 *   Pin 1 - Gnd
 *   Pin 2 - GPIO Pin 8, 4.7k pull-up to 5V (White)
 *   Pin 3 - Vcc (Optional - not used, parasitic mode)
 * LCD via I2C
 *  SDA - Pin A4 (Orange)
 *  SCL - Pin A5 (Yellow)
 *  Gnd - Gnd
 *  Vcc - Vcc +5v
 * RTC via I2C
 *  SDA - Pin A4
 *  SCL - Pin A5
 *  Gnd - Gnd
 *  Vcc - Vcc +5v
 * SD card attached to SPI bus as follows:
 *  CS   - pin 10  (Blue)
 *  MOSI - pin 11  (Yellow)
 *  MISO - pin 12  (Orange)
 *  CLK  - pin 13  (Green) *** Note that this is also the on-board LED pin!!!
 * Vref
 *  LM4040 Anode - GND
 *  LM4040 Cathode - VREF
 *  560R resistor - btwn VREF & 5V Vcc
 * Analog in
 *  Can be specified in the the variable definitions or in the module call
 *  
 * 
 * FYI...Arduino Uno Datatypes
 *  
 *  Byte             0 to 255 (2^8 - 1)
 *  Word             0 to 65,535
 *  Short            -32,768 to 32,767 
 *  Int              -32,768 to 32,767 
 *  Unsigned Int     0 to 65,535 (2^16 - 1)
 *  Long             -2,147,483,648 to 2,147,483,647
 *  Unsigned Long    0 to 4,294,967,295 (2^32 - 1)
 *  Float            -3.4028235E+38 to 3.4028235E+38
*/

// $$$$$$$$$$$$$$$  Start Declarations   $$$$$$$$$$$$$$$$$//

// -------------( Import Libraries )------------- //

// String functions
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Load i2c library
#include <Wire.h>

// Load OneWire for the Dallas DS18B20 Temp Sensor  !!! Old Library !!!
#include <OneWire.h>
 // TO DO LIST:  update to new library
 // http://www.milesburton.com/?title=Dallas_Temperature_Control_Library

// Load I2C version of the LCD library !!! Old Library !!!
#include <LiquidCrystal_I2C.h>
 // If continuing to use the LCD, need to update to the newer libs

// Load DS1307 RTC library (https://github.com/adafruit/RTClib.git)
#include <RTClib.h>

// Load SD library
#include <SD.h>


// -------------( Declare Constants )------------- //
// LED
const byte ledPin = 13;

// Time interval
const unsigned long timeDelay = 5000;  // in miliseconds
                    // 30 sec = 30000
                    //  1 min = 60000
                    //  5 min = 300000
                    // 10 min = 600000
// If I do make this switch programmable, remember to remove "const"

// Error codes:
byte errorCode  = 0;  // No error
              //  1 - DS18B20 Error
              //  2 - SD Card Error
              //  3 - 
              
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const byte chipSelect = 10;

// DS18B20 Temperature Variables
  byte type_s;
  byte addr[8];
  float celsius, fahrenheit;
  int   cInt, fInt;
  float temp = 0.00;

// May use this for SD logging, available from the RTClib
long unixTime = 0;    // now.unixtime()
// Lest we forget, seconds from midnight 1/1/1970!

// SD Card
const char fileName[9]  = "LOG6.TXT";

//** ADC Voltage Variables
// Using a 4.096 vRef LM4040AIZ set ref to 4096/1023
const float refVoltage = 4.004;  // using the LM4040AIZ on VREF
//const float refVoltage = 5.0;  // using the LM4040AIZ on VREF
// Can be changed here or in the call to the module
byte adcPin = 0;          // currently set to A0
unsigned int adcDelay = 1;        // delay btwn ADC reads
unsigned int adcSamples = 20;     // number of ADC samples to average
// Other variables:
unsigned int adcValue = 0;        // the returned sensor value (0-1023)
float adcVoltage = 0;    // sensor value * reference voltage
float volts = 0;         // x.xx volts
unsigned int mvolts = 0;          // xxxx millivolts

// -------------( Declare Objects )------------- //

// DS 18B20 Temp Sensor - Initialize OneWire for Pin 8
OneWire  ds(8);  // on pin 8 (a 4.7K resistor is necessary)

// Define DS1307
RTC_DS1307 rtc;

//** Global LCD Variables
// #define BACKLIGHT_PIN     13  // The i2c module I have uses pin 3
// Set the address and pins on the I2C chip used for LCD connections:
//                   addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// ^^^ works for the green 1602 LCD + I2C module
//LiquidCrystal_I2C lcd(0x20, 4, 5, 6, 0, 1, 2, 3, 7, NEGATIVE);  // For the green QC2004A
// ^^^ works for the red Sparkfun 1602 LCD + I2C module

// 01/16/2016 ML  Issues with Arduino 1.6.5 - 1.6.7: 
// "DS18B20_RTC_LCD_Logging_V5:119: error: 'POSITIVE' was not declared in this scope"
//  Getting this error with nothing connected, just doing a verify.  Will try this:
//LiquidCrystal_I2C lcd(0x27,20,4);
// set the LCD address to 0x27 for a 16 chars and 2 line display
// ^^^ which seems to pass verify, not tested yet on device.
// 01/17/2017 ML  Found and reinstalled old LiquidCrystal library...too many issues!

// $$$$$$$$$$$$$$$  End Declarations   $$$$$$$$$$$$$$$$$//

// $SSSS$$$$$$$$$$$$$$$  setup()   $$$$$$$$$$$$$$$$$$$$$//
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // Wait for serial port to connect.
    }

  //DEBUG Serial.println("void setup() starting...");
 
  // Wire library setup for I2C
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif

  // Initialize RTC DS1307
  Serial.println("Initializing RTC DS1307...");
  rtc.begin();

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    //    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2016 at 3am you would call:
    // rtc.adjust(DateTime(2016, 1, 21, 3, 0, 0));
  }

  // Initialize LCD
  Serial.println("Initializing LCD...");
  lcd.begin(16,2);        // initialize the lcd
  lcd.clear();
  lcd.backlight();        // finish with backlight on

  // Initialize ADC
  analogReference(EXTERNAL); // use AREF for reference voltage
                             // prototype Nano has LM4040 soldered to board
  //analogReference(DEFAULT); // use Vcc for reference voltage
  //analogReference(INTERNAL); // use 1.1V internal reference voltage

  // Initialize SD Card
  Serial.println("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  // chipSelect = 10 previously set
  pinMode(chipSelect, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    errorCode = 2;
  }

  Serial.println("...SD card initialized.");

  /*----------- Initilize DS18B20 -------------*/
  
  Serial.println("Initializing DS18B20...");
  // This function is required
  if ( !ds.search(addr))
    {
    Serial.println(" ERROR: No more addresses.");
    // Serial.println();
    ds.reset_search();
    delay(250);
    errorCode = 1;
    //return;      // this is ok when in loop(), freaks when in module
    //return -1.0;    // return a negative temp
    }

   Serial.print(" DS18B20 Address: ");
   for (byte j=0; j<8; j++) {
    Serial.write(' ');
    Serial.print(addr[j], HEX);
   }
   Serial.println("");

  if (OneWire::crc8(addr, 7) != addr[7])
    {
    Serial.println(" ERROR: CRC is not valid!");
    lcd.print("CRC is not valid");
    errorCode = 1;
    // return;  // this is ok when in loop(), freaks when in module
    // return -2.0;    // return a negative temp
    }

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println(" ERROR: Device is not a DS18x20 family device.");
     errorCode = 1;
      //return -3.0;
  } 

  Serial.println("Setup complete.");

} //$$$$$$$$$$$$$$$  end setup()   $$$$$$$$$$$$$$$$$//

// $$$$$$$$$$$$$$$$  loop()    $$$$$$$$$$$$$$$$$$$$$//
void loop() {  
  
  Serial.println("");
  
  switch (errorCode) {
    case 0:
      break;
    case 1:
      digitalWrite(ledPin, HIGH);
      Serial.println("ERROR - DS18B20");
      delay(5000);
      digitalWrite(ledPin,LOW);
      delay(1000);
      digitalWrite(ledPin, HIGH);
      delay(1000);
      digitalWrite(ledPin, LOW);
      break;
    case 2:
      digitalWrite(ledPin, HIGH);
      Serial.println("ERROR - SD Card");
      delay(5000);
      digitalWrite(ledPin,LOW);
      delay(1000);
      digitalWrite(ledPin, HIGH);
      delay(1000);
      digitalWrite(ledPin, LOW);
      delay(1000);
      digitalWrite(ledPin, HIGH);
      delay(1000);
      digitalWrite(ledPin, LOW);
      break;
    default:
      digitalWrite(ledPin, HIGH);
      Serial.println("ERROR - Not defined");
      delay(5000);
      digitalWrite(ledPin,LOW);
      delay(1000);
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      break;
    }

  //    Serial.println("Getting Temp from DS 18B20...");
  /*-------------( DS18B20 Temp Section )-------------*/
  celsius = getTemp();
  //if (celsius == -1.0 ) Serial.println("DS18B20 Temp Sensor NOT FOUND.");
  //if (celsius == -2.0 ) Serial.println("DS18B20 Temp Sensor has BAD CRC.");
  
  fahrenheit = celsius * 1.8 + 32.0;
  fInt = ((int)(fahrenheit * 10 + .5) / 10.0);
  cInt = ((int)(celsius * 10 + .5) / 10.0);
  
  /* Testing, works!
  Serial.print("C = ");
  Serial.print(celsius);
  Serial.print("\t");
  Serial.println(cInt);
  Serial.print("F = ");
  Serial.print(fahrenheit);
  Serial.print("\t");
  Serial.println(fInt);
  */
  serialShowTemp(celsius);
  lcdShowTemp(celsius);
  

  /*-------------( RTC DS1307 Section )-------------*/
  
  Serial.println("Getting Time from DS1307...");

  DateTime now = rtc.now();

  lcdShowTime();

  //if (second < 10)
  //  lcd.print("0");
  //  lcd.print(now.second(), DEC);
  

  // This converts RTC info to a string to be used
  /* But for some reason, when this is enabled, writing to the SD Fails!!!
  // Convert YYYY MM DD to YYYMMDD string
  char yearStr[4];
  char monthStr[2];
  char dayStr[2];
  char dateStr[10];
  sprintf(yearStr, "%d", now.year());
  strcpy(dateStr, yearStr);
  if(now.month() < 10) {
    sprintf(monthStr, "0%d", now.month());}
    else {
    sprintf(monthStr, "%d", now.month());}
  strcat(dateStr, monthStr);
  if(now.day() < 10) {
    sprintf(dayStr, "0%d", now.day()); }
    else {
    sprintf(dayStr, "%d", now.day()); }
  strcat(dateStr, dayStr);

  // Convert HH MM SS to HHMMSS string
  char hourStr[2];
  char minStr[2];
  char secStr[2];
  char timeStr[8];  // 6 without second, 8 with
  if(now.hour() < 10) {
    sprintf(hourStr, "0%d", now.hour()); }
    else {
    sprintf(hourStr, "%d", now.hour()); }
  strcpy(timeStr, hourStr);
  if(now.minute() < 10) {
    sprintf(minStr, "0%d", now.minute()); }
    else {
    sprintf(minStr, "%d", now.minute()); }
  strcat(timeStr, minStr);
  if(now.second() < 10) {
    sprintf(secStr, "0%d", now.second()); }
    else {
    sprintf(secStr, "%d", now.second()); }
  strcat(timeStr, secStr);
  
  Serial.print(dateStr);
  Serial.print("\t");
  Serial.println(timeStr);
  Serial.println();
  */

  /*-------------( ADC Section )-------------*/

  // format sampleADC(adc, adcSamples, adcDelay)
  //  Note that these are set above, can be passed too!
  adcValue = sampleADC(adcPin, adcSamples, adcDelay);
  
  adcVoltage = adcValue*refVoltage;  // in mv (float)
  volts = adcVoltage/1000.0;         // (float)
  mvolts = adcVoltage;               // (int) <-- (float)
  
  serialShowVolts(mvolts,volts);


  /*-------------( SC Card Logging Section )-------------*/
  // Log data to SD Card
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // this opens the file and appends to the end of file
  // if the file does not exist, this will create a new file.
  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {  
    dataFile.print(now.unixtime());
    dataFile.print("\t");
    dataFile.print(now.year(), DEC);  // RTClib gives full 4-digit year
    if (now.month() < 10)
      dataFile.print("0");
    dataFile.print(now.month(), DEC);
    if (now.day() < 10)
      dataFile.print("0");
    dataFile.print(now.day(), DEC);
    dataFile.print("\t");
    if (now.hour() < 10)          // 24-hour
      dataFile.print("0");       // 24-hour
    dataFile.print(now.hour(), DEC);
    dataFile.print(":");
    if (now.minute() < 10)
      dataFile.print("0");
    dataFile.print(now.minute(), DEC);
    dataFile.print(":");
    if (now.second() < 10)
      dataFile.print("0");
    dataFile.print(now.second(), DEC);
    dataFile.print("\t");
    dataFile.print("C: ");
    dataFile.print(celsius);
    dataFile.print("\t");
    dataFile.print("F: ");
    dataFile.print(fahrenheit);
    dataFile.print("\t");
    //dataFile.print("A0 mv: ");
    dataFile.print(mvolts);
    dataFile.print("\t");
    //dataFile.print("A0 V: ");
    dataFile.print(volts);
    dataFile.println();

    Serial.print("Data written to:  ");
    Serial.println(fileName);
    Serial.print(now.unixtime());
    Serial.print("\t");
    Serial.print(now.year(), DEC);  // RTClib gives full 4-digit year
    if (now.month() < 10)
      Serial.print("0");
    Serial.print(now.month(), DEC);
    if (now.day() < 10)
      Serial.print("0");
    Serial.print(now.day(), DEC);
    Serial.print("\t");
    if (now.hour() < 10)          // 24-hour
      Serial.print("0");       // 24-hour
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    if (now.minute() < 10)
      Serial.print("0");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    if (now.second() < 10)
      Serial.print("0");
    Serial.print(now.second(), DEC);
    Serial.print("\t");
    Serial.print(celsius);
    Serial.print("\t");
    Serial.println(fahrenheit);
    Serial.print("\t");
    //Serial.print("A0 mv: ");
    Serial.print(mvolts);
    Serial.print("\t");
    //Serial.print("A0 V: ");
    Serial.print(volts);
    Serial.println();

    dataFile.close();
    Serial.println("Data file closed");
 
    // print to the serial port too:
  }  
  // if the file isn't open, pop up an error:
  else
  {
    Serial.println("error opening data file");
  } 

  delay(timeDelay);

}  //$$$$$$$$$$$$$$$  end loop()   $$$$$$$$$$$$$$$$$//


//$$$$$$$$$$$$$$$  FUNCTIONS   $$$$$$$$$$$$$$$$$//
/*------------- getTemp() -------------*/
float getTemp() {
  // Code for DS 18B20 Temp Sensor
  byte present = 0;
  byte data[12];
  float temp = 0.00;
  

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  // Troubleshooting - Diagnostics
  Serial.print("DS18B20 Data = ");
  Serial.print(present, HEX);
 
  for (byte i = 0; i < 9; i++)
    {
    // we need 9 bytes, this gets the temp data
    data[i] = ds.read();

    //Troubleshooting - Diagnostics
    Serial.print(data[i], HEX);
    Serial.print(" ");
    }
    Serial.println("");

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  
  if (type_s)
    {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10)
      {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else
      {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
  temp = (float)raw / 16.0;
  return temp;
}


/*------------- lcdShowTemp() -------------*/
void lcdShowTemp(float c) {
  // LCD display on first row 
  // Displays three digits "F: ff.f  C: cc.c"
  // Does rounding, but..
  //  Cheating: spacing over 100th place .ff, truncating .cc

  float f = c * 1.8 + 32.0;

  //int cInt, fInt;
  //fInt = ((int)(fahrenheit * 10 + .5) / 10.0);
  //cInt = ((int)(celsius * 10 + .5) / 10.0);

  lcd.setCursor(0,0);
  lcd.print("F:");
  //lcd.setCursor(3,0);
  //lcd.print(fInt);
  lcd.print(f);
  lcd.setCursor(8,0);
  lcd.print("C:");
  //lcd.setCursor(12,0);
  //lcd.print(cInt);
  lcd.print(c);
}  


/*------------- serialShowTemp() -------------*/
void serialShowTemp(float c) {
  
  int cInt = ((int)(c * 10 + .5) / 10.0);
  float f = c * 1.8 + 32.0;
  int fInt = ((int)(f * 10 + .5) / 10.0);

  Serial.print("C = ");
  Serial.print(c);
  Serial.print("\t");
  Serial.println(cInt);
  Serial.print("F = ");
  Serial.print(f);
  Serial.print("\t");
  Serial.println(fInt);
}


/*------------- lcdShowTime() -------------*/
void lcdShowTime() {
  // LCD with DS1307 RTC on Second row

  DateTime now = rtc.now();
  
  lcd.setCursor(0,2);        // Go to second line
  lcd.print(now.year(), DEC);  // RTClib gives full 4-digit year
  if (now.month() < 10)
    lcd.print("0");
    lcd.print(now.month(), DEC);
  if (now.day() < 10)
    lcd.print("0");
    lcd.print(now.day(), DEC);
    
  lcd.print(" ");

  if (now.hour() < 10)          // 24-hour
    lcd.print("0");       // 24-hour
    lcd.print(now.hour(), DEC);
  if (now.minute() < 10)
    lcd.print("0");
    lcd.print(now.minute(), DEC);
}


/*------------- sampleADC() -------------*/
// Returns averaged reading from ADC
int sampleADC(byte adc, unsigned int samples, unsigned int wait) {
  //digitalWrite(ledPin, HIGH);  //we're sampling
  int value = 0;
  unsigned int i;         // 65K iterations!
  unsigned long sum = 0;  // handle ~4M interations

  // Warmup ADC
  for (byte j=0; j < 10; j++) {
    value = analogRead(adc);
    delay(1);
  }
  
  for (i=0; i < samples ;i++) {
    value = analogRead(adc);
    delay(wait);        //wait x milliseconds
    sum += value;
  }
  value = sum / samples;
  //digitalWrite(ledPin, LOW);
  return value;
}


/*------------- serialShowVolts() -------------*/
void serialShowVolts (int mvolts,float volts) {
  Serial.print("Reading: ");
  Serial.print(mvolts);
  Serial.print(" mv    ");
  Serial.print(volts);
  Serial.println(" V");
}


