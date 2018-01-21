//Displaytest
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include "SdFat.h"
#include "FreeStack.h"
//#include "UserTypes.h"
#include <ADC.h>
#include <string.h>
#include <Time.h>
#include <SdFatUtil.h>
#include <BufferedWriter.h>
SdFatSdio sd;
//SdFile myFile;
// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
/* Uncomment this block to use hardware SPI
  #define OLED_DC     6
  #define OLED_CS     7
  #define OLED_RESET  8
  Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);
*/
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
void setup()   {
  sd.begin();
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  Serial.begin(115200);
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  //Startup splash screen
  //Check for card. If no card, display message. Else proceed
  //Display idle screen: time/date, battery state, “ready for test”
  // WeldBuddy splash screen
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Buddy Industries, LLC");
  display.setCursor(10, 20);
  display.setTextSize(2);
  display.println("WeldBuddy");
  display.setCursor(4, 40);
  display.setTextSize(1);
  display.println("Physics Test Rig 2.0");
  display.display();
  delay(5000);
  display.clearDisplay();
binaryToText();
}
void loop() {
  digitalClockDisplay();
  delay(1000);
  //Idle loop: sense if amps>10, if yes, go to datalogger loop. If no, keep looping.
  //check ADC amps value, check VRef, save current time to use as string for filename, update time on display, 
  //Main datalogger loop: sense amps, buffer, write buffer to card.
  //Continue til amps drop below 10A for 1s, then: end test, display “ending” screen, run
  
//BINtoCSV converter
  //binaryToCSV(filename);
 
 //Display current ss
  //Display idle screen: last test time/date, average amps, average volts
}
//.bin TO .csv
//------------------------------------------------------------------------------
// convert binary file to text file
void binaryToText() {
  uint8_t lastPct;
  uint32_t buf[2];
  uint32_t t = 0;
  uint32_t syncCluster = 0;
  SdFile binFile;
  SdFile textFile;
  BufferedWriter bw;
  if (!binFile.open("TeensyDataLogger.bin", O_READ)) {
    Serial.println("failed to open .bin");
  }
  // create a new binFile
  char name[13];
  strcpy_P(name, PSTR("DATA000.TXT"));
  for (uint8_t n = 0; n < 100; n++) {
    name[4] = '0' + n / 10;
    name[5] = '0' + n % 10;
    if (textFile.open(name, O_WRITE | O_CREAT | O_EXCL)) break;
  }
  if (!textFile.isOpen()) Serial.println("failed to open textFile");
  Serial.println("Writing: ");
  Serial.println(name);
  bw.init(&textFile);
  while (!Serial.available() && binFile.read(&buf, 8) == 8) {
    uint16_t i;
// put timestamp
bw.putNum(buf[0]);
bw.putStr(", ");
// convert to float
float data = buf[1];
// multiply
data *= 16.7359854184137; //voltage multiplier
char stringBuf[12];
String(data).toCharArray(stringBuf, 12);
bw.putStr(stringBuf);
bw.putCRLF();
   /* for (i = 0; i < 4; i++) {
      bw.putNum(buf);
      bw.putCRLF();
    }*/
    if (textFile.curCluster() != syncCluster) {
      bw.writeBuf();
      textFile.sync();
      syncCluster = textFile.curCluster();
    }
    if ((millis() - t) > 1000) {
      uint8_t pct = binFile.curPosition() / (binFile.fileSize() / 100);
      if (pct != lastPct) {
        t = millis();
        lastPct = pct;
        Serial.print(pct, DEC);
        Serial.println('%');
      }
    }
    if (Serial.available()) break;
  }
  bw.writeBuf();
  textFile.close();
  Serial.println("Done");
}
//------------------
//Serial Datalogger
//RTC
/*
   TimeRTC.pde
   example code illustrating Time library with Real Time Clock.
*/
void timeloop() {
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
}
void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message
unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}
void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

