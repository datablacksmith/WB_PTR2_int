//WeldBuddy PTR
//A0: stud volts
//A1: Amp sensor Vout (sensor's pin 3)
//A2: Amp sensor Vref (sensor's pin 4)

#include <SPI.h>
#include <Wire.h>
//#include <Adafruit_GFXs.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include "SdFat.h"
#include "FreeStack.h"
#include <ADC.h>
#include <string.h>
#include <Time.h>
#include <SdFatUtil.h>
#include <BufferedWriter.h>
SdFatSdio sd;

//Voltage multiplier (for A0)
const float vmult = 16.7359854184137;

//Amperage multiplier (for A1 and A2)
const uint8_t amult = 480;

// Pin to record
const int SENSOR_PIN = A0;

// Pin with LED, which flashes whenever data is written to card, and does a
// slow blink when recording has stopped.
const int LED_PIN = 13;

// 16 KiB buffer.
const size_t BUF_DIM = 16384;

// Sampling rate
const uint32_t sampleIntervalMicros = 10000;
// 200 us interval = 5 kHz
// 10000 us = 100 Hz

// Use total of four buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;

// Number of data points per record
const uint8_t ADC_DIM = 1;

// Format for one data record
struct data_t {
  uint32_t time;
  uint32_t adc[ADC_DIM];
};
// Warning! The Teensy allocates memory in chunks of 4 bytes!
// sizeof(data_t) will always be a multiple of 4. For example, the following
// data record will have a size of 12 bytes, not 9:
// struct data_t {
//   uint32_t time; // 4 bytes
//   uint8_t  adc[5]; // 5 bytes
// }

File file;

// Number of data records in a block.
const uint16_t DATA_DIM = (BUF_DIM - 4) / sizeof(data_t);

//Compute fill so block size is BUF_DIM bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = BUF_DIM - 4 - DATA_DIM * sizeof(data_t);

// Format for one block of data
struct block_t {
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};

// Intialize all buffers
block_t block[BUFFER_BLOCK_COUNT];

// Initialize full queue
const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 1;

// Index of last queue location.
const uint8_t QUEUE_LAST = QUEUE_DIM - 1;

block_t* curBlock = 0;

block_t* emptyStack[BUFFER_BLOCK_COUNT];
uint8_t emptyTop;
uint8_t minTop;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead = 0;
uint8_t fullTail = 0;

uint32_t nextSampleMicros = 0;
bool fileIsClosing = false;
bool collectingData = false;
bool isSampling = false;
bool justSampled = false;

// Display: If using software SPI (the default case):
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
#error("Display: Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//-----------------------------------------------------------------------------
//                               SETUP
//-----------------------------------------------------------------------------

void setup()   {
  sd.begin();

  Serial.println("Deleting bin file...");
  sd.remove("TeensyDataLogger.bin");


  Serial.begin(9600);
  while (!Serial) {
  }
  pinMode(LED_PIN, OUTPUT); //from TeensyDataLogger
  // Put all the buffers on the empty stack. from TeensyDataLogger
  for (int i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    emptyStack[i] = &block[i - 1];
  }
  emptyTop = BUFFER_BLOCK_COUNT;

  if (!file.open("TeensyDataLogger.bin", O_RDWR | O_CREAT)) {
    error("Datalogger setup: failed to open .bin file");
  }
  Serial.print("Block size: ");
  Serial.println(BUF_DIM);
  Serial.print("Record size: ");
  Serial.println(sizeof(data_t));
  Serial.print("Records per block: ");
  Serial.println(DATA_DIM);
  Serial.print("Record bytes per block: ");
  Serial.println(DATA_DIM * sizeof(data_t));
  Serial.print("Fill bytes per block: ");
  Serial.println(FILL_DIM);
  Serial.println("Recording. Enter any key to stop.");
  delay(100);
  collectingData = true;
  nextSampleMicros = micros() + sampleIntervalMicros;

  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  if (timeStatus() != timeSet) {
    error("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  
  splashscreen(); //Startup splash screen
}

//need to add check for card. If no card, display message. Else proceed
//Display idle screen: time/date, battery state, “ready for test”

//-----------------------------------------------------------------------------
//                            MAIN LOOP
//-----------------------------------------------------------------------------
void loop() {
  // Write the block at the tail of the full queue to the SD card
  if (fullHead == fullTail) { // full queue is empty
    if (fileIsClosing) {
      file.close();
      Serial.println("Datalogger: File complete.");
      BINtoCSV();
      blinkForever();
    } else {
      yield(); // acquire data etc.
    }
  } else { // full queue not empty
    // write buffer at the tail of the full queue and return it to the top of
    // the empty stack.
    digitalWrite(LED_PIN, HIGH);
    block_t* pBlock = fullQueue[fullTail];
    fullTail = fullTail < QUEUE_LAST ? fullTail + 1 : 0;
    if ((int)BUF_DIM != file.write(pBlock, BUF_DIM)) {
      error("Datalogger: write failed");
    }
    emptyStack[emptyTop++] = pBlock;
    digitalWrite(LED_PIN, LOW);
  }

  fileIsClosing = Serial.available();
  //digitalClockDisplay(); //display clock in serial every second
  //delay(1000);
  //Idle loop: sense if amps>10, if yes, go to datalogger loop. If no, keep looping.
  //check ADC amps value, check VRef, save current time to use as string for filename, update time on display,
  //Main datalogger loop: sense amps, buffer, write buffer to card.
  //Continue til amps drop below 10A for 1s, then: end test, display “ending” screen, run
  //Display idle screen: last test time/date, average amps, average volts
}
//-----------------------------------------------------------------------------
//                            DATALOGGER
//-----------------------------------------------------------------------------
void datalogger() {

}
//-----------------------------------------------------------------------------
void yield() {
  // This does the data collection. It is called whenever the teensy is not
  // doing something else. The SdFat library will call this when it is waiting
  // for the SD card to do its thing, and the loop() function will call this
  // when there is nothing to be written to the SD card.

  if (!collectingData || isSampling)
    return;

  isSampling = true;

  // If file is closing, add the current buffer to the head of the full queue
  // and skip data collection.
  if (fileIsClosing) {
    if (curBlock != 0) {
      putCurrentBlock();
    }
    collectingData = false;
    return;
  }

  // If we don't have a buffer for data, get one from the top of the empty
  // stack.
  if (curBlock == 0) {
    curBlock = getEmptyBlock();
  }

  // If it's time, record one data sample.
  if (micros() >= nextSampleMicros) {
    if (justSampled) {
      error("rate too fast");
    }
    acquireData(&curBlock->data[curBlock->count++]);
    nextSampleMicros += sampleIntervalMicros;
    justSampled = true;
  } else {
    justSampled = false;
  }

  // If the current buffer is full, move it to the head of the full queue. We
  // will get a new buffer at the beginning of the next yield() call.
  if (curBlock->count == DATA_DIM) {
    putCurrentBlock();
  }

  isSampling = false;
}
//-----------------------------------------------------------------------------
block_t* getEmptyBlock() {
  /*
     Takes a block form the top of the empty stack and returns it
  */
  block_t* blk = 0;
  if (emptyTop > 0) { // if there is a buffer in the empty stack
    blk = emptyStack[--emptyTop];
    blk->count = 0;
  } else { // no buffers in empty stack
    error("All buffers in use");
  }
  return blk;
}
//-----------------------------------------------------------------------------
void putCurrentBlock() {
  /*
     Put the current block at the head of the queue to be written to card
  */
  fullQueue[fullHead] = curBlock;
  fullHead = fullHead < QUEUE_LAST ? fullHead + 1 : 0;
  curBlock = 0;
}

//-----------------------------------------------------------------------------
void acquireData(data_t* data) {
  data->time = micros();
  // for testing purposes, set this value to some fixed number, so you can
  // figure out the bin conversion
  //data->adc[0] = 789;
  data->adc[0] = analogRead(SENSOR_PIN);
}
//-----------------------------------------------------------------------------
void blinkForever() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}
//-----------------------------------------------------------------------------
//                            SPLASH SCREEN
//-----------------------------------------------------------------------------
void splashscreen() {
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
}
//-----------------------------------------------------------------------------
//                            ERROR HANDLING
//-----------------------------------------------------------------------------
void error(String msg) {
  Serial.print("ERROR: ");//need to change to display
  Serial.println(msg);//need to change to display
  blinkForever();
}
//-----------------------------------------------------------------------------
//                              RTC STUFF
//-----------------------------------------------------------------------------
void timeloop() {
  if (Serial.available()) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }
}
//------------------------------------------------------------------------------
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
//-----------------------------------------------------------------------------
//                            BIN to  CSV
//-----------------------------------------------------------------------------
void BINtoCSV() {
  uint8_t lastPct;
  uint32_t buf[2];
  uint32_t t = 0;
  uint32_t syncCluster = 0;
  SdFile binFile;
  SdFile textFile;
  BufferedWriter bw;
  if (!binFile.open("TeensyDataLogger.bin", O_READ)) {
    error("BINtoCSV: failed to open .bin");
  }
  // create a new CSV file
  char name[13];
  strcpy_P(name, PSTR("DATA000.CSV"));//need to change naming to be date/time
  for (uint8_t n = 0; n < 100; n++) {
    name[4] = '0' + n / 10;
    name[5] = '0' + n % 10;
    if (textFile.open(name, O_WRITE | O_CREAT | O_EXCL)) break;
  }
  if (!textFile.isOpen()) error("BINtoCSV failed to open textFile");
  Serial.println("BINtoCSV: Writing: "); //need to switch to print to screen
  Serial.println(name); //need to switch to print to screen
  bw.init(&textFile);
  while (binFile.read(buf, 8) == 8) {
    //uint16_t i;
    bw.putNum(buf[0]);  // put timestamp
    bw.putStr(",");  // put comma
    float data = buf[1];    // convert to float
    data *= vmult;   // multiply by voltage multiplier
    char stringBuf[12];  //
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
        Serial.print(pct, DEC); //need to switch to print to screen
        Serial.println('%'); //need to switch to print to screen
      }
    }
  }
  bw.writeBuf();
  textFile.close();
  Serial.println("BINtoCSV: Done"); //need to switch to print to screen
}
