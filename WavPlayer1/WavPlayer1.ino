/*
  WavPlayer1

  Play 8 bit 16KBit WAV files

  The circuit:
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10

  Audio out on pin D3. Use a MOSFET to make it capable of driving a loudspeaker
  
  created   Aug 2020
  by Jon
*/
#include <SPI.h>
#include "SdFat.h"

#define RIFF_ID_POS 0
#define WAVEfmt_ID_POS 8
#define DATA_ID_POS 0x24
#define DATASIZE_POS 0x28
#define DATA_BUFFER_SIZE 100

inline void fastWriteD3(int value)
{
  if (value) PORTD |= 1 << 3;
  else PORTD &= ~(1 << 3);
}

const int chipSelect = SS;
const uint8_t RIFF_ID[4] = {0x52, 0x49, 0x46, 0x46};
const uint8_t WAVEfmt_ID[7] = {0x57, 0x41, 0x56, 0x45, 0x66, 0x6d, 0x74};
const uint8_t DATA_ID[4] = {0x64, 0x61, 0x74, 0x61};

SdFat SD;
File dir;
uint8_t dataBuffer[DATA_BUFFER_SIZE];
uint32_t dataSize;
int dataFirst;
int dataLast;
bool paused = false;
bool playNext = false;
bool noData = false;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial)
    ; // wait for serial port to connect. Needed for native USB port only

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect, SD_SCK_MHZ(2)))
  {
    Serial.println("initialization failed!");
    Serial.println(SS);
    while (1);
  }
  Serial.println("initialization done.");
  openDir("/");

  setupPWMTimer();
  setup16KHzTimer();
}

ISR(TIMER1_COMPA_vect)
{
  if (dataFirst != dataLast)
  {
    uint8_t val = dataBuffer[dataFirst++];
    if (dataFirst == DATA_BUFFER_SIZE)
    {
      dataFirst = 0;
    }
    updatePWMTimer(val);
  }
  else if (!paused)
  {
    noData = true;
  }
}

// Set timer1 interrupt at 16kHz
void setup16KHzTimer()
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;               //initialize counter value to 0
  OCR1A = 1000;             // = (16*10^6) / (1 * 16000) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);   // turn on CTC mode
  TCCR1B |= (1 << CS10);    // Set CS10 bit for no prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  sei();
}

// Using Timer 2 B = pin D3, 62.5KHz i.e. no prescaler
void setupPWMTimer()
{
  pinMode(3, OUTPUT);
  //pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2B = 0x80;
}

void updatePWMTimer(uint8_t val)
{
  if (val == 0)
  {
    TCCR2A &= ~_BV(COM2B1);
    fastWriteD3(0);
  }
  else if (val == 0xff)
  {
    TCCR2A &= ~_BV(COM2B1);
    fastWriteD3(1);
  }
  else
  {
    TCCR2A |= _BV(COM2B1);
    OCR2B = val;
  }
}

void loop()
{
  File file = getNextWavFile();
  printFileDetails(file);

  bool valid = file.read(dataBuffer, 0x2d) == 0x2d;
  if (valid)
  {
    for (int i = 0; i < 0x30; i++) { Serial.print(dataBuffer[i], HEX); Serial.print(" "); }      // Test code
    if (memcmp(&dataBuffer[RIFF_ID_POS], RIFF_ID, sizeof(RIFF_ID)) ||
        memcmp(&dataBuffer[WAVEfmt_ID_POS], WAVEfmt_ID, sizeof(WAVEfmt_ID)) ||
        memcmp(&dataBuffer[DATA_ID_POS], DATA_ID, sizeof(DATA_ID)))
    {
      Serial.println("Not a valid WAV file");
      valid = false;
    }
  }
  if (valid)
  {
    readWavFile(file);
  }

  file.close();
  delay(1000);
}

File getNextWavFile()
{
  while (1)
  {
    File file = getNextFile();
    if (!file)
    {
      Serial.println("End of playlist");
      dir.rewindDirectory();
      delay(1000);
    }
    else if (file.isDirectory())
    {
      // ignore directories
      file.close();
    }
    else
    {
      return file;
    }
  }
}

void printFileDetails(File file)
{
  char longName[129];
  file.getName(longName, sizeof(longName)-1);
  Serial.print(longName);
  Serial.print("\t");
  Serial.println(file.size(), DEC);
}

// This method takes a long time because it reads the entire file. Note the call to processSerialCommands() in the middle
// TODO: Refactor this to make it a bit nicer.
void readWavFile(File file)
{
  dataSize = *(uint32_t*)(&dataBuffer[DATASIZE_POS]);
  dataFirst = 0;
  dataLast = 0;
  Serial.println(dataSize);
  
  uint32_t dataPtr = 0;
  uint32_t lastCheckPos = 0;
  while (dataPtr < dataSize)
  {
    int dataInBuf = dataLast >= dataFirst ? dataLast - dataFirst : DATA_BUFFER_SIZE - (dataFirst - dataLast);
    int toRead = paused ? 0 : (int)min((uint32_t)(DATA_BUFFER_SIZE - 1 - dataInBuf), dataSize - dataPtr);
    int dataLastPartial = dataLast;
    if (dataLast + toRead > DATA_BUFFER_SIZE)
    {
      int partialRead = DATA_BUFFER_SIZE - dataLast;
      file.read(&dataBuffer[dataLast], partialRead);
      dataPtr += partialRead;
      toRead -= partialRead;
      dataLastPartial = 0;
    }
    if (toRead > 0)
    {
      file.read(&dataBuffer[dataLastPartial], toRead);
      dataLastPartial += toRead;
      dataLast = dataLastPartial;
      dataPtr += toRead;
    }
  
    if (paused || dataPtr - lastCheckPos > 16000)
    {
      processSerialCommands();
      lastCheckPos = dataPtr;
      if (playNext)
      {
        playNext = false;
        break;
      }
    }

    if (noData)
    {
      Serial.println("Warning: data buffer empty");
      noData = false;
    }
  }
}

void processSerialCommands()
{
  if (Serial.available())
  {
    String command = Serial.readString();
    if (command.startsWith("next"))
    {
      playNext = true;
      Serial.println("going to next file");
    }
    else if (command.startsWith("pause"))
    {
      paused = true;
      Serial.println("Paused...");
    }
    else if (command.startsWith("play"))
    {
      paused = false;
      Serial.println("Playing...");
    }
  }
}

void openDir(char* path)
{
  dir = SD.open(path);
}

File getNextFile()
{
  return dir.openNextFile();
}
