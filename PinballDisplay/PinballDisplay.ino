#include "SPI.h"
#include <SdFat.h>                // SD card & FAT filesystem library

// Adafruit Stuff
#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_ILI9341.h>     // Hardware-specific library
#include <Adafruit_SPIFlash.h>    // SPI / QSPI flash library
#include <Adafruit_ImageReader.h> // Image-reading functions

// Use hardware SPI (on Uno/NANO, #13, #12, #11)
//TFT Pins
#define TFT_CS 10 // TFT select pin
#define TFT_DC  9 // TFT display/command pin
#define TFT_RST 8   // You have to add this

// SD Pins
#define SD_CS   2 // SD card select pin

char * const splashImageName = "splash2.bmp";

SdFat SD;         // SD card filesystem
Adafruit_ImageReader reader(SD);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  startSD();
  startGameDisplay();
}

void startSD() {
  if(!SD.begin(SD_CS)) { // ESP32 requires 25 MHz limit
    Serial.println(F("SD begin() failed"));
    for(;;); // Fatal error, do not continue
  }
  Serial.println("SD Loaded");
}

void startGameDisplay() {
  // Image wont display if screen is not started
  tft.begin();
  ImageReturnCode stat;
  // Pass in true, SD/tft share SPI
  stat = reader.drawBMP(splashImageName, tft, 0, 0, true);
  Serial.println("Image Loaded");
}


void loop() {
  // put your main code here, to run repeatedly:
}
