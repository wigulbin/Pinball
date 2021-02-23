#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>
#include <LiquidCrystal.h>
#include "Adafruit_MCP23017.h"
#include <Servo.h>

#define NUM_LEDS 60
#define DATA_PIN 3
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

volatile boolean awakenByInterrupt = false;
byte arduinoInterrupt=1;

//Servo Pin
int servoPin = 5;

// LCD Pins
#define rs 7
#define en 8
#define d4 9
#define d5 10
#define d6 11
#define d7 12

// Pinball Input Pins
//#define laserPin 1
//MCP Pins
#define ramp1Pin 0
#define ramp2Pin 1
#define ramp3Pin 2
#define multiBallPin 3
#define target1Pin 4
#define popBumper1Pin 5
#define popTargetPin 6
int targetPins[]= {ramp1Pin, ramp2Pin, ramp3Pin, multiBallPin, target1Pin, popBumper1Pin, popTargetPin};
int inputPinCount = 7;

Servo myservo;
int servoPos = 0; 
int servoDelayTime = 3000;
int servoStartTime = 0;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
CRGB leds[NUM_LEDS];
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_MCP23017 mcp;
int score = 0;
int counter = 3;
int multiBallCounter = 0;
int targetCounter = 0;
int rampCounter = 0;
int popTargetCounter = 0;
int popBumperCounter = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
//  startSSD1306Screen()
  startMCP();
  startLED();

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Score:");
}

void loop() {
  // put your main code here, to run repeatedly:  
  // enable interrupts before going to sleep/wait
  // And we setup a callback for the arduino INT handler.
  attachInterrupt(arduinoInterrupt,intCallBack,RISING);
  
  // Simulate a deep sleep
  while(!awakenByInterrupt);
  // Or sleep the arduino, this lib is great, if you have it.
  //LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  
  // disable interrupts while handling them.
  detachInterrupt(arduinoInterrupt);
  
  if(awakenByInterrupt) handleInterrupt();
}

void checkBallDrain(){
}
void handleInterrupt(){
  uint8_t pin=mcp.getLastInterruptPin();
  uint8_t val=mcp.getLastInterruptPinValue();
  checkTarget(pin);

  while( ! (mcp.digitalRead(pin) ));
  // and clean queued INT signal
  cleanInterrupts();
}

void cleanInterrupts(){
  EIFR=0x01;
  awakenByInterrupt=false;
}  
void intCallBack(){
  awakenByInterrupt=true;
}

void checkTarget(int targetPin) {
  int targetHit = mcp.digitalRead(targetPin);
  if(targetHit > 0) {
    switch(targetPin){
      case target1Pin:
        updateScore(10);
        targetCounter++;
        flashLEDs();
        break;
      case popBumper1Pin:
        updateScore(3);
        popBumperCounter++;
        flashLEDs();
        break;
      case popTargetPin:
        updateScore(100);
        popTargetCounter++;
        flashLEDs();
        break;
        
      case multiBallPin:
        updateScore(50);
        multiBallCounter++;
        flashLEDs();
        break;
        
      case ramp1Pin:
        updateScore(20);
        rampCounter++;
        flashLEDs();
        break;
      case ramp2Pin:
        updateScore(20);
        rampCounter++;
        flashLEDs();
        break;
      case ramp3Pin:
        updateScore(20);
        rampCounter++;
        flashLEDs();
        break;
    }
  }
}

void handleMultiBall(){
  if(servoStartTime == 0){
    servoStartTime = millis();
    myservo.write(180);
  }

  if(servoStartTime + servoDelayTime <= millis()){
    myservo.write(-180); 
    servoStartTime = 0;
  }
}

void updateScore(int amount) {
  score += amount;
  updateScreen();
  flashLEDs();
}

void updateScreen() {
  lcd.setCursor(0,1);
  lcd.print(score);
}

void flashLEDs(){
    for(int dot = 0; dot < NUM_LEDS; dot++) {
      leds[dot] = CRGB::Red;
    }
    FastLED.show();
    delay(200);
    for(int dot = 0; dot < NUM_LEDS; dot++) {
      leds[dot] = CRGB::Black;
    }
     FastLED.show();
}

void startSSD1306Screen() {
 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();

  display.setTextSize(5);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println(counter);
  display.display();
}

void startMCP(){
  mcp.begin();

  for(int i = 0; i < inputPinCount; i++){
    mcp.pinMode(targetPins[0], INPUT);
//    mcp.pullUp(targetPins[0], LOW);  // turn on a 100K pullup internally
    mcp.setupInterruptPin(targetPins[0], RISING); 
  }
}

void startLED(){
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  playRunner();
  playRunnerReverse();
}

void playRunner(){
  for(int dot = 0; dot < NUM_LEDS; dot++) {
    leds[dot] = CRGB::Red;
    FastLED.show();
    // clear this led for the next time around the loop
    leds[dot] = CRGB::Black;
    delay(30);
  }
}

void playRunnerReverse(){
  for(int dot = NUM_LEDS-1; dot >= 0; dot--) {
    leds[dot] = CRGB::Red;
    FastLED.show();
    // clear this led for the next time around the loop
    leds[dot] = CRGB::Black;
    delay(30);
  }
}
