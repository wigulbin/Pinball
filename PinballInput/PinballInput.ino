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

byte arduinoIntPin = 3;
int interruptPin = 3;

volatile boolean awakenByInterrupt = false;
byte arduinoInterrupt=1;

//Servo Pin
int servoPin = 5;

// LCD Pins
//#define rs 7
//#define en 8
//#define d4 9
//#define d5 10
//#define d6 11
//#define d7 12
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 7, d7 = 2;

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
volatile int score = 0;
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
  startLED();

  pinMode(arduinoIntPin, INPUT);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Score:");
  startMCP();
  attachInterrupt(digitalPinToInterrupt(interruptPin), intCallBack, CHANGE);
}

void loop() {
  updateScreen();
  // put your main code here, to run repeatedly:  
  // enable interrupts before going to sleep/wait
  // And we setup a callback for the arduino INT handler.
//  updateScreen();
  // Simulate a deep sleep
//  while(!awakenByInterrupt);
  // disable interrupts while handling them.
//  detachInterrupt(arduinoInterrupt);
  
//  if(awakenByInterrupt) {
//    Serial.println("Woken by interrupt...");
//    handleInterrupt();
//  }

//delay(10);
}

void checkBallDrain(){
}
void handleInterrupt(){
}

void cleanInterrupts(){
  EIFR=0x01;
  awakenByInterrupt=false;
}  
void intCallBack(){
  awakenByInterrupt=true;
  uint8_t pin=mcp.getLastInterruptPin();
  uint8_t val=mcp.getLastInterruptPinValue();
  Serial.print("Pin: ");
  Serial.println(pin);
  Serial.print("Val: ");
  Serial.println(val);
  Serial.println("");
  
  if(val > 0){
    checkTarget(pin);
  }

//  while( ! (mcp.digitalRead(pin) ));
  // and clean queued INT signal
  cleanInterrupts();
}

void checkTarget(int targetPin) {
  Serial.print("Checking target: ");
  Serial.println(targetPin);
  Serial.print("Target 1: ");
  Serial.println(ramp2Pin);
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
//  noInterrupts();
  score += amount;
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
  Serial.println("Starting MCP...");
  mcp.begin();
  mcp.setupInterrupts(true, false, HIGH);
  for(int i = 0; i < inputPinCount; i++){
    Serial.print("Starting pin: ");
    Serial.println(targetPins[i]);
    mcp.pinMode(targetPins[i], INPUT);
    mcp.setupInterruptPin(targetPins[i], CHANGE); 
  }
  Serial.println("MCP Done");
}

void startLED(){
  Serial.println("Starting LEDs...");
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
