#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>

#define NUM_LEDS 60
#define DATA_PIN 3
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define buttonPin 7

CRGB leds[NUM_LEDS];
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int counter = 1;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(5);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println(counter);
  display.display();

  pinMode(buttonPin, INPUT);
//  pinMode(3, OUTPUT);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  playRunner();
  playRunnerReverse();

  tone(3,988,8);
  delay(1000);
  noTone(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  int input = digitalRead(buttonPin);
  Serial.println(input);
  if(input >0){
  Serial.println("Here");
    counter += 1;
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println(counter);
    display.display();

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

  tone(3,988);
  delay(1000);
  noTone(3);
  delay(100);
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
