#include <FastLED.h> //VSCode detects errors here, but it works on the Arduino, so no need to worry about it

FASTLED_USING_NAMESPACE

// FastLED "100-lines-of-code" demo reel, showing just a few 
// of the kinds of animation patterns you can quickly and easily 
// compose using FastLED.  
//
// This example also shows one easy way to define multiple 
// animations patterns and have them automatically rotate.
//
// -Mark Kriegsman, December 2014

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    3
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    11
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

void setup() {
  Serial.begin(9600); // Sets up communication with the RoboRIO
  delay(3000); // 3 second delay for recovery
  
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  
}

void loop() {
  // Gets value from RoboRIO and stores it in variable "value"
  if(Serial.available()) {
    byte value = Serial.read();
    Serial.println("Arduino received code '" + value + "'.");
  }

  // Call the current pattern function once, updating the 'leds' array
 if (value == 0) {
    // put light pattern for no input here
  }
 
  if (value == 1) {
    // put light pattern for Pixy here
  }

   leds[0] = CRGB::Red; 
   leds[1] = CRGB::Blue; 
   leds[2] = CRGB::Red; 
   leds[3] = CRGB::Orange; 
   leds[4] = CRGB::Green; 
   leds[5] = CRGB::Yellow; 
        FastLED.show(); 
        delay(30); 
  
  delay(50);
}
