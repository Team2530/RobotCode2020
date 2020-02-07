// Manages communication with the RoboRIO
#include <FastLED.h>

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
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(3000); // 3 second delay for recovery
  
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    byte value = Serial.read();

    if(value == 0x12) {
      Serial.println("Arduino says: 'Received code 0x12.'");
    }
  }
// Call the current pattern function once, updating the 'leds' array
 
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
