#include <FastLED.h> //VSCode detects errors here, but it works on the Arduino, so no need to worry about it

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    3
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    11
CRGB leds[NUM_LEDS];

#define BRIGHTNESS         50
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
  byte[] value = {0, 0};
  
  // Gets value from RoboRIO and stores it in variable "value"
  if(Serial.available()) {
    value = Serial.read();
    Serial.println("Arduino received data.");
  }

  // Call the current pattern function once, updating the 'leds' array
  for (int i = 0; i < 11; i++) {
    
    if (value[0] == 0 && value[1] == 0) {
      // put light pattern for no input here
      leds[i] = CHSV(105, 75, 255);
    }
    
    if (value[0] == 1) {
      // put light pattern for Pixy here
      leds[i] = CRGB::Yellow;
    }

    if (value[1] == 1) {
      // put light pattern for Limelight here
      leds[i] = CRGB::Green;
    }

    if (value[0] == 1 && value[1] == 1) {
      // put light pattern for Limelight + Pixy here
      if (i < 6) {
        leds[i] = CRGB::Yellow;
      } else {
        leds[i] = CRGB::Green;
      }
    }
  }
 
  FastLED.show(); 
  delay(30); 
}
