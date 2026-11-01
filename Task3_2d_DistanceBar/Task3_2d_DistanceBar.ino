#include <NewPing.h>
#include <Adafruit_NeoPixel.h>

const int TRIG_PIN = 4;
const int ECHO_PIN = 5;
const int LED_PIN = 14;
const int NUM_LEDS = 8;

NewPing sonar(TRIG_PIN, ECHO_PIN, 400);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  strip.begin();
}

void loop() {
  int cm = sonar.ping_cm();
  if (cm == 0) cm = 400;  // no echo = far away

  // Closer = more LEDs: 10 cm -> 8 LEDs, 100 cm -> 1 LED
  int leds = map(constrain(cm, 10, 100), 100, 10, 1, NUM_LEDS);

  // Red when near, yellow in the middle, green when far
  uint32_t colour;
  if (cm < 30)       colour = strip.Color(32, 0, 0);
  else if (cm <= 100) colour = strip.Color(32, 32, 0);
  else               colour = strip.Color(0, 32, 0);

  strip.clear();
  strip.fill(colour, 0, leds);
  strip.show();

  Serial.print(cm);
  Serial.print(" cm -> ");
  Serial.println(leds);

  delay(100);
}
