#include "led.h"

void initLed() {
  // Initialize the LED pin as an output
  pinMode(LED_PIN_GR, OUTPUT);
//   pinMode(LED_PIN_BL, OUTPUT);
}

void turnLedOn(int pin) {
  digitalWrite(pin, HIGH); // Set the pin voltage to HIGH (5V or 3.3V, depending on Arduino)
}

void turnLedOff(int pin) {
  digitalWrite(pin, LOW); // Set the pin voltage to LOW (0V)
}
