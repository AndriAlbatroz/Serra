#include <Wire.h>

byte x = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(8);
  Wire.onRequest(requestEvent);
}

void loop() {
  delay(100);  
}

void requestEvent() {
  Wire.write("Hello.");
}
