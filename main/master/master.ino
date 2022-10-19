#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  Wire.requestFrom(8, 6);

  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }

  delay(500);
  Serial.println("");
}
