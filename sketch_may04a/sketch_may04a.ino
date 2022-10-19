void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  int v = analogRead(A13);
  v = map(v, 0, 4096, 0, 100);
  Serial.println(v);
  delay(500);
}
