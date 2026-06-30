const int PIN_X = 1;  // X-osa (zamenjeno)
const int PIN_Y = 2;  // Y-osa (zamenjeno)

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // ESP32 ADC je 12-bitni (0-4095)
}

void loop() {
  int rawX = analogRead(PIN_X);
  int rawY = analogRead(PIN_Y);

  // Skaliranje sa 0-4095 na -20..20
  int xVal = map(rawX, 0, 4095, -20, 20);
  int yVal = map(rawY, 0, 4095, -20, 20);

  Serial.print("X: ");
  Serial.print(xVal);
  Serial.print("\tY: ");
  Serial.println(yVal);

  delay(50);
}
