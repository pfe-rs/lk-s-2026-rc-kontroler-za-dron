const int PIN_X = 1;  // X-osa
const int PIN_Y = 2;  // Y-osa

float mapFloat(float val, float inMin, float inMax, float outMin, float outMax) {
  return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // ESP32 ADC je 12-bitni (0-4095)
}

void loop() {
  int rawX = analogRead(PIN_X);
  int rawY = analogRead(PIN_Y);

  float xVal = mapFloat(rawX, 0, 4095, -20.0, 20.0);
  float yVal = mapFloat(rawY, 0, 4095, -20.0, 20.0);

  Serial.print("X: ");
  Serial.print(xVal, 2);
  Serial.print("\tY: ");
  Serial.println(yVal, 2);

  delay(50);
}
