const int PIN_X = 2;  // X-osa
const int PIN_Y = 1;  // Y-osa
const int PIN_X2 = 3;
const int PIN_Y2 = 4;

float mapFloat(float val, float inMin, float inMax, float outMin, float outMax) {
  return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void setup() {  
  Serial.begin(115200);
  analogReadResolution(12); // ESP32 ADC je 12-bitni (0-4095)
  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);
  pinMode(PIN_X2, INPUT);
  pinMode(PIN_Y2, INPUT);
}

void loop() {
  int rawX = analogRead(PIN_X);
  int rawY = analogRead(PIN_Y);
  int rawX2 = analogRead(PIN_X2);
  int rawY2 = analogRead(PIN_Y2);

  float xVal = mapFloat(rawX, 0, 4095, -20.0, 20.0);
  float yVal = mapFloat(rawY, 0, 4095, -20.0, 20.0);
  float xVal2 = mapFloat(rawX2, 0, 4095, -20.0, 20.0);
  float yVal2 = mapFloat(rawY2, 0, 4095, -20.0, 20.0);


  Serial.print("X: ");
  Serial.print(rawX);
  Serial.print("\tY: ");
  Serial.print(rawY);
  Serial.print("\tX2: ");
  Serial.print(rawX2);
  Serial.print("\tY2: ");
  Serial.println(rawY2);

  delay(50);
}
