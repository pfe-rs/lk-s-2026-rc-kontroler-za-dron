void setup() {
  // put your setup code here, to run once:
  pinMode(7, INPUT);
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(7);
  float v_adc = (raw / 4095.0) * 3.3;
  float v_bat = v_adc * (100.0 + 220.0) / 220.0;
  Serial.println(v_bat);
  delay(1000);  
}
