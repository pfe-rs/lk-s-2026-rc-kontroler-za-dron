#include <esp_now.h>
#include <WiFi.h>

#define PIN_YAW         1  
#define PIN_THROTTLE    2
#define PIN_ROLL        3
#define PIN_PITCH       4
#define PIN_SWITCH_ARM  7 

uint8_t broadcastAddress[] = {0xE4, 0xB3, 0x23, 0xF8, 0x2E, 0xD8}; // треба променити

struct DronPackage {
    float throttle;
    float yaw;
    float pitch;
    float roll;
    bool armStatus;
};

DronPackage packageForSend;
esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    rgbLedWrite(21, 0, 255, 0);
  } else {
    rgbLedWrite(21, 255, 0, 0);
  }
}

void makingPackage() {
  // ово је за тјуновање ПИДа
  /*
  if (Serial.available() > 0) {
    String readFromSerial = Serial.readStringUntil('\n');
    
    if (readFromSerial.charAt(0) == 'a') {
      packageForSend.armStatus = !packageForSend.armStatus;
    }

    if (readFromSerial.charAt(0) == 't') {
      packageForSend.throttle = readFromSerial.substring(1, readFromSerial.length()).toFloat();
    }

    if (readFromSerial.charAt(0) == 'p') {
      packageForSend.yaw = readFromSerial.substring(1, readFromSerial.length()).toFloat();
    }

    if (readFromSerial.charAt(0) == 'i') {
      packageForSend.pitch = readFromSerial.substring(1, readFromSerial.length()).toFloat();
    }

    if (readFromSerial.charAt(0) == 'd') {
      packageForSend.roll = readFromSerial.substring(1, readFromSerial.length()).toFloat();
    }
  }
  */

  // ово је за контролу кретања
  /**/
  packageForSend.yaw   = mapFloat(analogRead(PIN_YAW), 0, 4095, -20, 20);
  packageForSend.pitch = mapfloat(analogRead(PIN_PITCH), 0, 4095, -20, 20);
  packageForSend.roll  = mapfloat(analogRead(PIN_ROLL), 0, 4095, -20, 20);
  
  packageForSend.throttle = mapfloat(analogRead(PIN_THROTTLE), 0, 4095, 15, 80);

  packageForSend.armStatus = (digitalRead(PIN_SWITCH_ARM) == LOW);
  /**/
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printPackage() {
  Serial.print("Throttle: ");       Serial.println(packageForSend.throttle);
  Serial.print("Yaw: ");            Serial.println(packageForSend.yaw, 5);
  Serial.print("Pitch: ");          Serial.println(packageForSend.pitch, 5);
  Serial.print("Roll: ");           Serial.println(packageForSend.roll, 5);
  Serial.print("Motor Status: ");   Serial.println(packageForSend.armStatus ? "ОТКЉУЧАН" : "ЗАКЉУЧАН");
  Serial.println("-------------");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Грешка при иницијализацији комуникације.");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Неуспешно додавање peer-а");
    return;
  }

  packageForSend.armStatus = 0;
  packageForSend.throttle = 15;
  packageForSend.yaw = 0.04;
  packageForSend.pitch = 0.025;
  packageForSend.roll = 0.03;
}

void loop() { 
  makingPackage();
  esp_now_send(broadcastAddress, (uint8_t *) &packageForSend, sizeof(packageForSend));
  printPackage();
  
  delay(100);
}
