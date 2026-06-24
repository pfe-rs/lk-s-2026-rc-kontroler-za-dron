#include <esp_now.h>
#include <WiFi.h>

uint8_t transmiterAddress[] = {0xE4, 0xB3, 0x23, 0xF8, 0x37, 0xD0}; 

struct DronPackage {
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    bool armStatus;
    uint32_t vremeSlanja;
};

DronPackage primljeniPodaci;
unsigned long vremePoslednjegPaketa = 0;
esp_now_peer_info_t peerInfo;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(DronPackage)) {
    memcpy(&primljeniPodaci, incomingData, sizeof(primljeniPodaci));
    vremePoslednjegPaketa = millis(); 
    
    esp_now_send(transmiterAddress, (uint8_t *) &primljeniPodaci, sizeof(primljeniPodaci));
  }
}

void proveriFailsafe() {
  if (millis() - vremePoslednjegPaketa > 1500) {
    primljeniPodaci.throttle = 1000; 
    primljeniPodaci.yaw = 1500;      
    primljeniPodaci.pitch = 1500;    
    primljeniPodaci.roll = 1500;     
    primljeniPodaci.armStatus = false; 
    
    Serial.println("!!! FAILSAFE AKTIVIRAN (GUBITAK SIGNALA) !!!");
  }
}

void ispisiDebugReceiver() {
  Serial.println("NOVI PAKET PRIMLJEN");
  Serial.print("Throttle: ");       Serial.println(primljeniPodaci.throttle);
  Serial.print("Yaw: ");      Serial.println(primljeniPodaci.yaw);
  Serial.print("Pitch: ");        Serial.println(primljeniPodaci.pitch);
  Serial.print("Roll: ");         Serial.println(primljeniPodaci.roll);
  Serial.print("Motor Status: ");         Serial.println(primljeniPodaci.armStatus ? "OTKLJUČAN" : "ZAKLJUČAN");
  Serial.println("---------------------------------");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Greska pri inicijalizaciji komunikacije");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, transmiterAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Neuspesno dodavanje transmitera u listu");
    return;
  }
}

void loop() {
  proveriFailsafe();       
  ispisiDebugReceiver();   
  
  delay(500); 
}