#include <esp_now.h>
#include <WiFi.h>

const int PIN_YAW      = 1;  
const int PIN_THROTTLE = 2;  
const int PIN_ROLL     = 3;  
const int PIN_PITCH    = 4;  
const int PIN_SWITCH_ARM = 7; 

const int LED_CRVENA   = 5;  
const int LED_ZELENA   = 6;  

uint8_t broadcastAddress[] = {0xE8, 0x06, 0x90, 0x95, 0x79, 0xD0}; 

struct DronPackage {
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    bool armStatus;
    uint32_t vremeSlanja; 
};

DronPackage podaciZaSlanje;
esp_now_peer_info_t peerInfo;

volatile float izmerenaLatencija = 0.0;
volatile bool noviPingStigao = false;

int16_t primeniDeadband(int16_t vrednost) {
  if (vrednost >= 1480 && vrednost <= 1520) {
    return 1500;
  }
  return vrednost; 
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(LED_ZELENA, HIGH);
    digitalWrite(LED_CRVENA, LOW);
  } else {
    digitalWrite(LED_ZELENA, LOW);
    digitalWrite(LED_CRVENA, HIGH);
  }
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  uint32_t trenutnoVreme = micros();
  DronPackage povratniPaket;
  
  if (len == sizeof(DronPackage)) {
    memcpy(&povratniPaket, incomingData, sizeof(povratniPaket));
    
    izmerenaLatencija = (trenutnoVreme - povratniPaket.vremeSlanja) / 2.0 / 1000.0;
    noviPingStigao = true;
  }
}

void citajHardver() {
  int16_t siroviYaw   = map(analogRead(PIN_YAW), 0, 4095, 1000, 2000);
  int16_t siroviPitch = map(analogRead(PIN_PITCH), 0, 4095, 1000, 2000);
  int16_t siroviRoll  = map(analogRead(PIN_ROLL), 0, 4095, 1000, 2000);
  
  podaciZaSlanje.throttle = map(analogRead(PIN_THROTTLE), 0, 4095, 1000, 2000);

  podaciZaSlanje.yaw   = primeniDeadband(siroviYaw);
  podaciZaSlanje.pitch = primeniDeadband(siroviPitch);
  podaciZaSlanje.roll  = primeniDeadband(siroviRoll);

  podaciZaSlanje.armStatus = (digitalRead(PIN_SWITCH_ARM) == LOW);
}

void ispisiDebugTransmiter() {
  Serial.print("Saljem -> Gas: ");  Serial.print(podaciZaSlanje.throttle);
  Serial.print(" | Yaw: ");        Serial.print(podaciZaSlanje.yaw);
  Serial.print(" | Pitch: ");      Serial.print(podaciZaSlanje.pitch);
  Serial.print(" | Roll: ");       Serial.print(podaciZaSlanje.roll);
  Serial.print(" | ARM: ");        Serial.print(podaciZaSlanje.armStatus ? "UPALJEN" : "UGASEN");
  
  Serial.println(); 
}

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_ZELENA, OUTPUT);
  pinMode(LED_CRVENA, OUTPUT);
  pinMode(PIN_SWITCH_ARM, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Greska pri inicijalizaciji komunikacije");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv); 
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Neuspesno dodavanje peera");
    return;
  }
}

void loop() {
  citajHardver();
  ispisiDebugTransmiter();
  
  if (noviPingStigao) {
    Serial.print("---> Izmerena Latencija: ");
    Serial.print(izmerenaLatencija, 2);
    Serial.println(" ms");
    noviPingStigao = false; 
  } else {
    Serial.println("Nema odgovora od drona (Proveri da li je Receiver upaljen)");
  }
  Serial.println("-----------------------------------------------------------------------");

  podaciZaSlanje.vremeSlanja = micros(); 
  esp_now_send(broadcastAddress, (uint8_t *) &podaciZaSlanje, sizeof(podaciZaSlanje));

  delay(1000); 
}