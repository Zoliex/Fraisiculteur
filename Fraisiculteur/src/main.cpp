#include <Arduino.h>
#include <esp_now.h>
#include "DHT.h"
#include <WiFi.h>

#define DHTPIN 15
int pompe = 13;

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

// REPLACE WITH YOUR SendR MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the Sendr structure
typedef struct Send_message {
  int a;
  int b;
  int c;
} Send_message;

// Create a struct_message called Send
Send_message Send;

typedef struct Receive_message {
    int a;
} Receive_message;

Receive_message Receive;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Receive, incomingData, sizeof(Receive));
  Serial.println("");
  Serial.println("|--------------------[ PACKET RECEIVED ]--------------------");
  Serial.println("|");
  Serial.println("|--> Packet received with Success");
    Serial.print("|--> Bytes : ");
  Serial.println(len);
    Serial.print("|--> Pompe : ");
  Serial.println(Receive.a);
  Serial.println("|");
  Serial.println("|-----------------------------------------------------------");
  if (Receive.a == 0){
    digitalWrite(pompe, HIGH);
  } else {
    digitalWrite(pompe, LOW);
  }


  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  // Set values to send
  Send.a = t;
  Send.b = h;
  Send.c = digitalRead(pompe);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Send, sizeof(Send));
   
  if (result == ESP_OK) {
    Serial.println("");
    Serial.println("|----------------------[ PACKET SENT ]----------------------");
    Serial.println("|");
    Serial.println("|--> Delivery Success");
      Serial.print("|--> Temperature : ");
    Serial.println(Send.a);
      Serial.print("|--> Humidite : ");
    Serial.println(Send.b);
      Serial.print("|--> Pompe : ");
    Serial.println(Send.c);
    Serial.println("|");
    Serial.println("|-----------------------------------------------------------");
  }
  else {
    Serial.println("");
    Serial.println("|----------------------[ PACKET SENT ]----------------------");
    Serial.println("|");
    Serial.println("|--> Delivery Failed");
    Serial.println("|");
    Serial.println("|-----------------------------------------------------------");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  dht.begin();
  pinMode(pompe, OUTPUT);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
void loop() {

}