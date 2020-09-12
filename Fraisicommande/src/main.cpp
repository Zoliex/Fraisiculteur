#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //MAC AUTRE ESP

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

typedef struct Sender_message {
  int a;
} Sender_message;

Sender_message Sender;

typedef struct Receive_message {
    int a;
    int b;
    int c;
} Receive_message;

// Create a struct_message called Sender
Receive_message Receive;

byte goutte[8] = {
	0b00100,
	0b00100,
	0b01010,
	0b01010,
	0b10001,
	0b10001,
	0b10001,
	0b01110
};

byte temp[8] = {
	0b00100,
	0b01010,
	0b01010,
	0b01010,
	0b01110,
	0b11111,
	0b11111,
	0b01110
};

byte prise[8] = {
	0b01010,
	0b01010,
	0b11111,
	0b10001,
	0b10001,
	0b01110,
	0b00100,
	0b00100
};

byte wifi[8] = {
	0b00000,
	0b00000,
	0b00001,
	0b00011,
	0b00111,
	0b01111,
	0b11111,
	0b11111
};

byte load1[8] = {
	0b01111,
	0b11000,
	0b10011,
	0b10111,
	0b10111,
	0b10011,
	0b11000,
	0b01111
};

byte load2[8] = {
	0b11111,
	0b00000,
	0b11011,
	0b11011,
	0b11011,
	0b11011,
	0b00000,
	0b11111
};

byte load3[8] = {
	0b11110,
	0b00011,
	0b11001,
	0b11101,
	0b11101,
	0b11001,
	0b00011,
	0b11110
};

byte degre[8] = {
	0b11100,
	0b10100,
	0b11100,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};

// variable for storing the pushbutton status
int buttonState = 0;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&Receive, incomingData, sizeof(Receive));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Temp: ");
  Serial.println(Receive.a);
  Serial.print("Hum: ");
  Serial.println(Receive.b);
  Serial.print("Pompe: ");
  Serial.println(Receive.b);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(3);
  lcd.setCursor(2, 0);
  lcd.print("Recu: ");
  lcd.print(len);
  lcd.print(" bytes");
  lcd.setCursor(0, 1);
  lcd.write(1);
  lcd.setCursor(2, 1);
  lcd.print("Temperature: ");
  lcd.print(Receive.a);
  lcd.write(7);
  lcd.print("C");
  lcd.setCursor(0, 2);
  lcd.write(0);
  lcd.setCursor(2, 2);
  lcd.print("Humidite: ");
  lcd.print(Receive.b);
  lcd.print("%");
  lcd.setCursor(0, 3);
  lcd.write(2);
  lcd.setCursor(2, 3);
  lcd.print("Pompe: ");
  String button;
  if (Receive.c == 1) {
    button = "ON";
  } else if (Receive.c == 0) {
    button = "OFF";
  } else {
    button = "???";
  }
  lcd.print(button);
}

void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.createChar(0, goutte);
  lcd.createChar(1, temp);
  lcd.createChar(2, prise);
  lcd.createChar(3, wifi);
  lcd.createChar(4, load1);
  lcd.createChar(5, load2);
  lcd.createChar(6, load3);
  lcd.createChar(7, degre);
  // initialize the pushbutton pin as an input
  lcd.setCursor(3, 1);
  lcd.print("Fraisicommande");
  lcd.setCursor(7, 2);
  lcd.print("v0.0.2");
  delay(2500);
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Attente de donnees");
  lcd.setCursor(1, 2);
  lcd.write(4);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(5);
  lcd.write(6);
  pinMode(D3, INPUT);
  Serial.begin(115200);

  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop()
{
  if ((millis() - lastTime) > timerDelay) {
    // read the state of the pushbutton value
    buttonState = digitalRead(D3);
    Sender.a = buttonState;
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &Sender, sizeof(Sender));
    lastTime = millis();
  }

}