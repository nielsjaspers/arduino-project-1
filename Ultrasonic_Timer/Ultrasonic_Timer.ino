#include <Arduino.h>
#include <WiFiS3.h>
#include <ArduinoBLE.h>

#include "WIFIPASS.h"
#include "BLUETOOTH.h"

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int BUZZ_PIN = 4;
const int RELAY_PIN = 2;

const int YELLOW_LED_PIN = 12;
const int RED_LED_PIN = 7;

const float SPEED_OF_SOUND = 0.0343;

unsigned long timePassed;
unsigned long triggerTime = 0;

float duration, distance;

int status = WL_IDLE_STATUS;

BLEService fileTransferService(dataCharacteristicsUUID);
BLECharacteristic dataCharacteristic(dataCharacteristicsUUID, BLEWrite | BLERead, "");
WiFiServer server(80);

void PrintWifiStatus();

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);  

  Serial.begin(9600);

  //while(status != WL_CONNECTED){
    //Serial.print("Attempting to connect to SSID: ");
    //Serial.println(ssid);

    //status = WiFi.begin(ssid, pass);

    //delay(10000);
  //}
  //server.begin();

  if (!BLE.begin()){
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName(peripheralName);
  BLE.setAdvertisedService(fileTransferService);

  fileTransferService.addCharacteristic(dataCharacteristic);

  BLE.advertise();

  PrintWifiStatus();
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    char recievedData [200];

    while (central.connect()) {
      if (dataCharacteristic.written()) {
        strcpy(recievedData, (const char*) dataCharacteristic.value());
        // Processes the recieved data
        Serial.print("Recieved data: ");
        Serial.println(recievedData);
      }
    }
  }

  WiFiClient client = server.available();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration*SPEED_OF_SOUND)/2;

  Serial.print("distance: ");
  Serial.println(distance);

  if (distance <= 50) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
    if (triggerTime == 0){
      triggerTime = millis();
    }
    if (millis() - triggerTime > 4000){
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(RELAY_PIN, HIGH);
        //tone(BUZZ_PIN, 3500, 500);
    }
    else{
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(RELAY_PIN, LOW);
    }
  }
  else{
    triggerTime = 0;
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
  }
}


void PrintWifiStatus(){
  Serial.print("IP Adress: ");
  Serial.println(WiFi.localIP());

  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}
