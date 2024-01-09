#include <Arduino.h>
#include <WiFiS3.h>
#include <ArduinoBLE.h>
#include <DS18B20.h>
#include <ArduinoMqttClient.h>

#include "BLUETOOTH.h"
#include "secrets.h"

void OnMqttMessage(int messageSize);

/* MQTT Setup */
char ssid[] = SSID;
char pass[] = PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);


const char BROKER[] = "192.168.144.1";
const int PORT = 1883;

  // Publish Topics
const char PUBLISH_TOPIC_DISTANCE[] = "nielsjaspers/distance";
const char PUBLISH_TOPIC_AWAKE[] = "nielsjaspers/awake";
const char PUBLISH_TOPIC_TEMP[] = "nielsjaspers/temperature";

  // Subscribe Topics
const char SUBSCRIBE_TOPIC_REGEN[] = "dionstroet/regen";
const char SUBSCRIBE_TOPIC_TEMPERATURE[] = "dionstroet/temperatuur";

long count = 0;
const long INTERVAL = 2000; // analog read interval
unsigned long currentMillis;
unsigned long previousMillis = 0;
/* MQTT Setup End */

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
const int BUZZ_PIN = 4;
const int RELAY_PIN = 2;
const int TEMP_PIN = 13;

const int YELLOW_LED_PIN = 12;
const int RED_LED_PIN = 7;

const float SPEED_OF_SOUND = 0.0343;

unsigned long timePassed;
unsigned long triggerTime = 0;

float duration, distance;

bool awake;

unsigned long timeDifference;

int status = WL_IDLE_STATUS;

BLEService fileTransferService(dataCharacteristicsUUID);
BLECharacteristic dataCharacteristic(dataCharacteristicsUUID, BLEWrite | BLERead, "");
WiFiServer server(80);
DS18B20 ds(TEMP_PIN);

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);  

  Serial.begin(115200);

  while (WiFi.begin(ssid, pass) != WL_CONNECTED){
    delay(5000);
  }

  bool mqttConnected = false;
  while (!mqttConnected){
    if (!mqttClient.connect(BROKER, PORT)){
      Serial.print("NOT CONNECTED!\n");
      delay(1000);
    }
    else{
      mqttConnected = true;
    }
    mqttClient.subscribe(SUBSCRIBE_TOPIC_REGEN);
    mqttClient.subscribe(SUBSCRIBE_TOPIC_TEMPERATURE);
    mqttClient.onMessage(OnMqttMessage);
  }

  Serial.print("CONNECTED!\n");

  if (!BLE.begin()){
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName(peripheralName);
  BLE.setAdvertisedService(fileTransferService);

  fileTransferService.addCharacteristic(dataCharacteristic);

  BLE.advertise();

}

void loop() {
  mqttClient.poll();

  currentMillis = millis();

  timeDifference = currentMillis - previousMillis;

  if (timeDifference >= INTERVAL){
    previousMillis = currentMillis;

    int value = distance;

    Serial.print("Sending message to topic: ");
    Serial.println(PUBLISH_TOPIC_DISTANCE);
    Serial.println(value);

    mqttClient.beginMessage(PUBLISH_TOPIC_DISTANCE, true, 0);
    mqttClient.print(value);
    mqttClient.endMessage();
  }
  delay(1);

  

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

  /* Temperatuur meting moet on andere plek! */
  // ds.selectNext();
  // Serial.print("Temp: ");
  // Serial.println(ds.getTempC());

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration*SPEED_OF_SOUND)/2;

  Serial.print("distance: ");
  Serial.println(distance);

  int buzzTimes = 0;

  if (distance <= 50) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
    if (triggerTime == 0){
      triggerTime = millis();
    }
    if (millis() - triggerTime > 4000){
      awake = true;
      mqttClient.beginMessage(PUBLISH_TOPIC_AWAKE, true, 0);
      mqttClient.print(awake);
      mqttClient.endMessage();
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("KOFFIEZETAPPARAAT AAN!!!");
      if(buzzTimes == 0){
        tone(BUZZ_PIN, 3500, 500);
        buzzTimes = 1;
      }
    }
    else{
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(RELAY_PIN, LOW);
      awake = false;
      buzzTimes = 0;
    }
  }
  else{
    triggerTime = 0;
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
    awake = false;
    mqttClient.beginMessage(PUBLISH_TOPIC_AWAKE, true, 0);
    mqttClient.print(awake);
    mqttClient.endMessage();
  }
}

void OnMqttMessage(int messageSize){
  Serial.print("Recieved a message with topic: ");
  Serial.println(mqttClient.messageTopic());

  String message = "";

  if (mqttClient.messageTopic() == SUBSCRIBE_TOPIC_REGEN){
    int num;

    while (mqttClient.available()){
      message.concat((char)mqttClient.read());
    }
    Serial.println(message);
    num = message.toInt();

    if (num == 0){
      Serial.println("REGEN");
    }
    else{
      Serial.println("DROOG");
    }
  }
  else if (mqttClient.messageTopic() == SUBSCRIBE_TOPIC_TEMPERATURE){
    float incomingTemp;

    while (mqttClient.available()){
      message.concat((char)mqttClient.read());
    }
    Serial.println(message);
    incomingTemp = message.toFloat();

    if (incomingTemp <= 15){
      Serial.println("Outside is Cold");
      tone(BUZZ_PIN, 4000, 2000);
    }
    else if (incomingTemp > 15 && incomingTemp < 20){
      Serial.println("Outside is Getting Warmer");
      tone(BUZZ_PIN, 3000, 2000);
    }
    else if (incomingTemp >= 20 && incomingTemp < 30){
      Serial.println("Outside is Warm");
      tone(BUZZ_PIN, 2000, 2000);
    }
    else if (incomingTemp >= 30){
      Serial.println("Outside is Hot");
      tone(BUZZ_PIN, 1000, 2000);
    }
    else{
      Serial.println("NaN, Try again later.");
    }
  }

  
}
