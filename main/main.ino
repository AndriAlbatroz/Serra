#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <DHT.h>

#define lcdAddress 0x00

#define wifiRetryCnt 10
#define wifiSsid "qualcosadistupido"
#define wifiPassword "FamigliaMicheli9801Potty!"

#define mqttRetryCnt 10
#define mqttClientName "Serra"
#define mqttServer "192.168.1.3"
#define mqttPort 1883
#define mqttUser "serra"
#define mqttPassword "test01"

#define mqttTopicTemperature "sensor/temperature_room"
#define mqttTopicHumidity "sensor/humidity_room"
#define mqttTopicRain "state/rain"
#define mqttTopicHygrometer "state/hygrometer"

#define lcdScl 22
#define lcdSda 21

#define dhtPin 5
#define dhtType DHT22

#define hygrometerPin A0
#define hygrometerTrigger 35 // percentage

#define raindropsPin A3
#define raindropsTrigger 60 // percentage

#define elettrovalvolaPin 23

#define tettoPinDir 19
#define tettoPinStep 18
#define tettoOpenFC 16
#define tettoCloseFC 17

float temperaturaInterna = 0.0;
float umiditaInterna = 0.0;
float umiditaTerra = 0.0;
bool isRain = false;

LiquidCrystal_I2C lcdClient(lcdAddress, 16, 2);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
DHT dht(dhtPin, dhtType);

long previousMillis = 0; //ms
long interval = 1500; //ms

void setupOTA() {
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void connectToLCD() {
  Wire.begin(lcdSda, lcdScl);
  lcdClient.init();
  lcdClient.backlight();
  lcdClient.setCursor(0, 0);
  lcdClient.print("Booting...");
}

void scrollTextLCD(int row, String message, int delayTime, int lcdColumns) {
  for (int i = 0; i < lcdColumns; i++) {
    message = " " + message;
  }
  message = message + " ";
  for (int pos = 0; pos < message.length(); pos++) {
    lcdClient.setCursor(0, row);
    lcdClient.print(message.substring(pos, pos + lcdColumns));
    delay(delayTime);
  }
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  for (int i = 0; i < wifiRetryCnt; i++) {
    if (WiFi.status() != WL_CONNECTED) {
      delay(5000);
    }
  }
}

void connectToMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  for (int i = 0; i < mqttRetryCnt; i++) {
    if (!mqttClient.connect(mqttClientName, mqttUser, mqttPassword)) {
      delay(5000);
    }
  }
}

void connectToDHT() {
  dht.begin();
}

void setup() {
  Serial.begin(115200);
  
  //connectToLCD();
  connectToWiFi();
  connectToMQTT();
  connectToDHT();
  setupOTA();

  pinMode(tettoPinDir, OUTPUT);
  pinMode(tettoPinStep, OUTPUT);
}

void loop() {
  ArduinoOTA.handle();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    
    /*
    isRain = map(analogRead(raindropsPin), 0 , 4096, 0, 100) > raindropsTrigger;
    
    if (isRain && digitalRead(tettoOpenFC)) { // pioggia e tetto aperto
      closeTetto();
    }

    // Irrigazione
    umiditaTerra = map(analogRead(hygrometerPin), 0, 4096, 0, 100);
    digitalWrite(elettrovalvolaPin, umiditaTerra < hygrometerTrigger ? HIGH : LOW);
    */

    temperaturaInterna = dht.readTemperature();
    umiditaInterna = dht.readHumidity();

    mqttClient.publish(mqttTopicTemperature, String(temperaturaInterna).c_str(), true);
    mqttClient.publish(mqttTopicHumidity, String(umiditaInterna).c_str(), true);
    //mqttClient.publish(mqttTopicRain, String(isRain).c_str(), true);
    //mqttClient.publish(mqttTopicHygrometer, String(umiditaTerra < hygrometerTrigger).c_str(), true);
  }
}

void pulse(int d = 500) {
  digitalWrite(tettoPinStep, HIGH);
  delayMicroseconds(d);
  digitalWrite(tettoPinStep, LOW);
  delayMicroseconds(d);
}

void closeTetto() {
  digitalWrite(tettoPinDir, LOW);
  while (!digitalRead(tettoCloseFC)) {
    pulse();
  }
}

void openTetto() {
  digitalWrite(tettoPinDir, HIGH);
  while (!digitalRead(tettoOpenFC)) {
    pulse();
  }
}
