/*
  Written originally for a WeMOS D1 ESP8266 Arduino board
  then ported to an Espressif Wrover dev module.

  Connects to WiFi
  Publishes a "connected" message to let MQTT know it's online.
  Watches for a signal from a radar motion sensor.
  Publishes a "motion" message each time it sees motion.

  It will reconnect to the server if the connection is lost using a blocking
  reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
  achieve the same result without blocking the main loop.
*/
#include <Arduino.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Tone32.h>

#include "localize.h"

// pinout reference https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

#define UNIQUE_ID  "ESP02"
#define TOPIC      "ESP32"
#define SUB_TOPIC  "HOME"

// HARDWARE
#define LED 33
#define RADAR 18
#define BUZZER 23
#define BUZZER_CHANNEL 0
#define BUTTON EN

#define QOS 1

// MQTT messages
#define MQTT_PUB_MOTION      TOPIC "/Motion"
#define MQTT_PUB_DISCOVERY   "HOME/device_automation/1/" UNIQUE_ID "/config"
#define MQTT_DISCOVERY_JSON  "{\"automation_type\":\"trigger\",\"topic\":\"" TOPIC "\",\"payload\":\"1\",\"uniq_id\":\"" UNIQUE_ID "\"}"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t discoveryTimer;           // send a discovery message periodically
TimerHandle_t radarDebounce;         // don't resend motion message too often

#define DISCOVERY_INTERVAL 30000  // send a discovery message

/* My thinking is that I want the motion sensor to send MQTT messages as long as
it's detecting movement so that the timer that turns the light off keeps getting
reset, so the light stays on as long as there is motion. But I don't want to 
hammer the server with messages either, so I have this debounce timer that makes
sure it does not send messages continuously. */
#define RADAR_INTERVAL 5000  // dont send RADAR messages more often than this

void mqtt_pub(const char *msg, uint8_t qos, const char *payload) {
    uint16_t id = mqttClient.publish(msg, qos, true, payload);                            
    Serial.printf("Pub \"%s\" QoS=%d, id %i: ", msg, qos, id);
}

void connectToWifi() {
  Serial.printf("Connecting to %s.. ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT broker.");
  mqttClient.connect();
}


void sendDiscovery() {
    // Every X seconds publish a "config" message
//    sprintf(msg, MQTT_PUB_DISCOVERY, UNIQUE_ID);
    mqtt_pub(MQTT_PUB_DISCOVERY, QOS, MQTT_DISCOVERY_JSON);
}

uint8_t retrigger_ok = 1;
void debounceTimeout() {
  retrigger_ok = 1;
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("WiFi connected, ");
        Serial.println("address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        
        // Stop these timers until WiFi comes back
        xTimerStop(discoveryTimer, 0);
        xTimerStop(mqttReconnectTimer, 0);

		    xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
    Serial.print("Connected to MQTT. Session present:");
    Serial.println(sessionPresent);
    sendDiscovery();
    xTimerStart(discoveryTimer, 0);
    tone(BUZZER, NOTE_G4, 1000, BUZZER_CHANNEL);
    tone(BUZZER, NOTE_C5,  500, BUZZER_CHANNEL);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
  tone(BUZZER, NOTE_C5,  500, BUZZER_CHANNEL);
  tone(BUZZER, NOTE_G4, 1000, BUZZER_CHANNEL);
}

void onMqttSubscribe(uint16_t id, uint8_t qos) {
  Serial.printf("SUB ACK. %d", id);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t id) {
  Serial.printf("PUB ACK %d\n", id);
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RADAR, INPUT);

  Serial.begin(115200);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  discoveryTimer = xTimerCreate("discoveryTimer", pdMS_TO_TICKS(DISCOVERY_INTERVAL), pdTRUE, // autoreload
       (void*)0, reinterpret_cast<TimerCallbackFunction_t>(sendDiscovery));
  radarDebounce = xTimerCreate("radarDebounce", pdMS_TO_TICKS(RADAR_INTERVAL), pdFALSE,
       (void*)0, reinterpret_cast<TimerCallbackFunction_t>(debounceTimeout));

  WiFi.onEvent(WiFiEvent);

  // Install MQTT event handlers.
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}


void loop() {
    int radar = digitalRead(RADAR);
    if (radar == 1) {
      if (retrigger_ok) {
        digitalWrite(LED, 1); // Turn on LED
        mqtt_pub(MQTT_PUB_MOTION, QOS, "1");
        xTimerStart(radarDebounce,0);
        retrigger_ok = 0;
      }
    } else {
        digitalWrite(LED, 0); // Turn off LED
    }
}

