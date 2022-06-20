/*
  Written originally for a WeMOS D1 ESP8266 Arduino board
  then ported to an Espressif Wrover DEV module.

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
#include <ArduinoJson.h>

#include "localize.h"

// pinout reference https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

#define TOPIC      "ESP32"

#define DISCOVERY_INTERVAL 30000  // send a discovery message

/* My thinking is that I want the motion sensor to send MQTT messages as long as
it's detecting movement so that the timer that turns the light off keeps getting
reset, so the light stays on as long as there is motion. But I don't want to 
hammer the server with messages either, so I have this debounce timer that makes
sure it does not send messages continuously. */
#define MOTION_INTERVAL 2100  // dont send RADAR messages more often than this

// HARDWARE
#define HAVE_NEOPIXEL 1
#define LED 33
#define MAXBRIGHT 50
#define NEOPIXEL_COUNT 1

#define HAVE_RADAR 0
#define RADAR 18

#define HAVE_PIR 1
#define PIR 23

#define BUZZER 26
#define BUZZER_CHANNEL 0
#define PITCH 1500

#define HAVE_TEMPSENSOR 0
#define TEMPSENSOR 34 // one wire sensor
#define BUTTON EN

#if HAVE_NEOPIXEL
#include <Adafruit_NeoPixel.h>

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, LED, NEO_GRBW + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

void setColor(uint32_t color) {
    strip.setPixelColor(0, color);   //  Set pixel's color (in RAM)
    strip.show();                    //  Update strip to match
}

void pulseWhite(uint8_t wait) {
  extern bool sleeping;
  for(int j=0; j<=MAXBRIGHT && sleeping; j++) { // Ramp up from 0 to 255
    // gamma-corrected brightness level 'j':
    setColor(strip.Color(0, 0, 0, strip.gamma8(j)));
    delay(wait);
  }

  for(int j=MAXBRIGHT; j>=0 && sleeping; j--) { // Ramp down from 255 to 0
    setColor(strip.Color(0, 0, 0, strip.gamma8(j)));
    delay(wait);
  }

  setColor(strip.Color(0, 0, 0, 0));
}
#endif

#if HAVE_TEMPSENSOR
OneWire oneWire(TEMPSENSOR);
DallasTemperature sensors(&oneWire);
#endif

bool sleeping = false;
uint8_t r,g,b,w;

#define QOS 0

// MQTT messages
#define NAME "ESP01"
#define MQTT_PUB_DISCOVERY   TOPIC "/binary_sensor/esp32/config"
#define MQTT_DISCOVERY_JSON  "{\"name\": \"ESP01\", \"device_class\": \"motion\", \"state_topic\": \"homeassistant/binary_sensor/esp32/state\"}"
#define MQTT_PUB_RADAR       TOPIC "/RADAR"
#define MQTT_PUB_PIR         TOPIC "/Motion"
#define MQTT_SUB_MOTION      NAME "/MotionSensor" // with "ON", power up motion sensor, with "OFF", power it off.

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t discoveryTimer;           // send a discovery message periodically
TimerHandle_t motionDebounce;         // don't resend motion message too often

void mqtt_pub(const char *msg, uint8_t qos, const char *payload) {
    mqttClient.publish(msg, qos, true, payload);                            
    Serial.printf("pub \"%s\" QoS=%d \"%s\"\n", msg, qos, payload);
}

void mqtt_sub(const char *msg, uint8_t qos) {
    mqttClient.subscribe(msg, qos);
    Serial.printf("sub \"%s\" QoS=%d\n", msg, qos);
}

void connectToWifi() {
#if HAVE_NEOPIXEL
  setColor(strip.Color(200,0,0,0)); // RED 
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  setColor(strip.Color(200,180,0,0)); // YELLOW
  mqttClient.connect();
}


void sendDiscovery() {
    // Every X seconds publish a "config" message
    //char msg[128];
    //const char *clientId = mqttClient.getClientId();
    //sprintf(msg, MQTT_DISCOVERY_JSON, clientId);
    mqtt_pub(MQTT_PUB_DISCOVERY, QOS, MQTT_DISCOVERY_JSON);
}

uint8_t retrigger_ok = 1;
void debounceTimeout() {
  retrigger_ok = 1;
}

void WiFiEvent(WiFiEvent_t event) {
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("[WiFi]; address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("[WiFi] lost connection");
        
        // Stop these timers until WiFi comes back
        xTimerStop(discoveryTimer, 0);
        xTimerStop(mqttReconnectTimer, 0);

		    xTimerStart(wifiReconnectTimer, 0);
        break;
    default:
        Serial.printf("[WiFi] event: %d\n", event);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
    Serial.print("[MQTT Connect] Session present:");
    Serial.println(sessionPresent);

    setColor(strip.Color(0,0,0,0)); // OFF

    tone(BUZZER, NOTE_G4, 100, BUZZER_CHANNEL);
    sendDiscovery();
    mqtt_sub(MQTT_SUB_MOTION, 1);

    xTimerStart(discoveryTimer, 0);
    tone(BUZZER, NOTE_C5, 500, BUZZER_CHANNEL);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[MQTT Disconnect]");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
  tone(BUZZER, NOTE_C5, 100, BUZZER_CHANNEL);
  tone(BUZZER, NOTE_G4, 200, BUZZER_CHANNEL);
}

void onMqttSubscribe(uint16_t id, uint8_t qos) {
  Serial.printf("[MQTT subscribe] %d\n", id);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.print("[MQTT Unsubscribe]");
  Serial.print("  packetId: ");  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {

  Serial.print("MQTT RECEIVE-- ");
  Serial.println(topic);
/*
  Serial.print("  qos: ");  Serial.print(properties.qos);
  Serial.print("  dup: ");  Serial.print(properties.dup);
  Serial.print("  retain: ");  Serial.println(properties.retain);
  
  Serial.print("  index: ");  Serial.print(index);
  Serial.print("  total: ");  Serial.println(total);
*/
  if (strncmp(payload, "ON", 2) == 0) {
    sleeping = false;
    tone(BUZZER, 1200, 25, BUZZER_CHANNEL);
    tone(BUZZER, 1300, 50, BUZZER_CHANNEL);
  } else if (strncmp(payload, "OFF", 3) == 0) {
    sleeping = true;
    tone(BUZZER, 1300, 50, BUZZER_CHANNEL);
    tone(BUZZER, 1200, 25, BUZZER_CHANNEL);
  } else {

    // Deserialize the JSON document
    const int jsonsize = JSON_OBJECT_SIZE(4); // one object with 4 elements in it
    StaticJsonDocument<jsonsize> doc;
    DeserializationError error = deserializeJson(doc, payload);
    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do   auto time = doc["time"].as<long>();

  // RGBW = set the neopixel.
  // if we're not sleeping this will only last until the next sensor reading
    if (doc.containsKey("r")) {
      //uint8_t r,g,b,w;
      r = doc["r"];
      g = doc["g"];
      b = doc["b"];
      w = doc["w"];
      Serial.printf("%d %d %d %d",r,g,b,w);
      setColor(strip.Color(r,g,b,w));

    // TONE, DURATION = play a tone
    } else if (doc.containsKey("tone")) {
      int t,d;
      t = doc["tone"];
      d = doc["duration"];
      tone(BUZZER, t,d, BUZZER_CHANNEL);
    }
  }
}

void onMqttPublish(uint16_t id) {
  //Serial.printf("[MQTT publish] %d\n", id);
}

void setup() {
#if HAVE_NEOPIXEL
#else
  pinMode(LED, OUTPUT);
#endif
  pinMode(BUZZER, OUTPUT);
  pinMode(RADAR, INPUT);

  Serial.begin(115200);

#if HAVE_NEOPIXEL
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(MAXBRIGHT); // Set BRIGHTNESS (max = 255)
#endif

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  discoveryTimer = xTimerCreate("discoveryTimer", pdMS_TO_TICKS(DISCOVERY_INTERVAL), pdTRUE, // autoreload
       (void*)0, reinterpret_cast<TimerCallbackFunction_t>(sendDiscovery));
  motionDebounce = xTimerCreate("radarDebounce", pdMS_TO_TICKS(MOTION_INTERVAL), pdFALSE,
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

  Serial.print("[setup] MQTT Client ID=");
  Serial.println(mqttClient.getClientId());

  connectToWifi();

#if HAVE_TEMPSENSOR
  sensors.begin(); // defaults to 9 bit precision, up to 12 is available
  int count = sensors.getDeviceCount();
  haveSensor = count > 0;
  sensors.setResolution(12);

  Serial.print("  Sensors found: ");
  Serial.println(count);
#endif

#if HAVE_NEOPIXEL==1
// Set a default color 
  r = 20; g = b = w = 0;
  setColor(strip.Color(r,g,b,w));
  strip.clear(); // go dark 
  strip.show();
#endif  
}


void loop() {
    // I left out some code for reading the DS18S20 temp
    // sensor since I am pretty sure the sensor does not work anyway!

// "sleeping" means don't blink the neopixel
// to show motion, because it's annoying when we're sleeping
// also though it makes testing colors easier with the JSON command over MQTT
// since motion

#if HAVE_RADAR
  int radar = digitalRead(RADAR);
  if (radar == 1) {
    if (retrigger_ok) {
      if (!sleeping) {
#if HAVE_NEOPIXEL
        setColor(strip.Color(r,g,b,w));
#else
        digitalWrite(LED, 1); // Turn on LED
#endif
      }
      mqtt_pub(MQTT_PUB_RADAR, 1, "ON");
      xTimerStart(motionDebounce,0);
      retrigger_ok = 0;
    }
  } else {
    if (!sleeping && retrigger_ok) {
#if HAVE_NEOPIXEL
      setColor(strip.Color(0, 0, 0, 0)); // DARK
#else
      digitalWrite(LED, 0); // Turn off LED
#endif
    }
  }
#endif

#if HAVE_PIR
  int pir = digitalRead(PIR);
  if (pir == 1) {
    if (retrigger_ok) {
#if HAVE_NEOPIXEL
      setColor(strip.Color(r,g,b,w));
#else
      digitalWrite(LED, 1); // Turn on LED
#endif
      mqtt_pub(MQTT_PUB_PIR, 1, "ON");
      xTimerStart(motionDebounce,0);
      retrigger_ok = 0;
    }
  } else {
#if HAVE_NEOPIXEL
    setColor(strip.Color(0, 0, 0, 0)); // DARK
#else
    digitalWrite(LED, 0); // Turn off LED
#endif
  }
#endif
}
