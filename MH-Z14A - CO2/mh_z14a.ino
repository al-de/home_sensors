#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "wifi-ssid"  // CHANGE: WIFI SSID
#define WIFI_PASSWORD "wifi-password"  // CHANGE: WIFI Password

#define MQTT_HOST "0.0.0.0"  // CHANGE: IP of MQTT Broker
#define MQTT_PORT 1883  // CHANGE: Port of MQTT Broker

// CO2 MQTT Topic
#define MQTT_PUB_CO2 "esp32/mh_z14a/co2"  // CHANGE: MQTT Topic to publish the CO2 value to

#define CHILD_ID_AIQ 0
// Digital pin connected to the MH-Z14A sensor
#define AIQ_SENSOR_ANALOG_PIN 4  // Connect MH-Z14A PWM pin to pin 4 of the ESP32

// Interval at which to publish sensor readings
const long interval = 10000;  // in ms

float valAIQ = 0.0;
float lastAIQ = 0.0;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;  // Stores last time CO2 was published

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // OPTINOAL: if the broker requires authentication (username and password), uncomment below
  //mqttClient.setCredentials("mqtt-user", "mqtt-password");  // CHANGE: MQTT User, MQTT Password
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New co2 sensor readings
    while(digitalRead(AIQ_SENSOR_ANALOG_PIN) == HIGH) {
    ;
    }
    uint32_t duration = pulseIn(AIQ_SENSOR_ANALOG_PIN, HIGH);
    long co2ppm = 5 * ((duration / 1000) - 2);
    if ((co2ppm != lastAIQ) && (abs(co2ppm - lastAIQ) >= 10)) {
      lastAIQ = ceil(co2ppm);
    }

    // Check if any reads failed and exit early (to try again).
    if (isnan(co2ppm)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    
    // Publish an MQTT message on CO2 topic
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_CO2, 1, true, String(co2ppm).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_CO2, packetIdPub1);
    Serial.printf("Message: %.2f \n", co2ppm);
  }
}