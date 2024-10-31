#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoOTA.h>
#include "config.h"

// Global objects
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

// State Variables
unsigned long lastRPMTime = 0;
unsigned long lastReadingTime = 0;
volatile int fan1_pulse_count = 0, fan2_pulse_count = 0;
int fan1_rpm = 0, fan2_rpm = 0;

// Helper function prototypes
void setupWifi();
void reconnectMQTT();
void publishData();
void updateFanSpeed();
void updateBuzzer(bool state);
void checkFanStatus();

// ISR for fan RPM counting
void IRAM_ATTR fan1TachISR() {
  fan1_pulse_count++;
}

void IRAM_ATTR fan2TachISR() {
  fan2_pulse_count++;
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Configure pins
  pinMode(FANPIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN1_TACH_PIN, INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN, INPUT_PULLUP);

  // Attach interrupts for fan tachometer signals
  attachInterrupt(digitalPinToInterrupt(FAN1_TACH_PIN), fan1TachISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACH_PIN), fan2TachISR, FALLING);

  // Setup WiFi and MQTT
  setupWifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);

  // Initialize OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void loop() {
  unsigned long currentMillis = millis();

    // Handle OTA updates
  ArduinoOTA.handle();

  // Reconnect MQTT if needed
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Update RPM and check fan status at intervals
  if (currentMillis - lastRPMTime >= INTERVAL) {
    checkFanStatus();
    lastRPMTime = currentMillis;
  }

  // Read temperature, humidity, and control fan speed at intervals
  if (currentMillis - lastReadingTime >= READING_INTERVAL) {
    updateFanSpeed();
    lastReadingTime = currentMillis;
  }
}

// Connect to WiFi
void setupWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");
}

// Reconnect to MQTT broker
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (client.connect(MQTT_DEVICE_NAME, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");
      delay(5000);
    }
  }
}

// Update fan speed based on temperature and humidity
void updateFanSpeed() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    analogWrite(FANPIN, 254);
    updateBuzzer(true);
    client.publish("fanControlSystem/status", "Sensor error: Failed to read from the DHT sensor!");
    return;
  }

  int fanSpeed = 0;
  if (humidity >= 60) {
    fanSpeed = 0;
    updateBuzzer(false);
  } else if (temperature > 35) {
    fanSpeed = 254;
    updateBuzzer(true);
  } else if (temperature > 30) {
    fanSpeed = map(temperature, 30, 35, 178, 229);
    updateBuzzer(false);
  } else if (temperature > 25) {
    fanSpeed = map(temperature, 25, 30, 127, 178);
    updateBuzzer(false);
  } else {
    fanSpeed = 0;
    updateBuzzer(false);
  }

  analogWrite(FANPIN, fanSpeed);

  // Publish data
  String message = "Temperature: " + String(temperature) + "C, Humidity: " + String(humidity) + "%";
  client.publish("fanControlSystem/status", message.c_str());
}

// Check fan status and calculate RPM
void checkFanStatus() {
  fan1_rpm = (fan1_pulse_count * 60) / 2;
  fan2_rpm = (fan2_pulse_count * 60) / 2;
  fan1_pulse_count = 0;
  fan2_pulse_count = 0;

  String fanStatus;
  if (fan1_rpm == 0 || fan2_rpm == 0) {
    fanStatus = "Fan failure detected!";
    updateBuzzer(true);
  } else {
    fanStatus = "Fans are operational";
    updateBuzzer(false);
  }

  client.publish("fanControlSystem/fan1RPM", String(fan1_rpm).c_str());
  client.publish("fanControlSystem/fan2RPM", String(fan2_rpm).c_str());
  client.publish("fanControlSystem/fanStatus", fanStatus.c_str());
}

// Publish data to MQTT
void publishData() {
  client.publish("fanControlSystem/temperature", String(dht.readTemperature()).c_str());
  client.publish("fanControlSystem/humidity", String(dht.readHumidity()).c_str());
}

// Control buzzer state
void updateBuzzer(bool state) {
  digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}