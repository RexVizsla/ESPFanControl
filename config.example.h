// Wi-Fi Configuration
#define WIFI_SSID "your_ssid_here"          // Your Wi-Fi SSID
#define WIFI_PASSWORD "your_password_here"  // Your Wi-Fi Password

// MQTT Configuration
#define MQTT_SERVER "YOUR_MQTT_BROKER_IP"  // Replace with your MQTT broker IP
#define MQTT_PORT 1883  // Default MQTT port, change if needed
#define MQTT_USER "YOUR_MQTT_USERNAME"  // Optional: MQTT username
#define MQTT_PASSWORD "YOUR_MQTT_PASSWORD"  // Optional: MQTT password

// DHT Sensor Configuration
#define DHTPIN D4                           // Pin for the DHT22 sensor
#define DHTTYPE DHT22                       // DHT22 sensor type

// Fan Control Pins
#define FANPIN D2                           // Fan's PWM control pin
#define FAN1_TACH_PIN D5                    // TACH signal pin for Fan 1
#define FAN2_TACH_PIN D6                    // TACH signal pin for Fan 2

// Buzzer Configuration
#define BUZZER_PIN D7                       // Pin for the buzzer