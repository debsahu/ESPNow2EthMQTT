#include <pgmspace.h>

const char FRIENDLY_NAME[] = "ESPNow2EthMQTT"; // do not uses spaces
const char DEVICE_NAME_FULL[] = "ESP Now SErver to ETH MQTT Client";

#define WIFI_CHANNEL 1

const char ESPSSID[] = "ESPNOWNETWORK";
const char ESPPASS[] = "ESP826632";

const char MQTT_HOST[] = "mqtt_ip";
int        MQTT_PORT   = 1883;
const char MQTT_USER[] = "mqtt_user_name";
const char MQTT_PASS[] = "mqtt_password";

struct __attribute__((packed)) SENSOR_DATA
{
    bool motion;
    char friendlyName[20];
    char deviceName[20];
    uint8_t battery_percent;
    float temperature[10];
    float humidity[10];
    float pressure[10];
} sensorData;

uint8_t mac[] = {0x36, 0x33, 0x33, 0x33, 0x33, 0x33}; // This is mac of ESPNow Server