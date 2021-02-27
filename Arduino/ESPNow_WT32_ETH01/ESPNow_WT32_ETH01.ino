#include <ETH.h>
#include <ArduinoJson.h>
#include <MQTT.h>
#include "config.h"

#include <esp_wifi.h>
#include <esp_now.h>

#ifdef ETH_CLK_MODE
#undef ETH_CLK_MODE
#endif

// IDEA: ESP-Now >> WT32-ETH01 >> ETH0 >> MQTT

/*  reference //
Very limited info on WT32-ETH01 at https://www.seeedstudio.com/Ethernet-module-based-on-ESP32-series-WT32-ETH01-p-4736.html
LAN8720 code: https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/ETH_LAN8720_internal_clock/ETH_LAN8720_internal_clock.ino
LAN8720 Pins: https://esphome.io/components/ethernet.html#configuration-for-wireless-tag-wt32-eth01
              https://github.com/arendst/Tasmota/issues/9496#issuecomment-715338713
ESP Now code: https://github.com/HarringayMakerSpace/ESP-Now
//            */
#define ETH_CLK_MODE  ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN GPIO_NUM_16
#define ETH_TYPE      ETH_PHY_LAN8720
#define ETH_ADDR      1
#define ETH_MDC_PIN   GPIO_NUM_23
#define ETH_MDIO_PIN  GPIO_NUM_18

WiFiClient net;
MQTTClient client(256);

volatile boolean haveReading = false;
char mqtt_msg[256];

static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_ETH_START:
    Serial.println("ETH Started");
    //set eth hostname here
    ETH.setHostname(FRIENDLY_NAME);
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected");
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    if (ETH.fullDuplex())
    {
      Serial.print(", FULL_DUPLEX");
    }
    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps");
    eth_connected = true;
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected");
    eth_connected = false;
    break;
  case SYSTEM_EVENT_ETH_STOP:
    Serial.println("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}

void initEspNow()
{
  if (esp_now_init() != 0)
  {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }

  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    Serial.print("$$"); // $$ just an indicator that this line is a received ESP-Now message

    String deviceMac = "";
    deviceMac += String(mac[0], HEX);
    deviceMac += String(mac[1], HEX);
    deviceMac += String(mac[2], HEX);
    deviceMac += String(mac[3], HEX);
    deviceMac += String(mac[4], HEX);
    deviceMac += String(mac[5], HEX);
    const size_t temp_size = sizeof(sensorData.temperature) / sizeof(sensorData.temperature[0]);
    const size_t pres_size = sizeof(sensorData.pressure) / sizeof(sensorData.pressure[0]);
    const size_t hum_size = sizeof(sensorData.humidity) / sizeof(sensorData.humidity[0]);
    const size_t capacity = JSON_ARRAY_SIZE(temp_size) + JSON_ARRAY_SIZE(pres_size) + JSON_ARRAY_SIZE(hum_size) + JSON_OBJECT_SIZE(8) + 200;
    DynamicJsonDocument doc(capacity);
    doc["mac"] = deviceMac;
    memcpy(&sensorData, data, sizeof(sensorData));
    doc["motion"] = sensorData.motion;
    doc["friendly_name"] = sensorData.friendlyName;
    doc["name"] = sensorData.deviceName;
    doc["battery_percent"] = sensorData.battery_percent;
    JsonArray temperature = doc.createNestedArray("temperature");
    for (int i = 0; i < temp_size; i++)
    {
      temperature.add(sensorData.temperature[i]);
    }
    JsonArray pressure = doc.createNestedArray("pressure");
    for (int i = 0; i < pres_size; i++)
    {
      pressure.add(sensorData.pressure[i]);
    }
    JsonArray humidity = doc.createNestedArray("humidity");
    for (int i = 0; i < hum_size; i++)
    {
      humidity.add(sensorData.humidity[i]);
    }
    serializeJson(doc, Serial);
    serializeJson(doc, mqtt_msg);
    Serial.println();

    haveReading = true;
  });
}

void configDeviceAP()
{
  bool result = WiFi.softAP(ESPSSID, ESPPASS, WIFI_CHANNEL, 1); //hidden network
  if (!result)
  {
    Serial.println("AP Config failed.");
  }
  else
  {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(ESPSSID));
  }
}

void connectMQTT()
{
  Serial.print("\nMQTT .");
  while (!client.connect(FRIENDLY_NAME, MQTT_USER, MQTT_PASS) && eth_connected)
  {
    Serial.print(".");
    delay(500);
  }
  if (client.connected())
    Serial.println(" connected!");
}

void resetWiFi()
{
  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_AP);
}

void setup()
{
  Serial.begin(115200);
  WiFi.onEvent(WiFiEvent);

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println();
  WiFi.mode(WIFI_AP);
  esp_wifi_set_mac(WIFI_IF_AP, &mac[0]);
  Serial.print("This node AP mac: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("This node STA mac: ");
  Serial.println(WiFi.macAddress());

  configDeviceAP();
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
  client.begin(MQTT_HOST, MQTT_PORT, net);
  initEspNow();
}

unsigned long heartBeat;

void loop()
{
  if (eth_connected)
  {
    if (!client.connected())
    {
      connectMQTT();
    }
    else
    {
      client.loop();
    }

    if (millis() - heartBeat > 30000)
    {
      Serial.println("Waiting for ESP-NOW messages...");
      heartBeat = millis();
    }

    if (haveReading)
    {
      haveReading = false;
      client.publish("home/espnow/" + String(sensorData.friendlyName), mqtt_msg);
    }
  }
}