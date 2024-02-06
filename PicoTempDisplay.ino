#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <deque>

U8G2_SH1107_64X128_F_4W_SW_SPI u8g2(U8G2_R1, 10, 11, 9, 8, 12);

JsonDocument config;

WiFiMulti wifiMulti;
WiFiUDP udp;

const char* wifiName;
const char* wifiPassword;
int port;

std::deque<std::string> temps;

void setup() {
  Serial.begin();
  u8g2.begin();
  LittleFS.begin();

  char configFileName[32] = "config.json";
  auto f = LittleFS.open(configFileName, "r");
  if (!f) {
    Serial.println("Failed to find config");
    while (1) { delay(10); }
  }

  std::string jsonString;
  char buffer[16];
  while(f.available())
  {
    int bytesRead = f.readBytes(buffer, 16);
    jsonString.append(buffer, bytesRead);
  }

  auto jsonError = deserializeJson(config, jsonString.c_str());
  if (jsonError) {
    Serial.println("Failed to load config");
    Serial.println(jsonError.f_str());
    while (1) { delay(10); }
  }


  wifiName = config["wifiName"];
  wifiPassword = config["wifiPassword"];
  port = config["port"];

  wifiMulti.addAP(wifiName, wifiPassword);
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Failed to connect to wifi, rebooting");
    delay(10000);
    rp2040.reboot();
  }

  udp.begin(port);
}

char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];

double i = 0;
void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    // read the packet into packetBufffer
    int n = udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    temps.push_front(packetBuffer);
  }
  temps.resize(4);

  ++i;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR12_tf);
    for(int idx=0; idx < temps.size(); ++idx)
    {
      u8g2.drawStr(0, 16 * (idx + 1), temps[idx].c_str());
    }
  } while ( u8g2.nextPage() );
}
