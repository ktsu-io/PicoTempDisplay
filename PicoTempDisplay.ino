#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>

U8G2_SH1107_64X128_F_4W_SW_SPI u8g2(U8G2_R1, 10, 11, 9, 8, 12);

JsonDocument config;

WiFiMulti wifiMulti;
WiFiUDP udp;

const char* wifiName;
const char* wifiPassword;
int port;

std::map<std::string, float> prevTempsFloat;
std::map<std::string, float> tempsFloat;
std::map<std::string, std::string> temps;

//https://stackoverflow.com/a/27511119
std::vector<std::string> split(const std::string &s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim)) {
    //elems.push_back(item);
    elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
  }
  return elems;
}

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

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    // read the packet into packetBufffer
    int n = udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;

    std::string packet(packetBuffer);
    std::vector<std::string> parts = split(packet, ' ');
    if(parts.size() >= 2)
    {
      auto& zone = parts[0];
      auto& tempStr = parts[1];
      float tempFloat = atof(tempStr.c_str());
      float prevTempFloat = tempsFloat[zone];
      if(tempFloat != prevTempFloat)
      {
        prevTempsFloat[zone] = prevTempFloat;
      }
      tempsFloat[zone] = tempFloat;
      temps[zone] = tempStr;
    }
  }

  
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    std::string indicator = "\u004E";
    u8g2.drawStr(0, 64, indicator.c_str());
    u8g2.setFont(u8g2_font_open_iconic_www_2x_t);
    indicator = "\u0051";
    u8g2.drawStr(20, 64, indicator.c_str());
    if(!WiFi.isConnected())
    {
      indicator = "\u004A";
      u8g2.drawStr(20, 64, indicator.c_str());
    }

    int line = 1;
    u8g2.setFont(u8g2_font_helvR10_tf);
    for (const auto& [zone, tempStr]: temps) {
      u8g2.drawStr(20, 14 * line++, std::string(tempStr + " " + zone).c_str());
    }
    line = 1;
    u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);
    for (const auto& [zone, tempStr]: temps) {
      indicator = "\u004C";
      float prev = prevTempsFloat[zone];
      if(prev == 0)
      {
        indicator = "\u0057";
      }
      else if(prev < tempsFloat[zone])
      {
        indicator = "\u004F";
      }

      u8g2.drawStr(0, 16 * line++, indicator.c_str());
    }
  } while ( u8g2.nextPage() );
}
