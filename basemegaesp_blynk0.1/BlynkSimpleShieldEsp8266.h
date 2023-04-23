/**
 * @file       BlynkSimpleShieldEsp8266.h
 * @author     Volodymyr Shymanskyy
 * @license    This project is released under the MIT License (MIT)
 * @copyright  Copyright (c) 2015 Volodymyr Shymanskyy
 * @date       Jun 2015
 * @brief
 *
 */

#ifndef BlynkSimpleShieldEsp8266_h
#define BlynkSimpleShieldEsp8266_h

#ifdef ESP8266
#error This code is not intended to run on the ESP8266 platform! Please check your Tools->Board setting.
#endif

#ifndef BLYNK_INFO_CONNECTION
#define BLYNK_INFO_CONNECTION  "ESP8266"
#endif

#ifndef BLYNK_ESP8266_MUX
#define BLYNK_ESP8266_MUX  1
#endif

#define BLYNK_SEND_ATOMIC
#define BLYNK_SEND_CHUNK 40

#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <utility/BlynkFifo.h>
#include <ESP8266_Lib.h>

void http_process(ESP8266* client, uint8_t mux_id, uint32_t len);

class BlynkTransportShieldEsp8266
{
    static void onData(uint8_t mux_id, uint32_t len, void* ptr) {
        ((BlynkTransportShieldEsp8266*)ptr)->onData(mux_id, len);
    }

    void onData(uint8_t mux_id, uint32_t len) {
        //BLYNK_LOG1(mux_id);
        if (mux_id != BLYNK_ESP8266_MUX) {
            http_process(client, mux_id, len);
            return;
        }
        //BLYNK_LOG2("Got ", len);
        while (len) {
            if (client->getUart()->available()) {
                uint8_t b = client->getUart()->read();
                 if(!buffer.push(b)) {
                    BLYNK_LOG1(BLYNK_F("Buffer overflow"));
                }
                len--;
            }
        }
    }

public:
    BlynkTransportShieldEsp8266()
        : client(NULL)
        , status(false)
        , domain(NULL)
        , port(0)
    {}

    void setEsp8266(ESP8266* esp8266) {
        client = esp8266;
        client->setOnData(onData, this);
    }

    void begin(const char* d,  uint16_t p) {
        domain = d;
        port = p;
    }

    bool connect() {
        if (!domain || !port)
            return false;
        status = client->createTCP(BLYNK_ESP8266_MUX, domain, port);
        return status;
    }

    void disconnect() {
        status = false;
        buffer.clear();
        client->releaseTCP(BLYNK_ESP8266_MUX);
    }

    size_t read(void* buf, size_t len) {
        uint32_t start = millis();
        //BLYNK_LOG4("Waiting: ", len, " Occuied: ", buffer.getOccupied());
        while ((buffer.getOccupied() < len) && (millis() - start < 1500)) {
            client->run();
        }
        return buffer.read((uint8_t*)buf, len);
    }
    size_t write(const void* buf, size_t len) {
        if (client->send(BLYNK_ESP8266_MUX, (const uint8_t*)buf, len)) {
            return len;
        }
        return 0;
    }

    bool connected() { return status; }

    int available() {
        client->run();
        //BLYNK_LOG2("Still: ", buffer.getOccupied());
        return buffer.getOccupied();
    }

private:
    ESP8266* client;
    bool status;
    BlynkFifo<uint8_t,256> buffer;
    const char* domain;
    uint16_t    port;
};

class BlynkWifi
    : public BlynkProtocol<BlynkTransportShieldEsp8266>
{
    typedef BlynkProtocol<BlynkTransportShieldEsp8266> Base;
public:
    BlynkWifi(BlynkTransportShieldEsp8266& transp)
        : Base(transp)
        , wifi(NULL)
    {}

    bool connectWiFi(const char* ssid, const char* pass)
    {
        ::delay(500);
        BLYNK_LOG2(BLYNK_F("Connecting to "), ssid);
        /*if (!wifi->restart()) {
            BLYNK_LOG1(BLYNK_F("Failed to restart"));
            return false;
        }*/
        if (!wifi->kick()) {
             BLYNK_LOG1(BLYNK_F("ESP is not responding"));
             return false;
        }
        if (!wifi->setEcho(0)) {
            BLYNK_LOG1(BLYNK_F("Failed to disable Echo"));
            return false;
        }
        String ver = wifi->ESP8266::getVersion();
        BLYNK_LOG1(ver);
        if (!wifi->enableMUX()) {
            BLYNK_LOG1(BLYNK_F("Failed to enable MUX"));
        }
        if (!wifi->setOprToStation()) {
            BLYNK_LOG1(BLYNK_F("Failed to set STA mode"));
            return false;
        }
        if (wifi->joinAP(ssid, pass)) {
            String my_ip = wifi->getLocalIP();
            BLYNK_LOG1(my_ip);
        } else {
            BLYNK_LOG1(BLYNK_F("Failed to connect WiFi"));
            return false;
        }
        /*if (!wifi->setStationIp("192.168.188.45","192.168.188.1","255.255.255.0",3)) {
            BLYNK_LOG1(BLYNK_F("Failed to set IP"));
            return false;
        }*/
        BLYNK_LOG1(BLYNK_F("Connected to WiFi"));
        return true;
    }

    void config(ESP8266&    esp8266,
                const char* auth,
                const char* domain = BLYNK_DEFAULT_DOMAIN,
                uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
        Base::begin(auth);
        wifi = &esp8266;
        this->conn.setEsp8266(wifi);
        this->conn.begin(domain, port);
    }

    void begin(const char* auth,
               ESP8266&    esp8266,
               const char* ssid,
               const char* pass,
               const char* domain = BLYNK_DEFAULT_DOMAIN,
               uint16_t    port   = BLYNK_DEFAULT_PORT)
    {
         /*Serial.println(wifi->getIPStatus());
        connectWiFi(ssid, pass);
        Serial.println(wifi->getIPStatus());
        while(this->connect() != true) {
          Serial.println(wifi->getIPStatus());
          if (!wifi->getIPStatus().startsWith("STATUS:2")) {
            connectWiFi(ssid, pass);
            Serial.println("WIFI");
          }
          BLYNK_LOG1(BLYNK_F("..."));
        }*/
        config(esp8266, auth, domain, port);
        Serial.println(wifi->getIPStatus());
        int mytimeout = millis() / 1000;
        if (!wifi->getIPStatus().startsWith("STATUS:2")) {
          while (connectWiFi(ssid, pass) == false) { 
            if((millis() / 1000) > mytimeout + 8) {  // try for less than 9 seconds
              break;
            }
          }
          //connectWiFi(ssid, pass);
        }
        Serial.println(wifi->getIPStatus());
        if (wifi->getIPStatus().startsWith("STATUS:2")) {
          /*if (this->connect()) {
            BLYNK_LOG1(BLYNK_F("Connected!"));
          } else {
            BLYNK_LOG1(BLYNK_F("Connection failed!"));
          }*/
          mytimeout = millis() / 1000;
          while (this->connect() == false) {
            if((millis() / 1000) > mytimeout + 8) {  // try for less than 9 seconds
              BLYNK_LOG1(BLYNK_F("Connection failed!"));
              break;
            }
          }
        }
    }

    void reconnect(const char* ssid, const char* pass)
    {
      Serial.println(wifi->getIPStatus());
      int mytimeout = millis() / 1000;
      if (!wifi->getIPStatus().startsWith("STATUS:2")) {
        while (connectWiFi(ssid, pass) == false) { 
          if((millis() / 1000) > mytimeout + 8) {  // try for less than 9 seconds
            break;
          }
        }
        //connectWiFi(ssid, pass);
      }
      Serial.println(wifi->getIPStatus());
      if (wifi->getIPStatus().startsWith("STATUS:2")) {
        if (wifi->startTCPServer(8080)) {
          Serial.print("start tcp server ok\r\n");
        } else {
          Serial.print("start tcp server err\r\n");
        }
        /*if (this->connect()) {
          BLYNK_LOG1(BLYNK_F("Connected!"));
        } else {
          BLYNK_LOG1(BLYNK_F("Connection failed!"));
        }*/
        mytimeout = millis() / 1000;
        while (this->connect() == false) {
          if((millis() / 1000) > mytimeout + 8) {  // try for less than 9 seconds
            BLYNK_LOG1(BLYNK_F("Connection failed!"));
            break;
          }
        }
      }
    }

    void restart() {
      if (!wifi->restart()) {
        BLYNK_LOG1(BLYNK_F("Failed to restart"));
      }
    }

private:
    ESP8266* wifi;
};

static BlynkTransportShieldEsp8266 _blynkTransport;
BlynkWifi Blynk(_blynkTransport);

#include <BlynkWidgets.h>

#endif