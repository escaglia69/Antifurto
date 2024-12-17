#define BLYNK_TEMPLATE_ID "TMPL4B5_GeqGz"
#define BLYNK_TEMPLATE_NAME "ArduinoR4"
#define TRX_PIN  12

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <SPI.h>
#include <WiFiS3.h>
#include "BlynkSimpleWiFiShieldUnoR4.h"
#include "RTC.h"
#include <WiFiUdp.h>
#include "secret.h"
#include "RF24.h"
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
//#include "DigitalIO.h"
#include <nRF24L01.h>
#include <RCSwitch.h>


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

constexpr unsigned int LOCAL_PORT = 2390;      // local port to listen for UDP packets
constexpr int NTP_PACKET_SIZE = 48; // NTP timestamp is in the first 48 bytes of the message
int wifiStatus = WL_IDLE_STATUS;
IPAddress timeServer(162, 159, 200, 123); // pool.ntp.org NTP server
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
RTCTime currentTime;
bool isFirstConnect = true;
WidgetLCD wlcd(V5);
RF24 radio(5,9);
byte saddr[][6] = { "s0001" };
struct sensorDataRecord {
  int sid;
  float temp;
  boolean intDoor;
  boolean extDoor;
  float vcc;
  char dateTime[20];
};
struct dataRecord {
  uint16_t sid;
  uint8_t temp[4];
  uint8_t intDoor;
  uint8_t extDoor;
  uint8_t vcc[4];
  char dateTime[20];
};
const int SENSOR_NUM = 14;
sensorDataRecord sensorData[SENSOR_NUM] = {{0,0.0,1,1,0.0,"01/01/2016 00:00:00"},{1,0.0,1,1,0.0,"01/01/2016 00:00:00"},{2,0.0,1,1,0.0,"01/01/2016 00:00:00"},{3,0.0,1,1,0.0,"01/01/2016 00:00:00"},{4,0.0,1,1,0.0,"01/01/2016 00:00:00"},{5,0.0,1,1,0.0,"01/01/2016 00:00:00"},{6,0.0,1,1,0.0,"01/01/2016 00:00:00"},{7,0.0,1,1,0.0,"01/01/2016 00:00:00"},{8,0.0,1,1,0.0,"01/01/2016 00:00:00"},{9,0.0,1,1,0.0,"01/01/2016 00:00:00"},{10,0.0,1,1,0.0,"01/01/2016 00:00:00"},{11,0.0,1,1,0.0,"01/01/2016 00:00:00"},{12,0.0,1,1,0.0,"01/01/2016 00:00:00"},{13,0.0,1,1,0.0,"01/01/2016 00:00:00"}};
dataRecord tempData;
int sid;
int sidOnDisplay = SENSOR_NUM - 1;
//LiquidCrystal_I2C lcd(0x27, 16,2);  // Set the LCD I2C address
LiquidCrystal_I2C lcd(0x3F, 16,2);  // Set the LCD I2C address

const int rftxdatapin = 12;
const int highButtonPin = A3;
const int lowButtonPin = A2;
const int greenLedPin = A1;
const int yellowLedPin = A0;
int greenLedState = LOW;         // the current state of the output pin
int yellowLedState = LOW;         // the current state of the output pin
int displayOn = HIGH;
int highButtonState;             // the current reading from the input pin
int lastHighButtonState = HIGH;   // the previous reading from the input pin
long lastHighDebounceTime = 0;  // the last time the output pin was toggled
int lowButtonState;             // the current reading from the input pin
int lastLowButtonState = HIGH;   // the previous reading from the input pin
long lastLowDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
long longPressTime = 1000;
long lowButtonTimer = 0;
boolean lowButtonActive = false;
boolean lowLongPressActive = false;
long highButtonTimer = 0;
boolean highButtonActive = false;
boolean highLongPressActive = false;
boolean dhcpConnected = false;
boolean armed = false;
boolean alarm = false;
boolean alarmSent = false;
char lcdl[33];
bool Connected2Blynk = false;
char startedAt[20] = "01/01/2016 00:00:00";
float tempN;
float tempS;
float tempE;
char tmpid[2] = "";
char jsonline[128] ={0};
char prefix[9] = "01000101";
char suffixup[9] = "10010001";
char suffixdown[9] = "10010100";
char code[25] = {0};
bool firstsync = true;
String inputString        = "";
char packet_buf[256]      = {0};        // packet payload buffer
unsigned int bit_pos      = 0;          // bit reader bit position
unsigned int curtain      = 1;

BlynkTimer timer;
BlynkTimer ttimer;
RCSwitch mySwitch = RCSwitch();
WiFiServer server(80);

/**
 * Calculates the current unix time, that is the time in seconds since Jan 1 1970.
 * It will try to get the time from the NTP server up to `maxTries` times,
 * then convert it to Unix time and return it.
 * You can optionally specify a time zone offset in hours that can be positive or negative.
*/
unsigned long getUnixTime(int8_t timeZoneOffsetHours = 0, uint8_t maxTries = 5){
  // Try up to `maxTries` times to get a timestamp from the NTP server, then give up.
  for (size_t i = 0; i < maxTries; i++){
    sendNTPpacket(timeServer); // send an NTP packet to a time server
    // wait to see if a reply is available
    delay(1000);

    if (Udp.parsePacket()) {
      Serial.println("packet received");
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      //or two words, long. First, extract the two words:
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      
      // Combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;

      // Now convert NTP time into everyday time:
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      unsigned long secondsSince1970 = secsSince1900 - seventyYears + (timeZoneOffsetHours * 3600);
      return secondsSince1970;
    }
  }

  return 0;
}


void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // You can also specify server:
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, IPAddress(192,168,1,100), 8080);

  Serial.println("\nStarting connection to ntp server...");
  Udp.begin(LOCAL_PORT);
  RTC.begin();

  // Get the current date and time from an NTP server and convert
  // it to UTC +2 by passing the time zone offset in hours.
  // You may change the time zone offset to your local one.
  auto unixTime = getUnixTime(2);
  Serial.print("Unix time = ");
  Serial.println(unixTime);
  RTCTime timeToSet = RTCTime(unixTime);
  RTC.setTime(timeToSet);

  // Retrieve the date and time from the RTC and print them
  RTC.getTime(currentTime); 
  Serial.println("The RTC was just set to: " + String(currentTime));
  //String(currentTime).toCharArray(startedAt,20);
  sprintf(startedAt, "%02d/%02d/%04d %02d:%02d:%02d", currentTime.getDayOfMonth(), currentTime.getMonth(), currentTime.getYear(), currentTime.getHour(), currentTime.getMinutes(), currentTime.getSeconds());
  
  Serial.println(F("Setup!"));
  lcd.init();
  lcd.noBacklight(); // finish with backlight on
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print(F("Avviato!"));
  Serial.print(F("Started at: "));
  Serial.println(startedAt);
  
  pinMode(highButtonPin, INPUT);
  //digitalWrite(highButtonPin, HIGH); //su UNO R4 ora i pin Ax sono solo analogici
  pinMode(lowButtonPin, INPUT);
  //digitalWrite(lowButtonPin, HIGH); //su UNO R4 ora i pin Ax sono solo analogici
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  // set initial LED state
  digitalWrite(greenLedPin, greenLedState);
  digitalWrite(yellowLedPin, yellowLedState);
  /*####################*/
  //digitalWrite(13, LOW);
  /*####################*/
  //mySwitch.enableTransmit(rftxdatapin);
  //mySwitch.setPulseLength(399);
  pinMode(TRX_PIN, OUTPUT);
  inputString.reserve(200);
  mySwitch.enableTransmit(rftxdatapin);
  mySwitch.setPulseLength(399);

  radio.begin();
  //radio.setChannel(70); //************ATTENZIONE*****************
  //radio.setPayloadSize(16);
  
  // Min speed (for better range I presume)
  radio.setDataRate( RF24_250KBPS );
  // 8 bits CRC
  radio.setCRCLength( RF24_CRC_8 );
  // increase the delay between retries & # of retries
  radio.setRetries(15,15);
  //radio.setChannel(72);
  radio.openReadingPipe(1,saddr[0]);
  radio.startListening();


  //Blynk.begin(auth, wifi, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, wifi, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, wifi, ssid, pass, IPAddress(192,168,1,100), 8080);

  if (Blynk.connected()) {
    Serial.println(F("Rete in setup"));
    Blynk.virtualWrite(V9, 0);
  }
  timer.setInterval(60000L, CheckConnection);
  ttimer.setInterval(301000L, SendTemp);
  server.begin();
 
  /*if (wifi.startTCPServer(8080)) {
      Serial.print(F("start tcp server ok\r\n"));
  } else {
      Serial.print(F("start tcp server err\r\n"));
  }
  
  if (wifi.setTCPServerTimeout(6)) { 
      Serial.print(F("set tcp server timout 6 seconds\r\n"));
  } else {
      Serial.print(F("set tcp server timout err\r\n"));
  }*/
}

void CheckConnection(){
  if (firstsync) {
    firstsync = false;
    sync();
    RTC.getTime(currentTime); 
    String(currentTime).toCharArray(startedAt,20);
  }
  Connected2Blynk = Blynk.connected();
  if(!Connected2Blynk){
    Serial.println(F("Not connected"));
    //Blynk.reconnect(ssid, pass);
    Blynk.connectWiFi(ssid,pass);
    Blynk.connect();
  }
}

void SendTemp() {
  Blynk.virtualWrite(V10, sensorData[1].temp);
  Blynk.virtualWrite(V11, sensorData[3].temp);
  Blynk.virtualWrite(V12, sensorData[4].temp);
  Blynk.virtualWrite(V13, sensorData[6].temp);
  Blynk.virtualWrite(V14, sensorData[7].temp);
  Blynk.virtualWrite(V15, sensorData[8].temp);
  Blynk.virtualWrite(V16, sensorData[10].temp);
  Blynk.virtualWrite(V17, sensorData[11].temp);
}

void sync() {
}

void loop()
{
  pushButtonHigh();
  pushButtonLow();
  int i=0;
  if (armed) {
    for (i=0; i<SENSOR_NUM; i++) {
      if ((sensorData[i].intDoor == 0) && (EEPROM.read(i*2) == 1)) {
        alarm = true;
        break;
      }
      if ((sensorData[i].extDoor == 0) && (EEPROM.read((i*2)+1) == 1)) {
        alarm = true;
        break;
      }
    }
  }
  yellowLedState = alarm;
  digitalWrite(yellowLedPin, yellowLedState);
  if ((alarm) && !alarmSent) {
    Serial.println("OK4");
    Blynk.virtualWrite(V6, 1);
    //String text = "Allarme varco ";
    //Blynk.notify(text+=sid);
    Serial.println(String("Allarme varco: ") + sid);
    Blynk.logEvent("alarm", String("Allarme varco: ") + sid);
    alarmSent = true;
  }

  if( radio.available()){
    while (radio.available()) {                                   // While there is data ready
      //Serial.println("OK2");
      radio.read( &tempData, sizeof(tempData) );                  // Get the payload
    }
    if ((tempData.sid<SENSOR_NUM) && (tempData.sid>=0)) {
      sid = tempData.sid;
      copySensorData(sid);
      lcdLine(sid);
      printToSerial(lcdl,sensorData[sid].dateTime);
      printOnLCD(lcdl);
      printOnWLCD(lcdl);
      delay(10);
    } else if((tempData.sid<100) && (tempData.sid>=90)) {
      /*Serial.print(tempData.sid);
      Serial.print(" ");
      Serial.println(tempData.temp);*/
      if(tempData.sid==91) {
        //tempN=tempData.temp;
        memcpy(&tempN, &tempData.temp, 4);
        //sender2.send(tempN,tempData.vcc > VOLTAGE_THRESH);
        Serial.print(F("TEMP NORD: "));
        Serial.println(tempN);
        tempLine(tempData.sid,'N');
      }
      if(tempData.sid==92) {
        //tempS=tempData.temp;
        memcpy(&tempS, &tempData.temp, 4);
        Serial.print(F("TEMP SUD: "));
        Serial.println(tempS);
        tempLine(tempData.sid,'S');
      }
      if(tempData.sid==90) {
        //tempE=tempData.temp;
        memcpy(&tempE, &tempData.temp, 4);
        //sender1.send(tempE,tempData.vcc > VOLTAGE_THRESH);
        Serial.print(F("TEMP EST: "));
        Serial.println(tempE);
        tempLine(tempData.sid,'E');
      }
      printToSerial(lcdl,tempData.dateTime);
      printOnLCD(lcdl);
      printOnWLCD(lcdl);
    } else {
      Serial.print(F("####SID out of range!#### "));
      Serial.println(tempData.sid);
      //printf("####SID: %2d out of range!####\n",tempData.sid);
    }
  }
  
  WiFiClient client = server.available();
  if (client) {
    bool hom = false;
    bool printAllEnabled = false;
    bool help = false;
    bool json = false;
    Serial.println("new client");
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            if (hom) {
              hom = false;
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println();
              // the content of the HTTP response follows the header:
              client.print("<p style=\"font-size:3vw;\"><a href=\"/help\">Help</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/arm\">Arm</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/unarm\">Unarm</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/alarmReset\">Alarm reset</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/sync\">Sync</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/reset\">Restart</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/allIntEnabled\">All internal enabled</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/allIntDisabled\">All internal disabled</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/allExtEnabled\">All external enabled</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/allExtDisabled\">All external disabled</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/printEnabled\">Print enabled</a><br></p>");
              client.print("<p style=\"font-size:3vw;\"><a href=\"/json\">JSON</a><br></p>");
              client.print("<form action=intEnable>\n\t<label>Enable Int Id: </label>\n\t<input type=text name=id required>\n\t<input type=submit value=Submit>\n</form>");
              client.print("<form action=intDisable>\n\t<label>Disable Int Id: </label>\n\t<input type=text name=id required>\n\t<input type=submit value=Submit>\n</form>");
              client.print("<form action=extEnable>\n\t<label>Enable Ext Id: </label>\n\t<input type=text name=id required>\n\t<input type=submit value=Submit>\n</form>");
              client.print("<form action=extDisable>\n\t<label>Disable Int Id: </label>\n\t<input type=text name=id required>\n\t<input type=submit value=Submit>\n</form>");
              client.print("<form>\n\t<label>Awning Id: </label>\n\t<input type=text name=id required>\n\t<button formaction=awningUp>UP</button>\n\t<button formaction=awningStop>STOP</button>\n\t<button formaction=awningDown>DOWN</button>\n</form>");
              client.print("<form>\n\t<label>Shutter Id: </label>\n\t<input type=text name=id required>\n\t<button formaction=shutterOpen>OPEN</button>\n\t<button formaction=shutterClose>CLOSE</button>\n</form>");
              //client.print("\t<input type=submit value=Submit>\n</form>");
              client.print("<br>");
              client.print("<script>\n\tfunction getId(){\n\tvar id = document.getElementById('awid').value;\n\t}\n\t</script>");
              // The HTTP response ends with another blank line:
              client.println();
            }
            if (printAllEnabled) {
              printAllEnabled = false;
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println();
              char line[13] = {0};
              client.print("<p style=\"font-size:3vw;\"><a>Internal</a><br></p>");
              for (int id=0; id<SENSOR_NUM*2; id+=2) {
                sprintf(line,"%2d\t%s\n",id/2,EEPROM.read(id)==0?"Disabled":"Enabled");
                client.print(line);
                client.print("<br>");
              }
              client.print("<p style=\"font-size:3vw;\"><a>External</a><br></p>");
              for (int id=1; id<SENSOR_NUM*2; id+=2) {
                sprintf(line,"%2d\t%s\n",id/2,EEPROM.read(id)==0?"Disabled":"Enabled");
                client.print(line);
                client.print("<br>");
              }
            } else if (help) {
              help = false;
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println();
              client.print("<p style=\"font-size:3vw;\">arm<br>unarm<br>alarmreset<br>allIntEnabled<br>allIntDisabled<br>");
              client.print("allExtEnabled<br>allExtDisabled<br>printEnabled<br>intEnable?id=<br>");
              client.print("intDisable?id=<br>extEnable?id=<br>extDisable?id=<br>sync<br>reset<br></p>");
            } else if (json) {
              json = false;
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println();
              client.print("{\"Sensor\": [\n");
              for (int id=0; id<SENSOR_NUM; id++) {
                printLineToJSON(id);
                client.print(jsonline);
              }
              char temp[6];
              dtostrf(tempN,5, 2, temp);
              client.print("],\n\"Temp N\":\"");
              client.print(temp);
              dtostrf(tempS,5, 2, temp);
              client.print("\",\n\"Temp S\":\"");
              client.print(temp);
              dtostrf(tempE,5, 2, temp);
              client.print("\",\n\"Temp E\":\"");
              client.print(temp);
              client.print("\",\n\"Started at\":\"");
              client.print(startedAt);
              client.print("\"\n}\n");
            }

            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see what is the client request":
        if (currentLine.endsWith("GET /")) {
          hom=true;
        }
        if (currentLine.endsWith("GET /help")) {
          hom=false;
          help=true;
        }
        else if (currentLine.endsWith("GET /arm")) {
          arm();
        }
        else if (currentLine.endsWith("GET /unarm")) {
          unarm();
        }
        else if (currentLine.endsWith("GET /alarmReset")) {
          resetAlarm();
        }
        else if (currentLine.endsWith("GET /sync")) {
          sync();
        }
        else if (currentLine.endsWith("GET /reset")) {
          NVIC_SystemReset();
        }
        else if (currentLine.endsWith("GET /allIntEnabled")) {
          setAllIntEnabled();
        }
        else if (currentLine.endsWith("GET /allIntDisabled")) {
          setAllIntDisabled();
        }
        else if (currentLine.endsWith("GET /allExtEnabled")) {
          setAllExtEnabled();
        }
        else if (currentLine.endsWith("GET /allExtDisabled")) {
          setAllExtDisabled();
        }
        else if (currentLine.endsWith("GET /printEnabled")) {
          hom=false;
          printAllEnabled=true;
        }
        else if (currentLine.endsWith("GET /json")) {
          hom=false;
          json=true;
        }
        else if (currentLine.indexOf("GET /intEnable?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < SENSOR_NUM)) {
            enableIntId(id);
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /intDisable?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < SENSOR_NUM)) {
            disableIntId(id);
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /extEnable?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < SENSOR_NUM)) {
            enableExtId(id);
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /extDisable?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < SENSOR_NUM)) {
            disableExtId(id);
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /awningUp?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < 16)) {
            if ((id==1) || (id > 4)) {
              old_curtain_command(1,id);
            } else {
              curtain_command(1,id);
            }
            Serial.println("UP "+String(id));
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /awningStop?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < 16)) {
            if ((id==1) || (id > 4)) {
              old_curtain_command(0,id);
            } else {
              curtain_command(0,id);
            }
            Serial.println("STOP "+String(id));
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /awningDown?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < 16)) {
            if ((id==1) || (id > 4)) {
              old_curtain_command(-1,id);
            } else {
              curtain_command(-1,id);
            }
            Serial.println("DOWN "+String(id));
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /shutterOpen?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < 14)) {
            openShutter(id);
            Serial.println("OPEN "+String(id));
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
        else if (currentLine.indexOf("GET /shutterClose?id=") >= 0) {
          int id = getId(client, currentLine);
          if ((id >=0) && (id < 14)) {
            closeShutter(id);
            Serial.println("CLOSE "+String(id));
          } else {
            Serial.println("ID "+String(id)+" invalid!");
          }
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
  timer.run();
  if(Blynk.connected()){
    Blynk.run();
    ttimer.run();
  }
}

int getId(WiFiClient &client, String &currentLine) {
  char chid[2] = {0};
  if (client.available()) {
    chid[0]=client.read();
  }
  if (client.available()) {
    chid[1]=client.read();
    if (chid[1]!=' ') {
      if (client.available()) {
        client.read();
      }
    }
  }
  int enid = atoi(&chid[0]);
  Serial.println("XXX");
  Serial.println(enid);
  currentLine = "";
  return enid;
}

void pushButtonHigh() {
  // read the state of the switch into a local variable:
  int highReading = analogRead(highButtonPin);
  if (highReading>127) {
    highReading=HIGH;
  } else {
    highReading=LOW;
  }

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:
  // If the switch changed, due to noise or pressing:
  if (highReading != lastHighButtonState) {
    // reset the debouncing timer
    lastHighDebounceTime = millis();
  }
  if ((millis() - lastHighDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // if the button state has changed:
    if (highReading != highButtonState) {
      highButtonState = highReading;
      // only toggle the LED if the new button state is HIGH
      if ((highButtonState == LOW) && ! highLongPressActive) {
        armed = !armed;
        greenLedState = !greenLedState;
        Blynk.virtualWrite(V0,armed);
        digitalWrite(greenLedPin, greenLedState);
        Serial.println(greenLedState?"Armed":"Unarmed");
      }
    } else {
        if (highButtonState == LOW) {
        if (highButtonActive == false) {
          highButtonActive = true;
          highButtonTimer = millis();
        }
        if (((millis() - highButtonTimer) > longPressTime) && (highLongPressActive == false)) {
          highLongPressActive = true;
          resetAlarm();
        }
      } else {
        if (highButtonActive == true) {
          if (highLongPressActive == true) {
            highLongPressActive = false;
          }
          highButtonActive = false;
        }
      }
    }
  }
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastHighButtonState = highReading;
}

void pushButtonLow() {
  int lowReading = analogRead(lowButtonPin);
  if (lowReading!=0) {
    lowReading=HIGH;
  } else {
    lowReading=LOW;
  }
  if (lowReading != lastLowButtonState) {
    // reset the debouncing timer
    lastLowDebounceTime = millis();
  }
  if ((millis() - lastLowDebounceTime) > debounceDelay) {
    if (lowReading != lowButtonState) {
      lowButtonState = lowReading;
      if ((lowButtonState == LOW) && !lowLongPressActive) {
        sidOnDisplay+=1;
        if (sidOnDisplay >= SENSOR_NUM) {
          sidOnDisplay -= SENSOR_NUM;
        }
        lcdLine(sidOnDisplay);
        printOnLCD(lcdl);
        printOnWLCD(lcdl);
      }
    } else {
      if (lowButtonState == LOW) {
        if (lowButtonActive == false) {
          lowButtonActive = true;
          lowButtonTimer = millis();
        }
        if (((millis() - lowButtonTimer) > longPressTime) && (lowLongPressActive == false)) {
          lowLongPressActive = true;
          displayOn = !displayOn;
          if (displayOn) {
            lcd.backlight();
            Blynk.virtualWrite(V9, 1);
          } else {
            lcd.noBacklight();
            Blynk.virtualWrite(V9, 0);
          }
        }
      } else {
        if (lowButtonActive == true) {
          if (lowLongPressActive == true) {
            lowLongPressActive = false;
          }
          lowButtonActive = false;
        }
      }
    }
  }
  lastLowButtonState = lowReading;
}

void arm() {
  armed = true;
  greenLedState = HIGH;
  digitalWrite(greenLedPin,greenLedState);
  Blynk.virtualWrite(V0,1);
  //Serial.println(F("Armato!"));
}

void unarm() {
  armed = false;
  greenLedState = LOW;
  digitalWrite(greenLedPin,greenLedState);
  Blynk.virtualWrite(V0,0);
  //Serial.println(F("Disarmato!"));
}

void resetAlarm() {
  Serial.println("Alarm reset!");
  alarm = false;
  alarmSent = false;
  yellowLedState = alarm;
  digitalWrite(yellowLedPin, yellowLedState);
  for (int i=0; i<SENSOR_NUM; i++) {
    sensorData[i].intDoor = 1;
    sensorData[i].extDoor = 1;
  }
  Blynk.virtualWrite(V6,0);
}

void copySensorData(int sid) {
  RTC.getTime(currentTime);
  /*if ((tempData.temp < 0) || (tempData.temp > 100)) {
    tempData.temp=0.0;
  }
  if ((tempData.vcc < 0) || (tempData.vcc > 10)) {
    tempData.vcc=0.0;
  }
  if (tempData.intDoor != 0) {
    tempData.intDoor=1;
  }
  if (tempData.extDoor != 0) {
    tempData.extDoor=1;
  }*/
  /*sensorData[sid].temp = tempData.temp;
  sensorData[sid].intDoor = tempData.intDoor;
  sensorData[sid].extDoor = tempData.extDoor;
  sensorData[sid].vcc = tempData.vcc;*/
  memcpy(&sensorData[sid].temp, &tempData.temp, 4);
  sensorData[sid].intDoor = tempData.intDoor;
  sensorData[sid].extDoor = tempData.extDoor;
  memcpy(&sensorData[sid].vcc, &tempData.vcc, 4);
  if ((sensorData[sid].temp < 0) || (sensorData[sid].temp > 100)) {
    sensorData[sid].temp=0.0;
  }
  if ((sensorData[sid].vcc < 0) || (sensorData[sid].vcc > 10)) {
    sensorData[sid].vcc=0.0;
  }
  if (tempData.intDoor != 0) {
    tempData.intDoor=1;
  }
  if (tempData.extDoor != 0) {
    tempData.extDoor=1;
  }
  
  char buffer [20] = "";
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", currentTime.getDayOfMonth(), currentTime.getMonth(), currentTime.getYear(), currentTime.getHour(), currentTime.getMinutes(), currentTime.getSeconds());
  //String(currentTime).toCharArray(buffer,20);
  strcpy(sensorData[sid].dateTime,buffer);
  //delay(5);
}

void printToSerial(char *line, char time[20]) {
  Serial.print(line);
  Serial.print(F(" "));
  Serial.println(time);
  //Serial.println(freeRam());
}

char *lcdLine(int sid) {
    char temp[6];
    char vcc[5];
    dtostrf(sensorData[sid].temp,5, 2, temp);
    dtostrf(sensorData[sid].vcc,4, 2, vcc);
    sprintf(lcdl,"Id:%2d Temp:%5.5sIn:%1d Ex:%1d V:%4.4s",sid,temp,sensorData[sid].intDoor,sensorData[sid].extDoor,vcc);
    return lcdl;
}

char *tempLine(int sid, char cp) {
    char temp[6];
    char vcc[5];
    float effim;
    memcpy(&effim, &tempData.temp, 4);
    dtostrf(effim,5, 2, temp);
    memcpy(&effim, &tempData.vcc, 4);
    dtostrf(effim,4, 2, vcc);
    sprintf(lcdl,"Temp Est %c:%5.5s          V:%4.4s",cp,temp,vcc);
    return lcdl;
}

void printOnLCD(char *line) {
  if (!alarm) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(line);
    lcd.setCursor(0,1);
    lcd.print(&line[16]);
  }
}

void printOnWLCD(char *line) {
  if (!alarm) {
    wlcd.clear();
    wlcd.print(0,0,line);
    Blynk.virtualWrite(V8, EEPROM.read(sidOnDisplay*2));
  }
}

void enableIntId(int id) {
  EEPROM.write(id*2, 1);  
}

void disableIntId(int id) {
  EEPROM.write(id*2, 0);  
}

void enableExtId(int id) {
  EEPROM.write((id*2)+1, 1);  
}

void disableExtId(int id) {
  EEPROM.write((id*2)+1, 0);  
}

void setAllIntEnabled() {
  for (int i=0; i<SENSOR_NUM*2; i+=2) {
    EEPROM.write(i, 1);
  }
}

void setAllIntDisabled() {
  for (int i=0; i<SENSOR_NUM*2; i+=2) {
    EEPROM.write(i, 0);
  }
}

void setAllExtEnabled() {
  for (int i=1; i<SENSOR_NUM*2; i+=2) {
    EEPROM.write(i, 1);
  }
}

void setAllExtDisabled() {
  for (int i=1; i<SENSOR_NUM*2; i+=2) {
    EEPROM.write(i, 0);
  }
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
    if (isFirstConnect) 
    {
      // Request server to re-send latest values for all pins
      //Blynk.syncAll();
      //Blynk.SyncVirtual(V0, V1,...);
      Blynk.syncVirtual(V5,V22);
      //brtc.begin();
      isFirstConnect = false;
    }
    wlcd.clear();
}


BLYNK_WRITE(V0) {
  if (param.asInt()) {
    //HIGH
    arm();
  } else {
    //LOW
    unarm();
  }
}

BLYNK_WRITE(V6) {
  if (param.asInt()) {
    Blynk.virtualWrite(V6, 0);
    Serial.println("V6");
  } else {
    resetAlarm();
  }
}

BLYNK_WRITE(V3) {
  if (param.asInt()) {
        sidOnDisplay+=1;
        if (sidOnDisplay >= SENSOR_NUM) {
          sidOnDisplay -= SENSOR_NUM;
        }
        lcdLine(sidOnDisplay);
        printOnLCD(lcdl);
        printOnWLCD(lcdl);
  }
}

BLYNK_WRITE(V4) {
  if (param.asInt()) {
        sidOnDisplay-=1;
        if (sidOnDisplay <0) {
          sidOnDisplay += SENSOR_NUM;
        }
        lcdLine(sidOnDisplay);
        printOnLCD(lcdl);
        printOnWLCD(lcdl);
  }
}

/*BLYNK_WRITE(V7) {
  if (param.asInt()) {
    clockDisplay();
    sync();
  }
}*/

BLYNK_WRITE(V8) {
  if (param.asInt()) {
    enableIntId(sidOnDisplay);
  } else {
    disableIntId(sidOnDisplay);
  }
}

BLYNK_WRITE(V9) {
  if (param.asInt()) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
  }
}

BLYNK_WRITE(V18) {
  if (param.asInt()) {
    openShutter(sidOnDisplay);
  }
}

BLYNK_WRITE(V19) {
  if (param.asInt()) {
    closeShutter(sidOnDisplay);
  }
}

/*BLYNK_WRITE(V20) {
  if (param.asInt()) {
    openShutter(param.asInt());
  }
}

BLYNK_WRITE(V21) {
  if (param.asInt()) {
    closeShutter(param.asInt());
  }
}*/

BLYNK_WRITE(V22) {
  if (param.asInt()) {
    curtain=param.asInt();
  }
}
BLYNK_WRITE(V24) {
  if (param.asInt()) {
    curtain=10+param.asInt();
  }
}
BLYNK_WRITE(V23) {
  if (param.asInt()) {
    if (curtain==1 || curtain>4) {
      if (param.asInt()==1) {
        old_curtain_command(1, curtain);
      } else if (param.asInt()==2) {
        old_curtain_command(0, curtain);
      } else {
        old_curtain_command(-1, curtain);
      }
    } else {
      if (param.asInt()==1) {
        curtain_command(1, curtain);
      } else if (param.asInt()==2) {
        curtain_command(0, curtain);
      } else {
        curtain_command(-1, curtain);
      }
    }
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void printLineToJSON(int sid) {
  jsonline[0] = '\0';
  char temp[6];
  char vcc[5];
  dtostrf(sensorData[sid].temp,5, 2, temp);
  dtostrf(sensorData[sid].vcc,4, 2, vcc);
  snprintf(jsonline,128,"\t{\"Id\":\"%02d\", \"Temp\":\"%s\", \"Int door\":\"%d\", \"Ext door\":\"%d\", \"Bat V\":\"%s\", \"Read time\":\"%s",sensorData[sid].sid,temp,sensorData[sid].intDoor,sensorData[sid].extDoor,vcc,sensorData[sid].dateTime);
  if (sid == SENSOR_NUM-1 ) {
    strcat(jsonline,"\"}\n");
  } else {
    strcat(jsonline,"\"},\n");
  }
}

void openShutter(int gate) {
  Serial.println("APRI");
  char sensorBin[9] = {0};
  toBitsArray(gate, sensorBin);
  code[0] = '\0';
  strcat(code, prefix);
  strcat(code, sensorBin);
  strcat(code, suffixup);
  mySwitch.send(code);
  delay(10);
}

void closeShutter(int gate) {
  Serial.println("CHIUDI");
  char sensorBin[9] = {0};
  toBitsArray(gate, sensorBin);
  code[0] = '\0';
  strcat(code, prefix);
  strcat(code, sensorBin);
  strcat(code, suffixdown);
  mySwitch.send(code);
  delay(10);
}

void toBitsArray(int val, char* ca) {
  //byte ba[8];
  //char ca[9] = {0};
  //byte state;
  for (int i=7; i>=0; i--) {
    ca[7-i] = bitRead(val, i)?'1':'0';
    Serial.print(ca[7-i]);
  }
  ca[8] = '\0';
}

char hextoInt(char hex_nibble) {
  switch (hex_nibble) {
    case '0': return 0;
    case '1': return 1;
    case '2': return 2;
    case '3': return 3;
    case '4': return 4;
    case '5': return 5;
    case '6': return 6;
    case '7': return 7;
    case '8': return 8;
    case '9': return 9;
    case 'A': return 0xA;
    case 'B': return 0xB;
    case 'C': return 0xC;
    case 'D': return 0xD;
    case 'E': return 0xE;
    case 'F': return 0xF;
    case 'a': return 0xA;
    case 'b': return 0xB;
    case 'c': return 0xC;
    case 'd': return 0xD;
    case 'e': return 0xE;
    case 'f': return 0xF;
    default: return 0;
  }
}

// sprintf bugs made me do this, the object code is smaller without sprintf also
char get_hex_char(char hchar){
  if (hchar>9)
    return hchar+'A'-10;
  else
    return hchar+'0';
}

int get_bit() {
  unsigned int invert       = 0;          // invert the bits before transmit
  int ret;
  int byte_pos     = bit_pos / 8;
  int byte_bit_pos = 7 - (bit_pos % 8);     // reverse indexing to send the bits msb
  bit_pos++;
  ret = (packet_buf[byte_pos] & (1<<byte_bit_pos)) ? 1 : 0;
  return ret^invert;
}

int old_curtain_command(int command, int channel) {
  Serial.print(F("Curtain: "));
  Serial.print(channel);Serial.print(F(" "));
  Serial.println(command);
  //String inputString = "";
  unsigned int modulation   = 0;          // PWM = 0, PPM = 1
  unsigned int repeats      = 8;          // signal repeats
  unsigned int bits         = 40;         // amount of bits in a packet
  unsigned int pd_len       = 1064;  //1088      // pulse/distance length (in us)
  unsigned int zero_len     = 700;   //716     // length of 0 (in us)
  unsigned int zero_len_left= pd_len-zero_len;        // length of 0 (in us)
  unsigned int one_len      = 345;   //360    // length of 1 (in us)
  unsigned int one_len_left = pd_len-one_len;        // length of 0 (in us)
  unsigned int pause_len    = 7500;  //7500    // pause length (in us), time between packets
  unsigned int pbuf_len     = 0;          // payload buffer length
  int i,j;
  int bit;
  int pwm_bl;

  // send preamble - not implemented
  if (command>=1) { //up
    inputString = "c2ce7230ee";
  } else if (command<=-1) { //down
    inputString = "c2ce7230cc";
  } else {
    inputString = "c2ce7230aa";
  }
  inputString[7]=get_hex_char(15-channel);
  for (int i=0 ; i<inputString.length()-1 ; i++){
    packet_buf[i]  = hextoInt((char)inputString[(i*2)]) << 4;
    packet_buf[i] |= hextoInt((char)inputString[(i*2) + 1]);
  }
  // TODO clear the packet_buf buffer 
  pbuf_len = ((inputString.length()-1)+1)/2;  //round up

  // repeats
  for (j=0; j<(2*repeats); j++) {
    digitalWrite(TRX_PIN, HIGH);
    delayMicroseconds(pd_len*4);
    digitalWrite(TRX_PIN, LOW);
    delayMicroseconds(pd_len*1);
    // reset bit reader
    bit_pos = 0;
    //At the middle complements last nibble
    if (j==repeats) {
      if (command==0)
        break;
      packet_buf[4] &= 240;
      packet_buf[4] |= (15-hextoInt((char)inputString[9]));
    }
    // send bits
    for (i=0; i<bits; i++) {
      bit = get_bit();
      digitalWrite(TRX_PIN, HIGH);
      if (bit) {
        delayMicroseconds(one_len);
        pwm_bl = one_len;
        digitalWrite(TRX_PIN, LOW);
        delayMicroseconds(pd_len-pwm_bl);
      } else {
        delayMicroseconds(zero_len);
        pwm_bl = zero_len;
        digitalWrite(TRX_PIN, LOW);
        delayMicroseconds(pd_len-pwm_bl);
      }
    }
    // delay between packets
    delayMicroseconds(pause_len);
  }
  return 0;
}

int curtain_command(int comm, int channel) {
  Serial.print(F("Curtain: "));
  Serial.print(channel);Serial.print(F(" "));
  Serial.println(comm);
  //String inputString        = "";         // a string to hold incoming data
  unsigned int modulation   = 0;          // PWM = 0, PPM = 1
  unsigned int repeats      = 2;          // signal repeats
  unsigned int bits         = 25;         // amount of bits in a packet
  unsigned int pd_len       = 1296;        // pulse/distance length (in us)
  unsigned int zero_len     = 986;        // length of 0 (in us)
  unsigned int zero_len_left= pd_len-zero_len;        // length of 0 (in us)
  unsigned int one_len      = 320;       // length of 1 (in us)
  unsigned int one_len_left = pd_len-one_len;        // length of 0 (in us)
  unsigned int pause_len    = 7450;      // pause length (in us), time between packets
  unsigned int pbuf_len     = 0;          // payload buffer length
  int i,j, jj;
  int bit;
  int pwm_bl;

  // send preamble - not implemented
  if (comm>=1) { //up
    inputString = "d0f7fb80";
  } else if (comm<=-1) { //down
    inputString = "d0f7fd80";
  } else {
    inputString = "d0f7f780";
  }
  inputString[1]=get_hex_char(5-channel);
  for (int i=0 ; i<inputString.length()-1 ; i++){
    packet_buf[i]  = hextoInt((char)inputString[(i*2)]) << 4;
    packet_buf[i] |= hextoInt((char)inputString[(i*2) + 1]);
  }
  // TODO clear the packet_buf buffer 
  pbuf_len = ((inputString.length()-1)+1)/2;  //round up
  for (j=0; j<(repeats); j++) {
    // reset bit reader
    bit_pos = 0;
    //At the middle complements last nibble
    if (j==repeats) {
      //Serial.println(15-hextoInt((char)inputString[9]));
      packet_buf[4] &= 240;
      packet_buf[4] |= (15-hextoInt((char)inputString[9]));
    }
    // send bits
    for (i=0; i<bits; i++) {
      bit = get_bit();
      digitalWrite(TRX_PIN, HIGH);
      if (bit) {
        delayMicroseconds(one_len);
        pwm_bl = one_len;
        digitalWrite(TRX_PIN, LOW);
        delayMicroseconds(pd_len-pwm_bl);
      } else {
        delayMicroseconds(zero_len);
        pwm_bl = zero_len;
        digitalWrite(TRX_PIN, LOW);
        delayMicroseconds(pd_len-pwm_bl);
      }
    }
    // delay between packets
    delayMicroseconds(pause_len);
  }
  
  delay(200);
  
  inputString[1]=get_hex_char(5-channel);
  for (int i=0 ; i<inputString.length()-1 ; i++){
    packet_buf[i]  = hextoInt((char)inputString[(i*2)]) << 4;
    packet_buf[i] |= hextoInt((char)inputString[(i*2) + 1]);
  }
  // TODO clear the packet_buf buffer 
  pbuf_len = ((inputString.length()-1)+1)/2;  //round up
  for (j=0; j<(repeats); j++) {
    // reset bit reader
    bit_pos = 0;
    //At the middle complements last nibble
    if (j==repeats) {
      //Serial.println(15-hextoInt((char)inputString[9]));
      packet_buf[4] &= 240;
      packet_buf[4] |= (15-hextoInt((char)inputString[9]));
    }
    // send bits
    for (i=0; i<bits; i++) {
      bit = get_bit();
      digitalWrite(TRX_PIN, HIGH);
      if (bit) {
        delayMicroseconds(one_len);
        pwm_bl = one_len;
        digitalWrite(TRX_PIN, LOW);
        delayMicroseconds(pd_len-pwm_bl);
      } else {
        delayMicroseconds(zero_len);
        pwm_bl = zero_len;
        digitalWrite(TRX_PIN, LOW);
        delayMicroseconds(pd_len-pwm_bl);
      }
    }
    // delay between packets
    delayMicroseconds(pause_len);
  }
  return 0;
}
