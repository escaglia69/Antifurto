#define BLYNK_PRINT Serial
//#include <SPI.h>
//#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
//#include <Ethernet.h>
//#include <DigitalIO.h>
#include <EEPROM.h>
//#include <BlynkSimpleEthernet.h>
#include <ESP8266_Lib.h>
#include "BlynkSimpleShieldEsp8266.h"
#include <TimeLib.h>
#include <WidgetRTC.h>
#include "secret.h"

bool isFirstConnect = true;
WidgetLCD wlcd(V5);

RTC_DS1307 rtc;
WidgetRTC brtc;

RF24 radio(5,9);

byte saddr[][6] = { "s0001" };
struct sensorDataRecord {
  int sid;
  float temp;
  boolean intDoor;
  boolean extDoor;
  float vcc;
  String dateTime;
};
const int SENSOR_NUM = 14;
sensorDataRecord sensorData[SENSOR_NUM] = {{0,0.0,1,1,0.0,"01/01/2016 00:00:00"},{1,0.0,1,1,0.0,"01/01/2016 00:00:00"},{2,0.0,1,1,0.0,"01/01/2016 00:00:00"},{3,0.0,1,1,0.0,"01/01/2016 00:00:00"},{4,0.0,1,1,0.0,"01/01/2016 00:00:00"},{5,0.0,1,1,0.0,"01/01/2016 00:00:00"},{6,0.0,1,1,0.0,"01/01/2016 00:00:00"},{7,0.0,1,1,0.0,"01/01/2016 00:00:00"},{8,0.0,1,1,0.0,"01/01/2016 00:00:00"},{9,0.0,1,1,0.0,"01/01/2016 00:00:00"},{10,0.0,1,1,0.0,"01/01/2016 00:00:00"},{11,0.0,1,1,0.0,"01/01/2016 00:00:00"},{12,0.0,1,1,0.0,"01/01/2016 00:00:00"},{13,0.0,1,1,0.0,"01/01/2016 00:00:00"}};
sensorDataRecord tempData;
int sid;
int sidOnDisplay = SENSOR_NUM - 1;

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

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

byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x5A, 0x82 };
/*IPAddress manualIP(192,168,178,50);
IPAddress gateway(192,168,178,1);
IPAddress subnet(255, 255, 255, 0);
EthernetServer server(80);*/
boolean dhcpConnected = false;
boolean armed = false;
boolean alarm = false;
boolean alarmSent = false;
String readString = String(100);
char lcdl[33];
bool Connected2Blynk = false;
char startedAt [20] = "";

#define EspSerial Serial3
#define ESP8266_BAUD 115200

ESP8266 wifi(&EspSerial);
BlynkTimer timer;

/*char timeServer[] = "time.nist.gov";  // NTP server
unsigned int localPort = 2390;        // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48;  // NTP timestamp is in the first 48 bytes of the message
const int UDP_TIMEOUT = 2000;    // timeout in miliseconds to wait for an UDP packet to arrive
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets*/

void setup(void){
  Serial.begin(115200);
  Serial.println("Setup!");
  //delay(10);
  //setAllIntEnabled();
  //getAllEnabled();
  //ApplicationMonitor.Dump(Serial);
  //ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_8s);
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.noBacklight(); // finish with backlight on
  lcd.setCursor(0,0); //Start at character 4 on line 0
  //lcd.print(F("Allarme 1.0"));
  //lcd.setCursor(0,1);
  lcd.print(F("Avviato!"));
  //delay(30);  
  rtc.begin();
  //delay(30);
  if (! rtc.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  //delay(30);
  DateTime now = rtc.now();
  sprintf(startedAt, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
  Serial.print("Started at: ");
  Serial.println(startedAt);
  
  pinMode(highButtonPin, INPUT);
  digitalWrite(highButtonPin, HIGH);
  pinMode(lowButtonPin, INPUT);
  digitalWrite(lowButtonPin, HIGH);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  // set initial LED state
  digitalWrite(greenLedPin, greenLedState);
  digitalWrite(yellowLedPin, yellowLedState);

  radio.begin();

  // Min speed (for better range I presume)
  radio.setDataRate( RF24_250KBPS );
  // 8 bits CRC
  radio.setCRCLength( RF24_CRC_8 );
  // increase the delay between retries & # of retries
  radio.setRetries(15,15);

  //radio.setPALevel(RF24_PA_LOW);
  //radio.enableDynamicPayloads();
  radio.openReadingPipe(1,saddr[0]);
  //radio.openReadingPipe(2,saddr[1]);
  radio.startListening();

  //if(!Ethernet.begin(mac)) {
    //Ethernet.begin(mac, manualIP, gateway, gateway, subnet);
  //}
  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  //delay(10);
  //wifi.enableMUX();
  Blynk.begin(auth, wifi, ssid, pass);
  //Blynk.config(wifi, ssid);
  /*if (wifi.setOprToStation()) {
      Serial.print("to station ok\r\n");
  } else {
      Serial.print("to station err\r\n");
  }
  delay(1000);
  if (wifi.joinAP(ssid, pass)) {
      Serial.print("Join AP success\r\n");
      Serial.print("IP: ");       
      Serial.println(wifi.getLocalIP().c_str());
  } else {
      Serial.print("Join AP failure\r\n");
  }
  //Blynk.connectWiFi(ssid, pass);*/
  //delay(4000);
  //CheckConnection();
  if (Blynk.connected()) {
    Serial.println(F("Rete in setup"));
    brtc.begin();
  }
  timer.setInterval(60000L, CheckConnection);
  //Blynk.config(wifi, auth);
  /*if (wifi.enableMUX()) {
      Serial.print("multiple ok\r\n");
  } else {
      Serial.print("multiple err\r\n");
  }*/
  
  if (wifi.startTCPServer(8080)) {
      Serial.print("start tcp server ok\r\n");
  } else {
      Serial.print("start tcp server err\r\n");
  }
  
  if (wifi.setTCPServerTimeout(5)) { 
      Serial.print("set tcp server timout 10 seconds\r\n");
  } else {
      Serial.print("set tcp server timout err\r\n");
  }

  /*sendNTPpacket(&wifi, timeServer); // send an NTP packet to a time server
  // wait for a reply for UDP_TIMEOUT miliseconds
  unsigned long startMs = millis();
  //while (!wifi.available() && (millis() - startMs) < UDP_TIMEOUT) {}

  //Serial.println(Udp.parsePacket());
  //if (Udp.parsePacket()) {
    Serial.println("packet received");
    // We've received a packet, read the data from it into the buffer
    wifi.recv(packetBuffer, NTP_PACKET_SIZE);

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
  //}
  // wait ten seconds before asking for the time again
  delay(10000);*/

}

void CheckConnection(){
  rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
  Connected2Blynk = Blynk.connected();
  if(!Connected2Blynk){
    Serial.println(F("Not connected"));
    //Blynk.connect(10000);  // timeout set to 10 seconds and then continue without Blynk
    Blynk.reconnect(ssid, pass);
  }
}

void loop() {
  //ApplicationMonitor.IAmAlive();
  ////ApplicationMonitor.SetData(10);
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
    Blynk.virtualWrite(V6, 1);
    String text = "Allarme varco ";
    Blynk.notify(text+=sid);
    alarmSent = true;
  }

  if( radio.available()){
    while (radio.available()) {                                   // While there is data ready
      radio.read( &tempData, sizeof(tempData) );                  // Get the payload
    }
    if ((tempData.sid<SENSOR_NUM) && (tempData.sid>=0)) {
      sid = tempData.sid;
      copySensorData(sid);
      lcdLine(sid);
      printToSerial(lcdl,sensorData[sid].dateTime);
      printOnLCD(lcdl);
      printOnWLCD(lcdl);
      //clockDisplay();
      //CheckConnection();
    } else {
        printf("####SID: %d out of range!####\n",tempData.sid);
    }
  }
  /*uint8_t buffer[128] = {0};
  uint8_t mux_id = 0;
  uint32_t len = wifi.recv(mux_id, buffer, sizeof(buffer), 100);
  //uint32_t len = wifi.recv(&mux_id, buffer, sizeof(buffer), 100);
  //uint32_t len = wifi.recv(buffer, sizeof(buffer), 100);
  if (len > 0) {
    Serial.print("Status:[");
    Serial.print(wifi.getIPStatus().c_str());
    Serial.println("]");
    
    Serial.print("Received from :");
    Serial.print(mux_id);
    Serial.print("[");
    for(uint32_t i = 0; i < len; i++) {
        Serial.print((char)buffer[i]);
    }
    Serial.print("]\r\n");
    
    uint8_t header[] = "HTTP/1.1 200 OK\r\n"
    "Content-Length: 24\r\n"
    "Server: ESP8266\r\n"
    "Content-Type: text/html\r\n"
    "Connection: keep-alive\r\n\r\n";
    uint8_t hello[] = "<h1>Hello ESP8266!!</h1>";
    wifi.send(mux_id, header, sizeof(header));
    wifi.send(mux_id, hello, sizeof(hello));
    
    if (wifi.releaseTCP(mux_id)) {
        Serial.print("release tcp ");
        Serial.print(mux_id);
        Serial.println(" ok");
    } else {
        Serial.print("release tcp");
        Serial.print(mux_id);
        Serial.println(" err");
    }
    
    Serial.print("Status:[");
    Serial.print(wifi.getIPStatus().c_str());
    Serial.println("]");
  }*/
  /*EthernetClient client = server.available();
  if (client) {
    boolean currentLineIsBlank = true;
    readString=F("");
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        //Serial.write(c);
        if (readString.length() < 100) {
          readString+=c;
        }
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: application/json;charset=utf-8"));
          //client.println(F("Server: Arduino"));
          client.println(F("Connnection: close"));
          client.println();

          if (readString.startsWith(F("GET /set?"))) {
            int id = getId(9);
            if ((id >=0) && (id < SENSOR_NUM)) {
              char rs[32] = "";
              readString.toCharArray(rs, readString.length()+1);
              String temp1 = strtok(rs,"|");
              temp1 = strtok(NULL,"|");
              tempData.temp = temp1.toFloat();
              temp1 = strtok(NULL,"|");
              tempData.intDoor = temp1=="1"?1:0;
              //Serial.print(F("IntDoor: "));
              //Serial.println(temp1);
              temp1 = strtok(NULL,"|");
              tempData.extDoor = temp1=="1"?1:0;
              //Serial.print(F("ExtDoor: "));
              //Serial.println(temp1);
              temp1 = strtok(NULL,"|");
              tempData.vcc = temp1.toFloat();
              copySensorData(id);
              lcdLine(id);
              printToSerial(lcdl,sensorData[id].dateTime);
              printOnLCD(lcdl);
              printOnWLCD(lcdl);
            } else {
              client.println(F("ERROR: Unvalid id"));
            }
          } else if (readString.startsWith(F("GET /help"))) {
            //client.println(F("arm/unarm/resetalarm/allIntEnabled/allIntDisabled/allExtEnabled/allExtDisabled/printEnabled/intEnable?id=/intDisable?id=/extEnable?id=/extDisable?id="));
            client.println(F("arm"));
            client.println(F("unarm"));
            client.println(F("resetalarm"));
            client.println(F("allIntEnabled"));
            client.println(F("allIntDisabled"));
            client.println(F("allExtEnabled"));
            client.println(F("allExtDisabled"));
            client.println(F("printEnabled"));
            client.println(F("intEnable?id="));
            client.println(F("intDisable?id="));
            client.println(F("extEnable?id="));
            client.println(F("extDisable?id="));
          } else if (readString.startsWith(F("GET /arm"))) {
            arm();
            client.println(F("Armed"));
          } else if (readString.startsWith(F("GET /unarm"))) {
            unarm();
            client.println(F("Unarmed"));
          } else if (readString.startsWith(F("GET /resetalarm"))) {
            resetAlarm();
            client.println(F("Alarm reset"));
          } else if (readString.startsWith(F("GET /allIntEnabled"))) {
            setAllIntEnabled();
            client.println(F("All internal enabled"));
          } else if (readString.startsWith(F("GET /allIntDisabled"))) {
            setAllIntDisabled();
            client.println(F("All internal disabled"));
          } else if (readString.startsWith(F("GET /allExtEnabled"))) {
            setAllExtEnabled();
            client.println(F("All external enabled"));
          } else if (readString.startsWith(F("GET /allExtDisabled"))) {
            setAllExtDisabled();
            client.println(F("All external disabled"));
          } else if (readString.startsWith(F("GET /printEnabled"))) {
            printAllEnabled(client);
          } else if (readString.startsWith(F("GET /intEnable?id="))) {
            int id = getId(18);
            if ((id >=0) && (id < SENSOR_NUM)) {
              enableIntId(id);
              client.print(F("Int.Gate "));
              client.print(id);
              client.println(F(" enabled"));              
            } else {
              client.println(F("ERROR: Unvalid id"));
            }
          } else if (readString.startsWith(F("GET /intDisable?id="))) {
            int id = getId(19);
            if ((id >=0) && (id < SENSOR_NUM)) {
              disableIntId(id);
              client.print(F("Int.Gate "));
              client.print(id);
              client.println(F(" disabled"));              
            } else {
              client.println(F("ERROR: Unvalid id"));
            }
          } else if (readString.startsWith(F("GET /extEnable?id="))) {
            int id = getId(18);
            if ((id >=0) && (id < SENSOR_NUM)) {
              enableExtId(id);
              client.print(F("Ext.Gate "));
              client.print(id);
              client.println(F(" enabled"));              
            } else {
              client.println(F("ERROR: Unvalid id"));
            }
          } else if (readString.startsWith(F("GET /extDisable?id="))) {
            int id = getId(19);
            if ((id >=0) && (id < SENSOR_NUM)) {
              disableExtId(id);
              client.print(F("Ext.Gate "));
              client.print(id);
              client.println(F(" disabled"));              
            } else {
              client.println(F("ERROR: Unvalid id"));
            }
          } else {
            client.print(F("{\"Sensor\": [\n"));
            for (int id=0; id<SENSOR_NUM; id++) {
              printToJSON(client, id);
            }
            client.print(F("],\n\"Started at:\":\""));
            client.print(startedAt);
            client.println("\"\n}");        
          }
          break;
        }
        if (c == '\n') {
          currentLineIsBlank = true;
        }
        else if (c!= '\r') {
          currentLineIsBlank = false;
        }
      }


      
    }
    delay(1);
    client.stop();
    //Serial.println(F("Client disconnected"));
  }*/
  timer.run();
  if(Blynk.connected()){
    Blynk.run();
  }
}

void pushButtonHigh() {
  // read the state of the switch into a local variable:
  int highReading = digitalRead(highButtonPin);
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
  int lowReading = digitalRead(lowButtonPin);
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
          } else {
            lcd.noBacklight();
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
  Serial.println(F("Armato!"));
}

void unarm() {
  armed = false;
  greenLedState = LOW;
  digitalWrite(greenLedPin,greenLedState);
  Blynk.virtualWrite(V0,0);
  Serial.println(F("Disarmato!"));
}

void resetAlarm() {
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
  DateTime now = rtc.now();
  sensorData[sid].temp = tempData.temp;
  sensorData[sid].intDoor = tempData.intDoor;
  sensorData[sid].extDoor = tempData.extDoor;
  sensorData[sid].vcc = tempData.vcc;
  char buffer [20] = "";
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
  sensorData[sid].dateTime = buffer;
  //delay(5);
}

void printToSerial(char *line, String time) {
  Serial.print(line);
  Serial.println(" "+time);
}


void printToSerial(int sid) {
    Serial.print(F("Id: "));
    Serial.print(sensorData[sid].sid);
    Serial.print(F(" Temp: "));
    Serial.print(sensorData[sid].temp, DEC);
    Serial.print(F( " Int: "));
    Serial.print(sensorData[sid].intDoor);  
    Serial.print(F(" Ext: "));
    Serial.print(sensorData[sid].extDoor);
    Serial.print(F(" V: "));
    Serial.print(sensorData[sid].vcc);
    Serial.print(F(" Time: "));
    Serial.println(sensorData[sid].dateTime);
    //delay(100);
}
char *lcdLine(int sid) {
    char temp[6];
    char vcc[5];
    dtostrf(sensorData[sid].temp,5, 2, temp);
    dtostrf(sensorData[sid].vcc,4, 2, vcc);
    sprintf(lcdl,"Id:%2d Temp:%sIn:%1d Ex:%1d V:%s",sid,temp,sensorData[sid].intDoor,sensorData[sid].extDoor,vcc);
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
  }
}

void printToJSON(String* msg, int sid) {
    *msg+= "\t{\"Id\":\"";
    *msg+= String(sensorData[sid].sid);
    *msg+= "\", \"Temp\":\"";
    *msg+= String(sensorData[sid].temp);
    *msg+= "\", \"Int door\":\"";
    *msg+= String(sensorData[sid].intDoor);
    *msg+= "\", \"Ext door\":\"";
    *msg+= String(sensorData[sid].extDoor);
    *msg+= "\", \"Battery V\":\"";
    *msg+= String(sensorData[sid].vcc);
    *msg+= "\", \"Read time\":\"";
    *msg+= String(sensorData[sid].dateTime);
    if (sid == SENSOR_NUM-1 ) {
      *msg+= "\"}\n";
    } else {
      *msg+= "\"},\n";
    }
}

int getId(int offset) {
  String inString="";
  char inChar = readString.charAt(offset);
  if (isDigit(inChar)) {
    // convert the incoming byte to a char
    // and add it to the string:
    inString += inChar;
    inChar = readString.charAt(offset+1);
    if (isDigit(inChar)) {
      inString += inChar;
    }
    return inString.toInt();
  } else {
    return -1;
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

void printAllEnabled(String* msg) {
  for (int addr=0; addr<SENSOR_NUM*2; addr++) {
    *msg+= String(addr);
    *msg+= "\t";
    *msg+= String(EEPROM.read(addr));
    *msg+= "\n";
  }
}

BLYNK_CONNECTED() // runs every time Blynk connection is established
{
    //if (isFirstConnect) 
    {
      // Request server to re-send latest values for all pins
      Blynk.syncAll();
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
    Blynk.virtualWrite(V6, 0);;
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

void http_process(ESP8266* client, uint8_t mux_id, uint32_t len) {
      BLYNK_LOG1(BLYNK_F("Arriva!"));
      boolean currentLineIsBlank = true;
      readString=F("");
      String msg = "HTTP/1.1 200 OK\n";
      msg+= "Server: ESP8266\n";
      msg+= "Content-Type: text/plain;charset=utf-8\n";
      msg+= "Connnection: close\n\n";
      while (len) {
        if (client->getUart()->available()) {
          char c = client->getUart()->read();
          if (readString.length() < 100) {
            readString+=c;
          }
          len--;
          if (c == '\n' && currentLineIsBlank) {
            // send a standard http response header
            if (readString.startsWith(F("GET /set?"))) {
              int id = getId(9);
              if ((id >=0) && (id < SENSOR_NUM)) {
                char rs[32] = "";
                readString.toCharArray(rs, readString.length()+1);
                String temp1 = strtok(rs,"|");
                temp1 = strtok(NULL,"|");
                tempData.temp = temp1.toFloat();
                temp1 = strtok(NULL,"|");
                tempData.intDoor = temp1=="1"?1:0;
                //Serial.print(F("IntDoor: "));
                //Serial.println(temp1);
                temp1 = strtok(NULL,"|");
                tempData.extDoor = temp1=="1"?1:0;
                //Serial.print(F("ExtDoor: "));
                //Serial.println(temp1);
                temp1 = strtok(NULL,"|");
                tempData.vcc = temp1.toFloat();
                copySensorData(id);
                lcdLine(id);
                printToSerial(lcdl,sensorData[id].dateTime);
                printOnLCD(lcdl);
                printOnWLCD(lcdl);
              } else {
                msg+= "ERROR: Unvalid id\n";
              }
            } else if (readString.startsWith(F("GET /help"))) {
              //client.println(F("arm/unarm/resetalarm/allIntEnabled/allIntDisabled/allExtEnabled/allExtDisabled/printEnabled/intEnable?id=/intDisable?id=/extEnable?id=/extDisable?id="));
              msg+= "arm\n";
              msg+= "unarm\n";
              msg+= "resetalarm\n";
              msg+= "allIntEnabled\n";
              msg+= "allIntDisabled\n";
              msg+= "allExtEnabled\n";
              msg+= "allExtDisabled\n";
              msg+= "printEnabled\n";
              msg+= "intEnable?id=\n";
              msg+= "intDisable?id=\n";
              msg+= "extEnable?id=\n";
              msg+= "extDisable?id=\n";
            } else if (readString.startsWith(F("GET /arm"))) {
              arm();
              msg+= "Armed\n";
            } else if (readString.startsWith(F("GET /unarm"))) {
              unarm();
              msg+= "Unarmed\n";
            } else if (readString.startsWith(F("GET /resetalarm"))) {
              resetAlarm();
              msg+= "Alarm reset\n";
            } else if (readString.startsWith(F("GET /allIntEnabled"))) {
              setAllIntEnabled();
              msg+= "All internal enabled\n";
            } else if (readString.startsWith(F("GET /allIntDisabled"))) {
              setAllIntDisabled();
              msg+= "All internal disabled\n";
            } else if (readString.startsWith(F("GET /allExtEnabled"))) {
              setAllExtEnabled();
              msg+= "All external enabled\n";
            } else if (readString.startsWith(F("GET /allExtDisabled"))) {
              setAllExtDisabled();
              msg+= "All external disabled\n";
            } else if (readString.startsWith(F("GET /printEnabled"))) {
              printAllEnabled(&msg);
            } else if (readString.startsWith(F("GET /intEnable?id="))) {
              int id = getId(18);
              if ((id >=0) && (id < SENSOR_NUM)) {
                enableIntId(id);
                msg+= "Int.Gate ";
                msg+= String(id);
                msg+= " enabled\n";
              } else {
                msg+= "ERROR: Unvalid id\n";
              }
            } else if (readString.startsWith(F("GET /intDisable?id="))) {
              int id = getId(19);
              if ((id >=0) && (id < SENSOR_NUM)) {
                disableIntId(id);
                msg+= "Int.Gate ";
                msg+= String(id);
                msg+= " disabled\n";            
              } else {
                msg+= "ERROR: Unvalid id\n";
              }
            } else if (readString.startsWith(F("GET /extEnable?id="))) {
              int id = getId(18);
              if ((id >=0) && (id < SENSOR_NUM)) {
                enableExtId(id);
                msg+= "Ext.Gate ";
                msg+= String(id);
                msg+= " enabled\n";            
              } else {
                msg+= "ERROR: Unvalid id\n";
              }
            } else if (readString.startsWith(F("GET /extDisable?id="))) {
              int id = getId(19);
              if ((id >=0) && (id < SENSOR_NUM)) {
                disableExtId(id);
                msg+= "Ext.Gate ";
                msg+= String(id);
                msg+= " disabled\n";            
              } else {
                msg+= "ERROR: Unvalid id\n";
              }
            } else {
              msg+= "{\"Sensor\": [\n";
              for (int id=0; id<SENSOR_NUM; id++) {
                printToJSON(&msg, id);
              }
              msg+= "],\n\"Started at\":\"";
              msg+= startedAt;
              msg+= "\"\n}";      
            }
            break;
          }
          if (c == '\n') {
            currentLineIsBlank = true;
          }
          else if (c!= '\r') {
            currentLineIsBlank = false;
          }

        }
      }

      /*uint8_t hello[] = "HTTP/1.1 200 OK\r\n"
      //"Content-Length: 24\r\n"
      "Server: ESP8266\r\n"
      "Content-Type: text/html\r\n"
      "Connection: keep-alive\r\n\r\n"
      "<h1>Hello ESP8266!!</h1>";
      //client->send(mux_id, head, sizeof(head));*/
      uint8_t data[msg.length()+1];
      //const char *mmsg;
      //Serial.print(msg);
      //mmsg = msg.c_str();
      msg.getBytes(data,sizeof(data));
      client->send(mux_id, data, sizeof(data)-1);
      if (client->releaseTCP(mux_id)) {
        BLYNK_LOG1(BLYNK_F("release tcp ok"));
      } else {
        BLYNK_LOG1(BLYNK_F("release tcp err"));
      }
      return;
}

/*void sendNTPpacket(ESP8266* client, String ntpSrv)
{
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
  client->registerUDP(ntpSrv, 123); //NTP requests are to port 123
  client->send(packetBuffer, NTP_PACKET_SIZE);
  client->unregisterUDP();
}*/

void clockDisplay()
{
  // You can call hour(), minute(), ... at any time
  // Please see Time library examples for details

  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  String currentDate = String(day()) + " " + month() + " " + year();
  Serial.print("Current time: ");
  Serial.print(currentTime);
  Serial.print(" ");
  Serial.print(currentDate);
  Serial.println();

  // Send time to the App
  Blynk.virtualWrite(V1, currentTime);
  // Send date to the App
  Blynk.virtualWrite(V2, currentDate);
}



