//#define BLYNK_PRINT Serial
#include "RF24.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <ESP8266_Lib.h>
#include "BlynkSimpleShieldEsp8266.h"
#include <DS1307RTC.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include "secret.h"

bool isFirstConnect = true;
WidgetLCD wlcd(V5);
WidgetRTC brtc;

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
const int SENSOR_NUM = 14;
sensorDataRecord sensorData[SENSOR_NUM] = {{0,0.0,1,1,0.0,"01/01/2016 00:00:00"},{1,0.0,1,1,0.0,"01/01/2016 00:00:00"},{2,0.0,1,1,0.0,"01/01/2016 00:00:00"},{3,0.0,1,1,0.0,"01/01/2016 00:00:00"},{4,0.0,1,1,0.0,"01/01/2016 00:00:00"},{5,0.0,1,1,0.0,"01/01/2016 00:00:00"},{6,0.0,1,1,0.0,"01/01/2016 00:00:00"},{7,0.0,1,1,0.0,"01/01/2016 00:00:00"},{8,0.0,1,1,0.0,"01/01/2016 00:00:00"},{9,0.0,1,1,0.0,"01/01/2016 00:00:00"},{10,0.0,1,1,0.0,"01/01/2016 00:00:00"},{11,0.0,1,1,0.0,"01/01/2016 00:00:00"},{12,0.0,1,1,0.0,"01/01/2016 00:00:00"},{13,0.0,1,1,0.0,"01/01/2016 00:00:00"}};
sensorDataRecord tempData;
int sid;
int sidOnDisplay = SENSOR_NUM - 1;

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
char startedAt[20] = "";
char tmpid[2] = "";
uint8_t jsonline[128];
int jsonlinel = 0;
int reslen = 0;
tmElements_t tm;

#define EspSerial Serial3
#define ESP8266_BAUD 115200

ESP8266 wifi(&EspSerial);
BlynkTimer timer;

void setup(void){
  Serial.begin(115200);
  Serial.println("Setup!");
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.noBacklight(); // finish with backlight on
  lcd.setCursor(0,0); //Start at character 4 on line 0
  //lcd.print(F("Allarme 1.0"));
  //lcd.setCursor(0,1);
  lcd.print(F("Avviato!"));
  if (RTC.read(tm)) {
    sprintf(startedAt, "%02d/%02d/%04d %02d:%02d:%02d", tm.Day, tm.Month, tmYearToCalendar(tm.Year), tm.Hour, tm.Minute, tm.Second);
    Serial.print("Started at: ");
    Serial.println(startedAt);    
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
    delay(100);
  }
  delay(100);
  
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
  radio.openReadingPipe(1,saddr[0]);
  radio.startListening();

  EspSerial.begin(ESP8266_BAUD);

  Blynk.begin(auth, wifi, ssid, pass);

  if (Blynk.connected()) {
    Serial.println(F("Rete in setup"));
    brtc.begin();
  }
  timer.setInterval(60000L, CheckConnection);
 
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

}

void CheckConnection(){
  tm.Year = CalendarYrToTm(year());
  tm.Month = month();
  tm.Day = day();
  tm.Hour = hour();
  tm.Minute = minute();
  tm.Second = second();
  RTC.write(tm);
  Connected2Blynk = Blynk.connected();
  if(!Connected2Blynk){
    Serial.println(F("Not connected"));
    Blynk.reconnect(ssid, pass);
  }
}

void loop() {
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
  //DateTime now = rtc.now();
  RTC.read(tm);
  sensorData[sid].temp = tempData.temp;
  sensorData[sid].intDoor = tempData.intDoor;
  sensorData[sid].extDoor = tempData.extDoor;
  sensorData[sid].vcc = tempData.vcc;
  char buffer [20] = "";
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", tm.Day, tm.Month, tmYearToCalendar(tm.Year), tm.Hour, tm.Minute, tm.Second);
  strcpy(sensorData[sid].dateTime,buffer);
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

int printAllEnabled(uint8_t (*msg)[2048]) {
  int offs = 91,addr=0;
  for (addr=0; addr<SENSOR_NUM*2; addr++) {
    tmpid[0]=' ';
    sprintf(tmpid,"%2d",addr);
    memcpy(*msg+offs+addr*5,tmpid,2);
    memcpy(*msg+offs+2+addr*5,"\t",1);
    memcpy(*msg+offs+3+addr*5,EEPROM.read(addr)==0?"0":"1",1);
    memcpy(*msg+offs+4+addr*5,"\n",1);
  }
  return offs+5*addr;
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
    //clockDisplay();
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

void clockDisplay()
{
  // You can call hour(), minute(), ... at any time
  // Please see Time library examples for details

  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  String currentDate = String(day()) + " " + month() + " " + year();
  /*Serial.print("Current time: ");
  Serial.print(currentTime);
  Serial.print(" ");
  Serial.print(currentDate);
  Serial.println();*/

  // Send time to the App
  Blynk.virtualWrite(V1, currentTime);
  // Send date to the App
  Blynk.virtualWrite(V2, currentDate);
}

int printToJSON(uint8_t (*row)[128], int sid) {
    char temp[5];
    char vcc[4];
    dtostrf(sensorData[sid].temp,5, 2, temp);
    dtostrf(sensorData[sid].vcc,4, 2, vcc);
    memcpy(*row,"\t{\"Id\":\"",8);
    tmpid[0]=' ';
    sprintf(tmpid,"%2d",sensorData[sid].sid);
    memcpy(*row+8,tmpid,2);
    memcpy(*row+10,"\", \"Temp\":\"",11);
    memcpy(*row+21,temp,5);
    memcpy(*row+26,"\", \"Int door\":\"",15);
    sprintf(tmpid,"%1d",sensorData[sid].intDoor,1);
    memcpy(*row+41,tmpid,1);
    memcpy(*row+42,"\", \"Ext door\":\"",15);
    sprintf(tmpid,"%1d",sensorData[sid].extDoor,1);
    memcpy(*row+57,tmpid,1);
    memcpy(*row+58,"\", \"Battery V\":\"",16);
    memcpy(*row+74,vcc,4);
    memcpy(*row+78,"\", \"Read time\":\"",16);
    memcpy(*row+94,sensorData[sid].dateTime,19);
    if (sid == SENSOR_NUM-1 ) {
      memcpy(*row+113,"\"}\n",3);
      return 116;
    } else {
      memcpy(*row+113,"\"},\n",4);
      return 117;
    }
}

void http_process(ESP8266* client, uint8_t mux_id, uint32_t len) {
      BLYNK_LOG1(BLYNK_F("Arriva!"));
      boolean currentLineIsBlank = true;
      readString=F("");
      uint8_t msg[2048] = "HTTP/1.1 200 OK\n"
      "Server: ESP8266\n"
      "Content-Type: text/plain;charset=utf-8\n"
      "Connnection: close\n\n";
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
                memcpy(msg+91,"ERROR: Unvalid id\n",18);
              }
            } else if (readString.startsWith(F("GET /help"))) {
              memcpy(msg+91,"arm\n",4);
              memcpy(msg+95,"unarm\n",6);
              memcpy(msg+101,"resetalarm\n",11);
              memcpy(msg+112,"allIntEnabled\n",14);
              memcpy(msg+126,"allIntDisabled\n",15);
              memcpy(msg+141,"allExtEnabled\n",14);
              memcpy(msg+155,"allExtDisabled\n",15);
              memcpy(msg+170,"printEnabled\n",13);
              memcpy(msg+183,"intEnable?id=\n",14);
              memcpy(msg+197,"intDisable?id=\n",15);
              memcpy(msg+212,"extEnable?id=\n",14);
              memcpy(msg+226,"extDisable?id=\n",15);
              memcpy(msg+241,"sync\n",5);
              reslen=246;
            } else if (readString.startsWith(F("GET /arm"))) {
              arm();
              memcpy(msg+91,"Armed\n",6);
              reslen=97;
            } else if (readString.startsWith(F("GET /unarm"))) {
              unarm();
              memcpy(msg+91,"Unarmed\n",8);
              reslen=99;
            } else if (readString.startsWith(F("GET /resetalarm"))) {
              resetAlarm();
              memcpy(msg+91,"Alarm reset\n",12);
              reslen=103;
            } else if (readString.startsWith(F("GET /allIntEnabled"))) {
              setAllIntEnabled();
              memcpy(msg+91,"All internal enabled\n",21);
              reslen=112;
            } else if (readString.startsWith(F("GET /allIntDisabled"))) {
              setAllIntDisabled();
              memcpy(msg+91,"All internal disabled\n",22);
              reslen=113;
            } else if (readString.startsWith(F("GET /allExtEnabled"))) {
              setAllExtEnabled();
              memcpy(msg+91,"All external enabled\n",21);
              reslen=112;
            } else if (readString.startsWith(F("GET /allExtDisabled"))) {
              setAllExtDisabled();
              memcpy(msg+91,"All external disabled\n",22);
              reslen=113;
            } else if (readString.startsWith(F("GET /printEnabled"))) {
              reslen=printAllEnabled(&msg);
            } else if (readString.startsWith(F("GET /sync"))) {
              clockDisplay();
              memcpy(msg+91,"Time synced\n",12);
              reslen=103;
            } else if (readString.startsWith(F("GET /intEnable?id="))) {
              int id = getId(18);
              if ((id >=0) && (id < SENSOR_NUM)) {
                enableIntId(id);
                memcpy(msg+91,"Int.Gate ",9);
                tmpid[0]=' ';
                sprintf(tmpid,"%2d",id);
                memcpy(msg+100,tmpid,2);
                memcpy(msg+102," enabled\n",9);
                reslen=111;
              } else {
                memcpy(msg+91,"ERROR: Unvalid id\n",18);
                reslen=109;
              }
            } else if (readString.startsWith(F("GET /intDisable?id="))) {
              int id = getId(19);
              if ((id >=0) && (id < SENSOR_NUM)) {
                disableIntId(id);
                memcpy(msg+91,"Int.Gate ",9);
                tmpid[0]=' ';
                sprintf(tmpid,"%2d",id);
                memcpy(msg+100,tmpid,2);
                memcpy(msg+102," disabled\n",10);
                reslen=112;         
              } else {
                memcpy(msg+91,"ERROR: Unvalid id\n",18);
                reslen=109;
              }
            } else if (readString.startsWith(F("GET /extEnable?id="))) {
              int id = getId(18);
              if ((id >=0) && (id < SENSOR_NUM)) {
                enableExtId(id);
                memcpy(msg+91,"Ext.Gate ",9);
                tmpid[0]=' ';
                sprintf(tmpid,"%2d",id);
                memcpy(msg+100,tmpid,2);
                memcpy(msg+102," enabled\n",9);
                reslen=111;
              } else {
                memcpy(msg+91,"ERROR: Unvalid id\n",18);
                reslen=109;
              }
            } else if (readString.startsWith(F("GET /extDisable?id="))) {
              int id = getId(19);
              if ((id >=0) && (id < SENSOR_NUM)) {
                disableExtId(id);
                memcpy(msg+91,"Ext.Gate ",9);
                tmpid[0]=' ';
                sprintf(tmpid,"%2d",id);
                memcpy(msg+100,tmpid,2);
                memcpy(msg+102," disabled\n",10);
                reslen=112;
              } else {
                memcpy(msg+91,"ERROR: Unvalid id\n",18);
                reslen=109;
              }
            } else {
              memcpy(msg+91,"{\"Sensor\": [\n",13);
              int offset=104;
              for (int idd=0; idd<SENSOR_NUM; idd++) {
                jsonline[0] = '\0';
                jsonlinel=printToJSON(&jsonline, idd);
                memcpy(msg+offset,jsonline,jsonlinel);
                offset+=jsonlinel;
              }
              memcpy(msg+offset,"],\n\"Started at\":\"",17);
              memcpy(msg+17+offset,startedAt,19);
              memcpy(msg+36+offset,"\"\n}",3);
              reslen=offset+39;
            }
            tmpid[0] = '\0';
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

      client->send(mux_id, msg, reslen);
      msg[0] ='\0';
      if (client->releaseTCP(mux_id)) {
        BLYNK_LOG1(BLYNK_F("release tcp ok"));
      } else {
        BLYNK_LOG1(BLYNK_F("release tcp err"));
      }
      return;
}

