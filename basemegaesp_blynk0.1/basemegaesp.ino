#define BLYNK_PRINT Serial
#include "RF24.h"
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <ESP8266_Lib.h>
#include "BlynkSimpleShieldEsp8266.h"
#include <DS1307RTC.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include <RCSwitch.h>
//#include "Oregon.h"
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
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

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

//byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x5A, 0x82 };
/*IPAddress manualIP(192,168,178,50);
IPAddress gateway(192,168,178,1);
IPAddress subnet(255, 255, 255, 0);
EthernetServer server(80);*/
boolean dhcpConnected = false;
boolean armed = false;
boolean alarm = false;
boolean alarmSent = false;
char lcdl[33];
bool Connected2Blynk = false;
char startedAt[20] = "01/01/2016 00:00:00";
float tempN;
float tempS;
float tempO;
char tmpid[2] = "";
char jsonline[128] ={0};
tmElements_t tm;
//uint8_t msg[2048] = "HTTP/1.1 200 OK\n"
char msg[2048] = "HTTP/1.1 200 OK\n"
"Server: ESP8266\n"
"Access-Control-Allow-Origin: *\n"
"Access-Control-Allow-Headers: Content-Type\n"
"Connection: close\n\0";
char received[32] = {0};
char prefix[9] = "01000101";
char suffixup[9] = "10010001";
char suffixdown[9] = "10010100";
char code[25] = {0};
bool firstsync = true;

#define EspSerial Serial3
#define ESP8266_BAUD 115200

ESP8266 wifi(&EspSerial);
BlynkTimer timer;
BlynkTimer ttimer;
RCSwitch mySwitch = RCSwitch();
#define TRX_PIN  12
#define VOLTAGE_THRESH 4.0
//THN132N sender1(TRX_PIN, 0xAA, 1);
//THN132N sender2(TRX_PIN, 0x20, 2);

String inputString        = "";         // a string to hold incoming data
unsigned int modulation   = 0;          // PWM = 0, PPM = 1
unsigned int repeats      = 6;          // signal repeats
unsigned int bits         = 40;         // amount of bits in a packet
unsigned int channel      = 3;          // remote channel
unsigned int pd_len       = 1064;  //1088      // pulse/distance length (in us)
unsigned int zero_len     = 700;   //716     // length of 0 (in us)
unsigned int zero_len_left= pd_len-zero_len;        // length of 0 (in us)
unsigned int one_len      = 345;   //360    // length of 1 (in us)
unsigned int one_len_left = pd_len-one_len;        // length of 0 (in us)
unsigned int pause_len    = 7500;  //7500    // pause length (in us), time between packets
unsigned int preamble     = 0;       // preamble length (in us)
unsigned int invert       = 0;          // invert the bits before transmit
//char packet_buf[256]      = {0};        // packet payload buffer
char packet_buf[256]      = {0};        // packet payload buffer
unsigned int pbuf_len     = 0;          // payload buffer length
unsigned int bit_pos      = 0;          // bit reader bit position
unsigned int curtain      = 1;

void setup(void){
  Serial.begin(115200);
  Serial.println(F("Setup!"));
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.noBacklight(); // finish with backlight on
  lcd.setCursor(0,0); //Start at character 4 on line 0
  //lcd.print(F("Allarme 1.0"));
  //lcd.setCursor(0,1);
  lcd.print(F("Avviato!"));
  if (RTC.read(tm)) {
    sprintf(startedAt, "%02d/%02d/%04d %02d:%02d:%02d", tm.Day, tm.Month, tmYearToCalendar(tm.Year), tm.Hour, tm.Minute, tm.Second);
    Serial.print(F("Started at: "));
    Serial.println(startedAt);    
  } else {
    if (RTC.chipPresent()) {
      Serial.println(F("The DS1307 is stopped.  Please run the SetTime"));
    } else {
      Serial.println(F("DS1307 read error!  Please check the circuitry."));
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
  /*####################*/
  //digitalWrite(13, LOW);
  /*####################*/
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

  EspSerial.begin(ESP8266_BAUD);

  Blynk.begin(auth, wifi, ssid, pass);

  if (Blynk.connected()) {
    Serial.println(F("Rete in setup"));
    Blynk.virtualWrite(V9, 0);
  }
  timer.setInterval(60000L, CheckConnection);
  ttimer.setInterval(301000L, SendTemp);
 
  if (wifi.startTCPServer(8080)) {
      Serial.print(F("start tcp server ok\r\n"));
  } else {
      Serial.print(F("start tcp server err\r\n"));
  }
  
  if (wifi.setTCPServerTimeout(6)) { 
      Serial.print(F("set tcp server timout 6 seconds\r\n"));
  } else {
      Serial.print(F("set tcp server timout err\r\n"));
  }

}

void CheckConnection(){
  if (firstsync) {
    firstsync = false;
    sync();
    sprintf(startedAt, "%02d/%02d/%04d %02d:%02d:%02d", tm.Day, tm.Month, tmYearToCalendar(tm.Year), tm.Hour, tm.Minute-1, tm.Second);      
  }
  Connected2Blynk = Blynk.connected();
  if(!Connected2Blynk){
    Serial.println(F("Not connected"));
    Blynk.reconnect(ssid, pass);
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
  //brtc.begin();
  clockDisplay();
  if (year()!=1970) {
    tm.Year = CalendarYrToTm(year());
    tm.Month = month();
    tm.Day = day();
    tm.Hour = hour();
    tm.Minute = minute();
    tm.Second = second();
    if (RTC.write(tm)) {
      Serial.println(F("RTC updated!"));
    } 
  }
}

void clockDisplay()
{
  // You can call hour(), minute(), ... at any time
  // Please see Time library examples for details

  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  String currentDate = String(day()) + " " + month() + " " + year();
  Serial.print(F("Current time: "));
  Serial.print(currentTime);
  Serial.print(F(" "));
  Serial.print(currentDate);
  Serial.println();

  // Send time to the App
  //Blynk.virtualWrite(V1, currentTime);
  // Send date to the App
  //Blynk.virtualWrite(V2, currentDate);
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
      delay(10);
    } else if((tempData.sid<100) && (tempData.sid>=90)) {
      /*Serial.print(tempData.sid);
      Serial.print(" ");
      Serial.println(tempData.temp);*/
      if(tempData.sid==91) {
        tempN=tempData.temp;
        //sender2.send(tempN,tempData.vcc > VOLTAGE_THRESH);
        Serial.print(F("TEMP NORD: "));
        Serial.println(tempN);
        tempLine(tempData.sid,'N');
      }
      if(tempData.sid==92) {
        tempS=tempData.temp;
        Serial.print(F("TEMP SUD: "));
        Serial.println(tempS);
        tempLine(tempData.sid,'S');
      }
      if(tempData.sid==90) {
        tempO=tempData.temp;
        //sender1.send(tempO,tempData.vcc > VOLTAGE_THRESH);
        Serial.print(F("TEMP EST: "));
        Serial.println(tempO);
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

  /*cli();
  word p = pulse;
  pulse = 0;
  sei();
  if (p != 0) {
    //Serial.print("[pulse]");
    //Serial.print(p, HEX);
    //Serial.println();
    if (orscV2.nextPulse(p))
      reportSerial("OSV2", orscV2);       
  }*/
    
  timer.run();
  if(Blynk.connected()){
    Blynk.run();
    ttimer.run();
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
  RTC.read(tm);
  if ((tempData.temp < 0) || (tempData.temp > 100)) {
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
  }
  sensorData[sid].temp = tempData.temp;
  sensorData[sid].intDoor = tempData.intDoor;
  sensorData[sid].extDoor = tempData.extDoor;
  sensorData[sid].vcc = tempData.vcc;
  char buffer [20] = "";
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", tm.Day, tm.Month, tmYearToCalendar(tm.Year), tm.Hour, tm.Minute, tm.Second);
  strcpy(sensorData[sid].dateTime,buffer);
  //delay(5);
}

void printToSerial(char *line, char time[20]) {
  Serial.print(line);
  Serial.print(F(" "));
  Serial.println(time);
  Serial.println(freeRam());
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
    dtostrf(tempData.temp,5, 2, temp);
    dtostrf(tempData.vcc,4, 2, vcc);
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

BLYNK_CONNECTED() // runs every time Blynk connection is established
{
    if (isFirstConnect) 
    {
      // Request server to re-send latest values for all pins
      //Blynk.syncAll();
      //Blynk.SyncVirtual(V0, V1,...);
      Blynk.syncVirtual(V22);
      brtc.begin();
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

BLYNK_WRITE(V7) {
  if (param.asInt()) {
    clockDisplay();
    sync();
  }
}

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

BLYNK_WRITE(V20) {
  if (param.asInt()) {
    openShutter(param.asInt());
  }
}

BLYNK_WRITE(V21) {
  if (param.asInt()) {
    closeShutter(param.asInt());
  }
}

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
    if (param.asInt()==1) {
      curtain_command(1, curtain);
    } else if (param.asInt()==2) {
      curtain_command(0, curtain);
    } else {
      curtain_command(-1, curtain);
    }
  }
}

void http_process(ESP8266* client, uint8_t mux_id, uint32_t len) {
      BLYNK_LOG1(BLYNK_F("Arriva!"));
      Serial.print(F("LEN: "));
      Serial.println(len);
      //char received[32] = {0};
      int reslen = 125;
      msg[reslen] = '\0';
      char c ;
      int ii=0;
      while (len) {
          if (client->getUart()->available()) {
              c = client->getUart()->read();
              if (ii<31) {
                received[ii++]=c;                
              }
              len--;
          }
      }
      received[ii]='\0';
      Serial.println(received);
      
      if (strncmp(received, "GET ", 4) == 0) {
        strcat(msg,"Content-Type: text/plain;charset=utf-8\n\n");
        reslen=164;
        if (strncmp(received+4,"/arm",4) == 0) {
            arm();
            strcat(msg,"Armed\n");
            reslen+=6;
        } else if (strncmp(received+4,"/unarm",6) == 0) {
            unarm();
            strcat(msg,"Unarmed\n");
            reslen+=8;
        } else if (strncmp(received+4,"/alarmreset",11) == 0) {
            resetAlarm();
            strcat(msg,"Alarm reset\n");
            reslen+=12;
        } else if (strncmp(received+4,"/sync",5) == 0) {
            sync();
            strcat(msg,"Synced\n");
            reslen+=7;
        } else if (strncmp(received+4,"/reset",6) == 0) {
            Blynk.restart();
            softReset();
        } else if (strncmp(received+4,"/help",5) == 0) {
            strcat(msg,"arm\nunarm\nalarmreset\nallIntEnabled\nallIntDisabled\n");
            strcat(msg,"allExtEnabled\nallExtDisabled\nprintEnabled\nintEnable?id=\n");
            strcat(msg,"intDisable?id=\nextEnable?id=\nextDisable?id=\nsync\nreset\n");
            reslen+=161;
        } else if (strncmp(received+4,"/allIntEnabled",14) == 0) {
            setAllIntEnabled();
            strcat(msg,"All internal enabled\n");
            reslen+=21;
        } else if (strncmp(received+4,"/allIntDisabled",15) == 0) {
            setAllIntDisabled();
            strcat(msg,"All internal disabled\n");
            reslen+=22;
        } else if (strncmp(received+4,"/allExtEnabled",14) == 0) {
            setAllExtEnabled();
            strcat(msg,"All external enabled\n");
            reslen+=21;
        } else if (strncmp(received+4,"/allExtDisabled",15) == 0) {
            setAllExtDisabled();
            strcat(msg,"All external disabled\n");
            reslen+=22;
        } else if (strncmp(received+4,"/printEnabled",13) == 0) {
            reslen=printAllEnabled(reslen);
        } else if (strncmp(received+4,"/intEnable?id=",14) == 0) {
            int id = atoi(&received[18]);
            if ((id >=0) && (id < SENSOR_NUM)) {
              enableIntId(id);
              sprintf(msg+reslen,"Int.Gate %2d enabled\n",id);
              reslen+=20;
            } else {
              strcat(msg,"ERROR: Unvalid id\n");
              reslen+=18;
            }
        } else if (strncmp(received+4,"/intDisable?id=",15) == 0) {
            int id = atoi(&received[19]);
            if ((id >=0) && (id < SENSOR_NUM)) {
              disableIntId(id);
              sprintf(msg+reslen,"Int.Gate %2d disabled\n",id);
              reslen+=21;
            } else {
              strcat(msg,"ERROR: Unvalid id\n");
              reslen+=18;
            }
        } else if (strncmp(received+4,"/extEnable?id=",14) == 0) {
            int id = atoi(&received[18]);
            if ((id >=0) && (id < SENSOR_NUM)) {
              enableExtId(id);
              sprintf(msg+reslen,"Ext.Gate %2d enabled\n",id);
              reslen+=20;            
            } else {
              strcat(msg,"ERROR: Unvalid id\n");
              reslen+=18;
            }
        } else if (strncmp(received+4,"/extDisable?id=",15) == 0) {
            int id = atoi(&received[19]);
            if ((id >=0) && (id < SENSOR_NUM)) {
              disableExtId(id);
              sprintf(msg+reslen,"Ext.Gate %2d disabled\n",id);
              reslen+=21;
            } else {
              strcat(msg,"ERROR: Unvalid id\n");
              reslen+=18;
            }
        } else {
          msg[reslen-39]='\0';
          strcat(msg,"Content-Type: application/json\n\n");
          printToJSON();
          reslen=117*SENSOR_NUM+52+reslen-8+54;
        }
      }
      
      client->send(mux_id, (const uint8_t*)msg, reslen);
      if (client->releaseTCP(mux_id)) {
        BLYNK_LOG1(BLYNK_F("release tcp ok"));
      } else {
        BLYNK_LOG1(BLYNK_F("release tcp err"));
      }
      Serial.println(freeRam());
      return;
}

void printLineToJSON(int sid) {
  jsonline[0] = '\0';
  char temp[6];
  char vcc[5];
  dtostrf(sensorData[sid].temp,5, 2, temp);
  dtostrf(sensorData[sid].vcc,4, 2, vcc);
  snprintf(jsonline,128,"\t{\"Id\":\"%02d\", \"Temp\":\"%s\", \"Int door\":\"%d\", \"Ext door\":\"%d\", \"Battery V\":\"%s\", \"Read time\":\"%s",sensorData[sid].sid,temp,sensorData[sid].intDoor,sensorData[sid].extDoor,vcc,sensorData[sid].dateTime);
  if (sid == SENSOR_NUM-1 ) {
    strcat(jsonline,"\"}\n");
  } else {
    strcat(jsonline,"\"},\n");
  }
}

void printToJSON() {
  strcat(msg,"{\"Sensor\": [\n");
  for (int id=0; id<SENSOR_NUM; id++) {
    //strcat(json,printLineToJSON(id));
    printLineToJSON(id);
    strcat(msg,jsonline);
    //printf("%s",line);
  }
  char temp[6];
  dtostrf(tempN,5, 2, temp);
  strcat(msg,"],\n\"Temp N\":\"");
  strcat(msg,temp);
  dtostrf(tempS,5, 2, temp);
  strcat(msg,"\",\n\"Temp S\":\"");
  strcat(msg,temp);
  dtostrf(tempO,5, 2, temp);
  strcat(msg,"\",\n\"Temp O\":\"");
  strcat(msg,temp);
  strcat(msg,"\",\n\"Started at\":\"");
  strcat(msg,startedAt);
  strcat(msg,"\"\n}\n");
  
  /*strcat(msg,"],\n\"Started at\":\"");
  strcat(msg,startedAt);
  strcat(msg,"\"\n}\n");*/
}

int printAllEnabled(int len) {
  char line[6] = {0};
  for (int id=0; id<SENSOR_NUM*2; id++) {
    sprintf(line,"%2d\t%u\n",id,EEPROM.read(id)==0?0:1);
    strcat(msg,line);
  }
  return len+(5*SENSOR_NUM*2);
}

int freeRam () {
   extern int __heap_start, *__brkval; 
   int v; 
   return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
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

void softReset(){
  asm volatile ("  jmp 0");
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
  int ret;
  int byte_pos     = bit_pos / 8;
  int byte_bit_pos = 7 - (bit_pos % 8);     // reverse indexing to send the bits msb
  bit_pos++;
  ret = (packet_buf[byte_pos] & (1<<byte_bit_pos)) ? 1 : 0;
  return ret^invert;
}

int curtain_command(int command, int channel) {
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
