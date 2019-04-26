#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(4,5);
byte saddr[][6] = { "s0001" };

// Sleep declarations
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;
void setup_watchdog(uint8_t prescalar);
void do_sleep(void);
//const short sleep_cycles_per_transmission = 4;
//volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;
struct dataStruct{
  int sid = 13;
  float temp;
  boolean intDoor;
  boolean extDoor;
  float vcc;
}myData;
int counter;
volatile int f_int=0;

float readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result/1000.0;
}

long readIntTemp() {
  long result;
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = (result - 125) * 1075;
  return result;
}

float readTempLM35(float vcc) {
  int reading = analogRead(3);  
  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * vcc;
  voltage /= 1024.0;
  float tempC = voltage * 100 ;
  return tempC;
}

float readTempTMP36(float vcc) {
  //getting the voltage reading from the temperature sensor
 int reading = analogRead(3);  
 // converting that reading to voltage, for 3.3v arduino use 3.3
 float voltage = reading * vcc;
 voltage /= 1024.0;
 float tempC = (voltage - 0.5) * 100 ;
 return tempC;
}
void setup(void){
  counter = 0;
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  //digitalWrite(2, HIGH);
  //digitalWrite(3, HIGH);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  attachInterrupt(0, transmitDoorState, CHANGE);
  attachInterrupt(1, transmitDoorState, CHANGE);
  Serial.begin(115200);
  setup_watchdog(wdt_8s);

  radio.begin();
  //radio.setPALevel(RF24_PA_MAX);
  //radio.setChannel(70); //************ATTENZIONE*****************
  //radio.setPayloadSize(16);
 
  // Min speed (for better range I presume)
  radio.setDataRate( RF24_250KBPS );
  // 8 bits CRC
  radio.setCRCLength( RF24_CRC_8 );
  // increase the delay between retries & # of retries
  radio.setRetries(15,15);
 
  radio.openWritingPipe(saddr[0]);
}

void loop() {
    if ((counter == 15) || (f_int)) {
    myData.vcc = readVcc();
    myData.temp = readTempTMP36(myData.vcc);
    Serial.println(myData.temp);
    delay(100);
    radio.powerUp();
    if (!radio.write(&myData, sizeof(myData))){
      Serial.println(F("Sent failed"));
    }
    delay(100);                     // Experiment with some delay here to see if it has an effect
    // Power down the radio.
    radio.powerDown();
    counter = 0;
    f_int = 0;
  }
  counter++;
  // Sleep the MCU.
    enterSleep();
}

// Sleep helpers
//Prescaler values
// 0=16ms, 1=32ms,2=64ms,3=125ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(uint8_t prescalar){
  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
}
ISR(WDT_vect)
{
  //--sleep_cycles_remaining;
  //Serial.println(F("WDT"));
}

/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  byte old_ADCSRA = ADCSRA;                        // disable ADC //
  ADCSRA = 0;                                      // disable ADC //
  
  byte old_PRR = PRR;                              // disable Internal modules//
  PRR = 0xFF;                                      // disable Internal modules//

  MCUSR = 0;                                       // clear various "reset" flags// 
  // turn off brown-out enable in software//
  MCUCR = bit (BODS) | bit (BODSE);                //Brown out settings
  MCUCR = bit (BODS);                              //Brown out set.

  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  PRR = old_PRR;
  ADCSRA = old_ADCSRA;

  /* Re-enable the peripherals. */
  power_all_enable();
}

void transmitDoorState() {
  if(f_int == 0)
  {
    myData.intDoor = digitalRead(2);
    myData.extDoor = digitalRead(3);
    f_int=1;
  }
  else
  {
    Serial.println("INT Overrun!!!");
  }
  /*if (myData.intDoor) {
    Serial.print("Internal Door: ");
    Serial.println(myData.intDoor);
    Serial.print("External Door: ");
    Serial.println(myData.extDoor);  }*/
}
