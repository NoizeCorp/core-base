/*********************************************************************/
/*                             - TheBox -                            */
/*                                                                   */
/*             DHT22 - Temperature and humidity sensor               */
/*                       uSD reader logging sys                      */
/*                                Lux                                */
/*                                                                   */
/*             Contacts:                                             */
/*                      Alessio Aka BrainDev                         */
/*                         Ivan B  Aka KillerBM                      */
/*                            Gepp Aka UserK                         */
/*                                                                   */
/*             PinOUT  RTC                                           */
/*                        Vcc  - 5 V                                 */
/*                         SDA - A4                                  */
/*                         SCL = A5                                  */
/*                                                                   */
/*             PinOUT  DHT22                                         */
/*                        Vcc  - 5 V                                 */
/*                         DATA - 2                                  */
/*                                                                   */
/*             PinOUT  Blue                                          */
/*                        RX   -  11                                 */
/*                        TX   -  10                                 */
/*                                                                   */
/*             PinOUT  GAS                                           */
/*                        A0   -  A0                                 */
/*                                                                   */
/*             PinOUT  LUX                                           */
/*                       DATA  -  A1                                 */
/*                                                                   */
/*                                                                   */
/*********************************************************************/

/*********************************************************************/
/*                        INCLUDE                                    */
/*********************************************************************/
#include "DHT.h"
#include "SoftwareSerial.h"
#include <EEPROM.h>
#include <Wire.h>
#include <LowPower.h>
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <avr/sleep.h>



// Update Current Version
#define VERSION 1.01
#define DEBUG true

/*********************************************************************/
/*                           DEFINITION                              */
/*********************************************************************/

#define SERIAL_BAUDRATE 9600
#define WAKE_LED 13
/********DHT********/
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 5     // what digital pin we're connected to
#define DHT_POWER_PIN 9
DHT dht(DHTPIN, DHTTYPE); // DHT22 temperature/humidity sensor definition

typedef struct dhtData {
  float temp;
  float humidity;
} dhtData_t;


/********LUX********/
#define LUX_PIN A1
#define LUX_POWER_PIN 12

/********GAZ********/
#define GAZ_PIN A0
#define GAZ_POWER_PIN 8

/********RTC********/
#define wakePin 3
#define RTC_LED 10

/********BLU********/
#define BLU_POWER_PIN 11
#define BLU_WAKE_PIN 2
#define BLU_BAUDRATE 9600
#define BLU_LED 4
unsigned long blu_time_out = 60000;

/********Bluetooth********/
SoftwareSerial blu(6, 7); // RX, TX

/*********************************************************************/
/*                            CONFIG                                 */
/*********************************************************************/

boolean printCSV = false;
boolean printSerial = false;
boolean verbosity = false;
int verbosityLevel = 2;


/*********************************************************************/
/*                          RTC CONFIG                               */
/*********************************************************************/

byte AlarmFlag = 0;

/*********************************************************************/
/*                          EEPROM CONFIG                            */
/*********************************************************************/

/* Current address in the EEPROM */
unsigned int address = 2;


/*********************************************************************/
/*                          GLOBAL VARS                              */
/*********************************************************************/

//Initial value for sensors
int luxVal = 0;
int gazVal = 0;
dhtData_t dhtData;

// Strings Log
char temp[13];
char hum[13];
char lux[13];
char timeK[13];

String tempT;
String luxT;
String humT;
String timeT;
volatile bool rtc_wake_up = true;
volatile bool blu_wake_up = false;

volatile int countblu = 0;
volatile int countsleep = 0;


// Timers Countera
long timerLoop, timer0, timer1=0, timer2;
int TLperiod = 5;
long int periodeTimerLoopMillis = 0;
long int SdTimerPeriod = 5000;


/*********************************************************************/
/*                 INTERUPT CALLBACK ROUTINE                         */
/*********************************************************************/

/********RTC********/
void wake_up_rtc(){ 
  rtc_wake_up = true;
  countsleep++;
}

/********BLU********/

void wake_up_blu(){
  blu_wake_up = true; 
  countblu++;
}


/*********************************************************************/
/*                          SETUPS ROUTINE                           */
/*********************************************************************/

void serial_setup(){
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("Setup [..]");
}

void setup_rtc() {
    // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
    RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
    RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
    RTC.alarm(ALARM_1);
    RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, false);
    RTC.alarmInterrupt(ALARM_2, false);
    RTC.squareWave(SQWAVE_NONE);

    // configure an interrupt on the falling edge from the SQW pin
    pinMode(wakePin, INPUT_PULLUP);

    // set alarm 1 for 20 seconds after every minute
    RTC.setAlarm(ALM1_MATCH_SECONDS, 20, 0, 0, 1);
    RTC.alarm(ALARM_1);                   // ensure RTC interrupt flag is cleared
    RTC.alarmInterrupt(ALARM_1, true);

    RTC.alarmInterrupt(ALARM_2, false);
}

void setup_dht(){
  dht.begin();
  pinMode(DHTPIN,INPUT);
}

void setup_eeprom(){
 /* for(int i=0; i <= EEPROM.length(); i--){
    if((int)EEPROM.read(i) != 255){
      if(i == (((EEPROM.read(0) << 8) & 0xFF) + EEPROM.read(1))){
          address = i;
          break;
      } 
    }
  }*/
}

void setup_blu(){
  blu.begin(BLU_BAUDRATE);
  pinMode(BLU_WAKE_PIN, INPUT_PULLUP);
}

void setup_power_pin(){
  pinMode(DHT_POWER_PIN, OUTPUT);
  pinMode(GAZ_POWER_PIN, OUTPUT);
  pinMode(LUX_POWER_PIN, OUTPUT);
  pinMode(BLU_POWER_PIN, OUTPUT);
  if(DEBUG){
    pinMode(BLU_LED, OUTPUT);
    pinMode(RTC_LED, OUTPUT);
    pinMode(WAKE_LED, OUTPUT);
  }

}

/*
 * Function: setup_interrupt
 * ----------------------------
 *   Attach or detach interupt. 
 *   This function is user when the chip go to sleep or wakeUp
 *
 *   go_to_sleep: true if the ship will go to sleep
 *
 */
void setup_priority_interrupt(){
  cli();
      attachInterrupt(INT1, wake_up_rtc, FALLING);
  sei();
}

void secondary_interrupt(bool go_to_sleep){
    if(go_to_sleep){
      cli();
      attachInterrupt(INT0, wake_up_blu, FALLING);
      sei();
      return;
    }
    cli();
    detachInterrupt(INT0);
    sei();
}


/******* MAIN SETUP *******/
void setup() {
  serial_setup();
  //setup_eeprom();
  setup_dht();    
  setup_power_pin();  
  setup_rtc();
  setup_blu();
  setup_priority_interrupt();
  digitalWrite(WAKE_LED, HIGH);
}

/*********************************************************************/
/*                          SLEEP ROUTINE                            */
/*********************************************************************/
/*
 * Function: sleep
 * ----------------------------
 *   Allow chip to enter in the most powered safe mode
 *
 */
void sleep()        
{
    digitalWrite(WAKE_LED, LOW);
    Serial.println("Enter in sleep mode"); delay(1000);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    secondary_interrupt(true);
    sleep_mode(); // <- put device to sleep
                  // <- position after wake up 
    delay(1000);Serial.println("Wake up");   
    sleep_disable();
    secondary_interrupt(false);
    digitalWrite(WAKE_LED, HIGH);
}


/*********************************************************************/
/*                          LOOP ROUTINE                             */
/*********************************************************************/

void loop(){    
  //**********RTC**********//
  if(rtc_wake_up) rtc_interrupt_routine();
  
  
  //**********BLU**********//
  else if(blu_wake_up) blu_interrupt_routine();
  

  //**********SLEEP**********//
  if(!rtc_wake_up) sleep();
}


/*********************************************************************/
/*                      RTC INTERRUPT ROUTINE                        */
/*********************************************************************/

void rtc_interrupt_routine(){
    digitalWrite(RTC_LED, HIGH);
    Serial.println("rtc_interrupt_wake_up");
    safe_modify_bool_volatile(&rtc_wake_up, false);
    RTC.alarm(ALARM_1);
    sensorRoutine();
    digitalWrite(RTC_LED, LOW);
}
  
/*********************************************************************/
/*                      BLU INTERRUPT ROUTINE                        */
/*********************************************************************/
void blu_interrupt_routine(){
    digitalWrite(BLU_LED, HIGH);
    Serial.println("blu_interrupt_wake_up");
    
    safe_modify_bool_volatile(&blu_wake_up, false);
    
    unsigned long start_time = millis();
    digitalWrite(BLU_POWER_PIN, HIGH);
    
    while(blu_routine() != 0) if (millis() - start_time >= blu_time_out) break;
    
    digitalWrite(BLU_LED, LOW);
    digitalWrite(BLU_POWER_PIN, LOW);
}


int blu_routine(){
  if (blu.available()){
    
    blu.println("d : download all data");
    blu.println("p : state ");
    blu.println("q : quit blu mode");
    
    char t = blu.read();
   
    switch(t){
      case 'd':
        //read_all_eeprom_blu();
        break;
        
      case 'q':
        return 0;
        break;

      case 'p':
        blu.print(blu_wake_up);
        blu.print("\t Sleep :");
        blu.println(rtc_wake_up);
        
        blu.print("BluCloop: ");
        blu.print(countblu);
        blu.print("\tSleepCloop :");
        blu.println(countsleep);
        break;
        
      default:
        blu.println("Wrong cmd");
        blu.println("h for help");
        return 1;
        break;  
      }

  }
  return 1;
}

/*********************************************************************/
/*                                 UTIL                              */
/*********************************************************************/

/*
 * Function: safe_modify_bool_volatile
 * ----------------------------
 *   Allow to modify safly bool volatile var
 *
 *   var: pointer from variable to modify
 *   val : bool value
 *
 */
void safe_modify_bool_volatile(volatile bool* var,  bool val){
  cli();
  *var = val;
  sei();
} 


/*********************************************************************/
/*                          EEPROM UTIL                              */
/*********************************************************************/
/*
int writeToEeprom(float value){
  if (address == EEPROM.length()) return -1;
  EEPROM.write(address, value);
  address = address + sizeof(value);
  EEPROM.write(0,address);
}

void fillEepromMemory(){
  for(int i=0; i <= EEPROM.length(); i++){
    EEPROM.write(i,0);
  }
}

void read_all_eeprom_blu(){
  blu.println("Downloading starting");
  for(int i=0; i <= EEPROM.length(); i++){
    blu.println(EEPROM.read(i));
  }
  blu.println("Downloading finish");
  delay(500);
}


double convertSec2millisTimerL(int a)
{
  periodeTimerLoopMillis = 1000*a;
  return periodeTimerLoopMillis;
}

int convertMillis2Min(float a)
{
  return  a/1000;
}

double convertMin2Millis(double a)
{
  return a*1000;
}*/

/*********************************************************************/
/*                          SENSORS ROUTINE                          */
/*********************************************************************/
//  #SENSOR #ROUTINE
void sensorRoutine()
{

 /* convertSec2millisTimerL(TLperiod);
  if ( timerLoop % periodeTimerLoopMillis == 0)
  {*/
    readLux();
    getTempHum();
    readGaz();
//    printData();
  //}
}

// Update timers
void updateTimer()
{
  timerLoop = millis();
  if (verbosityLevel >=8)
  {
    Serial.println("[Sakura] Timer updates");
  }
}

//# GAZ
int readGaz()
{
  digitalWrite(GAZ_POWER_PIN, HIGH);
  delay(2000);
  gazVal = analogRead(GAZ_PIN);  
  Serial.println( gazVal);
  delay(100);
  digitalWrite(GAZ_POWER_PIN, LOW);
  if (verbosityLevel >= 3)
     Serial.println("Ok Gaz");
     
  //if( writeToEeprom(gazVal/4) == -1 ) return -1;
}

// # LUX
int readLux()
{
  digitalWrite(LUX_POWER_PIN, HIGH);
  delay(800);
  luxVal = analogRead(LUX_PIN);  
  Serial.println(luxVal);
  delay(100);
  digitalWrite(LUX_POWER_PIN, LOW);

  if (verbosityLevel >= 3)
     Serial.println("Ok LUx");
  //if( writeToEeprom(luxVal/4) == -1 ) return -1;
}

// #TEMP #HUMIDITY #HUM
int getTempHum()
{
  digitalWrite(DHT_POWER_PIN, HIGH);
  delay(800);
  dhtData.humidity = dht.readHumidity();
  dhtData.temp = dht.readTemperature();
  Serial.println( dhtData.temp);
  digitalWrite(DHT_POWER_PIN, LOW);
  if (verbosityLevel >= 3)
     Serial.println("Ok Temp");
   
  //if( writeToEeprom(dhtData.temp) == -1 ) return -1;
  //if( writeToEeprom(dhtData.humidity) == -1 ) return -1;
}

