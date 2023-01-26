/*******************************************************************************
   Copyright (c) 2021 Paul Brouwer

   Adapted from Meet Je Stad Firmware
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   See README.md for the required libraries and other components for this sketch.
 *******************************************************************************/

// include external libraries
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <NMEAGPS.h>

#if defined(ARDUINO_ARCH_STM32L0)
  #include "STM32L0.h"
  #define USERBUTTON true;
  const int USERBUTTON_PIN=30;
#endif

#define DEBUG true
#include "usb_serial.h"
#include "mjs_lmic.h"

//Firmware version - change BOTH when updating
const String Firmware_version="SL.B.1.1";
const uint8_t firmware=111;

// Baudrate for hardware serial port
const uint16_t SERIAL_BAUD = 9600;

// When this voltage is reached, skip power hungry sensors
uint16_t const MIN_BATT_VOLT = 3300;

float const BATTERY_DIVIDER_RATIO = (1.0 + 1.0) / 1.0;
uint8_t const BATTERY_DIVIDER_PIN = PIN_BATTERY;
auto const BATTERY_DIVIDER_REF = AR_DEFAULT;
uint16_t const BATTERY_DIVIDER_REF_MV = 3000;

float const SOLAR_DIVIDER_RATIO = (2.0 + 1.0) / 1.0;
uint8_t const SOLAR_DIVIDER_PIN = PIN_SOLAR;
auto const SOLAR_DIVIDER_REF = AR_DEFAULT;
uint16_t const SOLAR_DIVIDER_REF_MV = 3000;

#define GPS_SERIAL Serial2
uint8_t const GPS_ENABLE_PIN = PIN_ENABLE_3V_GPS;

// define various pins
uint8_t const LED_PIN = LED_BUILTIN;
uint8_t const LED_ON = LOW;

uint8_t const SM1_PIN=A0;
uint8_t const SM2_PIN=A1;
uint8_t const NTC1_PIN=A2;
uint8_t const NTC2_PIN=A3;

uint8_t const EXTERNAL_WATCHDOG_RESET_PIN=44;

// Sensor object
HTU21D htu;

// GPS reader
NMEAGPS gps;

//gps measurements
int32_t lat24 = 0;
int32_t lng24 = 0;
int32_t alt24 = 0;
uint8_t sats=0;

//system status
uint8_t reset_reason;
uint8_t updatesWithoutReset = 0;

// Standard measurements
float temperature;
float humidity;
uint16_t vcc = 0;
uint16_t vbatt = 0;
uint16_t vsolar = 0;

// Soil moisture measurements
const int levels=2;
const int freqs=1;
const int byte_num = (levels)+(levels*freqs*2)+(1); //bytes expected in TTL package
const int smLen = byte_num-1;

struct l{
  uint16_t f[freqs];
  uint8_t t;
};

struct l L[levels];
uint8_t samples;
uint8_t freq_sent;

//get the size of the packages to be sent over LORA
const uint8_t gpsLen=13;
uint8_t gpsData[gpsLen];
const uint8_t stdLen=8;
const uint8_t txLen=stdLen+smLen; //bytes
uint8_t txData[txLen];

// setup timing variables
uint16_t const UPDATE_INTERVAL_MIN =60; 
uint32_t const UPDATE_INTERVAL = UPDATE_INTERVAL_MIN*60000;
uint32_t const GPS_TIMEOUT = 120000;

// Update GPS position after transmitting this many updates
uint16_t const GPS_UPDATE_RATIO = (24*60)/UPDATE_INTERVAL_MIN;
uint32_t lastUpdateTime = 0;
uint32_t updatesBeforeGpsUpdate = 0;
gps_fix gps_data;

uint8_t const LORA_PORT = 13;

/**
 * Forward declarations for functions. Not strictly needed for Arduino,
 * but makes the code easier to use with IDEs that do less
 * preprocessing.
 */

//system functions
void doSleep(uint32_t time);
void dumpData();
void getPosition();
void queueData();
uint16_t readVcc();
uint16_t readVsolar();
uint16_t readVbatt();
bool batteryVoltageOk(uint16_t voltage, const __FlashStringHelper* device);
void writeLed(uint32_t rgb);
void resetExternalWatchdog();

//measurement functions
float readSM(uint8_t apin);
uint8_t readNTC(uint8_t apin);
float readAv(uint8_t apin,uint8_t samp);
bool do_measurement();

void setup() {

  //get the reset reason and reset the reset flag
  reset_reason = STM32L0.resetCause();
  RCC->CSR |= RCC_CSR_RMVF;

  //write the LED
  writeLed(0xff0000); // red

  // when in debugging mode start serial connection
  if(DEBUG) {
    setup_serial();
    Serial.print(F("Starting, firmware version = "));
    Serial.print(Firmware_version);
    Serial.println();
    Serial.println("-----------------------------------");
    Serial.print(F("Reset reason = "));
    Serial.print(reset_reason);
    Serial.println();
    Serial.print(F("Updates Without Reset= "));
    Serial.print(updatesWithoutReset);
    Serial.println();
    Serial.print(F("Update_interval = "));
    Serial.print(UPDATE_INTERVAL_MIN);
    Serial.print(F(" Minutes"));
    Serial.println();
    Serial.println("-----------------------------------");
    Serial.println();
  }

  writeLed(0xff0c00); // orange

  // setup LoRa transceiver
  mjs_lmic_setup();

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(GPS_ENABLE_PIN , OUTPUT);
  digitalWrite(GPS_ENABLE_PIN, LOW);

  //set the optional 3V and 5V
  pinMode(PIN_ENABLE_5V, OUTPUT);
  digitalWrite(PIN_ENABLE_5V, LOW);
  pinMode(PIN_ENABLE_3V_SENS, OUTPUT);
  digitalWrite(PIN_ENABLE_3V_SENS, LOW);

  // start communication to sensors
  htu.begin();

  writeLed(0x803000); // yellow

  //if the user presses the user button or gives an m as serial input
  //within 5 seconds then perform a measurement without sending data over LoRa 
  Serial.println(F("Joining to LoRa network, press 'm' to do a measurement without sending data"));
  if(do_measurement()){
    Serial.println(F("Performing measurement without LoRa"));
    measure();
    dumpData();
  }
 
  // Start join
  LMIC_startJoining();
  
  // Wait for join to complete
  while ((LMIC.opmode & (OP_JOINING)))
    os_runloop_once();
    
  writeLed(0x000000); // off (will be turned back on for GPS directly)
}

void loop() {
  
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();

  //measure battery voltage
  vbatt = readVbatt();

  // Send GPS or send the measurement data
  bool send_gps=false;
  
  if (updatesBeforeGpsUpdate == 0) {
    if (!BATTERY_DIVIDER_RATIO || batteryVoltageOk(vbatt, F("GPS"))) {
      writeLed(0x408080); // cyan
      getPosition();
      writeLed(0x000000); // off
      updatesBeforeGpsUpdate = GPS_UPDATE_RATIO;
      updatesWithoutReset++;
      // Use the lowest datarate, to maximize range. This helps for
      // debugging, since range problems can be more easily distinguished
      // from other problems (lockups, downlink problems, etc).
      LMIC_setDrTxpow(DR_SF12, 14);
    } else {
      // Clear gps fix to prevent sending stale data
      memset(&gps_data, 0, sizeof(gps_data));
    } //if battery voltage ok

    //send the GPS data only if there is a fix
    if(lat24!=0 && lng24!=0){
      
      //send the gps data
      Serial.println(F("--> Only sending GPS"));
      queueData(true);
      mjs_lmic_wait_for_txcomplete();

      //wait at least for tx-timeout
      delay(TX_TIMEOUT);
      os_runloop_once();
    }
  } //if GPS update
  else {
    
    //set SF to 9
    LMIC_setDrTxpow(DR_SF9, 14);
    
  } //if no GPS update needed
  updatesBeforeGpsUpdate--;

 //perform the measurements
 measure();

 //reset the watchdog timer
 resetExternalWatchdog();

  if (DEBUG)
    dumpData();

  writeLed(0x0000ff); // blue
  
  // Work around a race condition in LMIC, that is greatly amplified
  // if we sleep without calling runloop and then queue data
  // See https://github.com/lmic-lib/lmic/issues/3
  os_runloop_once();

  // We can now send the data
  Serial.println(F("--> Sending Measurement Data"));
  queueData(false);
  mjs_lmic_wait_for_txcomplete();

  // Schedule sleep
  unsigned long sleepDuration = UPDATE_INTERVAL;
  unsigned long msPast = millis() - startMillis;
  if (msPast < sleepDuration)
    sleepDuration -= msPast;
  else
    sleepDuration = 1000;

  writeLed(0x000000); // off

  if (DEBUG) {
    Serial.print(F("Sleeping for "));
    Serial.print(sleepDuration);
    Serial.println(F("ms..."));
    Serial.flush();
  }
  delay(100);
  doSleep(sleepDuration);
  if (DEBUG) {
    Serial.println(F("Woke up."));
  }
}

bool do_measurement(){
  int t=0;
  while(t<100){

    //If a user button was added 
    #if defined(USER_BUTTON)
    if(DEBUG && digitalRead(D1)==0){
      return true;
    }
    #endif

    //Or wait for an 'm' as serial input
    if(DEBUG && tolower(Serial.read()) == 'm'){
      return true;
    }
    t++;
    delay(50);
  }
  return false;
  
}

void measure(){

  //start
  if (DEBUG)
  Serial.println(F("Starting to read the sensors"));
    
  // Activate and read standard sensors
  temperature = htu.readTemperature();
  humidity = htu.readHumidity();
  vcc = readVcc();
  vsolar = readVsolar();

  //power up the 3V sensors
  digitalWrite(PIN_ENABLE_3V_SENS, HIGH);
  delay(1000);

  L[0].f[0]=readSM(SM1_PIN);
  L[0].t=readNTC(NTC1_PIN);
  L[1].f[0]=readSM(SM2_PIN);
  L[1].t=readNTC(NTC2_PIN);
  
  //power down the 3V sensors
  digitalWrite(PIN_ENABLE_3V_SENS, LOW);

}


void doSleep(uint32_t time) {
  #if defined(__AVR__)
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();

  while (time > 0) {
    uint16_t slept;
    if (time < 8000)
      slept = Watchdog.sleep(time);
    else
      slept = Watchdog.sleep(8000);

    // Update the millis() and micros() counters, so duty cycle
    // calculations remain correct. This is a hack, fiddling with
    // Arduino's internal variables, which is needed until
    // https://github.com/arduino/Arduino/issues/5087 is fixed.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      extern volatile unsigned long timer0_millis;
      extern volatile unsigned long timer0_overflow_count;
      timer0_millis += slept;
      // timer0 uses a /64 prescaler and overflows every 256 timer ticks
      timer0_overflow_count += microsecondsToClockCycles((uint32_t)slept * 1000) / (64 * 256);
    }

    if (slept >= time)
      break;
    time -= slept;
  }

  power_adc_enable();
  ADCSRA |= (1 << ADEN);
  #else

  // Shut down USB. Without this, sleeping is completely prevented (and
  // the USB block also consumes power).
  // By doing this here, rather than in setup_serialusb(), firmware
  // uploads remain possible until the first sleep (and also usb is now
  // detached even when not using serialusb).
  // Note that after sleep, USB is not reattached. When USB is already
  // detached, this is just a no-op.
  USBDevice.detach();

  // No need to update the millis counter, STM32L0 uses the RTC for
  // millis and keeps it running during sleep for wakeup.
  if (time)
    STM32L0.stop(time);
  #endif
}

void dumpData() {
  Serial.println();
  Serial.println(F("--- GPS Data--- "));
  if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
    Serial.print(F("lat/lon: "));
    Serial.print(gps_data.latitudeL()/10000000.0, 6);
    Serial.print(F(","));
    Serial.println(gps_data.longitudeL()/10000000.0, 6);
  } else {
    Serial.println(F("No GPS fix"));
  }
  Serial.println();

  Serial.println(F("--- Standard Measurements--- "));
  Serial.print(F("temp="));
  Serial.print(temperature, 1);
  Serial.println();
  
  Serial.print(F("hum="));
  Serial.print(humidity, 1);
  Serial.println();
  
  Serial.print(F("vbatt="));
  Serial.print(vbatt);
  Serial.println();
  
  Serial.print(F("vsolar="));
  Serial.print(vsolar);
  Serial.println();

  Serial.print(F("vcc="));
  Serial.print(vcc, 1);
  Serial.println(); 
  Serial.println(); 

  //print soil moisture measurements
  Serial.println(F("--- Soil Moisture Measurements--- "));
  for(int lev=0;lev<levels;lev++){;
    Serial.print(lev);
    Serial.print(F(" : "));
    
    for(int f=0;f<freqs;f++){
      Serial.print(L[lev].f[f]/10);
      Serial.print(F("mV | "));
    }
    Serial.print((float(L[lev].t)/4.0-20.0),2);
    Serial.print(F("C \n"));
  }
  
  Serial.println();
  Serial.flush();
}

void getPosition()
{
  #if defined(GPS_USE_SOFTWARE_SERIAL)
  // Setup GPS
  SoftwareSerial GPS_SERIAL(GPS_PIN, GPS_PIN);
  #endif

  GPS_SERIAL.begin(9600);
  memset(&gps_data, 0, sizeof(gps_data));
  gps.reset();
  gps.statistics.init();

  digitalWrite(GPS_ENABLE_PIN, HIGH);

  // Empty serial input buffer, so only new characters are processed
  while(Serial.read() >= 0) /* nothing */;

  if (DEBUG)
    Serial.println(F("Waiting for GPS, send 's' to skip..."));

  unsigned long startTime = millis();
  uint8_t valid = 0;
  while (millis() - startTime < GPS_TIMEOUT && valid < 10) {
    if (gps.available(GPS_SERIAL)) {
      gps_data = gps.read();
      if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
        valid++;
        lat24 = int32_t((int64_t)gps_data.latitudeL() * 32768 / 10000000);
        lng24 = int32_t((int64_t)gps_data.longitudeL() * 32768 / 10000000);
        alt24 = gps_data.altitude_cm();
        sats = int(gps_data.satellites);
      } else {
        lat24 = 0;
        lng24 = 0;
        alt24 = 0;
      }

      //set the sats 
      if (gps_data.valid.satellites) {
        Serial.print(F("Satellites: "));
        Serial.println(gps_data.satellites);
      } else{
        sats=0;
      }
    }
    if (DEBUG && tolower(Serial.read()) == 's')
      break;
  }
  digitalWrite(GPS_ENABLE_PIN, LOW);

  if (gps.statistics.ok == 0)
    Serial.println(F("No GPS data received, check wiring"));

  GPS_SERIAL.end();
}

void queueData(bool send_gps) {

  //create unions
  union m32{
    uint32_t m;
    byte m_byte[4];
  };

  union m16s{
    int16_t m;
    byte m_byte[2];
  };

  union m16{
    uint16_t m;
    byte m_byte[2];
  };

  //define measurements to be sent
  union m32 lat32;
  union m32 lon32;
  union m32 alt32;
  union m16s tmp16;
  union m16s hum16;
  union m16 sol;
  uint8_t vcc8 = (vcc - 1000) / 10;
  uint8_t batt = vbatt / 20;

  //get the measurement values
  lat32.m=lat24;
  lon32.m=lng24;
  alt32.m=alt24;
  tmp16.m = temperature * 16;
  hum16.m = humidity * 16;
  sol.m=vsolar;

  //add the GPS data to the package
  gpsData[0]=lat32.m_byte[0];
  gpsData[1]=lat32.m_byte[1];
  gpsData[2]=lat32.m_byte[2];
  gpsData[3]=lon32.m_byte[0];
  gpsData[4]=lon32.m_byte[1];
  gpsData[5]=lon32.m_byte[2];
  gpsData[6]=alt32.m_byte[0];
  gpsData[7]=alt32.m_byte[1];
  gpsData[8]=alt32.m_byte[2];
  gpsData[9]=sats;
  gpsData[10]=reset_reason;
  gpsData[11]=firmware;
  gpsData[12]=updatesWithoutReset;

  //add the standard measurements to the data-package
  txData[0]=tmp16.m_byte[0];
  txData[1]=tmp16.m_byte[1];
  txData[2]=hum16.m_byte[0];
  txData[3]=hum16.m_byte[1];
  txData[4]=sol.m_byte[0];
  txData[5]=sol.m_byte[1];
  txData[6]=batt;
  txData[7]=vcc8;
  
  //add soil moisture measurements to the package
  int count=8;
  for(int lev=0;lev<levels;lev++){;
    for(int f=0;f<freqs;f++){

      //append the capacity value to data
      union m16 C;
      C.m=L[lev].f[f];
      txData[count]=C.m_byte[0];
      txData[count+1]=C.m_byte[1];
      count=count+2;
    }

    //append the temperature value to data
    txData[count]=L[lev].t;
    count=count+1;
  }

  //variables for showing packet output 
  uint8_t * data;
  uint8_t data_size;

  // Prepare upstream data transmission at the next possible time.
  if(send_gps){
    LMIC_setTxData2(LORA_PORT, gpsData, sizeof(gpsData), 0);
    data = gpsData;
    data_size=sizeof(gpsData);
  } else{
    LMIC_setTxData2(LORA_PORT, txData, sizeof(txData), 0);
    data = txData;
    data_size=sizeof(txData);
  }
  
  if (DEBUG){
    Serial.print(F("Packet queued: "));
    Serial.print(data_size);
    Serial.print(F(" bytes\n"));
    for (int i = 0; i < data_size; i++)
    {
      if (data[i] < 0x10)
        Serial.write('0');
      Serial.print(data[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
    Serial.flush();
  }
}

uint16_t readVcc(){
  return STM32L0.getVDDA() * 1000;
}

uint16_t readVsolar() {
    analogReference(SOLAR_DIVIDER_REF);
    uint16_t reading = analogRead(SOLAR_DIVIDER_PIN);
    return (uint32_t)(reading*SOLAR_DIVIDER_RATIO*SOLAR_DIVIDER_REF_MV)/1023;
}

uint16_t readVbatt() {
    analogReference(BATTERY_DIVIDER_REF);
    uint16_t reading = analogRead(BATTERY_DIVIDER_PIN);
    return (uint32_t)(reading*BATTERY_DIVIDER_RATIO*BATTERY_DIVIDER_REF_MV)/1023;
}

/**
 * Check if the battery voltage is high enough for power-hungry
 * measurements. Return true if so, or print an error message and return
 * false if not.
 */
bool batteryVoltageOk(uint16_t voltage, const __FlashStringHelper* device) {
    if (voltage < MIN_BATT_VOLT) {
        Serial.print(F("Battery voltage too low, skipping "));
        Serial.print(device);
        Serial.print(F(" ("));
        Serial.print(voltage);
        Serial.print(F(" < "));
        Serial.print(MIN_BATT_VOLT);
        Serial.println(F(" mV)"));
        return false;
    }
    return true;
}

void writeSingleLed(uint8_t pin, uint8_t val) {

  if (val == 0) {
    pinMode(pin, HIGHZ);
  } else {
    analogWriteResolution(10);
    analogWrite(pin, 1023 - val);
    
    // Reset back to the default, which other code probably expects.
    analogWriteResolution(8);
  }
}

void writeLed(uint32_t rgb) {
  writeSingleLed(PIN_LED_RED, (rgb >> 16));
  writeSingleLed(PIN_LED_GREEN, (rgb >> 8));;
  writeSingleLed(PIN_LED_BLUE, (rgb >> 0));
}

uint8_t readNTC(uint8_t apin){
  
  //ADC values, assuming Vcc=1024
  float steps = 1024;

  //NTC parameters
  float r0=10000;
  float T0=25+273;
  float r_ref=15000; //adjust
  float beta=3950;

  //measure the voltage
  float Tint=readAv(apin,10);

  //calculate the temperature 
  float Tv=Tint/steps;
  float Tr=r_ref*(1-Tv)/Tv;
  float Tc=1/(log(Tr/r0)/beta+1/T0)-273;
  uint8_t Traw = (Tc+20)*4;

  return Traw;
  
}

float readSM(uint8_t apin) {

  float steps=1024;
  float Sraw=readAv(apin,10);
  float Svolt=10*(vcc/steps)*Sraw; //0.1mV
  
  return Svolt;
  
}

float readAv(uint8_t apin,uint8_t samp){
  
  float sum=0;
  int i=0;
  for(i;i<samp;i++){
    sum+=analogRead(apin);
  }

  return (sum/float(i));
  
}

void resetExternalWatchdog(){
  
  pinMode(EXTERNAL_WATCHDOG_RESET_PIN,OUTPUT);
  digitalWrite(EXTERNAL_WATCHDOG_RESET_PIN, LOW);
  delay(100);
  pinMode(EXTERNAL_WATCHDOG_RESET_PIN,INPUT);
  
}
