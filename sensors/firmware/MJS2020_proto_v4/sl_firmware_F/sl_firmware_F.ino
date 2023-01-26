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
#include "SparkFun_SCD30_Arduino_Library.h"

#if defined(ARDUINO_ARCH_STM32L0)
  #include "STM32L0.h"
  #define USERBUTTON true;
  const int USERBUTTON_PIN=30;
#endif

#define DEBUG true
#include "usb_serial.h"
#include "mjs_lmic.h"

//Firmware version
const String Firmware_version = "SL.F.1.1";
const uint8_t firmware=611;

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

#define SM_SERIAL Serial3

// Sensor objects
SCD30 airSensor;

// GPS reader
NMEAGPS gps;

//gps measurements
int32_t lat24 = 0;
int32_t lng24 = 0;
int32_t alt24 = 0;
uint8_t sats = 0;

//system status
uint8_t reset_reason;
uint8_t updatesWithoutReset = 0;

// Standard measurements
uint16_t vcc = 0;
uint16_t vbatt = 0;
uint16_t vsolar = 0;
float temperature = 0;
float humidity = 0;
float CO2 = 0;
float CO2_var = 0;
float soilM1 = 0;
float soilM2 = 0;
uint8_t soilT1 = 0;
uint8_t soilT2 = 0;

//get the size of the packages to be sent over LORA
const uint8_t gpsLen = 13;
uint8_t gpsData[gpsLen];
const uint8_t stdLen = 17;
const uint8_t txLen = stdLen; //bytes
uint8_t txData[txLen];

// setup timing variables
uint16_t const UPDATE_INTERVAL_MIN =15; 
uint32_t const UPDATE_INTERVAL = UPDATE_INTERVAL_MIN*60000;
uint32_t const GPS_TIMEOUT = 120000;
uint32_t const CO2_SAMPLES = 10;
uint32_t const CO2_TIMEOUT = CO2_SAMPLES*3000;   //ms
uint32_t const CO2_DELAY = 30000;      //ms

// Update GPS position after transmitting this many updates
uint16_t const GPS_UPDATE_RATIO = (24*60)/UPDATE_INTERVAL_MIN; //24h
uint32_t lastUpdateTime = 0;
uint32_t updatesBeforeGpsUpdate = 0;
gps_fix gps_data;

uint8_t const LORA_PORT = 12;

/**
   Forward declarations for functions. Not strictly needed for Arduino,
   but makes the code easier to use with IDEs that do less
   preprocessing.
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
bool do_measurement();
bool beginCO2();
uint32_t readCO2(uint32_t del, int s);
void endCO2();
float readSM(uint8_t apin);
uint8_t readNTC(uint8_t apin);
float readAv(uint8_t apin, uint8_t samp);

void setup() {

  //get the reset reason and reset the reset flag
  reset_reason = STM32L0.resetCause();
  RCC->CSR |= RCC_CSR_RMVF;

  writeLed(0xff0000); // red

  // when in debugging mode start serial connection
  if (DEBUG) {
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
  //htu.begin();

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
  // TODO: Sleep between join attempts
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
  bool send_gps = false;

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
    if (lat24 != 0 && lng24 != 0) {

      //send the gps data
      Serial.println(("--> Only sending GPS"));
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
    sleepDuration = 1000; //minimum may be higher 

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

void measure() {

  Serial.println(F("Starting to read the sensors"));

  //---- Standard sensors ----//

  // Activate and read the standard sensors
  //temperature = htu.readTemperature();
  //humidity = htu.readHumidity();
  vcc = readVcc();
  vsolar = readVsolar();

  //---- 3V sensors ----//

  //power up the 3V sensors
  digitalWrite(PIN_ENABLE_3V_SENS, HIGH);
  delay(1000);

  soilM1 = readSM(SM1_PIN);
  soilT1 = readNTC(NTC1_PIN);
  soilM2 = readSM(SM2_PIN);
  soilT2 = readNTC(NTC2_PIN);

  digitalWrite(PIN_ENABLE_3V_SENS, LOW);

  //---- 5V sensors ----//

  //power up the 5V sensors
  digitalWrite(PIN_ENABLE_5V, HIGH);

  //read CO2 sensor
  uint32_t t = 0;
  delay(CO2_DELAY);
  if (beginCO2()) {
    t = readCO2(CO2_TIMEOUT, CO2_SAMPLES);
  }
  endCO2();

  //power down the 5V sensors
  digitalWrite(PIN_ENABLE_5V, LOW);

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
    Serial.print(gps_data.latitudeL() / 10000000.0, 6);
    Serial.print(F(","));
    Serial.println(gps_data.longitudeL() / 10000000.0, 6);
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

  Serial.print(F("CO2="));
  Serial.print(CO2, 1);
  Serial.print(" +/- ");
  Serial.print(CO2_var, 1);
  Serial.println(F("ppm"));

  Serial.print(F("Soil Moisture 1="));
  Serial.print(soilM1, 1);
  Serial.println(F("Volt"));
  Serial.print(soilM2, 1);
  Serial.println(F("Volt"));

  Serial.print(F("Soil Temperature 1="));
  Serial.print(float(soilT1)/4-20, 1);
  Serial.println("Volt");
  Serial.print(float(soilT2)/4-20, 1);
  Serial.println("Volt");

  Serial.print("vbatt=");
  Serial.print(vbatt);
  Serial.println();

  Serial.print("vsolar=");
  Serial.print(vsolar);
  Serial.println();

  Serial.print(F("vcc="));
  Serial.print(vcc, 1);
  Serial.println();
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
  while (Serial.read() >= 0) /* nothing */;

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
      } else {
        sats = 0;
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
  union m32 {
    uint32_t m;
    byte m_byte[4];
  };

  union m16s {
    int16_t m;
    byte m_byte[2];
  };

  union m16 {
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

  union m16 cotwo;
  uint8_t cotwo_var;
  union m16 sm1;
  union m16 sm2;

  //get the measurement values
  lat32.m = lat24;
  lon32.m = lng24;
  alt32.m = alt24;
  sol.m = vsolar;
  tmp16.m = temperature * 16;
  hum16.m = humidity * 16;
  cotwo.m = CO2 * 10;         //max value=5500 ppm
  cotwo_var = CO2_var;        //max value=256ppm
  sm1.m = soilM1 * 10;        //max value=6.6V
  sm2.m = soilM2 * 10;        //max value=6.6V

  //add the GPS data to the package
  gpsData[0] = lat32.m_byte[0];
  gpsData[1] = lat32.m_byte[1];
  gpsData[2] = lat32.m_byte[2];
  gpsData[3] = lon32.m_byte[0];
  gpsData[4] = lon32.m_byte[1];
  gpsData[5] = lon32.m_byte[2];
  gpsData[6] = alt32.m_byte[0];
  gpsData[7] = alt32.m_byte[1];
  gpsData[8] = alt32.m_byte[2];
  gpsData[9] = sats;
  gpsData[10]=reset_reason;
  gpsData[11]=firmware;
  gpsData[12]=updatesWithoutReset;

  //add the standard measurements to the data-package
  txData[0] = tmp16.m_byte[0];
  txData[1] = tmp16.m_byte[1];
  txData[2] = hum16.m_byte[0];
  txData[3] = hum16.m_byte[1];
  txData[4] = sol.m_byte[0];
  txData[5] = sol.m_byte[1];
  txData[6] = batt;
  txData[7] = vcc8;

  // add the extra measurements
  txData[8] = cotwo.m_byte[0];
  txData[9] = cotwo.m_byte[1];
  txData[10] = cotwo_var;
  txData[11] = sm1.m_byte[0];
  txData[12] = sm1.m_byte[1];
  txData[13] = soilT1;
  txData[14] = sm2.m_byte[0];
  txData[15] = sm2.m_byte[1];
  txData[16] = soilT2;

  //variables for showing packet output
  uint8_t * data;
  uint8_t data_size;

  // Prepare upstream data transmission at the next possible time.
  if (send_gps) {
    LMIC_setTxData2(LORA_PORT, gpsData, sizeof(gpsData), 0);
    data = gpsData;
    data_size = sizeof(gpsData);
  } else {
    LMIC_setTxData2(LORA_PORT, txData, sizeof(txData), 0);
    data = txData;
    data_size = sizeof(txData);
  }

  if (DEBUG) {
    Serial.print(F("Packet queued: "));
    Serial.print(data_size);
    Serial.print(F(" bytes\n"));
    for (int i = 0; i < data_size; i++)
    {
      if (data[i] < 0x10)
        Serial.write('0');
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.flush();
  }
}

uint16_t readVcc() {
  return STM32L0.getVDDA() * 1000;
}

uint16_t readVsolar() {
  analogReference(SOLAR_DIVIDER_REF);
  uint16_t reading = analogRead(SOLAR_DIVIDER_PIN);
  return (uint32_t)(reading * SOLAR_DIVIDER_RATIO * SOLAR_DIVIDER_REF_MV) / 1023;
}

uint16_t readVbatt() {
  analogReference(BATTERY_DIVIDER_REF);
  uint16_t reading = analogRead(BATTERY_DIVIDER_PIN);
  return (uint32_t)(reading * BATTERY_DIVIDER_RATIO * BATTERY_DIVIDER_REF_MV) / 1023;
}

/**
   Check if the battery voltage is high enough for power-hungry
   measurements. Return true if so, or print an error message and return
   false if not.
*/
bool batteryVoltageOk(uint16_t voltage, const __FlashStringHelper* device) {
  if (voltage < MIN_BATT_VOLT) {
    Serial.print(F("Battery voltage too low, skipping "));
    Serial.print(device);
    Serial.print(F(" ("));
    Serial.print(voltage);
    Serial.print(F(" < "));
    Serial.print(MIN_BATT_VOLT);
    Serial.println(" mV)");
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

float readSM(uint8_t apin) {

  float steps = 1024;
  float Sraw = readAv(apin, 10);
  float Svolt = (vcc / steps) * Sraw; //0.1mV

  return Svolt;

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

float readAv(uint8_t apin, uint8_t samp) {

  float sum = 0;
  int i = 0;
  for (i; i < samp; i++) {
    sum += analogRead(apin);
  }

  return (sum / float(i));

}

boolean beginCO2() {

  //start wire protocol
  Wire.begin();

  //start the air_sensor
  if (airSensor.begin() == false) {
    Serial.println(F("   SCD30: not found"));
    return false;
  }

  return true;

}

uint32_t readCO2(uint32_t del, int s) {

  //timing variables
  uint32_t start = millis();
  uint32_t t = 0;

  //averaging function variables
  int c = 0;
  float CO2_sum = 0;
  float temp_sum = 0;
  float hum_sum = 0;

  float CO2_sam[s];
  float reject = 0.3;
  bool timeout = false;

  //main loop untill s samples are taken or timeout is triggered
  while (c < s && not timeout) {

    //while no data is available and time is within timeout
    while (not airSensor.dataAvailable() && t < del) {
      t = millis() - start;
      delay(500);
    }

    //if the timeout was triggered
    if (t > del) {
      Serial.println(F("   SCD30: sensor timeout"));
      timeout = true;

    } else {

      //add data to the average
      CO2_sam[c] = airSensor.getCO2();
      CO2_sum += CO2_sam[c];
      temp_sum += airSensor.getTemperature();
      hum_sum += airSensor.getHumidity();

      //increment time and count
      t = millis() - start;
      c += 1;
    }
  }

  if (c > 0) {

    //average of measurements
    CO2 = CO2_sum / float(c);
    temperature = temp_sum / float(c);
    humidity = hum_sum / float(c);

    //standard deviation CO2 measurements
    int r = 0;
    float CO2_sam_sum = 0;
    float CO2_var_sum = 0;

    for (int i = 0; i < c; i++) {
      CO2_sam_sum += CO2_sam[i];
      CO2_var_sum += pow((CO2_sam[i] - CO2), 2);

      //reject outliers and calculate standard deviation
      if ((abs(CO2_sam[i] - CO2) / CO2) < reject) {
        CO2_var += pow((CO2_sam[i] - CO2), 2);

      } else {
        Serial.println(F("   SCD30: rejecting outlier"));
        CO2_sum = CO2_sum - CO2_sam[i];
        r += 1;
        CO2 = CO2_sum / float(c - r);
      }
    }

    //calculate corrected values or all values if all but one
    //measurement is rejected
    if (r < (c - 1)) {
      CO2_var = sqrt((CO2_var / (c - r)));
    } else {
      CO2 = CO2_sam_sum / float(c);
      CO2_var = sqrt((CO2_var_sum) / c);
    }
    //report
    Serial.print(F("   SCD30: "));
    Serial.print(c - r);
    Serial.print(F(" Samples, time: "));
    Serial.print(t);
    Serial.print(F(" ms\n"));

  } else {

    //no data
    Serial.println(F("   SCD30: No data from sensor"));
  }

  //return the time needed for the measurement
  return t;

}

void endCO2() {
  airSensor.StopMeasurement();
  Wire.end();
}

void resetExternalWatchdog(){
  
  pinMode(EXTERNAL_WATCHDOG_RESET_PIN,OUTPUT);
  digitalWrite(EXTERNAL_WATCHDOG_RESET_PIN, LOW);
  delay(100);
  pinMode(EXTERNAL_WATCHDOG_RESET_PIN,INPUT);
  
}
