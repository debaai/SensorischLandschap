//Written for ATTINY24

#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int pinRST=1;
volatile boolean f_wdt = 1;
long counter = 0;
long timer_minutes=360; // 6h=360

void setup(){

  //set RESET pin as output
  pinMode(pinRST,INPUT);
  setup_watchdog(9); // approximately 8 seconds sleep

}

void loop(){

  //wait for timed out watchdog
  if (f_wdt==1) { 
    counter++; 
    f_wdt=0;       // reset flag
  
    //if the counter is a certain value then excecute the function
    if (counter > 7*timer_minutes){ 

      //set pinRST LOW for 100ms
      pinMode(pinRST,OUTPUT);
      digitalWrite(pinRST,LOW);
      delay(100);
    
      //set RST pin to input to save power
      pinMode(pinRST,INPUT); // set all used port to intput to save power
    
      counter = 0;
    }

    //do the actual sleep again
    system_sleep();
  }
}

// set system into the sleep state 
// system wakes up when wtchdog is timed out

void system_sleep() {

  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

void setup_watchdog(int ii) {

  byte bb;
  int ww;

  if (ii > 9 ) ii=9;

  bb=ii & 7;

  if (ii > 7) bb|= (1<<5);

  bb|= (1<<WDCE);

  ww=bb;

  MCUSR &= ~(1<<WDRF);

  // start timed sequence (WDTCSR = WDTCR on ATTINY85)

  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog timeout value

  WDTCSR = bb;

  WDTCSR |= _BV(WDIE);

}

// Watchdog Interrupt Service / is executed when watchdog timed out

ISR(WDT_vect) {

  f_wdt=1;  // set global flag

}
