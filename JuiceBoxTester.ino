#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "ATM90E26.h" // located in sketch folder

ATM90E26 eMeter;  

const int defaultUgain = 0x6EFE;
const int defaultIgain = 0x7530;

// WHAT KIND OF BOARD IS THIS?
//--- v8.12.1 ~ v8.12.3
#define trim120current
//--- v8.12.1 only
//#define V8121
//--- v8.12.4+ no options

//#define CLASSIC

// Buzzer?
#define BUZZER_PUI
//#define BUZZER_JAMECO

// DIGITAL PINS
// 0 = hardware serial Rx
// 1 = hardware serial Tx
// 2 = free pin, optional debug serial data stream output
const byte pin_GFI=3; // GFI trip input pin - attached to the GFI interrupt
const byte pin_sRX=4; // SoftSerial RX - used for ATM90E26 
const byte pin_sTX=5; // SoftSerial TX - used for ATM90E26
#ifdef V8121
const byte pin_GFItest=A5; // pin is A5 on v8.12.1
#else
const byte pin_GFItest=6; // pin wired to a GFCI-tripping relay - for the periodic testing of the GFCI circuit. relay takes 10mA input
#endif
// GFI trip pin - goes high on GFI fault, driven by the specialized circuit based on LM1851 
// has to be pin 3 as only pin 2 and 3 are available for interrupts on Pro Mini
const byte pin_GFIreset=7; // this is new for 8.9+ boards - reset pin (active LOW) for the D-latch that gets set upon GFI trip
const byte pin_pilot_break=8; // pin D9 controls solid-state switch to break pilot line
const byte pin_PWM=9; // J pilot PWM pin
const byte pin_LED_A=10; // Currently used as Calibration indication pin, can be reset after calibration code in setup() to be used as LED input
const byte pin_eMeter_calibrate=10; // Also redefined as programming pin....
const byte pin_ctrlBtn_A=11; // control button 3 ("A" on the remote, receiver pin 0) 
const byte pin_WiFi_reset=12; // WiFi module reset pin - active either LOW or HIGH depending on the board version
const byte pin_Buzzer=13; // control button 3 ("A" on the remote, receiver pin 0) 
// ANALOG PINS
#ifdef trim120current
  const byte pin_throttle120=0; // when RTC is not used, this is an input used to set 120V target current (0-30A range)
#else
const byte pin_GFI_voltage=0;
#endif
const byte pin_V = 1; // GMI input (misleading label)
const byte pin_therm = 2; // thermistor
const byte pin_relay=A3; // main relay 
const byte pin_throttle=4; // wired to a R10 trimpot - setting 240V current
const byte pin_pV=6; // pilot signal sensed through a 3-element divider 
const byte pin_stuck_relay = 7;

int PCBtemp=0;
int power=0; // instantaneous

//--------- Energy metering variables ------------------
const float ppwh = 3.2; // 3200 pulses/kwh = 3.2 pulse per wh

int ii=0, zz=0;
unsigned long tempLong;
float tempFloat;

#ifdef CLASSIC
int GFI_V_SENSE = 0;
int max_GFI_V_SENSE = 0;
unsigned int classicVSenseStop;
#endif

boolean dropOut; // flag if only one test is being run
unsigned int minefield = 0; // bitmask of failed tests

char str[160]; // main temp str buffer - do not go below 100, careful of going too high as may start having memory issues

SoftwareSerial MeterSerial(pin_sRX, pin_sTX);

byte testState = 0;
#define TEST_INIT 0
#define TEST_PILOT 1
#define TEST_GMI 2
#define TEST_GFI 3
#define TEST_RELAY 4
#define TEST_VSENSE 10
#define TEST_EMETER 5
#define TEST_TEMP 6
#define TEST_BUZZER 7
#define TEST_TRIMPOT 8
#define TEST_EEPCLEAR 9
#define FN_RLHIGH 'H'
#define FN_RLLOW 'L'
#define TEST_DONE 254

#define BUZZ_HAPPY 0
#define BUZZ_SAD 1
#define BUZZ_TEST 2

const char failed[] PROGMEM = "(#-FAIL-#)";
const char passed[] PROGMEM = "( OK )";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  MeterSerial.begin(9600); // meter IC runs at 9600 baud
  if (EEPROM.read(0) == 69) testState = TEST_DONE;
}

void loop() {
  while (Serial.available()) Serial.read(); // clear buffer
  // put your main code here, to run repeatedly:
  if (testState <= 9) Serial.print(testState,DEC);
  else Serial.print(testState);
  Serial.print(": ");
  switch (testState) {
    case TEST_INIT:
      Serial.println(F("Setting up pins..."));
      pinMode(pin_V, INPUT); // Set analog pin 1 as input
      // set digital output pins
      pinMode(pin_PWM, OUTPUT);
      pinMode(pin_relay, OUTPUT);
      pinMode(pin_GFI,INPUT);
      // pinMode(pin_WPS, OUTPUT); // do NOT do this if there is a remote installed!
      pinMode(pin_GFItest, OUTPUT);
      pinMode(pin_GFIreset, OUTPUT);
      pinMode(pin_eMeter_calibrate, INPUT_PULLUP);
      pinMode(pin_Buzzer, OUTPUT);
      testState = TEST_PILOT;
      delay(1000);
      break;
    case TEST_PILOT:
      Serial.print(F("Testing pilot signal: "));
      digitalWrite(pin_PWM,0);
      delay(100);
      ii = analogRead(pin_pV);
      Serial.print(F("-12V pin_pV = ")); Serial.print(ii); Serial.print(' ');
      passFail(ii < 170);
      digitalWrite(pin_PWM,1);
      delay(100);
      ii = analogRead(pin_pV);
      Serial.print(F("+12V pin_pV = ")); Serial.print(ii); Serial.print(' ');
      passFail(ii > 730);
      testState = TEST_GMI;
      Serial.println();
      break;
    case TEST_GMI:
      Serial.print(F("Testing GMI (ground)..."));
      ii = analogRead(pin_V);
      Serial.print(F(" pin_V = ")); Serial.print(ii); Serial.print(' ');
      passFail(ii < 840);
      testState = TEST_GFI;
      Serial.println();
      break;
    case TEST_GFI:
      Serial.print(F("Testing GFI - reset, "));
      digitalWrite(pin_GFIreset, LOW); // active LOW
      delay(5); // so it takes
      digitalWrite(pin_GFIreset, HIGH); 
      delay(5); // so it takes
      Serial.print(F("test not tripped "));
      passFail(!digitalRead(pin_GFI));
      Serial.print(F(", set timer for 400msec, enable GFI test relay, test for trip "));
      digitalWrite(pin_GFItest,HIGH);
      tempLong = millis() + 400; // JB firmware only gives it 400msec
#ifdef CLASSIC
      classicVSenseStop = millis() + 130; // JB firmware runs approx 130msec of testing
#endif
      ii = 0;
#ifdef CLASSIC
// sense voltage while doing GFI test
      while (millis() < classicVSenseStop) {
        if (millis() < classicVSenseStop) {
          GFI_V_SENSE = analogRead(pin_GFI_voltage);
          if(GFI_V_SENSE > max_GFI_V_SENSE) {
              max_GFI_V_SENSE = GFI_V_SENSE;
          }
        }
      }
// then, complete the GFI test
      while (millis() < tempLong) {
        if (digitalRead(pin_GFI)) {
          ii = (tempLong - millis());
          tempLong = millis();
        }
      }
#else
// just run GFI test
      while (millis() < tempLong) {
        if (digitalRead(pin_GFI)) {
          ii = (tempLong - millis());
          tempLong = millis();
        }
      }
#endif
      digitalWrite(pin_GFItest,LOW);
      passFail(ii);
      Serial.print(F(" in ")); Serial.print(400-ii); Serial.println(" msec");
      // reset GFI
      delay(100); // wait for it to open lololol
      digitalWrite(pin_GFIreset, LOW); // active LOW
      delay(5); // so it takes
      digitalWrite(pin_GFIreset, HIGH); 
      testState = TEST_RELAY;
      break;
    case TEST_RELAY:
      Serial.print(F("Testing relay - open = "));
      ii = analogRead(pin_stuck_relay);
      Serial.print(ii); Serial.print(' ');
      passFail(ii >= 840);
      Serial.print(F(", closed = "));
      digitalWrite(pin_relay,HIGH);
      delay(1000);
      ii = analogRead(pin_stuck_relay);
      Serial.print(ii); Serial.print(' ');
      passFail(ii < 840);
      // reset relay
      Serial.print(F(", GFI trip = "));
      Serial.print(digitalRead(pin_GFI),DEC);
      digitalWrite(pin_relay,LOW);
      Serial.println();
#ifdef CLASSIC
      testState = TEST_VSENSE;
#else
      testState = TEST_EMETER;
#endif
      break;
#ifdef CLASSIC
    case TEST_VSENSE:
      Serial.print(F("Verifying v8.12.4 Vsense: "));
      tempFloat = 5.0*((float)max_GFI_V_SENSE/1024.0);
      tempFloat *= 100;
      Serial.print(tempFloat); Serial.print("V ");
      passFail(tempFloat > 200);
      Serial.println();
      testState = TEST_TEMP;
      break;
#else
    case TEST_EMETER:
      Serial.print(F("Testing Emeter - init "));
      eMeter.begin(&MeterSerial,&Serial); // No debug serial in production
      delay(500);
      passFail(calibrateMeter());
      Serial.print(F(", Volts = "));
      tempFloat = eMeter.readVolts();
      Serial.print(tempFloat); Serial.print(' ');
      passFail(tempFloat > 200);
      Serial.print(F(", Freq = "));
      tempFloat = eMeter.readFrequency();
      Serial.print(tempFloat); Serial.print(' ');
      passFail((tempFloat > 59) && (tempFloat < 61));
      Serial.println();
#ifdef V8121
      testState = TEST_BUZZER;
#else
      testState = TEST_TEMP;
#endif
      break;
#endif
#ifndef V8121
    case TEST_TEMP:
      Serial.print(F("Testing temp sensor... "));
      ii = analogRead(pin_therm);
      tempFloat = (1/( log(1024./ii-1) /4540.+1/298.)-273);
      Serial.print(tempFloat); Serial.print("C ");
      passFail((tempFloat > 12.7) && (tempFloat < 32.2));
      Serial.println();
      testState = TEST_BUZZER;
      break;
#endif
    case TEST_BUZZER:
      Serial.println(F("Testing buzzer..."));
      buzz(BUZZ_TEST);
      testState = TEST_TRIMPOT;
      break;
    case TEST_TRIMPOT:
      Serial.println(F("Test trimpots"));
      // pin_throttle
      // pin_throttle120
      #ifdef trim120current
        Serial.print(F("Testing 120 - set to 0%"));
        tempLong = millis();
        while (analogRead(pin_throttle120) > 64) {
          if (millis() > tempLong) {
            Serial.print(analogRead(pin_throttle120)); Serial.print(' ');
            tempLong = millis() + 1000;
          }
          if (Serial.available()) {
            testState = TEST_DONE;
            return;
          }
        }
        Serial.println();
        Serial.print(F("Testing 120 - set to 50%"));
        tempLong = millis();
        while (analogRead(pin_throttle120) < 500) {
          if (millis() > tempLong) {
            Serial.print(analogRead(pin_throttle120)); Serial.print(' ');
            tempLong = millis() + 1000;
          }
        }
        Serial.println();
        Serial.print(F("Testing 120 - set to 100%"));
        tempLong = millis();
        while (analogRead(pin_throttle120) < 980) {
          if (millis() > tempLong) {
            Serial.print(analogRead(pin_throttle120)); Serial.print(' ');
            tempLong = millis() + 1000;
          }
        }
        Serial.println();
        Serial.print(F("Testing 120 - set to 15A"));
        tempLong = millis();
        while (getAmps(pin_throttle120) != 15) {
          if (millis() > tempLong) {
            Serial.print(getAmps(pin_throttle120),DEC); Serial.print(' ');
            tempLong = millis() + 1000;
          }
        }
        Serial.println();        
      #endif
      Serial.print(F("Testing 240 - set to 0%"));
      tempLong = millis();
      while (analogRead(pin_throttle) > 64) {
        if (millis() > tempLong) {
          Serial.print(analogRead(pin_throttle)); Serial.print(' ');
          tempLong = millis() + 1000;
        }
        if (Serial.available()) {
          testState = TEST_DONE;
          return;
        }
      }
      Serial.println();
      Serial.print(F("Testing 240 - set to 50%"));
      tempLong = millis();
      while (analogRead(pin_throttle) < 500) {
        if (millis() > tempLong) {
          Serial.print(analogRead(pin_throttle)); Serial.print(' ');
          tempLong = millis() + 1000;
        }
      }
      Serial.println();
      Serial.print(F("Testing 240 - set to 100%"));
      tempLong = millis();
      while (analogRead(pin_throttle) < 980) {
        if (millis() > tempLong) {
          Serial.print(analogRead(pin_throttle)); Serial.print(' ');
          tempLong = millis() + 1000;
        }
      }
      Serial.println();
      testState = TEST_EEPCLEAR;
      break;
    case TEST_EEPCLEAR:
      Serial.print(F("Clearing EEPROM... "));
      for (ii=0; ii<1024; ii++) {
        EEPROM.write(ii,0xFF);
      }
      if (minefield == 0) EEPROM.write(0,69); // tag the header that testing was successful and reset is completed
      Serial.println(F("Done."));
      testState = TEST_DONE;
      break;
    case TEST_DONE:
      Serial.println(F("Test complete! Enter 0-9 to re-run specific test."));
      if (minefield) buzz(BUZZ_SAD); else buzz(BUZZ_HAPPY);
      while (!Serial.available()) ;
      break;
    case FN_RLHIGH:    
      Serial.print(F("Relay is ON, GFI trip = "));
      Serial.println(digitalRead(pin_GFI),DEC);
      digitalWrite(pin_relay,HIGH);
      break;
    case FN_RLLOW:
      Serial.print(F("Relay is OFF, GFI trip = "));
      Serial.println(digitalRead(pin_GFI),DEC);
      digitalWrite(pin_relay,LOW);
      break;
    default:
      testState++; // skip whatever
      break;
  }
  if (dropOut) testState = TEST_DONE;
  if (Serial.available()) {
    str[0] = Serial.read();
    str[1] = 0;
    if ((str[0] >= '0') && (str[0] <= '9')) testState = atoi(str);
    else if ((str[0] >= 'A') && (str[0] <= 'Z')) testState = str[0];
    dropOut = true;
  }
  Serial.flush();
}

void passFail(boolean checkValue) {
  if (checkValue) Serial.print((const __FlashStringHelper *)passed);
  else {
    Serial.print((const __FlashStringHelper *)failed);
    minefield |= 1 << testState;
  }
}

boolean calibrateMeter() {
  zz = 0;
  ii = 0;
  while (ii <= 5) {
    //Begin Metering Calibration using values stored in EEPROM - Program constants and configuration, in addition to calibrating line gains
    zz = eMeter.write(0x20, 0x5678);
    
    // Write PL constant to registers 0x21 and 0x22 
    // PL contstant calcualted per ATM90E26 app notes and converted to HEX
    zz |= eMeter.write(0x21, 0x15);
    zz |= eMeter.write(0x22, 0x5F29);
    
    // Write MMode configuration word to appropriate register to set up metering configuration
    zz |= eMeter.write(0x2B, 0x9022);
    
    // Begin Measurement calibration - Calibrate voltage and current readings 
    zz |= eMeter.write(0x30, 0x5678);
    
    // Calibrate voltage - read gain value from EEPROM and program meter
    zz |= eMeter.write(0x31, defaultUgain);
    
    // Calibrate current - read gain value from EEPROM and program meter
    zz |= eMeter.write(0x33, defaultIgain);
    // if successful, stop trying; if not, try again and increment
    if (zz) ii = 6; else ii++;
  }
  if (ii != 6) return false; else return true;
}

void buzz(byte disposition) {
// pleasant tones
// happiness = 880, 988, 1318
// unhappiness = 880, 622 | 554
// test = 932
  switch (disposition) {
    case BUZZ_HAPPY:
      tone(pin_Buzzer,880);
      delay(333);
      tone(pin_Buzzer,988);
      delay(333);
      tone(pin_Buzzer,1318,333);
      delay(333);
      break;
    case BUZZ_SAD:
      tone(pin_Buzzer,880);
      delay(500);
      tone(pin_Buzzer,622,500);
      delay(1000);
      for (ii = 0; ii<16; ii++) {
        if (minefield & 1) {
          for (zz=0; zz<ii; zz++) {
            tone(pin_Buzzer,554,333);
            delay(500);
          }
          delay(500);
        }
        minefield >>= 1;
      }
      break;
    case BUZZ_TEST:
      tone(pin_Buzzer,932,1000); // beep 250ms
      break;
  }      
}

char getAmps(byte analogPin) {
  const float nominal_outC_120V = 12;
  float throttle;
  float outC;
#ifdef trim120current  
  throttle=analogRead(analogPin)/1024.;
  outC=throttle*nominal_outC_120V*2; // full range is 2x of nominal
  return outC;
#else
  return 15;
#endif
}

