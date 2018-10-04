#include <Wire.h>
#include "RTClib.h"
#include "LowPower.h"

// Other files specific to this project
#include "Schedule.h"

#define PCF8523_CONTROL_1 0x00
#define PCF8523_CONTROL_2 0x01
RTC_PCF8523 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

volatile boolean alarmFlagNeedsReset = false;
boolean lightsOn = false;
int onboardLEDState = LOW;
DateTime sunrise, sunset, startTime;
boolean isVerbose = true;

// Iteration variable we'll use to track where we are in the schedule
int i;

void hardwareSetup(){
  pinMode(13, OUTPUT); // the on-board LED
  pinMode(9, OUTPUT); // the LED strip
  pinMode(1, INPUT_PULLUP); // external interrupt from the RTC

  onboardLEDState = HIGH;
  digitalWrite(13, onboardLEDState);
  analogWrite(9, 31);
  
  Serial.begin(57600);
  Serial.println("setup running");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  delay(10000);
  Serial.print("pin 1 = ");
  if (digitalRead(1) == LOW) {
    Serial.println("LOW");
  } else {
    Serial.println("HIGH");
  }
  
  resetAlarmFlag();
  Serial.print("pin 1 after write = ");
  if (digitalRead(1) == LOW) {
    Serial.println("LOW");
  } else {
    Serial.println("HIGH");
  }

  // Make sure the square wave output is off (because we want to use
  // that pin for an alarm interrupt).
  if (rtc.readSqwPinMode() != PCF8523_OFF) {
    Serial.println("Turning off square wave output.");
    rtc.writeSqwPinMode(PCF8523_OFF);
  } else {
    Serial.println("Square wave output already off.");
  }

  byte val;
  byte reg = 0x0;

  val = read_i2c_register(PCF8523_ADDRESS, reg);
  Serial.print("val = ");
  Serial.println(val);

  val = read_i2c_register(PCF8523_ADDRESS, PCF8523_CONTROL_1);
  Serial.print("Control_1 = ");
  Serial.println(val);

  write_i2c_register(PCF8523_ADDRESS, PCF8523_CONTROL_1, (1 << 1 | val));

  val = read_i2c_register(PCF8523_ADDRESS, PCF8523_CONTROL_1);
  Serial.print("Control_1, after write = ");
  Serial.println(val);

  val = read_i2c_register(PCF8523_ADDRESS, PCF8523_CONTROL_2);
  Serial.print("Control_2 = ");
  Serial.println(val);

  val = read_i2c_register(PCF8523_ADDRESS, 0x07);
  Serial.print("Weekday = ");
  Serial.println(val);

  //printAlarmSetting();
}

void setup() {

  hardwareSetup();

  // Set the RTC to a certain time for debugging.
  //rtc.adjust(DateTime(2018, 10, 4, 6, 38, 50));
  
  analogWrite(9, 0);  
  
  DateTime now = rtc.now();
  for (i=0; i<220; i++){
    
    if (isVerbose) {
      Serial.print("setup loop iteration ");
      Serial.println(i);
    }
    setLightingTimes();
    if (isVerbose){
      showDate("now", now);
      showDate("sunrise", sunrise);
    }
                            
    // Check whether this day's lighting period is already past.
    if (now.unixtime() > sunrise.unixtime()){
      if (isVerbose){
        Serial.print("Lighting period for ");
        Serial.print(sunTimes[i][0]);
        Serial.print("/");
        Serial.print(sunTimes[i][1]);
        Serial.println(" is already past.");
      }
      if (i<219){ continue; }
      else{
        Serial.println("ERROR: Finished iterating sunTimes. All times are past. Giving up.");
        showError();
      }
    }
  
    // Compare startTime to current time to see if the lights should be on now.
    if (now.unixtime() > startTime.unixtime()) {
      // We're starting up in the midst of a lighting period. Turn the lights on.
      lightsOn = true;
    }
    else{
      // This day's lights on time is still in the future. Set the alarm.
      setRTCAlarm(startTime.day(), startTime.hour(), startTime.minute());
    }
    break;
  }
}

void alarmISR() {
  alarmFlagNeedsReset = true;
}

uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }


void printAlarmSetting() {
  byte val = bcd2bin(read_i2c_register(PCF8523_ADDRESS, 0x0A));
  Serial.print("Minute alarm register = ");
  Serial.println(val);
  Serial.print("Minute alarm enable bit = ");
  Serial.println((val & B10000000) >> 7);

  val = bcd2bin(read_i2c_register(PCF8523_ADDRESS, 0x0B));
  Serial.print("Hour alarm register = ");
  Serial.println(val);
  Serial.print("Hour alarm enable bit = ");
  Serial.println((val & B10000000) >> 7);

  val = bcd2bin(read_i2c_register(PCF8523_ADDRESS, 0x0C));
  Serial.print("Day alarm register = ");
  Serial.println(val);
  Serial.print("Day alarm enable bit = ");
  Serial.println((val & B10000000) >> 7);

  val = bcd2bin(read_i2c_register(PCF8523_ADDRESS, 0x0D));
  Serial.print("Weekday alarm register = ");
  Serial.println(val);
  Serial.print("Weekday alarm enable bit = ");
  Serial.println((val & B10000000) >> 7);
}

void resetAlarmFlag() {
  write_i2c_register(PCF8523_ADDRESS, PCF8523_CONTROL_2, B01110000);
  alarmFlagNeedsReset = false;
}


/*
 * An infinite loop to fast-blink the onboard LED. Use for irretrievable failure states.
 */
void showError(){
  while (true){
    if (onboardLEDState == LOW) { onboardLEDState = HIGH; }
    else { onboardLEDState = LOW; }
    digitalWrite(13, onboardLEDState);
    delay(250);
  }
}

void showDate(const char* txt, const DateTime& dt) {
    Serial.print(txt);
    Serial.print(' ');
    Serial.print(dt.year(), DEC);
    Serial.print('/');
    Serial.print(dt.month(), DEC);
    Serial.print('/');
    Serial.print(dt.day(), DEC);
    Serial.print(' ');
    Serial.print(dt.hour(), DEC);
    Serial.print(':');
    Serial.print(dt.minute(), DEC);
    Serial.print(':');
    Serial.print(dt.second(), DEC);
    
    Serial.print(" = ");
    Serial.print(dt.unixtime());
    Serial.print("s / ");
    Serial.print(dt.unixtime() / 86400L);
    Serial.print("d since 1970");
    
    Serial.println();
}

void showTimeSpan(const char* txt, const TimeSpan& ts) {
    Serial.print(txt);
    Serial.print(" ");
    Serial.print(ts.days(), DEC);
    Serial.print(" days ");
    Serial.print(ts.hours(), DEC);
    Serial.print(" hours ");
    Serial.print(ts.minutes(), DEC);
    Serial.print(" minutes ");
    Serial.print(ts.seconds(), DEC);
    Serial.print(" seconds (");
    Serial.print(ts.totalseconds(), DEC);
    Serial.print(" total seconds)");
    Serial.println();
}
uint8_t read_i2c_register(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write((byte)reg);
  Wire.endTransmission();

  Wire.requestFrom(addr, (byte)1);
  return Wire.read();
}

void write_i2c_register(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write((byte)reg);
  Wire.write((byte)val);
  Wire.endTransmission();
}

void setRTCAlarm(uint8_t day, uint8_t hour, uint8_t minute){
  Serial.print("setting alarm to day = ");
  Serial.print(day);
  Serial.print(", hour = ");
  Serial.print(hour);
  Serial.print(", minute = ");
  Serial.println(minute);
  write_i2c_register(PCF8523_ADDRESS, 0x0C, bin2bcd(day));
  write_i2c_register(PCF8523_ADDRESS, 0x0B, bin2bcd(hour));
  write_i2c_register(PCF8523_ADDRESS, 0x0A, bin2bcd(minute));
}

void setLightingTimes(){
  int y;
  if (sunTimes[i][0] > 6){
      y = 2018;
    }
    else{
      y = 2019;
    }
  sunrise = DateTime(y, sunTimes[i][0], sunTimes[i][1],
                          sunTimes[i][2], sunTimes[i][3], 0);
  sunset = DateTime(y, sunTimes[i][0], sunTimes[i][1],
                         sunTimes[i][4], sunTimes[i][5], 0);
    
  // Calculate lighting duration and start time.
  TimeSpan daylightLength = sunset - sunrise;
  if (isVerbose){ showTimeSpan("daylightLength", daylightLength); }
  TimeSpan onDuration = TimeSpan(0, 14, 0, 0) - daylightLength;
  if (isVerbose){ showTimeSpan("onDuration", onDuration); }
  startTime = sunrise - onDuration;
  if (isVerbose){ showDate("startTime", startTime); }
}





void loop() {
  // If the light is already on from setup, finish the lighting session.
  if (lightsOn){
    turnLightsOn(); 
  }
  
  onboardLEDState = HIGH;
  digitalWrite(13, onboardLEDState);
  Serial.flush();
  attachInterrupt(digitalPinToInterrupt(1), alarmISR, FALLING);
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_OFF);

  /* For debugging (also comment out powerdown statement above)
  while (!alarmFlagNeedsReset){
    showDate("waiting for lights-on interrupt", rtc.now());
    delay(1000);
  }
  */
  
  detachInterrupt(digitalPinToInterrupt(1));
  onboardLEDState = LOW;
  digitalWrite(13, onboardLEDState);
  showDate("Wake from sleep at", rtc.now());

  /** This section will run when an interrupt from the RTC wakes the computer.
   *  
   *  Determine which situation we're in upon wakeup:
   *    1. Beginning of lighting window (now =~ start time & light is off)
   *    2. False start of lighting window (now !=~ start time, once a month during the summer)
   */
  
  TimeSpan ts = startTime - rtc.now();
  int daysDifference = ts.days();
  if (daysDifference > 1){
    // False start, no lighting needed
    if (isVerbose){ Serial.println("False start"); }
  }
  else{
    // Time to turn on the light
    resetAlarmFlag();
    lightsOn = true;
    turnLightsOn();
  }
   
} 


void turnLightsOn(){
  analogWrite(9, 31);
  if (isVerbose){ Serial.println("turnLightsOn()"); }

  // Set the RTC alarm so that it will tell us when to turn the light off.
  setRTCAlarm(sunrise.day(), sunrise.hour(), sunrise.minute());
  //setRTCAlarm(offDay, offHour, offMinute); // for testing
  if (isVerbose){
    Serial.println("Setting alarm to lights off time.");
    Serial.println("Beginning delay loop to wait for lights-off interrupt.");
  }
  attachInterrupt(digitalPinToInterrupt(1), alarmISR, FALLING);
  while (lightsOn){
    while (!alarmFlagNeedsReset){
      if (isVerbose){
        showDate("waiting for lights-off interrupt", rtc.now());
        showDate("alarm set to sunrise, ", sunrise);
      }
      if (onboardLEDState == LOW) {
        onboardLEDState = HIGH;
      } else {
        onboardLEDState = LOW;
      }
      digitalWrite(13, onboardLEDState);
      delay(1000);
    }  
    // TODO: Add a check here to make sure it's really time to turn the light off
    // i.e. handle spurious interrupts
    lightsOn = false;
  }
  

  detachInterrupt(digitalPinToInterrupt(1));
  // Turn the light off.    
  resetAlarmFlag();
  
  analogWrite(9, 0);
  if (isVerbose){ Serial.println("Turning light off."); }

  // Reset for next lighting period.
  i++;
  setLightingTimes();
  setRTCAlarm(startTime.day(), startTime.hour(), startTime.minute());
}








