#include <Wire.h>
#include "RTClib.h"
#include "LowPower.h"
#define PCF8523_CONTROL_1 0x00
#define PCF8523_CONTROL_2 0x01
RTC_PCF8523 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

volatile boolean alarmFlagNeedsReset = false;
volatile
boolean lightsOn = false;
int onboardLEDState = LOW;
DateTime sunrise, sunset, startTime;
byte duration;

boolean isVerbose = false;

/*
 * Sunrise and sunset schedule
 * We will calculate our lighting schedule from this.
 */
const byte sunTimes[16][6] = {
  // month, day, rise hour, rise minute, set hour, set minute
  // always in standard time (no DST)
  {8, 11, 5, 18, 19, 15},
  {8, 12, 5, 19, 19, 14},
  {9, 26, 6, 3, 18, 2},
  {9, 27, 6, 4, 18, 0},
  {9, 28, 6, 5, 17, 58},
  {9, 29, 6, 6, 17, 57},
  {9, 30, 6, 7, 17, 55},
  {10, 1, 6, 8, 17, 53},
  {10, 2, 6, 9, 17, 52},
  {10, 3, 6, 10, 17, 50},
  {10, 4, 6, 11, 17, 48},
  {10, 5, 6, 12, 17, 47},
  {10, 6, 6, 13, 17, 45},
  {10, 7, 6, 14, 17, 44},
  {10, 8, 6, 15, 17, 42},
  {10, 9, 6, 16, 17, 42}
};

// Iteration variable we'll use to track where we are in the schedule
int i;

/* Variables to override normal schedule for testing purposes
 */
 /*
byte onDay = 11;
byte onHour = 21;  
byte onMinute = 16;
byte offDay = 11;
byte offHour = 21;
byte offMinute = 17;
*/

void setup() {
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

  printAlarmSetting();

  /*
  write_i2c_register(PCF8523_ADDRESS, 0x0A, bin2bcd(56));
  val = read_i2c_register(PCF8523_ADDRESS, 0x0A);
  Serial.print("Minute alarm register after write = ");
  Serial.println(bcd2bin(val));
  */

  /*
  val = 2;
  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire.write((byte)reg);
  Wire.write((byte)val);
  Wire.endTransmission();
  Serial.println("register set");

  Wire.beginTransmission(PCF8523_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)PCF8523_ADDRESS, (uint8_t)1);
  val = Wire.read();

  Serial.print("val = ");
  Serial.println(val);
  */
  /*
  mode >>= 3;
  mode &= 0x7;
  */

  /*
  Pcf8523SqwPinMode mode = rtc.readSqwPinMode();
  Serial.print("Sqw Pin Mode: ");
  Serial.println(mode);
  */

  /*
  rtc.writeSqwPinMode(PCF8523_OFF);
  mode = rtc.readSqwPinMode();
  Serial.print("Sqw Pin Mode: ");
  Serial.println(mode);
  */
  analogWrite(9, 0);  
  
  DateTime now = rtc.now();
  //uint32_t nowUnix = now.unixtime();
  //DateTime now = DateTime(2018, 10, 1, 0, 0, 0); // for testing
  for (i=0; i<220; i++){
    
    if (isVerbose) {
      Serial.print("setup loop iteration ");
      Serial.println(i);
    }
    setLightingTimes();
    showDate("now", now);
    Serial.print("now.unixtime() = ");
    Serial.println(now.unixtime());
    showDate("sunrise", sunrise);
    Serial.print("sunrise.unixtime() = ");
    Serial.println(sunrise.unixtime());                            
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

/* Setup code from algorithm-based version
  showDate("now in setup", now);
  nextStartTime = getNextStartTime(now);
  showDate("nextStartTime in setup", nextStartTime);
  setRTCAlarm(nextStartTime.day(), nextStartTime.hour(), nextStartTime.minute());
  //setRTCAlarm(onDay, onHour, onMinute); // for testing
*/


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
  Serial.println("in setLightingTimes()");
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
  /*
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
    printAlarmSetting();
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




DateTime getNextStartTime(DateTime now){
  /** Data based on civil twilight for State College, PA
   *  Source: http://aa.usno.navy.mil/cgi-bin/aa_rstablew.pl?ID=AA&year=2018&task=2&state=PA&place=State+College
   *  
   *  Requirement: 14 hours of daylight+civil twilight
   *  RTC should be set to standard time (i.e. no DST adjustment)
   *  First fall date for supplemental lighting (9/2): 1 minute of lighting
   *    beginning at 05:10EST
   *  Winter solstice (12/21): 223 minutes beginning at 03:18EST
   *  Last spring date (4/9): 1 minute beginning at 05:13EST
   *  Start time on 12/21 is 112 minutes earlier than on 9/2.
   *  Duration on 12/21 is 222 minutes longer than on 9/2.
   *  12/21 is 110 days after 9/2, and 4/9 is 109 days after 12/21.
   *  So we can just start a minute earlier and add two minutes to the duration
   *    each day, then do the same in reverse.
   *  Note: This logic only works at State College's latitude.
   */

  int solsticeYear;
  // First figure out which winter solstice should be our reference date.
  if (now.month() < 6) {
    solsticeYear = now.year() - 1;
  }
  else {
    solsticeYear = now.year();
  }
  Serial.print("solsticeYear = ");
  Serial.println(solsticeYear);
  DateTime winterSolstice = DateTime(solsticeYear, 12, 21, 3, 18, 0);

  // If current date is within 110 days of the winter solstice, then we're in lighting season.
  TimeSpan ts = winterSolstice - now;
  int daysDifference = ts.days();
  Serial.print("daysDifference = ");
  Serial.println(daysDifference);
  if ( abs(daysDifference) <= 110 ){
    duration = 223 - (2 * abs(daysDifference));
    Serial.print("duration = ");
    Serial.println(duration);
    DateTime timeOfDay = winterSolstice + TimeSpan(0, 0, abs(daysDifference), 0);
    showDate("timeOfDay", timeOfDay);
    DateTime newStartTime = DateTime(now.year(), now.month(), now.day(), timeOfDay.hour(), timeOfDay.minute(), 0);
    if (now.unixtime() > newStartTime.unixtime()){
      // This start time is already past. Find a start time based on midnight tomorrow.
      DateTime tomorrow = rtc.now() + TimeSpan(1, 0, 0, 0);
      return getNextStartTime(DateTime(tomorrow.year(), tomorrow.month(), tomorrow.day(), 0, 0, 0));
    }
    else{
      return DateTime(now.year(), now.month(), now.day(), timeOfDay.hour(), timeOfDay.minute(), 0);
    }
  } 
  
  else {
    // If current date is more than 110 days before/after the winter solstice, then no lighting is needed until 9/2.
    return DateTime(now.year(), 9, 2, 5, 10, 0);
  }
  

  
}








