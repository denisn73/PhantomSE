
#ifdef USE_RTC


unsigned long rtcPrevMillis = 0;

void RTC_INIT() {
  printlnDebug("[RTC] Init DS1307 ");
  //--------RTC SETUP ------------
  Rtc.Begin();
  // if you are using ESP-01 then uncomment the line below to reset the pins to
  // the available pins for SDA, SCL
  Wire.begin(5, 4); // due to limited pins, use pin 0 and 2 for SDA, SCL
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDebug("[RTC] compiled TimeDate: ");
  printDateTime(compiled, true);
  if(!Rtc.IsDateTimeValid()) {
      printlnDebug("[RTC] lost confidence in the DateTime!");
      Rtc.SetDateTime(compiled);
  }
  if(!Rtc.GetIsRunning()) {
      printlnDebug("[RTC] was not actively running, starting now");
      Rtc.SetIsRunning(true);
  }
  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) {
      printlnDebug("[RTC] is older than compile time! (Updating DateTime)");
      //Rtc.SetDateTime(compiled);
      //now = compiled;
  } else if (now > compiled) {
      printlnDebug("[RTC] is newer than compile time. (this is expected)");
  } else if (now == compiled) {
      printlnDebug("[RTC] is the same as compile time! (not expected but all is fine)");
  }
  printDebug("[RTC] TimeDate: ");
  printDateTime(now, true);
}

void RTC_LOOP() {
  if(RTC_update_period) {
    if(millis()-rtcPrevMillis >= RTC_update_period*1000) {
      if(!Rtc.IsDateTimeValid()) printlnDebug("RTC lost confidence in the DateTime!");
      RtcDateTime now = Rtc.GetDateTime();
      printDebug("[RTC] Check TimeDate: ");
      printDateTime(now, true);
      rtcPrevMillis = millis();
    }
  }
}

void RTC_RESET() {
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  Rtc.SetDateTime(compiled);
  printDateTime(compiled, true);
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

String getDateTimeString() {
  RtcDateTime now = Rtc.GetDateTime();
  char datestring[20];
  snprintf_P( datestring, 
              countof(datestring),
              PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
              now.Month(),
              now.Day(),
              now.Year(),
              now.Hour(),
              now.Minute(),
              now.Second() );
  return String(datestring);
}

void printDateTime(const RtcDateTime& dt, bool new_line) {
  char datestring[20];
  snprintf_P( datestring, 
              countof(datestring),
              PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
              dt.Month(),
              dt.Day(),
              dt.Year(),
              dt.Hour(),
              dt.Minute(),
              dt.Second() );
  printDebug(datestring);
  if(new_line) printlnDebug();
}

void printDateTimeDebug(void) {
  RtcDateTime now = Rtc.GetDateTime();
  printDebug("[");
  printDateTime(now, false);
  printDebug("] ");
}

void RTC_updateUTC(uint32_t time) {
  RtcDateTime now = Rtc.GetDateTime();
  now.InitWithEpoch32Time(time);
  Rtc.SetDateTime(now);
}

void UTC_to_RTC(unsigned long _utc) {
  
}

unsigned long RTC_to_UTC() {
  unsigned long _utc = 0;
  return _utc;
}

#endif

