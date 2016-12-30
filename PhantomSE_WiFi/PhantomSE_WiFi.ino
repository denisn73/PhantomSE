
/*
  upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
  or you can upload the contents of a folder if you CD in that folder and run the following command:
  for file in `ls -A1`; do curl -F "file=@$PWD/$file" esp8266fs.local/edit; done
  
  access the sample web page at http://esp8266fs.local
  edit the page by going to http://esp8266fs.local/edit\
  C:\Users\silivanov_dv\AppData\Local\Temp\build7d2a7283f707ff5ebb6bee476cc9d70c.tmp
*/

#include <Wire.h>
#include <RtcDS1307.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFiMulti.h>
#include <FS.h>
#include <vector>
#include <algorithm>
#include <ArduinoJson.h>
#ifdef ESP8266
  extern "C" {
    #include "user_interface.h"
    #include "mem.h"
  }
#endif
#include "define.h"

#ifdef USE_RTC
RtcDS1307 Rtc;
#endif

ESP8266WiFiMulti wifiMulti;
ESP8266WebServer server(80);

IPAddress AP_IP;
byte mac[6];
unsigned long utc = 0;

const char * const OP_MODE_NAMES[] {
    "NULL_MODE",
    "STATION_MODE",
    "SOFTAP_MODE",
    "STATIONAP_MODE"
};

unsigned long wifi_begin_timeout;

//========================== ИНИЦИАЛИЗАЦИЯ МОДУЛЯ ==========================//
void setup(void) {

  //  *** WARNING ***  //
  // BE VERY CAREFULL! //
  MSound(3, 60, 2000, 80, 2250, 100, 2500);
  host += "_";
  host += String(ESP.getChipId(), HEX);
  beginDebug();
  printfDebug("\r\n\r\nBoot '%s' [v_%s.%s]\r\n", ESP_name.c_str(), FLASH_version.c_str(), SPIFFS_version.c_str());
  //setTimeDebug(true);
  RTC_INIT();
  USB_init();
  // Try pushing frequency to 160MHz.
  system_update_cpu_freq(SYS_CPU_160MHZ);
  btn_init(BTN_PIN);
  SPIFFS_init();
  //save_config();
  read_config();
  WebServer_init();
  WiFi_init();
  AVR_init();
  //*******************//
  
  Phoenix_setup();
  
  printlnDebug("Run programm");
  
}

bool connected_flag = false;
//========================== ГЛАВНЫЙ ЦИКЛ ==========================//
void loop(void) {
  yield();
  WiFi_loop();
  byte _mode = wifi_get_opmode();
  if(OP_MODE_NAMES[_mode]=="STATIONAP_MODE" || OP_MODE_NAMES[_mode]=="STATION_MODE") {
    if(!wifi_begin_timeout) {
      if(isSavedSSID) wifiMulti.run();
    } else {
      if(WiFi.status() == WL_CONNECTED) {
        wifi_begin_timeout = 0;
      } else {
        if(millis()-wifi_begin_timeout >= 10000) {
          wifi_begin_timeout = 0;
            printDebug("Failed connect to ");
            printlnDebug(WiFi.SSID());
        }
      }
    }
  }
  server.handleClient();
  btn_handle();
  serialEvent();
  
  if(!connected_flag) {
    if(WiFi.status()==WL_CONNECTED) {
      connected_flag = true;
      WiFi_connected();
    }
  } else {
    if(WiFi.status()!=WL_CONNECTED) {
      connected_flag = false;
    }
  }
  
  OTA_loop();
  RTC_LOOP();
  AVR_loop();
  Telnet_handle();
  USB_loop();
  
  // ==== USER CODE HERE ==== //
  
  Phoenix_loop();
  PhoenixCMD_loop();
  
  // === END OF USER CODE === //
}

//=============================== УХОД В СОН ===============================//
void sleep(unsigned long sleepTimeS) {
  printDebug("Go to deepSleep for ");
  printDebug(sleepTimeS);
  printlnDebug(" seconds");
  ESP.deepSleep(sleepTimeS * 1000000);
}

//========================== ИНИЦИАЛИЗАЦИЯ SPIFFS ==========================//
void SPIFFS_init() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  SPIFFS.begin();
  Dir dir = SPIFFS.openDir("/");
  while(dir.next()) {
    String fileName = dir.fileName();
    size_t fileSize = dir.fileSize();
    printfDebug("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
  }
  printlnDebug();
  #ifdef USE_RECOVER_PIN
    unsigned int recover_button_counter = RECOVER_BTN_TIME;
    pinMode(USE_RECOVER_PIN, INPUT);
    digitalWrite(USE_RECOVER_PIN, HIGH);
    while(!digitalRead(USE_RECOVER_PIN)) {
      if(recover_button_counter>0) {
        if(recover_button_counter==1) {
          digitalWrite(LED_PIN, HIGH);
        }
        recover_button_counter--;
        delay(1000);
      }
    }
    if(!recover_button_counter) {
      printlnDebug("Recover - Init proccess!");
      _blink(LED_PIN, 5, 200);
      digitalWrite(LED_PIN, HIGH);
      FIRMWARE_BACKUP();
    } else {
      printlnDebug("Recover - skiped.");
      _blink(LED_PIN, 2, 300);
    }
  #endif  
}
// END ____________________________________________________________//

void _blink(byte _pin, byte _cnt, unsigned int _delay) {
  for(byte i=0; i<_cnt; i++) {
    digitalWrite(_pin, HIGH);
    delay(_delay);
    digitalWrite(_pin, LOW);
    if(i<_cnt) delay(_delay);
  }
}

// print to DEBUG file ============================================//
void printLog(String data) {
  File file = SPIFFS.open(log_debug_path, "a"); 
  file.print(utc);
  file.print(": ");
  file.println(data);
}
// END ____________________________________________________________//

// convert IP to String ===========================================//
String ipToString(IPAddress ip){
  String s="";
  for (int i=0; i<4; i++) s += i  ? "." + String(ip[i]) : String(ip[i]);
  return s;
}
// END ____________________________________________________________//

// convert MAC to String ===========================================//
String macToString(byte *mac) {
  String s="";
  for(int i=0; i<6; i++) {
    if(mac[i]<16) s += "0";
    s += mac  ? ":" + String(mac[i], HEX) : String(mac[i], HEX);
  }
  return s;
}
// END ____________________________________________________________//

//struct UserData {
//  char type = 0;
//  String hash; // aasdas67asfasf5675fas67
//  int timeout = millis();
//}
//enum UserTypes {
//  User = 0,
//  Admin = 1,
//};
//UserData users[2] = { {User, ""}, {Admin, ""} };

