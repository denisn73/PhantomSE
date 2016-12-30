
void save_config();
void read_config();
bool wifi_showAP(unsigned int);
void wifi_wps(void);
void SPIFFS_init();
void serialEvent();
void WebServer_init();
void WiFi_init(void);
void WiFi_scan(void);
void user_setup();
void user_loop();
void FIRMWARE_BACKUP();
void WiFi_user_init();
void WiFi_user_loop();
String formatBytes(size_t);
void read_AP(String path);
void handleFileList();
void handleFileUpload();
void handleFileDelete();
void handleFileCreate();
bool handleFileRead(String path);
String getContentType(String filename);
bool FIRMWARE_UPDATE(String host, int port, String path);
void SPIFFS_UPDATE(void);
void checkUpdates(String host, int port);
void handleUpdate(int);
void handleUpdatePOST();
void handleUpdateFsPOST();

bool checkSignature(bool _stop = true);
bool eraseFlashContents();
bool writeFlashContents(String path);
bool verifyFlashContents(String path);
void showProcessors();
void chooseProcessor(byte numSig);

void sleep(unsigned long sleepTimeS);
void printDateTimeDebug(void);

//================================================================================//
#define BTN_PIN             16
#define LED_PIN             2
#define SOUND_PIN           15
#define USE_RECOVER_PIN     BTN_PIN
#define RECOVER_BTN_TIME    10 // секунд
//================================================================================//
#define DBG_OUTPUT_PORT Serial
//------------------------------------------------
#ifdef DBG_OUTPUT_PORT
  #define DBG_PORT_BAUD       115200
  #define SERIAL_CMD_DEBUG    "[DEBUG] "
#endif
//================================================================================//
#define USE_PHOENIX_SE
//------------------------------------------------
#ifdef USE_PHOENIX_SE
  #define USER_PIN           2
#else
  #define Phoenix_setup(...)
  #define Phoenix_loop(...)
#endif
//================================================================================//
//#define USE_USB
//------------------------------------------------
#ifdef USE_USB
#else
  #define USB_init(...)
  #define USB_loop(...)
#endif
//================================================================================//
#define USE_AVRPROG
//------------------------------------------------
#ifdef USE_AVRPROG
  #define AVRISP_SPI_FREQ   300e3
  #define AVRISP_PTIME      10
  #define AVRISP_ATTEMPTS   50
  #define AVRISP_RESET_PIN  BTN_PIN
#else
  #define AVR_init(...)
  #define AVR_loop(...)
  #define AVRListAvailable(...)
  #define AVRISP_init(...)
#endif
//================================================================================//
#define USE_RTC // address 0x68 (104)
//------------------------------------------------
#ifdef USE_RTC
  #define UTC_zone7   0//25200
  unsigned int RTC_update_period = 0; // in seconds
  void RTC_RESET();
#else
  #define RTC_RESET()
  #define RTC_INIT()
  #define RTC_LOOP()
  #define UTC_to_RTC()
  #define UTC_to_RTC()
  #define RTC_to_UTC()
#endif
//================================================================================//
#if defined(USE_PHOENIX_SE)
String      ESP_name       = "PhantomSE";
#elif defined(USE_PHOENIX_COMMANDER)
String      ESP_name       = "PhantomCommander";
#else
String      ESP_name       = "ESP";
#endif
//const char* PhantomSE_Host    = "PhantomSE.phoenix";
//const int   PhantomSE_Port    = 8083;
String      AP_ssid        = "PhantomSE";
const char* AP_password    = "1234567890";
const char* STA_ssid       = "LAN_Prodmash";
const char* STA_password   = "admin@local";
String      host           = ESP_name;
String      FLASH_version  = "1";
String      SPIFFS_version = "1";

String backup_path    = "/backup/SmartWiFi-1.1.flash.bin";
String config_path    = "/config/wifi_config.txt";
String log_debug_path = "/log/debug.log";
String log_data_path  = "/log/beerbox.log";

bool isSavedSSID = false;
unsigned int scan_period = 300; // секунд
struct WiFiItem {
  String ssid;
  String pass;
  int    rssi   = 0;
  bool   saved  = false;
  bool   secure = false;
  bool   used   = false;
};
std::vector<WiFiItem> wifis;

struct updateItem {
  String path = "/firmware/";
  String name;
  String version;
  bool   isSpiffs = false;
  bool   isAVR    = false;
};
std::vector<updateItem> updates;

//================================================================================//
//================================================================================//
//================================================================================//
#ifdef DBG_OUTPUT_PORT
  bool new_line = true;
  bool debug = true;
  bool debug_last_state = debug;
  bool printDebugTime = false;
  void setTimeDebug(bool state) {
    printDebugTime = state;
  }
  void printTime(void) {
    #ifdef USE_RTC
      if(printDebugTime) printDateTimeDebug();
    #endif
  }
  template<class... T>
  void printDebug(T... args) {
    if(debug) {
      if(new_line) {
        new_line = false;
        DBG_OUTPUT_PORT.print(SERIAL_CMD_DEBUG);
        printTime();
      }
      DBG_OUTPUT_PORT.print(args...);
    }
  }
  template<class... T>
  void printlnDebug(T... args) {
    if(debug) {
      if(new_line) {
        DBG_OUTPUT_PORT.print(SERIAL_CMD_DEBUG);
        printTime();
      }
      new_line = true;
      DBG_OUTPUT_PORT.println(args...);
    }
  }
  template<class... T>
  void printfDebug(T... args) {
    if(debug) {
      if(new_line) {
        new_line = false;
        DBG_OUTPUT_PORT.print(SERIAL_CMD_DEBUG);
        printTime();
      }
      DBG_OUTPUT_PORT.printf(args...);
    }
  }
  template<class... T>
  void writeDebug(T... args) {
    if(debug) {
      DBG_OUTPUT_PORT.write(args...);
    }
  }
  template<class... T>
  void setInternelDebug(T... args) {
    if(debug) {
      DBG_OUTPUT_PORT.setDebugOutput(args...);
    }
  }
  void setDebug(bool state) {
    if(state != debug_last_state) {
      debug = state;
      debug_last_state = debug;
      if(!debug) {
        DBG_OUTPUT_PORT.setDebugOutput(false);
        DBG_OUTPUT_PORT.println("[DEBUG OFF]");
      } else {
        DBG_OUTPUT_PORT.println("[DEBUG ON]");
      }
    }
  }
  //--------------- ИНИЦИАЛИЗАЦИЯ ПОРТА ДЛЯ ОТЛАДКИ ---------------//
  void beginDebug() {
    DBG_OUTPUT_PORT.begin(DBG_PORT_BAUD);
    setInternelDebug(false);
  }
  String cmd_checkUpdates     = "checkUpdates";
  String cmd_getSign          = "getSign";
  String cmd_eraseFlash       = "eraseFlash";
  String cmd_writeFlash       = "writeFlash";
  String cmd_verifyFlash      = "verifyFlash";
  String cmd_showCommands     = "showCommands";
  String cmd_showProcessors   = "showProcessors";
  String cmd_chooseProcessors = "choose=";
  String cmd_sleep            = "sleep";
  String cmd_ping             = "ping";
  String cmd_debug            = "debug";
  String cmd_resetRTC         = "resetRTC";
  String cmd_showRTC          = "showRTC";
  //--------------------- СОБЫТИЯ SERIAL ПОРТА --------------------//
  void formatShowCommands(String cmd) {
    DBG_OUTPUT_PORT.print(" '");
    DBG_OUTPUT_PORT.print(cmd);
    DBG_OUTPUT_PORT.print("' - ");
  }
  void showCommands() {
    DBG_OUTPUT_PORT.println("[ Serial commands list ]");
    formatShowCommands(cmd_checkUpdates);     printlnDebug("check updates from server");
    formatShowCommands(cmd_showProcessors);   printlnDebug("AVR show supported processors");
    formatShowCommands(cmd_chooseProcessors); printlnDebug("AVR choose processor by index");
    formatShowCommands(cmd_getSign);          printlnDebug("AVR chip get signature");
    formatShowCommands(cmd_eraseFlash);       printlnDebug("AVR chip erase");
    formatShowCommands(cmd_writeFlash);       printlnDebug("AVR chip write");
    formatShowCommands(cmd_verifyFlash);      printlnDebug("AVR chip verify");
    formatShowCommands(cmd_resetRTC);         printlnDebug("Reset RTC timeDate");
    DBG_OUTPUT_PORT.println("[ end of list ]");
  }
  void changeDebug() {
    if(debug) setDebug(false);
    else setDebug(true);
  }
  String inputString = "";
  void serialEvent() {
    if(debug) {
      while(DBG_OUTPUT_PORT.available()) {
        char inChar = (char)DBG_OUTPUT_PORT.read();
        if(inChar != '\n') {
          if(inChar != '\r') inputString += inChar;
        } else {
          DBG_OUTPUT_PORT.print("[SERIAL] --> ");
          DBG_OUTPUT_PORT.println(inputString);
               if(inputString.equals(cmd_debug))          changeDebug();
          else if(inputString.equals(cmd_resetRTC))       RTC_RESET();
          else if(inputString.equals(cmd_showRTC))        printDateTimeDebug();
          else if(inputString.equals(cmd_sleep))          sleep(5);
          else if(inputString.equals(cmd_getSign))        checkSignature();
          else if(inputString.equals(cmd_eraseFlash))     eraseFlashContents();
          else if(inputString.equals(cmd_writeFlash))     writeFlashContents("/smart_main.hex");
          else if(inputString.equals(cmd_verifyFlash))    verifyFlashContents("/smart_main.hex");
          else if(inputString.equals(cmd_showCommands))   showCommands();
          else if(inputString.equals(cmd_showProcessors)) showProcessors();
          else {
            int index = inputString.indexOf(cmd_chooseProcessors);
            if(index != -1) {
              inputString = inputString.substring(cmd_chooseProcessors.length());
              chooseProcessor(inputString.toInt());
            }
            else DBG_OUTPUT_PORT.println("[SERIAL] >>> error input <<<");
          }
          inputString = "";
        }
        delay(1);
      }
    }
  }
#else
  #define beginDebug(...)
  #define printDebug(...)
  #define printlnDebug(...)
  #define printfDebug(...)
  #define writeDebug(...)
  #define printTime(...)
  #define setInternalDebug(...)
  #define setTimeDebug(...)
  #define serialEvent(...)
#endif
//================================================================================//

//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency) {
  #ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
  #else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
  #endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;
  
  pinMode(SOUND_PIN, OUTPUT); // Set the pinMode as OUTPUT
  digitalWrite(SOUND_PIN, LOW);
  
  pin_port             = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask             = digitalPinToBitMask(SOUND_PIN);
  toggle_count         = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    *pin_port ^= pin_mask; // toggle the pin
    delayMicroseconds(lusDelayPerHalfCycle); // delay a half cycle
  }
  *pin_port &= ~(pin_mask);  // keep pin low after stop
}

void MSound(byte cNotes, ...) {
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);
  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...) {};
#endif
