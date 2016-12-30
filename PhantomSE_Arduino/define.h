//================================================================================//
String      ESP_name       = "PhantomSE";
//------------------------------------------------
#define BTN_PIN             16
#define LED_PIN             2
#define SOUND_PIN           15
//================================================================================//
//#define DBG_OUTPUT_PORT Serial
//------------------------------------------------
#ifdef DBG_OUTPUT_PORT
  #define DBG_PORT_BAUD       115200
  #define SERIAL_CMD_DEBUG    "[DEBUG] "
#endif
//------------------------------------------------
#ifdef DBG_OUTPUT_PORT
  bool new_line = true;
  bool debug = true;
  bool debug_last_state = debug;
  bool printDebugTime = false;
  void setTimeDebug(bool state) {
    printDebugTime = state;
  }
  template<class... T>
  void printDebug(T... args) {
    if(debug) {
      if(new_line) {
        new_line = false;
        DBG_OUTPUT_PORT.print(SERIAL_CMD_DEBUG);
      }
      DBG_OUTPUT_PORT.print(args...);
    }
  }
  template<class... T>
  void printlnDebug(T... args) {
    if(debug) {
      if(new_line) {
        DBG_OUTPUT_PORT.print(SERIAL_CMD_DEBUG);
      }
      new_line = true;
      DBG_OUTPUT_PORT.println(args...);
    }
  }
  template<class... T>
  void writeDebug(T... args) {
    if(debug) {
      DBG_OUTPUT_PORT.write(args...);
    }
  }
  void setDebug(bool state) {
    if(state != debug_last_state) {
      debug = state;
      debug_last_state = debug;
      if(!debug) {
        DBG_OUTPUT_PORT.println("[DEBUG OFF]");
      } else {
        DBG_OUTPUT_PORT.println("[DEBUG ON]");
      }
    }
  }
  //--------------- ИНИЦИАЛИЗАЦИЯ ПОРТА ДЛЯ ОТЛАДКИ ---------------//
  void beginDebug() {
    DBG_OUTPUT_PORT.begin(DBG_PORT_BAUD);
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
  #define writeDebug(...)
  #define serialEvent(...)
  #define setDebug(...)
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

