
#ifdef USE_AVRPROG

#include <FS.h>
#include "AVRutils.h"
#include <ESP8266AVRISP.h>

const char*    avrHost   = "esp8266-avrisp";
const uint16_t port      = 328;
const uint8_t  reset_pin = AVRISP_RESET_PIN;

ESP8266AVRISP avrprog(port, reset_pin);

unsigned long prevAvrMillis;

signatureType choosenProcessor;

bool formatListstate;

byte readProcessorsIndex = 0;

byte AVRListReadIndex() { return readProcessorsIndex; }

String AVRListReadString() {
  signatureType sign;
  memcpy_P (&sign, &signatures[readProcessorsIndex], sizeof sign);
  readProcessorsIndex++;
  return String(sign.desc);
}
bool AVRListAvailable() {
  if(readProcessorsIndex < processorsCount) return true;
  readProcessorsIndex = 0;
  return false;
}

void showProcessors() {
  printlnDebug("[ Processors list ]");
  for(unsigned int j = 0; j < NUMITEMS(signatures); j++) {
    printDebug(j);
    printDebug(". ");
    signatureType sign;
    memcpy_P (&sign, &signatures[j], sizeof sign);
    printlnDebug(sign.desc);
  }
  printlnDebug("[ end of list ]");
}

void chooseProcessor(byte numSig) {
  if(numSig >= processorsCount) {
    printDebug("Processor with index ");
    printDebug(numSig);
    printlnDebug(" doesn't exist");
    return;
  }
  choosenProcessor = signatures[numSig];
  checkSignature();
  printDebug("Choosen processor: ");
  printlnDebug(choosenProcessor.desc);
  printDebug("Current processor: ");
  if(foundSig != -1) printlnDebug(currentSignature.desc);
  else printlnDebug("unknown");
}

bool isProcessor(byte numSig) {
  return (foundSig == numSig);
}

bool readHexFile(String path, const byte action) {
  
  unsigned int lineNumber     = 0;
  const int    maxLine        = 80;
  byte         max_line_print = 5;
  String       line           = "";
  extendedAddress             = 0;
  lowestAddress               = 0xFFFFFFFF;
  highestAddress              = 0;
  bytesWritten                = 0;
  progressBarCount            = 0;
  pagesize                    = currentSignature.pageSize;
  pagemask                    = ~(pagesize - 1);
  oldPage                     = NO_PAGE;
  errors                      = 0;
  
  printDebug("Open file ");
  printDebug(path);
  printDebug(" ");
  File file = SPIFFS.open(path, "r");
  printlnDebug(formatBytes(file.size()));

  switch (action) {
    case verifyFlash:  printlnDebug("[Verify...]"); break;
    case writeToFlash: printlnDebug("[Write...]");  break;
  }

  while(file.available()) {
    char line_buffer[maxLine], c = ' ';
    byte index = 0;
    line = "";
    while(c != '\n') {
      c = (char) file.read();
      if(c != '\n') {
        line_buffer[index] = c;
        line += c;
        index++;
      }
    }
    char send_buffer[index];
    for(byte i=0; i<index; i++) send_buffer[i] = line_buffer[i];
    lineNumber++;
    if(!processLine(send_buffer, index, action)) {
      printDebug("[ error at line ");
      printDebug(lineNumber);
      printlnDebug(" ] ");
      break;
    }
  }
  printlnDebug("[end]");
  file.close();

  if(!gotEndOfFile) {
    printlnDebug("Did not get 'end of file' record.");
    return false;
  }
  
  switch (action) {
    case verifyFlash:  {
      if(!errors) printlnDebug("Flash verifed. No errors.");
      else {
        printlnDebug("Flash verifed errors.");
        return false;
      }
    } break;
    case writeToFlash: {
      if(oldPage != NO_PAGE) commitPage(oldPage);
      printlnDebug("Flash written.");
    } break;
  }
  return true;  
}

bool checkHexFile(String path) {
  printDebug("--> Check file '");
  printDebug(path);
  if(!SPIFFS.exists(path)) {    
    printlnDebug("' not exist");
    return false;
  }
  if(!path.endsWith(".hex")) {
    printlnDebug("' has bad format (not .hex)");
    return false;
  }
  printlnDebug();
  return true;
}

bool checkSignature(bool _stop) {
  if(startProgramming()) {
    getSignature();
    if(foundSig == -1) printlnDebug("Don't have signature!");
  }
  if(foundSig != -1) {
    if(_stop) stopProgramming();
    return true;
  }
  stopProgramming();
  return false;
}

bool eraseFlashContents() {
  bool state = false;
  if(!checkSignature()) return state;
  if(startProgramming()) {
    eraseMemory();
    state = true;
  }
  stopProgramming();
  return state;  
}

bool writeFlashContents(String path) {
  bool state = false;
  if(!checkHexFile(path)) return state;
  if(!checkSignature()) return state;
  if(startProgramming()) {
    eraseMemory();
    readHexFile(path, writeToFlash);
    readHexFile(path, verifyFlash);
    state = true;
  }
  stopProgramming();
  return state;
}

bool verifyFlashContents(String path) {
  bool state = false;
  if(!checkHexFile(path)) return state;
  if(!checkSignature()) return state;
  if(startProgramming()) {
    readHexFile(path, verifyFlash);
    state = true;
  }
  stopProgramming();
  return state;
}

bool writeDefFuses(byte low, byte high, byte extented) {
  bool state = false;
  if(startProgramming()) {
    state = true;
    writeFuse(low, lowFuse);
    writeFuse(high, highFuse);
    writeFuse(extented, extFuse);
  }
  stopProgramming();
  return state;
}

void AVR_init() {
  // *** avrprog *** //
  AVRISP_init();
  // *** other *** //
  prevAvrMillis = millis();
}

void AVR_loop() {
  // *** AVRISP *** //
  AVRISP_loop();
  if(wasAVRprogramming() || avrprog.wasProgramming()) btn_deInit();
}

void AVRISP_init() {
  MDNS.begin(avrHost);
  MDNS.addService("avrisp", "tcp", port);
  avrprog.begin();
}

void AVRISP_loop() {
  static AVRISPState_t last_state = AVRISP_STATE_IDLE;
  AVRISPState_t new_state = avrprog.update();
  if (last_state != new_state) {
      switch (new_state) {
          case AVRISP_STATE_IDLE: {
              printlnDebug("[AVRISP] now idle");
              // Use the SPI bus for other purposes
              break;
          }
          case AVRISP_STATE_PENDING: {
              printlnDebug("[AVRISP] connection pending");
              // Clean up your other purposes and prepare for programming mode
              break;
          }
          case AVRISP_STATE_ACTIVE: {
              printlnDebug("[AVRISP] programming...");
              // Stand by for completion
              break;
          }
      }
      last_state = new_state;
  }
  // Serve the client
  if (last_state != AVRISP_STATE_IDLE) {
      avrprog.serve();
  }
}

void AVRflash(String path) {
  printlnDebug(" >>> AVRflash <<<");
  if(!SPIFFS.exists(path)) {
    printDebug("[ERROR] --> File '");
    printDebug(path);
    printlnDebug("' not exist");
    return;
  }
  if(!path.endsWith(".hex")) {
    printDebug("[ERROR] --> File '");
    printDebug(path);
    printlnDebug("' has bad format (not .hex)");
    return;
  }
  
  printDebug("Open file ");
  printDebug(path);
  printDebug(" ");
  File file = SPIFFS.open(path, "r");
  printlnDebug(formatBytes(file.size()));
  
  unsigned int lineNumber     = 0;
  const int    maxLine        = 80;
  byte         max_line_print = 5;
  String       line           = "";
  extendedAddress             = 0;
  lowestAddress               = 0xFFFFFFFF;
  highestAddress              = 0;
  bytesWritten                = 0;
  progressBarCount            = 0;
  pagesize                    = currentSignature.pageSize;
  pagemask                    = ~(pagesize - 1);
  oldPage                     = NO_PAGE;
  
  printlnDebug("[begin file]");  
  while(file.available()) {
    char line_buffer[maxLine];
    char c = ' ';
    byte index = 0;
    line = "";
    while(c != '\n') {
      c = (char) file.read();
      if(c != '\n') {
        line_buffer[index] = c;
        line += c;
        index++;
      }
    }
    char send_buffer[index];
    for(byte i=0; i<index; i++) send_buffer[i] = line_buffer[i];
    lineNumber++;
    if(max_line_print) {
      max_line_print--;
      printDebug("[");
      printDebug(lineNumber);
      printDebug("]");
      printlnDebug(line);
    }
    if(processLine(send_buffer, index, writeToFlash)) {
      printDebug("[ at line ");
      printDebug(lineNumber);
      printDebug(" ] ");
      printDebug("size: ");
      printlnDebug(index);
      break;
    }
  }
  printlnDebug("...");
  printDebug("[");
  printDebug(lineNumber);
  printDebug("]");
  printlnDebug(line);
  printlnDebug("[end file]");
  file.close();

  if(gotEndOfFile) {
    if(oldPage != NO_PAGE) commitPage(oldPage);
    printlnDebug("Flash written.");
  } else {
    printlnDebug("Did not get 'end of file' record.");
  }

}

//////////////////////////////////////////////////////////////////////////////////////////

#endif
