
#include <ESP8266httpUpdate.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

String beerbox_path_flash  = "/firmware/SmartWiFi-1.1.flash.bin";
String beerbox_path_spiffs = "/firmware/SmartWiFi-1.spiffs.bin";

/////////////////////////////////////////////////////////////////////////////////////////////
// --- ArduinoOTA uptater -----------------------------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////
byte lastProgress = 0;
unsigned int ota_progress_count = 1;
byte new_progress = 0;
void OTA_init() {  
  ArduinoOTA.setPort(8266);                      // Port defaults to 8266
  ArduinoOTA.setHostname("esp8266");                  // Hostname defaults to esp8266-[ChipID]
  //ArduinoOTA.setPassword((const char *)"admin"); // No authentication by default
  ArduinoOTA.onStart([]() { setDebug(true); printlnDebug("[OTA] update ESP Start"); });
  ArduinoOTA.onEnd([]()   { printlnDebug("[OTA] update ESP End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//    byte new_progress = (progress / (total / 100));
//    if(new_progress != lastProgress) {
//      lastProgress = new_progress;
//      printfDebug("[OTA] Progress: %u%%\r\n", new_progress);
//    }
    if(!new_progress) {
      printDebug("[OTA] progress: ");
      new_progress = 1;
    }
    printDebug("#");
    if( (ota_progress_count++ % 64 == 0) || (progress==total) ) {
      new_progress = (progress / (total / 100));
      printDebug(" [ ");
      printDebug(new_progress);
      printlnDebug("% ]");
      new_progress = 0;
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    printfDebug("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)         printlnDebug("[OTA] Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   printlnDebug("[OTA] Begin Failed");
    else if (error == OTA_CONNECT_ERROR) printlnDebug("[OTA] Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) printlnDebug("[OTA] Receive Failed");
    else if (error == OTA_END_ERROR)     printlnDebug("[OTA] End Failed");
  });
  ArduinoOTA.begin();
}

void OTA_loop() {
  if(WiFi.status()==WL_CONNECTED) {
    ArduinoOTA.handle();
    yield();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// --- Update Flash from selected file ----------------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////
// http://192.168.89.58/update?pass=admin&host=beerbox.npmgroup.ru&port=8083&path=/firmware/SmartWiFi-1.1.flash.bin
// /update?pass=admin&host="beerbox.npmgroup.ru"&port="8083"&path="/firwmare/1.2.bin"
// /update?pass=admin&backup;
void handleUpdateGET() {
  #ifdef USE_AVRPROG
  byte access = false;
  if(server.hasArg("pass")) { if( server.arg("pass") == "admin" ) access = true; }
  if(!access) { server.send(404, "text/plain", "Access denied!!!"); return; }
  if(server.hasArg("backup")) {
    server.send(404, "text/plain", "Module recovering...");
    FIRMWARE_BACKUP();
    return;
  }
  if(server.hasArg("host") && server.hasArg("port") && server.hasArg("path")) {    
    if(!FIRMWARE_UPDATE(server.arg("host"), server.arg("port"), server.arg("path"))) {
      server.send(404, "text/plain", "Firmware update failed!");
    } else server.send(404, "text/plain", "Firwmare updated!");
    return;
  }
  //checkUpdates(beerBoxHost, beerBoxPort);
  String backupPage = "";
  backupPage += "<!DOCTYPE html><html><head>";
  backupPage += "<meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\">";
  backupPage += "<title>ESP update firmware</title></head>";

  backupPage += "<script type='text/javascript'> function confirm() { alert('clicked'); }";
  backupPage += "</script>";

  backupPage += "<body> Update AVR:<br>";
  backupPage += "<form method='POST' action='/updateAVR' enctype='multipart/form-data'>";
  backupPage += "<select name='processor'>";
  while(AVRListAvailable()) {
    backupPage += "<option value='";
    backupPage += AVRListReadIndex();
    if(isProcessor(AVRListReadIndex())) backupPage += "' selected>";
    else backupPage += "'>";
    backupPage += AVRListReadString();
  }
  backupPage += "</select>";
  backupPage += "<input type='file' name='updateAVR'>";
  backupPage += "<input type='submit' onclick='confirm();' value='Update'>";
  backupPage += "</form><br>";

  backupPage += "<form method='GET' action='/updateAVRfuses' enctype='multipart/form-data'>";
  backupPage += "<input type='submit' onclick='confirm();' value='Set Default Fuses'>";
  backupPage += "</form><br>";
  
  backupPage += "Update FLASH:<br>";
  backupPage += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  backupPage += "<input type='file' name='update'>";
  backupPage += "<input type='submit' onclick='confirm();' value='Update'>";
  backupPage += "</form><br>";
  
  backupPage += "Update SPIFFS:<br>";
  backupPage += "<form method='POST' action='/updateFS' enctype='multipart/form-data'>";
  backupPage += "<input type='file' name='updateFS'>";
  backupPage += "<input type='submit' onclick='confirm();' value='Update'>";
  backupPage += "</form><br>";
  
  backupPage += "<form method='GET' action='/update' enctype='multipart/form-data'>";
  backupPage += "<input type='hidden' name='pass' value='admin'>";
  backupPage += "<input type='submit' onclick='confirm();' name='backup' value='Backup Flash'>";
  backupPage += "</form><br>";
  
  backupPage += "</body></html>";
  server.send(200, "text/html", backupPage.c_str());
  #endif
}

void checkUpdates(String host, int port) {
  printDebug("Check updates... ");
  String ver = "unknown";
  printDebug("Connect to ");
  printDebug(host);
  printDebug(":");
  printDebug(port);
  printDebug("... ");
  WiFiClient client;
  if(!client.connect(host.c_str(), port)) {
    printlnDebug("Failed connect to server!");
    return;
  }
  printlnDebug("OK!");
  String requast = "GET /firmware/ HTTP/1.1\r\n";
  requast += "Host: ";
  requast += host;
  requast += ":";
  requast += port;
  requast += "\r\n";
  requast += "x-ESP8266-version: ";
  requast += FLASH_version;
  requast += "\r\n";
  requast += "Connection: close\r\n\r\n";
  client.print( requast );
  int ver_count = 0;
  int timeout = 5;
  while(!client.available() && timeout) {
    printDebug(".");
    delay(1000);
    timeout--;
  }
  if(!client.available()) printlnDebug(" Server not answer!");
  else printlnDebug(" OK!");
  updates.erase( std::remove_if(updates.begin(), updates.end(), [](updateItem& item) -> bool {
    return true;
  }), updates.end() );
  while(client.available()) {
    String line = client.readStringUntil('\n');
    int index = line.indexOf(".bin");
    if(index == -1) index = line.indexOf(".hex");
    if(index != -1) {
      ver_count++;
      ver = line.substring(0, index+4);
      printDebug(ver_count);
      printDebug(". ");
      printDebug(ver);
      int nameIndex = line.indexOf("-");
      int isSpiffsInsex = line.indexOf(".spiffs");
      bool isSpiffs = false;
      bool isAVR = false;
      if(isSpiffsInsex != -1) isSpiffs = true;
      else {
        isSpiffsInsex = line.indexOf(".avr");
        if(isSpiffsInsex != -1) isAVR = true;
        else {
          isSpiffsInsex = line.indexOf(".flash");
        }
      }
      if(isSpiffsInsex != -1) {
        updateItem item;
        item.path    += ver;
        item.name     = line.substring(0, nameIndex);
        item.version  = line.substring(nameIndex+1, isSpiffsInsex);
        item.isSpiffs = isSpiffs;
        item.isAVR = isAVR;
        updates.push_back(item);
      } else {
        printDebug(" (error type)");
      }
      printlnDebug();        
    }
  }
}

File avrUploadFile;

void handleUpdateAVRfuses() {
  #ifdef USE_AVRPROG
  String msg = "Change AVR fuses ";
  if(writeDefFuses(0xFF, 0xD6, 0x05)) msg += "OK!";
  else msg += "Failed! You may try again at prev page.";
  server.send(200, "text/plain", msg);
  #endif
}

bool updateAVR_OK = 0;

void handleUpdateAVRend() {
  String msg = "Update AVR ";
  if(updateAVR_OK) {
     updateAVR_OK = false ;
     msg += "completed!";
  } else msg += "Failed! Try again at prev page";
  server.send(200, "text/plain", msg);
}

void handleUpdateAVR() {
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    printfDebug("handleFileUpload: '%s'\r\n", filename.c_str());
    String isType = filename.substring(0, filename.indexOf(".hex"));
    if(isType.endsWith(".avr")) avrUploadFile = SPIFFS.open("/tmpAVR.hex", "w");
  } else if(upload.status == UPLOAD_FILE_WRITE) {
    if(SPIFFS.exists("/tmpAVR.hex")) {
      if(avrUploadFile) avrUploadFile.write(upload.buf, upload.currentSize);
    }
  } else if(upload.status == UPLOAD_FILE_END) {
    if(SPIFFS.exists("/tmpAVR.hex")) {
      if(avrUploadFile) avrUploadFile.close();
      if(writeFlashContents("/tmpAVR.hex")) updateAVR_OK = true;
      SPIFFS.remove("/tmpAVR.hex");
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
// --- Update FLASH or SPIFFS from selected file ------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////
extern "C" uint32_t _SPIFFS_start;
extern "C" uint32_t _SPIFFS_end;
void handleUpdatePOST()   { handleUpdate(U_FLASH);  }
void handleUpdateFsPOST() { handleUpdate(U_SPIFFS); }
void handleUpdate(int command) {  
  HTTPUpload& upload = server.upload();
  if(command != U_FLASH && command != U_SPIFFS) {
    if(upload.status == UPLOAD_FILE_START) {
      printlnDebug("Update command error.");
    }
    return;
  }
  if(upload.status == UPLOAD_FILE_START) {
    WiFiUDP::stopAll();
    printfDebug("Update: %s\n", upload.filename.c_str());
    size_t _size;
    if(command==U_FLASH) _size = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    else                 _size = ((size_t) &_SPIFFS_end - (size_t) &_SPIFFS_start);
    if(!Update.begin(_size, command)) { //start with max available size
      #ifdef DBG_OUTPUT_PORT
        Update.printError(DBG_OUTPUT_PORT);
      #endif
    }
  } else if(upload.status == UPLOAD_FILE_WRITE) {
    if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      #ifdef DBG_OUTPUT_PORT
        Update.printError(DBG_OUTPUT_PORT);
      #endif
    }
  } else if(upload.status == UPLOAD_FILE_END) {
    if(Update.end(true)){ //true to set the size to the current progress
      printfDebug("Update Success: %u\n", upload.totalSize);
      if(command==U_FLASH) printlnDebug("Rebooting...");
    } else {
      #ifdef DBG_OUTPUT_PORT
        Update.printError(DBG_OUTPUT_PORT);
      #endif
    }
  }
  yield();
}

/////////////////////////////////////////////////////////////////////////////////////////////
// --- Update FLASH from BeerBox.npmgroup.ru ----------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////
bool FIRMWARE_UPDATE(String host, String port, String path) {
  if(!path.startsWith("/") || !path.endsWith(".bin")) {
    printlnDebug("Firmware path error!");
    return false;
  }
  String firmware_path = "";
  if(!host.startsWith("http://") || !host.startsWith("https://")) firmware_path += "http://";
  firmware_path += host;
  firmware_path += ":";
  firmware_path += port;
  firmware_path += path;
  bool isSPIFFS = false;
  String isType = path.substring(0, path.indexOf(".bin"));
  if(isType.endsWith(".spiffs")) isSPIFFS = true;
  t_httpUpdate_return ret;
  String s;
  if(isSPIFFS) {
    s = "Start SPIFFS update: ";
    printDebug(s);
    printlnDebug(firmware_path);
    s += firmware_path;
    server.send(404, "text/plain", s);
    ret = ESPhttpUpdate.updateSpiffs(firmware_path, SPIFFS_version);
  } else {
    printDebug(s);
    String s = "Start FLASH update: ";
    printDebug(s);
    printlnDebug(firmware_path);
    s += firmware_path;
    server.send(404, "text/plain", s);
    ret = ESPhttpUpdate.update(firmware_path, FLASH_version);
  }
  switch(ret) {
    case HTTP_UPDATE_FAILED: {
        printfDebug("HTTP_UPDATE_FAILD Error (%d): %s\n",
                     ESPhttpUpdate.getLastError(),
                     ESPhttpUpdate.getLastErrorString().c_str());
    } break;
    case HTTP_UPDATE_NO_UPDATES: {
        printlnDebug("HTTP_UPDATE_NO_UPDATES");
    } break;
    case HTTP_UPDATE_OK: {
        printlnDebug("HTTP_UPDATE_OK");
        return true;
    } break;
    default: break;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//========================== ВОССТАНОВЛЕНИЕ ПРОШИВКИ ИЗ SPIFFS ============================//
/////////////////////////////////////////////////////////////////////////////////////////////
void FIRMWARE_BACKUP() {
  setInternelDebug(true);
  uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  if(!Update.begin(maxSketchSpace)){ // start with max available size
    #ifdef DBG_OUTPUT_PORT
      Update.printError(DBG_OUTPUT_PORT);
    #endif
    return;
  }
  if(!SPIFFS.exists(backup_path)) {
    printlnDebug("RECOVER failed: file not found");
    return;
  }
  printDebug("Open backup file for read... ");
  File file = SPIFFS.open(backup_path, "r");
  printDebug("OK! Size: ");
  size_t firmware_size = file.size();
  printlnDebug(firmware_size);
  printDebug("Start RECOVER... ");
  uint8_t data[100 + 1];
  while(file.available()) {
    firmware_size = file.read(data, 100);
    if(Update.write(data, firmware_size) != firmware_size) {
      file.close();
      printlnDebug(" Failed!");
      #ifdef DBG_OUTPUT_PORT
        Update.printError(DBG_OUTPUT_PORT); 
      #endif
      return;
    }
  }
  printlnDebug(" END!");
  file.close();
  if(Update.end(true)) { // true to set the size to the current progress
    printlnDebug("Recover Success! \nRebooting...\n");
    ESP.restart();
  } else {
    #ifdef DBG_OUTPUT_PORT
      Update.printError(DBG_OUTPUT_PORT);
    #endif
    return;
  }
  setInternelDebug(false);
}
// END ____________________________________________________________//

