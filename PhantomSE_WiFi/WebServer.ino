
// holds the current upload
File fsUploadFile;

MDNSResponder mdns;
MDNSResponder mdns2;

// SERVER INIT
void WebServer_init() {

  MDNS.begin(host.c_str());
  printDebug("Open http://");
  printDebug(host);
  printlnDebug(".local/edit to see the file browser");
  printDebug("Open http://");
  printDebug(host);
  printlnDebug(".local/update to upload new firmware");
  server.onNotFound([](){
    if(!handleFileRead(server.uri())) server.send(404, "text/plain", "FileNotFound<br>Check File system or update it if necessary");
  });
  server.on("/auth", HTTP_GET, []() {
    if(!handleFileRead("/auth.htm")) server.send(404, "text/plain", "FileNotFound<br>Check File system or update it if necessary");
  });
  server.on("/wifi_scan.htm", HTTP_GET, []() {
    if(!handleFileRead("/wifi_scan.htm")) server.send(404, "text/plain", "FileNotFound<br>Check File system or update it if necessary");
  });
  server.on("/FSbrowser", HTTP_GET, []() {
    if(!handleFileRead("/web/fs.htm")) server.send(404, "text/plain", "FileNotFound<br>Check File system or update it if necessary");
  });
  server.on("/info", HTTP_GET, []() {
    if(!handleFileRead("/info.htm")) server.send(404, "text/plain", "FileNotFound<br>Check File system or update it if necessary");
  });
  server.on("/edit", HTTP_GET, [](){
    if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound<br>Check File system or update it if necessary");
  });
  server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);
  server.on("/edit", HTTP_PUT, handleFileCreate);    // create file
  server.on("/edit", HTTP_DELETE, handleFileDelete); // delete file
  server.on("/list", HTTP_GET, handleFileList);
  server.on("/getConfig", HTTP_GET, handleAllconfig);
  server.on("/wifi_scan", HTTP_GET, handleWiFiScan);
  server.on("/wifi_connect", HTTP_POST, handleWiFiConnect);
  server.on("/fs", HTTP_GET, handleFSbrowser);
  server.on("/all", HTTP_GET, [](){
    String json = "{";
    json += "\"heap\":"+String(ESP.getFreeHeap());
    json += ", \"analog\":"+String(analogRead(A0));
    json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
    json += "}";
    server.send(200, "text/json", json);
    json = String();
  });
  server.on("/update", HTTP_GET, handleUpdateGET);
  server.on("/updateFS", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
  }, handleUpdateFsPOST);
  server.on("/updateAVR", HTTP_POST, handleUpdateAVRend, handleUpdateAVR);
  server.on("/updateAVRfuses", HTTP_GET, handleUpdateAVRfuses);
  server.on("/update", HTTP_POST, []() { 
      server.sendHeader("Connection", "close");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
      ESP.restart();
  }, handleUpdatePOST);
  server.on("/phoenix", HTTP_GET, handlePhoenix);
  server.on("/phoenixData", HTTP_GET, handlePhoenixData);
  server.begin();
  printlnDebug("HTTP server started");
  MDNS.addService("http", "tcp", 80);
}

void handlePhoenix() {
  if(!handleFileRead("/phoenix.htm")) server.send(404, "text/plain", "FileNotFound<br>Check File system or update it if necessary");
}

void handlePhoenixData() {
  #ifdef USE_PHOENIX_SE
  const char* template_str = "[{\"ID\":%d,\"SSID\":\"%s\",\"IP\":\"%s\",\"USB\":\"%s\",\"TIME\":\"%s\","
                             "\"MODE\":%d,\"BALANCE\":%d,\"SPEEDCTRL\":%d,\"WALKMETHOD\":%d,"
                             "\"GAIT\":%d,\"LEGNUM\":%d,\"LEGHOLD\":%d,\"GPSEQ\":%d,\"BODYY\":%d%s}]";

  String poses;
  for(int i=0; i<20; i++) poses += ",\"POS"+String(i)+"\":" + String(ax12get(i));
  
  char json_buffer[512];
  sprintf(json_buffer, template_str,
    ESP.getChipId(), WiFi.SSID().c_str(), ipToString(WiFi.localIP()).c_str(), 
    command.PS3connected ? "PS3 connected" : "PS3 disconnected",
    getDateTimeString().c_str(), ControlMode, g_InControlState.BalanceMode,
    g_InControlState.SpeedControl, bJoystickWalkMode, g_InControlState.GaitType, 
    g_InControlState.SelectedLeg, g_InControlState.fSLHold, GPSeq, g_BodyYOffset, poses.c_str());
    
  server.send(200, "application/json", json_buffer);
  #else
  server.send(404, "text/html", "page not found");
  #endif
  
  /*
  DynamicJsonBuffer jsonArrayBuffer;
  JsonArray& json = jsonArrayBuffer.createArray();
  JsonObject &obj = json.createNestedObject();
  obj["ID"]    = ESP.getChipId();
  obj["SSID"]  = WiFi.SSID();
  obj["IP"]    = ipToString(WiFi.localIP());
  if(command.PS3connected) obj["USB"] = "PS3 connected";
  else obj["USB"] = "PS3 disconnected";
  obj["TIME"]       = getDateTimeString();
  
  obj["MODE"]       = ControlMode;
  obj["BALANCE"]    = g_InControlState.BalanceMode;
  obj["SPEEDCTRL"]  = g_InControlState.SpeedControl;  
  obj["WALKMETHOD"] = bJoystickWalkMode;
  obj["GAIT"]       = g_InControlState.GaitType;
  obj["LEGNUM"]     = g_InControlState.SelectedLeg;
  obj["LEGHOLD"]    = g_InControlState.fSLHold;
  obj["GPSEQ"]      = GPSeq;
  obj["BODYY"]      = g_BodyYOffset;
  
  for(int i=0; i<20; i++) obj["POS"+String(i)] = ax12get(i);
    
//  obj["UP"]        = (command.buttons & BIT_UP);
//  obj["RIGHT"]     = (command.buttons & BIT_RIGHT);
//  obj["DOWN"]      = (command.buttons & BIT_DOWN);
//  obj["LEFT"]      = (command.buttons & BIT_LEFT);
//  obj["LT"]        = (command.buttons & BIT_LT);
//  obj["L1"]        = (command.buttons & BIT_L1);
//  obj["L2"]        = command.analogL2;
//  obj["LeftHatX"]  = command.leftHatX;
//  obj["LeftHatY"]  = command.leftHatY;
//  
//  obj["TRIANGLE"]  = (command.buttons & BIT_TRIANGLE);
//  obj["CIRCLE"]    = (command.buttons & BIT_CIRCLE);
//  obj["CROSS"]     = (command.buttons & BIT_CROSS);
//  obj["SQUARE"]    = (command.buttons & BIT_SQUARE);
//  obj["RT"]        = (command.buttons & BIT_RT);
//  obj["R1"]        = (command.buttons & BIT_R1);
//  obj["R2"]        = command.analogR2;
//  obj["RightHatX"] = command.rightHatX;
//  obj["RightHatY"] = command.rightHatY;
//  
//  obj["SELECT"]    = (command.buttons & BIT_SELECT);
//  obj["START"]     = (command.buttons & BIT_START);
//  obj["PS"]        = (command.buttons & BIT_PS);
  
  String t;
  json.printTo(t);
  server.send(200, "application/json", t);*/
}

void handleAllconfig() {
  DynamicJsonBuffer jsonArrayBuffer;
  JsonArray& json = jsonArrayBuffer.createArray();
  JsonObject &obj = json.createNestedObject();
  obj["ESP_NAME"]  = ESP_name;
  String version  = FLASH_version;
         version += ".";
         version += SPIFFS_version;
  obj["ESP_VER"]   = version;
  obj["ESP_ID"]    = ESP.getChipId();
  String MAC = "";
         MAC += mac[5]; MAC += ":";
         MAC += mac[4]; MAC += ":";
         MAC += mac[3]; MAC += ":";
         MAC += mac[2]; MAC += ":";
         MAC += mac[1]; MAC += ":";
         MAC += mac[0];
  obj["ESP_MAC"]   = macToString(mac);
  obj["WI_MODE"]   = OP_MODE_NAMES[wifi_get_opmode()];
  obj["AP_SSID"]   = AP_ssid;
  obj["AP_PASS"]   = AP_password;
  obj["AP_IP"]     = ipToString(WiFi.softAPIP());
  obj["AP_HIDDEN"] = false;
  obj["AP_CONF"]   = false;
  obj["STA_SSID"]  = WiFi.SSID();
  obj["STA_IP"]    = ipToString(WiFi.localIP());
  String t;
  json.printTo(t);
  server.send(200, "application/json", t);
}

void handleFSbrowser() {
  if(!server.hasArg("dir")) {
    server.send(500, "text/plain", "BAD ARGS");
    #ifdef DBG_OUTPUT_PORT
      DBG_OUTPUT_PORT.println("fs: BAD ARGS");
    #endif
    return;
  }
  String path = server.arg("dir");
  unsigned int path_size = path.length(); 
  printDebug("fs: " + path);
  printDebug(" ");
  printDebug(path_size);
  Dir dir = SPIFFS.openDir(path);
  path = String();
  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    String _name = String(entry.name()).substring(path_size);
    printDebug("name: " + _name);
    int index = _name.indexOf('/');
    if(index != -1) {
      isDir = true;
      _name = _name.substring(0, index);
    }
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    //output += String(entry.name()).substring(1);
    output += _name;
    output += "\"}";
    entry.close();
  }  
  output += "]";
  server.send(200, "text/json", output);
}

//

//========================== ПРОВЕРКА АВТОРИЗАЦИИ ==========================//
bool is_authentified() {
  printlnDebug("Enter is_authentified");
  if(server.hasHeader("Cookie")) {
    String cookie = server.header("Cookie");
    printDebug("Found cookie: ");
    printlnDebug(cookie);
    if(cookie.indexOf("ESPSESSIONID=1") != -1) {
      //admin_flag = 1;
      printlnDebug("Authentificated as 'Admin'");
      return true;
    } else if(cookie.indexOf("ESPSESSIONID=2") != -1) {
      //admin_flag = 0;
      printlnDebug("Authentificated as 'User'");
      return true;
    }
  }
  printlnDebug("Authentification Failed");
  return false;
}

// format bytes
String formatBytes(size_t bytes) {
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)) {
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename) {
  if(server.hasArg("download"))            return "application/octet-stream";
  else if(filename.endsWith(".a"))         return "secure";
  else if(filename.endsWith(".hex"))       return "hex";
  else if(filename.endsWith(".log"))       return "log";
  else if(filename.endsWith("config.txt")) return "config";
  else if(filename.endsWith(".htm"))       return "text/html";
  else if(filename.endsWith(".html"))      return "text/html";
  else if(filename.endsWith(".css"))       return "text/css";
  else if(filename.endsWith(".js"))        return "application/javascript";
  else if(filename.endsWith(".png"))       return "image/png";
  else if(filename.endsWith(".gif"))       return "image/gif";
  else if(filename.endsWith(".jpg"))       return "image/jpeg";
  else if(filename.endsWith(".ico"))       return "image/x-icon";
  else if(filename.endsWith(".xml"))       return "text/xml";
  else if(filename.endsWith(".pdf"))       return "application/x-pdf";
  else if(filename.endsWith(".zip"))       return "application/x-zip";
  else if(filename.endsWith(".gz"))        return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path) {
  printDebug("handleFileRead: ");
  printDebug(path);
  printDebug("; deform: ");
  if(path.endsWith("/")) path += "index.html";
  if(path.startsWith("/web")) path = path.substring(4);
  String contentType = getContentType(path);
  if(contentType.equals("config")) {}
  else if(contentType.equals("secure")) {}
  else if(contentType.equals("log")) {}
  else if(contentType.equals("hex")) {}
  else if( path.endsWith("mem.html") || path.endsWith("mmio.html") || path.endsWith("polyfill.js") || path.endsWith("script.js") || path.endsWith("style.css") ){}
  else path = "/web" + path;
  printlnDebug(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if(SPIFFS.exists(pathWithGz)) path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  printlnDebug("handleFileRead: FileNotFound!<br>Check File system or update it if necessary");
  return false;
}

void handleFileUpload() {
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/" + filename;
    printfDebug("handleFileUpload: '%s'\r\n", filename.c_str());
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE) {
    //DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if(fsUploadFile) fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END) {
    if(fsUploadFile) fsUploadFile.close();
    printfDebug("handleFileUpload Size: '%d'r\n", upload.totalSize);
  }
}

void handleFileDelete() {
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  printfDebug("handleFileDelete: '%s'\r\n", path.c_str());
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate() {
  if(server.args() == 0) {
    printlnDebug("FileCreate: BAD ARGS");
    return server.send(500, "text/plain", "BAD ARGS");
  }
  String path = server.arg(0);
  printfDebug("handleFileCreate: '%s'\r\n", path.c_str());
  if(path == "/") return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path)) return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file) file.close();
  else return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!server.hasArg("dir")) {
    server.send(500, "text/plain", "BAD ARGS");
    printlnDebug("handleFileList: BAD ARGS");
    return;
  }
  String path = server.arg("dir");
  printfDebug("handleFileList: '%s'\r\n", path.c_str());
  Dir dir = SPIFFS.openDir(path);
  path = String();
  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }  
  output += "]";
  server.send(200, "text/json", output);
}

void handleWiFiScan() {
  WiFi_scan();
  DynamicJsonBuffer jsonArrayBuffer;
  JsonArray& json = jsonArrayBuffer.createArray();
  for(int i=0; i<wifis.size(); i++) {
    JsonObject &obj = json.createNestedObject();
    obj["ssid"]   = wifis[i].ssid;
    obj["rssi"]   = wifis[i].rssi;
    obj["secure"] = wifis[i].secure;
    obj["saved"]  = (wifis[i].saved)?true:false;
    obj["used"]   = (wifis[i].ssid.equals(WiFi.SSID()))?true:false;
  }
  String t;
  json.printTo(t);
  server.send(200, "application/json", t);
}

void handleWiFiConnect() {
  server.send(204);
  if(server.hasArg("delete")) {
    printDebug("Server has arg 'delete': ");
    printlnDebug(server.arg("delete"));
    if(server.hasArg("ssid")) {
      wifis.erase( std::remove_if(wifis.begin(), wifis.end(), [](WiFiItem& item) -> bool {
        if(item.ssid==server.arg("ssid")) {
          item.saved = false;
          printDebug("Change network to not saved: ");
          printlnDebug(item.ssid);
          return !item.saved;
        }
        return false;
      }), wifis.end() );
      save_config();
    }
  } else if(server.hasArg("ssid")) {
    printDebug("Server has arg 'ssid': ");
    printlnDebug(server.arg("ssid"));
    String ssid = server.arg("ssid");
    String pass = "";
    for(WiFiItem& wifi: wifis) {
      if(wifi.ssid == ssid) 
      pass = wifi.pass;
      break;
    }
    if(server.hasArg("pass")) {
      printDebug("Server has arg 'pass': ");
      printlnDebug(server.arg("pass"));
      pass = server.arg("pass");
    }
    if(server.hasArg("save")) {
      printDebug("Server has arg 'save': ");
      printlnDebug(server.arg("save"));
      if(server.arg("save")=="true") {
        for(WiFiItem& wifi: wifis) {
          if(wifi.ssid == ssid) {
            printDebug("Add ");
            printDebug(ssid);
            printlnDebug(" as saved to config file");
            wifi.saved = true;
            wifi.pass = pass;
            wifi.secure = wifi.pass.length();
            save_config();
            break;
          }
        }
      }
    }
    if(ssid != WiFi.SSID()) {
      printDebug("Disconnect from ");
      printlnDebug(WiFi.SSID());
      WiFi_connect(ssid, pass);
      printDebug("Try connect to \"");
      printDebug(ssid);
      printDebug("\" with pass \"");
      printDebug(pass);
      printlnDebug("\"");
    } else {
      printDebug(WiFi.SSID());
      printlnDebug(" already used.");
    }
  } else {
    printDebug("Server has ");
    printDebug(server.args());
    printlnDebug("args.");
  }
  yield();
}

