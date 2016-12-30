
unsigned int scan_period_cnt = 0;
unsigned int scan_prev_millis = 0;

//========================== СОХРАНЕНИЕ НАСТРОЕК WiFi ==========================//
void save_config() {
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  JsonObject &j_tcp = root.createNestedObject("TCP");
  j_tcp["MAX_CLIENTS"] = 1;
  JsonObject &j_wifi = root.createNestedObject("wifi");
  j_wifi["ap_ssid"] = "BeerBoxESP";
  j_wifi["ap_pass"] = "1234567890";
  j_wifi["ap_hidden"] = false;
  JsonArray &j_wifis = j_wifi.createNestedArray("Saved");
  for(int i=0; i<wifis.size(); i++) {
    if(wifis[i].saved) {
      JsonObject &wifi = j_wifis.createNestedObject();
      wifi["ssid"] = wifis[i].ssid;
      wifi["pass"] = wifis[i].pass;
    }
  }
  File file = SPIFFS.open(config_path, "w");
  root.prettyPrintTo(file);
  file.close();
}

//========================== ЧТЕНИЕ НАСТРОЕК WiFi ==========================//
// read /wifi_config.txt
void read_config() {
  if(!SPIFFS.exists(config_path) ) {
    printlnDebug("File '/wifi_config.txt' not exist. Create new.");
    save_config();
    return;
  }
  printlnDebug("Read config from file '/wifi_config.txt'");
  File file = SPIFFS.open(config_path, "r");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(file.readString());
  file.close();  
  if(!json.success()) {
    printlnDebug("Parse '/wifi_config.txt' ERROR. Create new.");
    save_config();
    return;
  }
  JsonArray& json_wifis = json["wifi"]["Saved"];
  for(JsonObject& wifi: json_wifis) {
    WiFiItem item;
    item.saved = true;
    item.ssid = wifi["ssid"].as<const char*>();
    item.pass = wifi["pass"].as<const char*>();
    item.secure = item.pass.length();
    wifis.push_back(item);
  }
}

//========================== СОБЫТИЯ WiFi ==========================//
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case WIFI_EVENT_STAMODE_GOT_IP:
          WiFi_connected();
        break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
          isSavedSSID = false;
          printlnDebug("WiFi lost connection");
        break;
  }
}

void WiFi_connected() {
  isSavedSSID = true;
  printDebug("WiFi connected to: ");
  printlnDebug(WiFi.SSID());
  printDebug("IP-address: ");
  printlnDebug(WiFi.localIP());
  Telnet_init();
  AVRISP_init();
  OTA_init();
}

void WiFi_loop() {
  #ifndef DISABLE_WIFI
  if(!isSavedSSID) {
    if(millis() - scan_prev_millis >= 1000) {
      if(scan_period_cnt<scan_period) scan_period_cnt++;
      else {
        scan_period_cnt = 0;
        WiFi_scan();
      }
      scan_prev_millis = millis();
    }
  } else {
    scan_period_cnt = 0;
    scan_prev_millis = millis();
  }
  #endif
}

//========================== ИНИЦИАЛИЗАЦИЯ WIFI ==========================//
void WiFi_init(void) {
  #ifdef DISABLE_WIFI
  WiFi.mode(WIFI_OFF);
  #else
  wifi_showAP(0);
  WiFi.hostname(ESP_name);
  WiFi.macAddress(mac);
  //wifiMulti.addAP("NPMGroup-Guest");
  //wifiMulti.addAP("LAN_Prodmash", "admin@local");
  if(wifis.size()) {
    printlnDebug("Connecting multi...");
    for (WiFiItem& wifi: wifis) {
      if(wifi.saved) {
        wifiMulti.addAP(wifi.ssid.c_str(), wifi.pass.c_str());
      }
    }
    WiFi_scan();
    scan_prev_millis = millis();
    wifiMulti.run();
  } else {
    printlnDebug("No saved SSID's");
  }
  WiFi.onEvent(WiFiEvent);
  #endif
}

//========================== ПОДКЛЮЧЕНИЕ К WiFi СЕТИ ==========================//
void WiFi_connect(String ssid, String pass) {
  WiFi.disconnect();
  wifiMulti.addAP(ssid.c_str(), pass.c_str());
  wifi_begin_timeout = millis();
  WiFi.begin(ssid.c_str(), pass.c_str());
}

//========================== ОТКЛЮЧЕНИЕ ОТ WiFi СЕТИ ==========================//
bool wifi_disconnect() {
  byte i = 0;
  WiFi.disconnect();
  while(WiFi.status() == WL_CONNECTED) {
    if(i++ > 10) {
      printlnDebug(" -> disconnect error!");
      return false;
    } delay(1000);
  } return true;
}

//========================== СКАНИРОВАНИЕ WiFi СЕТИ ==========================//
void WiFi_scan(void) {
  wifis.erase( std::remove_if(wifis.begin(), wifis.end(), [](WiFiItem& item) -> bool {
    if (item.saved) item.rssi = 0;
    return !item.saved;
  }), wifis.end() );
  printDebug("Scan networks... ");
  int n = WiFi.scanNetworks();
  printlnDebug("Done!");
  if(n == 0) {
    printlnDebug(" no networks found");
  } else {
    printDebug(n);
    printlnDebug(" networks found:");
    // пробегаем по каждой найденной сети
    for(int i = 0; i < n; ++i) {
      // получаем SSID
      String ssid = WiFi.SSID(i);
      // проверяем наличие сети в списке
      auto it = std::find_if(wifis.begin(), wifis.end(), [&ssid](const WiFiItem& item) -> bool {
        return item.ssid.equals(ssid);
      });
      // если сети нет в списке - добавляем
      if(it == wifis.end()) {
        WiFiItem item;
        item.saved  = false;
        item.ssid   = ssid;
        item.secure = WiFi.encryptionType(i) != ENC_TYPE_NONE;
        item.rssi   = WiFi.RSSI(i);
        wifis.push_back(item);
      }
      // иначе если SSID совпадает то меняем уровень сигнала
      else {
        if(it->saved) isSavedSSID = true;
        if (it->rssi > WiFi.RSSI(i)) {
          it->rssi = WiFi.RSSI(i);
        }
      }
    }
    for(int i=0; i<wifis.size(); i++) {
      if(wifis[i].rssi) {
        printDebug(i);
        printDebug(". SSID: ");
        printDebug(wifis[i].ssid);
        printDebug(", RSSI: ");
        printDebug(wifis[i].rssi);
        printlnDebug(wifis[i].secure?"*":"");
      }
    }
  }
}

//================= АКТИВИРОВАТЬ ТОЧКУ ДОСТУПА НА ЗАДАННОЕ ВРЕМЯ =================//
bool wifi_showAP(unsigned int _timeout) {
  if(!wifi_disconnect()) return false;
  AP_ssid += "_";
  AP_ssid += ESP.getChipId();
  //if(OP_MODE_NAMES[wifi_get_opmode()]!="STATIONAP_MODE") {
    WiFi.mode(WIFI_AP_STA);
    delay(1000);
    printDebug("WiFi mode: ");
    printlnDebug(OP_MODE_NAMES[wifi_get_opmode()]);
    WiFi.softAP(AP_ssid.c_str(), AP_password, 1, false); // SSID, PASS, channel, ssid_hidden
    //WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    AP_IP = WiFi.softAPIP();
    printlnDebug("WiFi init as AP_STA mode!");
    printDebug("AP_SSID: ");
    printlnDebug(AP_ssid);
    printDebug("AP_PASS: ");
    printlnDebug(AP_password);
    printDebug("AP_IP-address: ");
    printlnDebug(AP_IP);
  //} else printlnDebug("AP_STA mode already configured!");
  return true;
}

//======================== ПОДКЛЮЧЕНИЕ К WiFi ПО WPS =========================//
void wifi_wps(void) {
  if(startWPSPBC()) printlnDebug("WPS sucsses!");
  else {
    printlnDebug("WPS failed! Reboot ESP...");
    ESP.restart();
  }
}
//----------------------------------------------------------------------------//
bool startWPSPBC() {
  printlnDebug("Start WPS configurating!");
  // WPS works in STA (Station mode) only -> not working in WIFI_AP_STA !!!
  if(!wifi_disconnect()) return false;
  printlnDebug("Go to WIFI_STA mode and try connect...");
  WiFi.mode(WIFI_STA);
  delay(1000);
  printDebug("WiFi mode: ");
  printlnDebug(OP_MODE_NAMES[wifi_get_opmode()]);
  WiFi.begin("foobar",""); // make a failed connection
  byte i = 0;
  while(WiFi.status() == WL_DISCONNECTED) {
    i++;
    if(i>10) {
      printlnDebug(" -> connect error!");
      return false;
    }
    delay(1000);
  }
  bool wpsSuccess = WiFi.beginWPSConfig();
  printDebug("Begin WPS config: ");
  printlnDebug(wpsSuccess);
  if(wpsSuccess) {
      // Well this means not always success :-/ in case of a timeout we have an empty ssid
      String newSSID = WiFi.SSID();
      if(newSSID.length() > 0) {
        // WPSConfig has already connected in STA mode successfully to the new station.
        printfDebug("WPS finished. Connected successfull to SSID '%s' \r\n", newSSID.c_str());
        //qConfig.wifiSSID = newSSID;
        //qConfig.wifiPWD = WiFi.psk();
      } else wpsSuccess = false;
  }
  return wpsSuccess; 
}
