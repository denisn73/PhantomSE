
#define SRV_CLIENTS   1
#define SRV_PORT     23

WiFiServer telnetServer(SRV_PORT);
WiFiClient telnetClients[SRV_CLIENTS];

void Telnet_init() {
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  printDebug("Telnet Ready! Use '");
  printDebug(WiFi.localIP());
  printlnDebug(": 23' to connect");
}

void Telnet_handle() {
  if(WiFi.status() == WL_CONNECTED) Telnet_loop();
}

void Telnet_loop() {
  uint8_t i;
  //check if there are any new clients
  if(telnetServer.hasClient()) {
    for(i=0; i < SRV_CLIENTS; i++) {
      //find free/disconnected spot
      if(!telnetClients[i] || !telnetClients[i].connected()) {
        if(telnetClients[i]) telnetClients[i].stop();
        telnetClients[i] = telnetServer.available();
        setDebug(false);
        continue;
      }
    }
    // no free/disconnected spot so reject
    WiFiClient telnetClient = telnetServer.available();
    telnetClient.stop();
  }
  // check clients for data
  for(i=0; i < SRV_CLIENTS; i++) {
    if(telnetClients[i] && telnetClients[i].connected()) {
      if(telnetClients[i].available()) {
        //get data from the telnet client and push it to the UART
        while(telnetClients[i].available()) Serial.write(telnetClients[i].read());
      }
    }
  }
  for(i=0; i < SRV_CLIENTS; i++) {
    if(telnetClients[i] && telnetClients[i].connected()) {
      size_t len = Serial.available();
      uint8_t sbuf[len];
      Serial.readBytes(sbuf, len);
      for(byte n = i; n < SRV_CLIENTS; n++) {
        if(telnetClients[n] && telnetClients[n].connected()) {
          telnetClients[n].write(sbuf, len);
          delay(1);
        }
      }
      continue;
    } else {
      
    }
  }
}

