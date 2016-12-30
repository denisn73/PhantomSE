
#include <Arduino.h>
#include <Wire.h>

int HexapodPort = 123;
WiFiServer hexapodServer(HexapodPort);
WiFiClient hexapodClient;

byte rcData[11]
int  index;     // -1 = waiting for new packet
int  checksum;

void Phoenix_setup() {
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void Phoenix_loop() {
  if(ReadMsgsTelnet()) {
    for(byte i=0; i<sizeof(rcData); i++) rcData[i] = vals[i];
  } else {
    
  }
}

void receiveEvent(int howMany) {
  byte buffer[howMany];
  for(int i=0; i<howMany; i++) buffer[i] = Wire.read();
}

void requestEvent() {
  Wire.write(rcData, sizeof(rcData));
}

bool HexapodTelnetInited = false;
byte vals[11];
int ReadMsgsTelnet() {
  byte ret = 0;
  if(WiFi.status() != WL_CONNECTED) return ret;
  if(!HexapodTelnetInited) {
    hexapodServer.begin();
    hexapodServer.setNoDelay(true);
    HexapodTelnetInited = true;
  }
  //check if there are any new clients
  if(hexapodServer.hasClient()) {
    //find free/disconnected spot
    for(byte i=0; i < 1; i++) {
      if(!hexapodClient || !hexapodClient.connected()) {
        if(hexapodClient) hexapodClient.stop();
        hexapodClient = hexapodServer.available();
        hexapodClient.setNoDelay(true);
        continue;
      }
    }
    // no free/disconnected spot so reject
    WiFiClient hexapodClient = hexapodServer.available();
    hexapodClient.stop();
  }
  //--------------------------------------------------------
  if(!hexapodClient || !hexapodClient.connected()) return ret;
  while(hexapodClient.available()) {
    byte in = hexapodClient.read();
    if(index == -1) {         // looking for new packet
      if(in == 0xff) {
        vals[0] = in;
        index = 1;
        checksum = 0;
      }
    }
    else if(index == 1) {
      vals[index] = in;
      if(vals[index] != 0xff) {
        checksum += (int) vals[index];
        index++;
      }
    } else if(index > 1) {
      vals[index] = in;
      checksum += (int) vals[index];
      index++;
      if(index == 11) { // packet complete
        if(checksum%256 != 255) {
          // packet error!
          index = -1;
          printlnDebug(" !!! checksum error !!!");
          return ret;
        } else {
          ret = 1;
        }
        index = -1;
        while(hexapodClient.read() != -1) BPS++;
        return ret;
      }
    }
  }
  return ret;
}
//==============================================================================

