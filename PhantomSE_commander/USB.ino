
void USB_handle() {
    
    Usb.Task();
    
    //--- PS3 --------------------------------------------
    if(PS3.PS3Connected || PS3.PS3NavigationConnected) {
      if(!PS3connected) {
        PS3connected = true;
        Serial.println("PS3 connected");
      }
      setCommand();
      HexapodTranslateData(command.rcData, sizeof(command.rcData));
    } else {
      if(PS3connected) {
        PS3connected = false;
        resetCommand();
        Serial.println("PS3 disconnected");
      }
    }
    //--- PS4 --------------------------------------------
    if(PS4.connected()) {
      if(!PS4connected) {
        PS4connected = true;
        Serial.println("PS4 connected");
      }
      setCommand();
    } else {
      if(PS4connected) {
        PS4connected = false;
        resetCommand();
        Serial.println("PS4 disconnected");
      }
    }
    //--- XBOX -------------------------------------------
    if(Xbox.Xbox360Connected) {
      if(!XBOXconnected) {
        XBOXconnected = true;
        Serial.println("XBOX connected");
      }
      setCommand();
    } else {
      if(XBOXconnected) {
        XBOXconnected = false;
        resetCommand();
        Serial.println("XBOX disconnected");
      }
    }
    //----------------------------------------------------
  
}
