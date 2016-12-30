#ifdef USE_USB

#include <PS3USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
PS3USB PS3(&Usb);

bool USB_inited = false;

void USB_init() {
  printDebug("Init USB host... ");
  if(Usb.Init() == -1) printlnDebug("OSC did not start!");
  else {
    USB_inited = true;
    printlnDebug("OK!");
  }
}

byte usbLedMode = 0;

void USB_loop() {
  if(USB_inited) {
    Usb.Task();
    if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
      if (PS3.getButtonClick(PS)) {
        Serial.print(F("\r\nPS"));
        if(++usbLedMode>3) usbLedMode = 0;
        setUSBled(usbLedMode);
      }
    }
  }
}

void setUSBled(byte _mode) {
  switch(_mode) {
    case 0 : PS3.setLedOn(LED1); break;
    case 1 : PS3.setLedOn(LED2); break;
    case 2 : PS3.setLedOn(LED3); break;
    case 3 : PS3.setLedOn(LED4); break;
    default : break;
  }
}

#endif
