
#include <Wire.h>
#include <PS3USB.h>
#include "define.h"
#include "Phoenix.h"

USB Usb;
PS3USB PS3(&Usb);
bool USBinited = false;

void setup(void) {

  MSound(3, 60, 2000, 80, 2250, 100, 2500);
  beginDebug();  
  Phoenix_setup();
  printlnDebug("Run programm");
  
}

//========================== ГЛАВНЫЙ ЦИКЛ ==========================//
void loop(void) {

  Phoenix_loop();
  if(USBinited) {
    Usb.Task();
    PS3_handle();
  }
  
}

void _blink(byte _pin, byte _cnt, unsigned int _delay) {
  for(byte i=0; i<_cnt; i++) {
    digitalWrite(_pin, HIGH);
    delay(_delay);
    digitalWrite(_pin, LOW);
    if(i<_cnt) delay(_delay);
  }
}

void PS3_handle() {
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if(PS3.getButtonClick(PS)){}
  }
}

