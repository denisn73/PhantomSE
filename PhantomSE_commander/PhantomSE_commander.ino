#define PROG_NAME   "PhantomSE_commander"

#define PhantomSE_commander_addr  0x30

#define RC_CHANNELS       8
#define RC_FAILTIME    1000

//===============================================//

#include <Wire.h>
#include <PPMIn.h>
#include <Timer1.h>
#include <PS3USB.h>
#include <PS4USB.h>
#include <XBOXUSB.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
PS3USB PS3(&Usb);
PS4USB PS4(&Usb);
XBOXUSB Xbox(&Usb);

bool USBinited = false;

bool PS3connected  = false;
bool PS4connected  = false;
bool XBOXconnected = false;
bool RCconnected   = false;

uint16_t  RC_channel[RC_CHANNELS];                      // output buffer for PPMIn
uint8_t   RC_workIn[PPMIN_WORK_SIZE(RC_CHANNELS)];      // we need to have a work buffer for the PPMIn class
rc::PPMIn RC_PPMIn(RC_channel, RC_workIn, RC_CHANNELS); //

// BYTE 7 (buttons_H)
#define BIT_PS        0
#define BIT_L1        1
#define BIT_LT        2
#define BIT_UP        3
#define BIT_RIGHT     4
#define BIT_DOWN      5
#define BIT_LEFT      6
#define BIT_SELECT    7

// BYTE 8 (buttons_L)
#define BIT_TYPE      0
#define BIT_R1        1
#define BIT_RT        2
#define BIT_TRIANGLE  3
#define BIT_CIRCLE    4
#define BIT_CROSS     5
#define BIT_SQUARE    6
#define BIT_START     7

// TYPE
#define TYPE_RC       1
#define TYPE_XBOX     2
#define TYPE_PS3      3
#define TYPE_PS4      4

struct joyStruct {
  // joystick values are -128 to 128
  signed char   rightHatX;    // vertical stick movement = forward speed
  signed char   rightHatY;    // horizontal stick movement = sideways or angular speed
  signed char   leftHatX;     // vertical stick movement = tilt    
  signed char   leftHatY;     // horizontal stick movement = pan (when we run out of pan, turn body?)
  signed char   analogL2;     // 
  signed char   analogR2;     // 
  unsigned char buttons_H;    // buttons are 0 or 1 (PRESSED), and bitmapped
  unsigned char buttons_L;    // Extended function set
  unsigned char external;     // Extended function set
  byte ledNum;
  byte ledMode;
  bool ledState;
  byte rcData[11];
} command;

void setup() {
  
  Serial.begin(115200);
  Serial.print("Boot ");
  Serial.print(PROG_NAME);
  Serial.println(":");

  Serial.print("Init I2C (");
  Serial.print("0x");
  if(PhantomSE_commander_addr<16) Serial.print("0");
  Serial.print(PhantomSE_commander_addr, HEX);
  Serial.print(")...");
  Wire.begin(PhantomSE_commander_addr);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.println("OK!");
  
  Serial.print("Init USB... ");
  if(Usb.Init() != -1) USBinited = true;
  if(USBinited) Serial.println("OK!");
  else Serial.println("Failed");
  
  Serial.print("Init RC_PPM... ");
  rc::Timer1::init();
  pinMode(8, INPUT);                // We use pin 8 as PPM input pin
  digitalWrite(8, LOW);
  PCMSK0 = (1 << PCINT0);           // allow pin change interrupts for PB0 (digital pin 8)
  PCICR = (1 << PCIE0);             // enable pin change interrupt 0
  RC_PPMIn.setTimeout(RC_FAILTIME); // set a timeout (default 500 milliseconds)
  RC_PPMIn.start();                 // start listening
  Serial.println("OK!");
  
  resetCommand();
  
}
void loop() {

  RC_handle();

  if(USBinited) USB_handle();
  
}

void setCommand() {
  if(PS3connected) {
    //--------------------------------------------------------------------//
    command.rightHatX = PS3.getAnalogHat(RightHatX);
    command.rightHatY = PS3.getAnalogHat(RightHatY);
    command.leftHatX  = PS3.getAnalogHat(LeftHatX);
    command.leftHatY  = PS3.getAnalogHat(LeftHatY);
    command.analogL2  = PS3.getAnalogButton(L2);
    command.analogR2  = PS3.getAnalogButton(R2);
    //--------------------------------------------------------------------//
    bitWrite(command.buttons_H, BIT_PS,       PS3.getButtonPress(PS));
    bitWrite(command.buttons_H, BIT_L1,       PS3.getButtonPress(L1));
    bitWrite(command.buttons_H, BIT_LT,       PS3.getButtonPress(L3));
    bitWrite(command.buttons_H, BIT_UP,       PS3.getButtonPress(UP));
    bitWrite(command.buttons_H, BIT_RIGHT,    PS3.getButtonPress(RIGHT));
    bitWrite(command.buttons_H, BIT_DOWN,     PS3.getButtonPress(DOWN));
    bitWrite(command.buttons_H, BIT_LEFT,     PS3.getButtonPress(LEFT));
    bitWrite(command.buttons_H, BIT_SELECT,   PS3.getButtonPress(SELECT));
    //--------------------------------------------------------------------//
    bitWrite(command.buttons_L, BIT_R1,       PS3.getButtonPress(R1));
    bitWrite(command.buttons_L, BIT_RT,       PS3.getButtonPress(R3));
    bitWrite(command.buttons_L, BIT_TRIANGLE, PS3.getButtonPress(TRIANGLE));
    bitWrite(command.buttons_L, BIT_CIRCLE,   PS3.getButtonPress(CIRCLE));
    bitWrite(command.buttons_L, BIT_CROSS,    PS3.getButtonPress(CROSS));
    bitWrite(command.buttons_L, BIT_SQUARE,   PS3.getButtonPress(SQUARE));
    bitWrite(command.buttons_L, BIT_START,    PS3.getButtonPress(START));
    //--------------------------------------------------------------------//
    command.external = TYPE_PS3;
    //--------------------------------------------------------------------//
  } else if(PS4connected) {
    command.external = TYPE_PS4;
  } else if(XBOXconnected) {
    command.external = TYPE_XBOX;
  } else if(RCconnected) {
    command.rightHatX  = map(RC_channel[0], 1000, 2000, -125, 125);
    command.rightHatY  = map(RC_channel[1], 1000, 2000, -125, 125);
    command.leftHatX   = map(RC_channel[2], 1000, 2000, -125, 125);
    command.leftHatY   = map(RC_channel[3], 1000, 2000, -125, 125);
    command.external = TYPE_RC;
  } else {
    resetCommand();
  }
}

void resetCommand() {
  command.rightHatX  = 0;
  command.rightHatY  = 0;
  command.leftHatX   = 0;
  command.leftHatY   = 0;
  command.analogL2   = 0;
  command.analogR2   = 0;
  command.buttons_H  = 0;
  command.buttons_L  = 0;
  command.external   = 0;
}

void setLed(byte ledNum, byte ledMode, bool ledState) {
  switch(ledNum) {
    case 1 : {
      if(PS3connected)  PS3.setLedOn(LED1);
      if(XBOXconnected) Xbox.setLedOn(LED1);
    } break;
    case 2 : {
      if(PS3connected)  PS3.setLedOn(LED2);
      if(XBOXconnected) Xbox.setLedOn(LED2);
    } break;
    case 3 : {
      if(PS3connected)  PS3.setLedOn(LED3);
      if(XBOXconnected) Xbox.setLedOn(LED3);
    } break;
    case 4 : {
      if(PS3connected)  PS3.setLedOn(LED4);
      if(XBOXconnected) Xbox.setLedOn(LED4);
    } break;
    default: break;
  }
}

// ***** Data Format ***** //
// BYTE 0  - start byte (0xFF)
// BYTE 1  - RightHatX  (signed)
// BYTE 2  - RightHatY  (signed)
// BYTE 3  - LeftHatX   (signed)
// BYTE 4  - LeftHatY   (signed)
// BYTE 5  - AnalogL2   (signed)
// BYTE 6  - AnalogR2   (signed)
// BYTE 7  - buttons_H  (button PS & Left side button mask)
// BYTE 8  - buttons_L  (Right side button mask)
// BYTE 9  - external   (controller type)
// BYTE 10 - checksum
//*************************//
void HexapodTranslateData(byte* data, size_t _size) {
  byte _value  = 0xFF;
  int checksum = 0;
  for(byte i=0; i<_size; i++) {
    switch(i) {
      case 1  : _value = command.rightHatX;  break;
      case 2  : _value = command.rightHatY;  break;
      case 3  : _value = command.leftHatX;   break;
      case 4  : _value = command.leftHatY;   break;
      case 5  : _value = command.analogL2;   break;
      case 6  : _value = command.analogR2;   break;
      case 7  : _value = command.buttons_H;  break;
      case 8  : _value = command.buttons_L;  break;
      case 9  : _value = command.external;   break;
      case 10 : _value = 0xFF-checksum;      break;
      default : break;
    }
    if(i) checksum += _value;
    data[i] = _value;
  }
}
