// Need G4P library
import g4p_controls.*;
import processing.serial.*;

import processing.net.*; 


int BUT_R1 = 0x01;
int BUT_R2 = 0x02;
int BUT_R3 = 0x04;
int BUT_L4 = 0x08;
int BUT_L5 = 0x10;
int BUT_L6 = 0x20;
int BUT_RT = 0x40;
int BUT_LT = 0x80;

//Serial serial;
Client serial; 
String inString;
int lf = 10;

long lastMillis = 0;

int rcData[] = {0,0,0,0,0,0,0,0,0,0,0};

int leftHatX  = 125;
int leftHatY  = 125;
int rightHatX = 125;
int rightHatY = 125;
int analogL2  = 0;
int analogR2  = 0;
int analogV = 0;

public void setup() {
  size(460, 380, JAVA2D);
  createGUI();
  customGUI();
  // Place your setup code here
  //printArray(Serial.list());
  //String portName = Serial.list()[1];
  //serial = new Serial(this, portName, 38400);
  //serial.bufferUntil(lf);
  serial = new Client(this, "192.168.202.38", 123);
}

int btn_mask = 0;

public void draw() {
  background(230);
  if(millis() - lastMillis >= 50) {
    int values[] = {rightHatY, rightHatX, leftHatY, leftHatX, 0x00, 0x00, btn_mask, 0x00, 5};
    int checksum = 0;
    serial.write(0xFF);
    for(int i=0; i<9; i++) {
      checksum += values[i];
      serial.write(values[i]);
    }
    checksum = 0xFF-checksum;
    serial.write(checksum);
    for(int i=0; i<20000; i++) serial.write(0);
    btn_mask = 0;
    lastMillis = millis();
  }
}

// Use this method to add additional statements
// to customise the GUI controls
public void customGUI() {

}

void serialEvent(Serial p) {
  println(p.readString());
}