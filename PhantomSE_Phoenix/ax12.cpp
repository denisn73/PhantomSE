/*
  ax12.cpp - ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
//=============================================================================
// *** EDIT for usage simple servo *** //
//=============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ax12.h"

#define FREQ          120       // = 120 Hz
#define FREQ_CALIB    FREQ / 10
#define FREQ_VALUE    FREQ + FREQ_CALIB
// 250 = 500us
// 1250 = 2500us
const int SERVOMIN    =  FREQ * 2 + FREQ_CALIB;
const int SERVOMAX    =  SERVOMIN * 5 + FREQ_CALIB;
const int SERVORANGE  =  SERVOMAX - SERVOMIN;
// 0 degress   = 500us  = 0
// 180 degress = 2500us = 1000
#define POS_VALUE     SERVORANGE / 2
#define POS_SIZE      30
#define PWM_ADDR_16   0x41
#define PWM_ADDR_20   0x42

int position_buf[POS_SIZE];
int position_buf_last[POS_SIZE];

Adafruit_PWMServoDriver pwm16 = Adafruit_PWMServoDriver(PWM_ADDR_16);
Adafruit_PWMServoDriver pwm20 = Adafruit_PWMServoDriver(PWM_ADDR_20);

bool SERVO16_inited = false;
bool SERVO20_inited = false;

bool ax12InitServo(unsigned int freq) {
  for(int i=0; i<POS_SIZE; i++) {
    position_buf[i] = POS_VALUE;
    position_buf_last[i] = POS_VALUE;
  }
  int error;
  Wire.beginTransmission(PWM_ADDR_16);
  error = Wire.endTransmission();
  if(error==0) SERVO16_inited = true;
  Wire.beginTransmission(PWM_ADDR_20);
  error = Wire.endTransmission();
  if(error==0) SERVO20_inited = true;
  if(SERVO16_inited) {
    pwm16.begin();
    pwm16.setPWMFreq(FREQ_VALUE);
  }
  if(SERVO20_inited) {
    pwm20.begin(); 
    pwm20.setPWMFreq(FREQ_VALUE);
  }
  if(SERVO16_inited || SERVO20_inited) return true;
  return false;
}

bool ax12set(int id, int data) {
  if(id<POS_SIZE) {
    position_buf[id] = data;
    if(SERVO16_inited && id<16)       pwm16.setPin(id, data + SERVOMIN);
    else if(SERVO20_inited && id>=16) pwm20.setPin(id-16, data + SERVOMIN);
    if(data != position_buf_last[id]) {
      position_buf_last[id] = data;
      return true;
    }
  }
  return false;
}

int ax12get(int id) {
  if(id<POS_SIZE) return position_buf[id];
  return 0;
}

//void setTX(int id);
//void setRX(int id);
//void setTXall();
//void ax12write(unsigned char data);
//void ax12writeB(unsigned char data);
//void ax12Init(long baud);
//void ax12SetRegister(int id, int regstart, int data);
//void ax12SetRegister2(int id, int regstart, int data);
//int ax12GetLastError() {return 0;}
//int ax12ReadPacket(int length) {return -1;}
//int ax12GetRegister(int id, int regstart, int length) {return -1;}

/******************************************************************************
 * Hardware Serial Level, this uses the same stuff as Serial1, therefore 
 *  you should not use the Arduino Serial1 library.
 */

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_tx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_rx_int_buffer[AX12_BUFFER_SIZE];

// making these volatile keeps the compiler from optimizing loops of available()
volatile int ax_rx_Pointer;
volatile int ax_tx_Pointer;
volatile int ax_rx_int_Pointer;
#if defined(AX_RX_SWITCHED)
unsigned char dynamixel_bus_config[AX12_MAX_SERVOS];
#endif

/** helper functions to switch direction of comms */
void setTX(int id) {
    //Serial.print("setTX: ");
    //Serial.println(id);
//    bitClear(UCSR1B, RXEN1); 
//  #if defined(AX_RX_SWITCHED)
//    if(dynamixel_bus_config[id-1] > 0)
//        SET_RX_WR;
//    else
//        SET_AX_WR;   
//  #else
//    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
//    #ifdef ARBOTIX_WITH_RX
//      PORTD |= 0x10;
//    #endif   
//    bitSet(UCSR1B, TXEN1);
//    bitClear(UCSR1B, RXCIE1);
//  #endif
//    ax_tx_Pointer = 0;
}
void setRX(int id){ 
    //Serial.print("setRX: ");
    //Serial.println(id);
//  #if defined(AX_RX_SWITCHED)
//    int i;
//    // Need to wait for last byte to be sent before turning the bus around.
//    // Check the Transmit complete flag
//    while (bit_is_clear(UCSR1A, UDRE1));
//    for(i=0; i<UBRR1L*15; i++)    
//        asm("nop");
//    if(dynamixel_bus_config[id-1] > 0)
//        SET_RX_RD;
//    else
//        SET_AX_RD;
//  #else
//    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
//    #ifdef ARBOTIX_WITH_RX
//      int i;
//      // Need to wait for last byte to be sent before turning the bus around.
//      // Check the Transmit complete flag
//      while (bit_is_clear(UCSR1A, UDRE1));
//      for(i=0; i<25; i++)    
//          asm("nop");
//      PORTD &= 0xEF;
//    #endif 
//    bitClear(UCSR1B, TXEN1);
//    bitSet(UCSR1B, RXCIE1);
//  #endif  
//    bitSet(UCSR1B, RXEN1);
//    ax_rx_int_Pointer = 0;
//    ax_rx_Pointer = 0;
}
// for sync write
void setTXall(){
    //Serial.println("setTXall");
//    bitClear(UCSR1B, RXEN1);    
//  #if defined(AX_RX_SWITCHED)
//    SET_RX_WR;
//    SET_AX_WR;   
//  #else
//    #ifdef ARBOTIX_WITH_RX
//      PORTD |= 0x10;
//    #endif
//    bitSet(UCSR1B, TXEN1);
//    bitClear(UCSR1B, RXCIE1);
//  #endif
//    ax_tx_Pointer = 0;
}
/** Sends a character out the serial port. */
void ax12write(unsigned char data){
//    while (bit_is_clear(UCSR1A, UDRE1));
//    UDR1 = data;
}
/** Sends a character out the serial port, and puts it in the tx_buffer */
void ax12writeB(unsigned char data){
//    ax_tx_buffer[(ax_tx_Pointer++)] = data; 
//    while (bit_is_clear(UCSR1A, UDRE1));
//    UDR1 = data;
}
/** We have a one-way recieve buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */
#if !defined(ESP8266)
ISR(USART1_RX_vect){
    //ax_rx_int_buffer[(ax_rx_int_Pointer++)] = UDR1;
}
#endif

/** read back the error code for our latest packet read */
int ax12Error;
int ax12GetLastError(){ return ax12Error; }
/** > 0 = success */
int ax12ReadPacket(int length){
    //Serial.print("ax12ReadPacket size ");
    //Serial.println(length);
    unsigned long ulCounter;
    unsigned char offset, blength, checksum, timeout;
    unsigned char volatile bcount; 

    offset = 0;
    timeout = 0;
    bcount = 0;
    while(bcount < length){
        ulCounter = 0;
        while((bcount + offset) == ax_rx_int_Pointer){
            if(ulCounter++ > 1000L){ // was 3000
                timeout = 1;
                break;
            }
        }
        if(timeout) break;
        ax_rx_buffer[bcount] = ax_rx_int_buffer[bcount + offset];
        if((bcount == 0) && (ax_rx_buffer[0] != 0xff))
            offset++;
        else if((bcount == 2) && (ax_rx_buffer[2] == 0xff))
            offset++;
        else
            bcount++;
    }

    blength = bcount;
    checksum = 0;
    for(offset=2;offset<bcount;offset++)
        checksum += ax_rx_buffer[offset];
    if((checksum%256) != 255){
        return 0;
    }else{
        return 1;
    }
}


/** initializes serial1 transmit at baud, 8-N-1 */
void ax12Init(long baud) {
  //Serial.begin(38400);
    //Serial.println("ax12Init");
    //delay(1000);
//    UBRR1H = (F_CPU / (8 * baud) - 1 ) >> 8;
//    UBRR1L = (F_CPU / (8 * baud) - 1 );
//    bitSet(UCSR1A, U2X1);
//    ax_rx_int_Pointer = 0;
//    ax_rx_Pointer = 0;
//    ax_tx_Pointer = 0;
//#if defined(AX_RX_SWITCHED)
//    INIT_AX_RX;
//    bitSet(UCSR1B, TXEN1);
//    bitSet(UCSR1B, RXEN1);
//    bitSet(UCSR1B, RXCIE1);
//#else
//  #ifdef ARBOTIX_WITH_RX
//    DDRD |= 0x10;   // Servo B = output
//    PORTD &= 0xEF;  // Servo B low
//  #endif
//    // set RX as pull up to hold bus to a known level
//    PORTD |= (1<<2);
//    // enable rx
//    setRX(0);
//#endif
}

/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int ax12GetRegister(int id, int regstart, int length) {

    if(regstart==AX_PRESENT_POSITION_L) {
      return position_buf[id];
    } else if(regstart==AX_PRESENT_VOLTAGE) {
      int volt = 120;
      return volt;
    }  
    else {
      setTX(id);
      // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
      int checksum = ~((id + 6 + regstart + length)%256);
      ax12writeB(0xFF);
      ax12writeB(0xFF);
      ax12writeB(id);
      ax12writeB(4);    // length
      ax12writeB(AX_READ_DATA);
      ax12writeB(regstart);
      ax12writeB(length);
      ax12writeB(checksum);  
      setRX(id);    
      if(ax12ReadPacket(length + 6) > 0){
          ax12Error = ax_rx_buffer[4];
          if(length == 1)
              return ax_rx_buffer[5];
          else
              return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
      }else{
          return -1;
      }
    }
}

/* Set the value of a single-byte register. */
void ax12SetRegister(int id, int regstart, int data){
    if(regstart==AX_PRESENT_POSITION_L) {
      position_buf[id] = data;
    } else if(regstart==AX_PRESENT_VOLTAGE) {
    } else if(regstart==AX_LED) {
    } else {
      setTX(id);    
      int checksum = ~((id + 4 + AX_WRITE_DATA + regstart + (data&0xff)) % 256);
      ax12writeB(0xFF);
      ax12writeB(0xFF);
      ax12writeB(id);
      ax12writeB(4);    // length
      ax12writeB(AX_WRITE_DATA);
      ax12writeB(regstart);
      ax12writeB(data&0xff);
      // checksum = 
      ax12writeB(checksum);
      setRX(id);
      //ax12ReadPacket();
    }
}
/* Set the value of a double-byte register. */
void ax12SetRegister2(int id, int regstart, int data){
    setTX(id);    
    int checksum = ~((id + 5 + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(5);    // length
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
    ax12writeB(data&0xff);
    ax12writeB((data&0xff00)>>8);
    // checksum = 
    ax12writeB(checksum);
    setRX(id);
    //ax12ReadPacket();
}

// general write?
// general sync write?


