
void RC_handle() {

  // update incoming values
  RC_PPMIn.update(); 

  // incoming values available in RC_channel
  if(RC_PPMIn.isStable()) {
    RCconnected = true;
    setCommand();
  }
  
  // signal has been lost
  else if(RC_PPMIn.isLost()) {
    if(RCconnected) {
      RCconnected = false;
      resetCommand();
    }
  }

}

static uint8_t PPMlastB = 0; // last read value of PINB
ISR(PCINT0_vect) {
  uint8_t PPMnewB = PINB;
  if((PPMlastB & (1 << 0)) != (PPMnewB & (1 << 0))) {
    // we tell PPMIn the pin has changed
    RC_PPMIn.pinChanged(PPMnewB & (1 << 0));
  }
  PPMlastB = PPMnewB;
}
