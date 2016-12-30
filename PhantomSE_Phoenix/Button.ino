
#define BTN_DREB 100 
#define BTN_HOLD 3000
unsigned int btn_dreb_counter = 0;
unsigned long prev_handle_micros = 0;
bool btn_state = false;
byte pin = 0;
byte pressed_num = 0;
unsigned int pressed_timeout = 0;
unsigned int hold_counter = 0;

void btn_init(byte _pin) {
  pin = _pin;
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  prev_handle_micros = micros();
  printDebug("BTN init pin : ");
  printlnDebug(pin);
}

void btn_deInit() {
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  prev_handle_micros = micros();
  btn_state          = false;
  btn_dreb_counter   = 0;
  pressed_num        = 0;
  pressed_timeout    = 0;
  hold_counter       = 0;
}

void btn_event(byte event) {
  if(event!=6) {
    printDebug("BTN pressed : ");
    printlnDebug(event);
  } else printlnDebug("BTN is HOLD");
  switch(event) {
    case 1 :
      break;
    case 3 :
      //wifi_showAP(120);
      break;
    case 5 :
      //FIRMWARE_BACKUP();
      break;
    case 6 : // HOLD
      //wifi_wps();
      break;
    default: break;
  }
}

void btn_handle() {
  if(micros() - prev_handle_micros >= 1000) {
    if(!pin) return;
    if(pressed_num) {
      if(pressed_timeout < 1000) pressed_timeout++;
      else {
        if(pressed_num != 6) {
          btn_event(pressed_num);
        }
        pressed_timeout = 0;
        pressed_num = 0;
      }
    }
    if(btn_state) {
      if(hold_counter < BTN_HOLD) hold_counter++;
      else if(hold_counter == BTN_HOLD) {
        hold_counter++;
        pressed_num = 0;
        btn_event(6);
      }
    }
    if(!digitalRead(pin)) {
      if(!btn_state) {
        if(btn_dreb_counter < BTN_DREB) btn_dreb_counter++;
        else {
          btn_dreb_counter = 0;
          btn_state = true;
        }
      } else {
        btn_dreb_counter = 0;
      }
    } else {
      if(btn_state) {
        if(btn_dreb_counter < BTN_DREB) btn_dreb_counter++;
        else {
          btn_dreb_counter = 0;
          btn_state = false;
          hold_counter = 0;
          pressed_num++;
          pressed_timeout = 0;
        }
      } else {
        btn_dreb_counter = 0;
      }
    }
    prev_handle_micros = micros();
  }
}

