
void receiveEvent(int howMany) {
  byte data[howMany];
  int index = 0;
  while(Wire.available()) {
    byte d = Wire.read();
    if(index < howMany) {
      data[index] = d;
      index++;
    }
  }
  if(index>=2) {
    command.ledNum = data[0];
    command.ledMode = data[1];
    if(command.ledMode>1) command.ledMode = 0;
    if(command.ledNum>4) command.ledNum = 0;
    setLed(command.ledNum, 0, 0);
  }
}

void requestEvent() {
  Wire.write(command.rcData, sizeof(command.rcData));
}
