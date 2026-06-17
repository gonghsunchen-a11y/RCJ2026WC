
void ballsensor() {
  uint8_t buffer[5];
  Serial6.write(0xDD);
  while(!Serial6.available());
  Serial6.readBytes(buffer,5);
  ballData.valid = 0;
  if(buffer[0] != 0xCC) return;
  if (buffer[4] != 0xEE) return;
  if(buffer[0] == 0xCC && buffer[4] == 0xEE){
      uint8_t found = buffer[1];
      uint16_t angle = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);

      ballData.valid = true;
      ballData.angle = angle;
  }
  else{
      ballData.valid = false;
  }
}
