void i2c_scanner(void) {
  //Let's declare some variables so we can use them in this subroutine.
  data = 0;
  uint8_t error, address, done;
  uint16_t nDevices;
  Serial.println("Scanning address 1 till 127...");
  Serial.println("");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    HWire.beginTransmission(address);
    error = HWire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  done = 1;
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
  delay(2000);
  print_intro();
}

