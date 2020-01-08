void activateESTOP(){
    digitalWrite(SABERTOOTHENABLE,LOW);
    desLeftMotorSpeed = 0.0;
    desRightMotorSpeed = 0.0;
    cmdLeftMotorSpeed = 0;
    cmdRightMotorSpeed = 0;
    resetPID();
    return;
}

void enableSabertooth(){
  digitalWrite(SABERTOOTHENABLE, HIGH);
  return;
}

void sabertoothInit(){
  //!====================Sabertootn Initial Flush====================
  //+++++++++++++++++++Serial Rate Select++++++++++++++++++++++++++++
  //? Set the Address
  sabertoothSerial.write(SabertoothAddress);
  //? Set the baud rate
  sabertoothSerial.write(SabertoothSetSerial);
  sabertoothSerial.write(SabertoothSerialOption); //Option 4: 38400 Baud
  //? Checksum
  sbCMDCheck = (SabertoothAddress + SabertoothSetSerial + SabertoothSerialOption);
  sabertoothSerial.write(sbCMDCheck & SabertoothChecksumMask);
  //+++++++++++++++++++timeout Select++++++++++++++++++++++++++++
  //? Set the Address
  sabertoothSerial.write(SabertoothAddress);
  //? Set the baud rate
  sabertoothSerial.write(SabertoothSetTimeout);
  sabertoothSerial.write(SabertoothTimeoutOption);
  //? Checksum
  sbCMDCheck = (SabertoothAddress + SabertoothSetTimeout + SabertoothTimeoutOption)& SabertoothChecksumMask;
  sabertoothSerial.write(sbCMDCheck);
  return;
}

void writeSabertoothMC(int leftSpeed, int rightSpeed)
{
  MotorCommandLastSent = millis();
  //? Final condom for accidentally giant value written
  leftSpeed = min(leftSpeed,SABERTOOTHMAX);
  rightSpeed = min(rightSpeed,SABERTOOTHMAX);

  //+++++++++++LEFT MOTOR+++++++++++++++++
  //? 1. Address select:
  sabertoothSerial.write(SabertoothAddress);
  //? 2. Direction(Mode) Select
  sabertoothSerial.write(leftMotorDirection);
  //? 3. Value write
  sabertoothSerial.write(cmdLeftMotorSpeed);
  //? 4. Checksum
  sbCMDCheck = (SabertoothAddress+leftMotorDirection+cmdLeftMotorSpeed)&SabertoothChecksumMask;
  sabertoothSerial.write(sbCMDCheck);

  //+++++++++++Right MOTOR+++++++++++++++++
  //? 1. Address select:
  sabertoothSerial.write(SabertoothAddress);
  //? 2. Direction(Mode) Select
  sabertoothSerial.write(rightMotorDirection);
  //? 3. Value write
  sabertoothSerial.write(cmdRightMotorSpeed);
  //? 4. Checksum
  sbCMDCheck = (SabertoothAddress+rightMotorDirection+cmdRightMotorSpeed)&SabertoothChecksumMask;
  sabertoothSerial.write(sbCMDCheck);
  return;
}