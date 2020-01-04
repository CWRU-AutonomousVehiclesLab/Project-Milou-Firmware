void activateESTOP(){
    digitalWrite(SOFTWAREENABLEPIN, 1); //output low when in EStop State

}

void writeSabertoothMC()
{
  /*
  //TODO - now that the PID is in place Motor Speeds could possibly be given a value higher than 127, impose a cap of some sort
  byte addressByte;
  byte leftCommandByte;
  byte leftDataByte;
  byte leftChecksumByte;
  byte rightCommandByte;
  byte rightDataByte;
  byte rightChecksumByte;

  MotorCommandLastSent = millis();

  //If robot is estopped, set the speed to 0 for outputting
  if (State == STATE_ESTOP)
  {
    MotorSpeeds[LEFTSPEED] = B00000000;
    MotorSpeeds[RIGHTSPEED] = B00000000;

  } else if(State==STATE_AUTONOMOUS){       //If running autonomously, call the PID function to get the actual speed we're currently outputting
    PID();   
  } else{
    MotorSpeeds[LEFTSPEED] = DesiredSpeeds[LEFTSPEED];   //no need to do PID for RC mode, so a simple passthrough from desired to output speed
    MotorSpeeds[RIGHTSPEED] = DesiredSpeeds[RIGHTSPEED];
  }

  //We're using the packetized serial mode to interface with the Sabertooth controller
  //This is a four byte packet:
  //byte 1: address of the controller, set by DIP switches 4, 5 and 6 on the controller
  addressByte = SabertoothAddress;
  //byte 2: the command, relevant ones here are 0 = motor 1 forward, 1 = motor 1 backwards, 4 = motor 2 forward, 5 = motor 2 backwards, they've already been encoded
  leftCommandByte = MotorSpeeds[LEFTDIRECTION];
  rightCommandByte = MotorSpeeds[RIGHTDIRECTION];
  //byte 3: the data, a speed from 0 - 127
  leftDataByte = MotorSpeeds[LEFTSPEED];
  rightDataByte = MotorSpeeds[RIGHTSPEED];
  //byte 4: the checksum, the sum of the previous 3 bytes, and ANDed with the mask 01111111b
  leftChecksumByte = (addressByte + leftCommandByte + leftDataByte) & SabertoothMask;
  rightChecksumByte = (addressByte + rightCommandByte + rightDataByte) & SabertoothMask;

  //output our packets, starting with left
  sabertoothSerial.write(addressByte);
  sabertoothSerial.write(leftCommandByte);
  sabertoothSerial.write(leftDataByte);
  sabertoothSerial.write(leftChecksumByte);
  //right packet
  sabertoothSerial.write(addressByte);
  sabertoothSerial.write(rightCommandByte);
  sabertoothSerial.write(rightDataByte);
  sabertoothSerial.write(rightChecksumByte);
  return;
  */
}

/*
  MotorCommandLastSent = millis();
  
  //If robot is estopped, send a braking command to the motor controller as a secondary safety stop
  if (Estopstatus == false)
  {
    LeftMotorSpeed = 0;
    RightMotorSpeed = 0;
    //When E-stopped, set PID error variables to 0 to wipe accumulated error to prevent sudden acceleration on start
    left_speed_error = 0;
    left_speed_error_old = 0;
    left_speed_error_sum = 0;
    right_speed_error = 0;
    right_speed_error_old = 0;
    right_speed_error_sum = 0;
  }
 
 //Motor Control Code  
// send packet to motor 1
  if (LeftMotorSpeed <= 127){
    Checksum1 = (SabertoothAddress + LeftMotorDirection + LeftMotorSpeed);
    SabertoothSerial.write(SabertoothAddress);
    SabertoothSerial.write(LeftMotorDirection);
    SabertoothSerial.write(LeftMotorSpeed);
    SabertoothSerial.write(Checksum1 & Mask);
  }

// send packet to motor 2
  if (RightMotorSpeed <= 127){
  Checksum2 = (SabertoothAddress + RightMotorDirection + RightMotorSpeed);
  SabertoothSerial.write(SabertoothAddress);
  SabertoothSerial.write(RightMotorDirection);
  SabertoothSerial.write(RightMotorSpeed);
  SabertoothSerial.write(Checksum2 & Mask);
  }
*/