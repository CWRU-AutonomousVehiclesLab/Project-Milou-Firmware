void setup() {
  //!====================Serial Setup====================
  //serial communication with the motor controller on pins 0/1
  // set baud rate to 38400 using the hardware DIP switches as per Sabertooth documentation

  //? Sabertooth Serial
  sabertoothSerial.begin(38400);
  //? PC Serial
  terminalSerial.begin(38400);
  /*
  //!====================Speed Init====================
  //Setting initial values for the motor speed both pre and post PID
  DesiredSpeeds[LEFTSPEED] = B00000000;      // set Motor 1 speed to 0 to start
  DesiredSpeeds[RIGHTSPEED] = B00000000;     // set Motor 2 speed to 0 to start
  DesiredSpeeds[LEFTDIRECTION] = B00000001;  // set Motor 1 backwards
  DesiredSpeeds[RIGHTDIRECTION] = B00000100; // set Motor 2 forwards
  
  //!====================Sabertooth Init====================  
  MotorSpeeds[LEFTSPEED] = B00000000;      // set Motor 1 speed to 0 to start
  MotorSpeeds[RIGHTSPEED] = B00000000;     // set Motor 2 speed to 0 to start
  MotorSpeeds[LEFTDIRECTION] = B00000001;  // set Motor 1 backwards
  MotorSpeeds[RIGHTDIRECTION] = B00000100; // set Motor 2 forwards
  */
  //!====================Pin Map====================
  //? RC
  pinMode(FWDPULSEPIN, INPUT_PULLUP);
  pinMode(ANGPULSEPIN, INPUT_PULLUP);
  pinMode(ESTOPPULSEPIN, INPUT_PULLUP);
  //? physical switch
  pinMode(SWITCHAPIN, INPUT);
  pinMode(SWITCHBPIN, INPUT);
  pinMode(SWITCHCPIN, INPUT);
  pinMode(SWITCHDPIN, INPUT);
  //? software enable pin
  pinMode(SOFTWAREENABLEPIN, OUTPUT); // set up pin for software enable as an output
  //? Estop Observer
  pinMode(ESTOPPIN, INPUT_PULLUP);
  //? LED Indicator
  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);
  //? Encoder
  pinMode(LEFTENCODER_A,INPUT);
  pinMode(LEFTENCODER_B,INPUT);
  pinMode(RIGHTENCODER_A,INPUT);
  pinMode(RIGHTENCODER_B,INPUT);



  //!====================Interrupt Setup====================
  //? RC Pins (HIGH LOW Interrupt)
  attachInterrupt(digitalPinToInterrupt(FWDPULSEPIN), FwdPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ANGPULSEPIN), AngPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ESTOPPULSEPIN), EStopPulse, CHANGE);
  //? Encoders
  attachInterrupt(digitalPinToInterrupt(LEFTENCODER_A), LeftEncoderA, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(LEFTENCODER_B), LeftEncoderB, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(RIGHTENCODER_A), RightEncoderA, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(RIGHTENCODER_B), RightEncoderB, CHANGE);  

  //? Switch State
  attachInterrupt(digitalPinToInterrupt(SWITCHAPIN), readSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHBPIN), readSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHCPIN), readSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHDPIN), readSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ESTOPPIN), readSwitches, CHANGE);

  //!====================Sabertootn Initial Flush====================
  //some serial setup stuff.  take a closer look at the values he's passing in at some point
  sabertoothSerial.write(SabertoothAddress);
  sabertoothSerial.write(B00001111);
  sabertoothSerial.write(B00000010);
  Checksum0 = (SabertoothAddress + B00000010 + B00001111);
  sabertoothSerial.write(Checksum0 & SabertoothMask);
  // set timeout to 200ms
  sabertoothSerial.write(SabertoothAddress);
  sabertoothSerial.write(B00001110);
  sabertoothSerial.write(B00000010);
  Checksum0 = (SabertoothAddress + B00000010 + B00001110);
  sabertoothSerial.write(Checksum0 & SabertoothMask);

  //!====================State initialization====================
  //initialize in the EStop state, no action until the physical estop is cycled
  State = STATE_ESTOP;
  NextState = STATE_ESTOP;
  

  //!====================Setup Halt====================
  //Delay 5 seconds to allow for initialization of other components
  delay(5000);

  //!====================Initial read State====================
  //run all of the various sub-functions and establish an initial run time
  digitalWrite(SOFTWAREENABLEPIN, HIGH); // All set indicator, enable motor
  ControlLoop();
  writeSabertoothMC();
  readSwitches();
  GetRCData();
  ROSPublish();
}
