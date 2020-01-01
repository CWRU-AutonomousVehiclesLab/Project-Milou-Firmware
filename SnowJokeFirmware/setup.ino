void setup() {
  //!====================Serial Setup====================
  //serial communication with the motor controller on pins 0/1
  // set baud rate to 38400 using the hardware DIP switches as per Sabertooth documentation
  //? Sabertooth Serial
  Serial1.begin(38400);
  //? PC Serial
  Serial.begin(38400);

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
  pinMode(ESTOPPIN, INPUT);

  //!====================Interrupt Setup====================
  //? RC Pins (HIGH LOW Interrupt)
  attachInterrupt(digitalPinToInterrupt(FWDPULSEPIN), FwdPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ANGPULSEPIN), AngPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ESTOPPULSEPIN), EStopPulse, CHANGE);
  //? Encoders
  attachInterrupt(digitalPinToInterrupt(LEFTENCODERPIN), LeftEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHTENCODERPIN), RightEncoderPulse, RISING);
  //? Switch State
  attachInterrupt(digitalPinToInterrupt(SWITCHAPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHBPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHCPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHDPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ESTOPPIN), ReadSwitches, CHANGE);

  //!====================Sabertootn Initial Flush====================
  //some serial setup stuff.  take a closer look at the values he's passing in at some point
  Serial1.write(SabertoothAddress);
  Serial1.write(B00001111);
  Serial1.write(B00000010);
  int Checksum0 = (SabertoothAddress + B00000010 + B00001111);
  Serial1.write(Checksum0 & SabertoothMask);
  // set timeout to 200ms
  Serial1.write(SabertoothAddress);
  Serial1.write(B00001110);
  Serial1.write(B00000010);
  Checksum0 = (SabertoothAddress + B00000010 + B00001110);
  Serial1.write(Checksum0 & SabertoothMask);

  //!====================State initialization====================
  //initialize in the EStop state, no action until the physical estop is cycled
  State = STATE_ESTOP;
  NextState = STATE_ESTOP;
  RCEStop = false;

  //!====================Setup Halt====================
  //Delay 5 seconds to allow for initialization of other components
  delay(5000);

  //!====================Initial read State====================
  //run all of the various sub-functions and establish an initial run time
  ControlLoop();
  SabertoothMotorCommandLoop();
  ReadSwitches();
}
