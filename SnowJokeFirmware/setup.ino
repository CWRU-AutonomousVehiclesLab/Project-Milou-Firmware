void setup() {
  //Main Setup function
  //serial communication with the motor controller on pins 0/1
  // set baud rate to 38400 using the hardware DIP switches as per Sabertooth documentation
  Serial1.begin(38400);
  //serial communication with the console for debug purposes
  Serial.begin(38400);

  
  //Setting initial values for the motor speed both pre and post PID
  DesiredSpeeds[LEFTSPEED] = B00000000;      // set Motor 1 speed to 0 to start
  DesiredSpeeds[RIGHTSPEED] = B00000000;     // set Motor 2 speed to 0 to start
  DesiredSpeeds[LEFTDIRECTION] = B00000001;  // set Motor 1 backwards
  DesiredSpeeds[RIGHTDIRECTION] = B00000100; // set Motor 2 forwards
  MotorSpeeds[LEFTSPEED] = B00000000;      // set Motor 1 speed to 0 to start
  MotorSpeeds[RIGHTSPEED] = B00000000;     // set Motor 2 speed to 0 to start
  MotorSpeeds[LEFTDIRECTION] = B00000001;  // set Motor 1 backwards
  MotorSpeeds[RIGHTDIRECTION] = B00000100; // set Motor 2 forwards
  //setup software enable pin
  pinMode(SOFTWAREENABLEPIN, OUTPUT); // set up pin for software enable as an output
  // setup physical switch input pins
  pinMode(SWITCHAPIN, INPUT);
  pinMode(SWITCHBPIN, INPUT);
  pinMode(SWITCHCPIN, INPUT);
  pinMode(SWITCHDPIN, INPUT);
  pinMode(ESTOPPIN, INPUT);
  //Set estop status pin as input
  //Setup RC Input Pins
  pinMode(FWDPULSEPIN, INPUT_PULLUP);
  pinMode(ANGPULSEPIN, INPUT_PULLUP);
  pinMode(ESTOPPULSEPIN, INPUT_PULLUP);
  //Attach High and Low Interrupts to each RC Input
  attachInterrupt(digitalPinToInterrupt(FWDPULSEPIN), FwdPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ANGPULSEPIN), AngPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ESTOPPULSEPIN), EStopPulse, CHANGE);
  //use a similar method for getting input from the encoders
  attachInterrupt(digitalPinToInterrupt(LEFTENCODERPIN), LeftEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHTENCODERPIN), RightEncoderPulse, RISING);
  //attach interrupts so the switches are read if any are changed
  attachInterrupt(digitalPinToInterrupt(SWITCHAPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHBPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHCPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCHDPIN), ReadSwitches, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ESTOPPIN), ReadSwitches, CHANGE);

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

  //initialize in the EStop state, no action until the physical estop is cycled
  State = STATE_ESTOP;
  NextState = STATE_ESTOP;
  RCEStop = false;
  //Delay 5 seconds to allow for initialization of other components
  delay(5000);
  //run all of the various sub-functions and establish an initial run time
  ControlLoop();
  SabertoothMotorCommandLoop();
  ReadSwitches();

  return;
}
