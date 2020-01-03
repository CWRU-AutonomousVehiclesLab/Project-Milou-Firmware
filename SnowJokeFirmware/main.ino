//main loop
void loop() {
  //To keep checking for whether the RC EStop is on
  long currentTime = millis();
  GetRCData();
  // check to see if enough time has passed to run the control loop
  //! robot loop runs at no faster than 50hz
  if ((currentTime - ControlLoopLastTime) >= CONTROLLOOPRATE)
  {
    ControlLoop();
  }


  if((currentTime - MotorCommandLastSent) >= SABERTOOTHLOOPRATE){
      writeSabertoothMC();
  }

  // If switch C is enabled the robot is in debug mode; dump output to console
  if (currentTime - LastDebug >= DEBUGOUTPUTRATE)  //add switch c check
  {
    debugOutput();
  }
}
