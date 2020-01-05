//main loop
void loop() {
  //To keep checking for whether the RC EStop is on
  long currentTime = millis();
  GetRCData();
  // check to see if enough time has passed to run the control loop
  //! robot loop runs at no faster than 50hz

  if ((currentTime - ControlLoopLastTime) >= CONTROLLOOPRATE)
  {
    if ((currentTime - ControlLoopLastTime) >= (CONTROLLOOPRATE*2)){
      DebugOutput("Control Loop Out of Sync. Delayed 2 Times!",4);
    }
    ControlLoop();
  }


  if((currentTime - MotorCommandLastSent) >= SABERTOOTHLOOPRATE){
    if ((currentTime - MotorCommandLastSent) >= (SABERTOOTHLOOPRATE*2)){
      DebugOutput("Sabertooth Write Loop Out of Sync. Delayed 2 Times!",4);
    }
      writeSabertoothMC(cmdLeftMotorSpeed,cmdRightMotorSpeed);
  }

  if ((currentTime - ROSLastSent) >= ROSPUBLISHRATE){
    if ((currentTime - MotorCommandLastSent) >= (SABERTOOTHLOOPRATE*2)){
      DebugOutput("ROS Feedback Loop Out of Sync. Delayed 2 Times!",5);
    }    
    ROSPublish();
  }

  // If switch C is enabled the robot is in debug mode; dump output to console
  if ((currentTime - LastDebug) >= DEBUGOUTPUTRATE)  //add switch c check
  {
    if ((currentTime - LastDebug) >= (DEBUGOUTPUTRATE*2)){
      DebugOutput("Debug display Loop Out of Sync. Delayed 2 Times!",4);
    }
    debugDump();
  }


}
