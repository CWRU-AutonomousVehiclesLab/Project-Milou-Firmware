void DebugOutput(String outputString, int level, bool debug=true, bool minimal=false)
{
  String prefix;
  String rosString = " [Teensy] "+outputString;
  switch (level)
  {
    case 0:
      if (debug) {
        prefix = "[DEBUG]";
        //nh.logdebug(rosString.c_str());
        outputString = prefix + outputString;
      }
      break;
    case 1:
      if (!minimal) {
        prefix = "[INFO]";
        //nh.loginfo(rosString.c_str());
        outputString = prefix + outputString;
      }
      break;
    case 2:
      prefix = "[WARNING]";
      //nh.logwarn(rosString.c_str());
      outputString = prefix + outputString;
      break;
    case 3:
      prefix = "[ERROR]";
      //nh.logerror(rosString.c_str());
      outputString = prefix + outputString;
      break;
    case 4:
      prefix = "[CRITICAL]";
      //nh.logfatal(rosString.c_str());
      outputString = prefix + outputString;
      break;
    case 5:
      prefix = "[FATAL]";
      //nh.logfatal(rosString.c_str());
      outputString = prefix + outputString;
      break;
  }
  terminalSerial.println(outputString);

  return;
}

void debugDump(){
  LastDebug = millis();
  terminalSerial.println("========================================");
  String debugString;
  
  //! State Check
  debugString = "  State is: "+String(State);
  DebugOutput(debugString,0);
  //! Check RC Recieve
  debugString = "  RC_SIG recieved is:  FWD: "+String(RCPulseData[FWDPULSEDATA])+"  ANG: "+String(RCPulseData[ANGPULSEDATA])+"  ESTOP: "+String(RCPulseData[ESTOPPULSEDATA]);
  DebugOutput(debugString,0);
  
  if (State == STATE_RC){
    //! Check RC Interpretation
    debugString = "  RC_CMD is:   V: "+String(rcLinearSpeed)+     "   W: "+String(rcAngularSpeed);
    DebugOutput(debugString,0);
  } else if(State == STATE_AUTONOMOUS) {
    //! Check ROS get State
    debugString = "  Autonomous_CMD is:   V: "+String(autoLinearVelocity)+"   W: "+String(autoAngularVelocity);
    DebugOutput(debugString,0);
  }
  //! Check IK Calculation
  debugString = "  IK is:             Left: "+String(desLeftMotorSpeed)+  "   Right: "+String(desRightMotorSpeed);
  DebugOutput(debugString,0);

  debugString = "  EncoderCount is:   Left: "+String(leftEncoderPos)+     "   Right: "+String(rightEncoderPos);
  DebugOutput(debugString,0);

  debugString = "  ActualSpeed is:    Left: "+String(obsLeftMotorSpeed)+  "   Right: "+String(obsRightMotorSpeed);
  DebugOutput(debugString,0);

  pidDebug();

  /*
  debugString = "  PID_DES_Speed is:  Left: "+String(pidedLeftMotorSpeed)+"   Right: "+String(pidedRightMotorSpeed);
  DebugOutput(debugString,0);
  */
 
  debugString = "  ActualCommand is:  Left: "+String(cmdLeftMotorSpeed)+  "   Right: "+String(cmdRightMotorSpeed);
  DebugOutput(debugString,0);  
  return;
}
