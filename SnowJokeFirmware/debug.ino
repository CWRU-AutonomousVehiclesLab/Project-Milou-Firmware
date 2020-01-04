void DebugOutput(String outputString, int level, bool debug=true, bool minimal=false)
{
  switch (level)
  {
    case 0:
      if (debug) {
        terminalSerial.print("[DEBUG]");
      }
      break;
    case 1:
      if (!minimal) {
        terminalSerial.print("[INFO]");
      }
      break;
    case 2:
      terminalSerial.print("[WARNING]");
      break;
    case 3:
      terminalSerial.print("[ERROR]");
      break;
    case 4:
      terminalSerial.print("[CRITICAL]");
      break;
    case 5:
      terminalSerial.print("[FATAL]");
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
    debugString = "  RC_CMD is:  V: "+String(rcLinearSpeed)+"   W: "+String(rcAngularSpeed);
    DebugOutput(debugString,0);
  } else if(State == STATE_AUTONOMOUS) {
    //! Check ROS get State
    debugString = "  RC_CMD is:   V: "+String(autoLinearVelocity)+"   W: "+String(autoAngularVelocity);
    DebugOutput(debugString,0);
  }
  //! Check IK Calculation
  debugString = "  IK is:   Left: "+String(desLeftMotorSpeed)+"   R: "+String(desRightMotorSpeed);
  DebugOutput(debugString,0);

  return;
}
