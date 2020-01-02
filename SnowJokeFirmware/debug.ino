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

void debugOutput(){
  String debugString;
  debugString = "  Current State is: "+String(State);
  DebugOutput(debugString,0);
  return;
}

// void DebugPrint()
// {
//   String OutputString = "";
//   LastDebug = millis();
  
//   //The below debugging info is for the purpose of the technical demonstration on 12/2/2019

//   //first demonstration: state switching between Autonomous, RC, and EStop
// /*
//   if(State==STATE_ESTOP)
//     OutputString = "State: EStop";
//   else if(State==STATE_RC)
//     OutputString = "State: RC";
//   else if(State==STATE_AUTONOMOUS)
//     OutputString = "State: Autonomous";
//   DebugOutput(OutputString, 0, true, false);
//   OutputString = "Switches (A, B, C, D, ESTOP): " + String(Switches[SWITCH_A]) + ", " + String(Switches[SWITCH_B]) + ", " + String(Switches[SWITCH_C]) + ", " + String(Switches[SWITCH_D]) + ", " + String(Switches[SWITCH_ESTOP]);
//   DebugOutput(OutputString, 0, true, false);
//  */

//   //second demonstration: reading RC signal lengths

//   /*
//   if (State == STATE_RC) {
//     OutputString = "Forward Pulse: " + String(RCPulseData[FWDPULSEDATA]);
//     DebugOutput(OutputString, 0, true, false);
//     OutputString = "Ang Pulse: " + String(RCPulseData[ANGPULSEDATA]);
//     DebugOutput(OutputString, 0, true, false);
//     OutputString = "EStop Pulse: " + String(RCPulseData[ESTOPPULSEDATA]);
//     DebugOutput(OutputString, 0, true, false);
//   }
//   */

//   //third demonstration: calculating wheel speeds in RC mode

//   /*OutputString = "Left wheel speed: " + String(MotorSpeeds[LEFTSPEED] + ", direction: " + String(MotorSpeeds[LEFTDIRECTION]));
//     DebugOutput(OutputString, 0, true, false);
//     OutputString = "Right wheel speed: " + String(MotorSpeeds[RIGHTSPEED] + ", direction: " + String(MotorSpeeds[RIGHTDIRECTION]));
//     DebugOutput(OutputString, 0, true, false);
//   */

//   //fourth demonstration: getting RPM from encoder input

  
//   float EncoderRPM = (60*1000000)/(EncoderPulseData[LEFTENCODER] * 256);
//   OutputString = ("Encoder RPM: " + String(EncoderRPM) + "RPM\n");
//   DebugOutput(OutputString, 0, true, false);
  
//   //fifth demonstration: comparing desired vs. actual speed in autonomous mode
// /*
//   AutonomousTestAccumulator = AutonomousTestAccumulator + (millis() - AutonomousTestTiming);
//   AutonomousTestTiming = millis();
//   if(AutonomousTestAccumulator > 5000){
//     AutonomousLinearVelocity = 1;
//     if(AutonomousTestAccumulator > 10000)
//       AutonomousTestAccumulator = 0;
//   }else
//     AutonomousLinearVelocity = 0;
//   OutputString = "Currently giving test speed: " + String(AutonomousLinearVelocity);
//   DebugOutput(OutputString, 0, true, false);
//   if(State==STATE_AUTONOMOUS){
//     OutputString = "Left wheel speed after PID: " + String(MotorSpeeds[LEFTSPEED]) + ", direction: " + String(MotorSpeeds[LEFTDIRECTION]));
//     DebugOutput(OutputString, 0, true, false);
//   }else{
//     OutputString = "Not in autonomous mode";
//     DebugOutput(OutputString, 0, true, false);
//   }
//   */
//   return;
// }
