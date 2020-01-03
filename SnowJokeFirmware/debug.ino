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