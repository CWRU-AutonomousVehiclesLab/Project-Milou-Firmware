//take the desired speed calculated so far, use PID functions to calculate the actual speed
void PID()
{
  //get time 
  float currentTime = millis();  
  float dt = currentTime-PIDLastTime;
  //PID variables
  left_speed_error = (desLeftMotorSpeed - obsLeftMotorSpeed)*100.0;     //This PID magic number is tuned in cm/s
  right_speed_error = (desRightMotorSpeed - obsRightMotorSpeed)*100.0;  //This PID magic number is tuned in cm/s

  left_speed_d = left_speed_error - left_speed_error_old;
  right_speed_d = right_speed_error - right_speed_error_old;

  //Add error to error sum for integral component
  left_speed_error_sum += left_speed_error;
  right_speed_error_sum += right_speed_error;

  pidedLeftMotorSpeed = round(left_speed_error*kP_left + left_speed_error_sum*kI_left + left_speed_d*kD_left);
  pidedRightMotorSpeed = round(right_speed_error*kP_right + right_speed_error_sum*kI_right + right_speed_d*kD_right);

  //Direction Set: 
  leftMotorDirection=(pidedLeftMotorSpeed>=0) ?  SB_leftForward: SB_leftBackward;
  rightMotorDirection=(pidedRightMotorSpeed>=0) ? SB_rightForward : SB_rightBackward;

  //? Wind up prevention: https://www.youtube.com/watch?v=NVLXCwc8HzM
  // Clamp
  cmdLeftMotorSpeed = min(abs(pidedLeftMotorSpeed),SABERTOOTHMAX);
  cmdRightMotorSpeed = min(abs(pidedRightMotorSpeed),SABERTOOTHMAX);

  int leftSaturated  = (cmdLeftMotorSpeed == pidedLeftMotorSpeed) ? 0 : 1;
  int rightSaturated = (cmdRightMotorSpeed == pidedRightMotorSpeed) ? 0 : 1;

  int leftSignCompare = (((left_speed_error>=0) & (pidedLeftMotorSpeed>=0))||((left_speed_error<0) & (pidedLeftMotorSpeed<0))) ? 1 : 0;
  int rightSignCompare = (((right_speed_error>=0) & (pidedRightMotorSpeed>=0))||((right_speed_error<0) & (pidedRightMotorSpeed<0))) ? 1 : 0;

  left_speed_error_sum = (leftSaturated & leftSignCompare) ? (left_speed_error_sum-= left_speed_error) : left_speed_error_sum;
  right_speed_error_sum = (rightSaturated & rightSignCompare) ? (right_speed_error_sum-= right_speed_error) :right_speed_error_sum;

  //cap error to prevent huge windup, using arbitrary magic number
  //left_speed_error_sum = max(min(left_speed_error_sum,pidErrorCap),pidErrorCap*-1.0);
  //right_speed_error_sum = max(min(right_speed_error_sum,pidErrorCap),pidErrorCap*-1.0);

  //save previous error state
  left_speed_error_old = left_speed_error;
  right_speed_error_old = right_speed_error;



  PIDLastTime = currentTime;
}

void resetPID(){
    //When E-stopped, set PID error variables to 0 to wipe accumulated error to prevent sudden acceleration on start
    left_speed_error = 0.0;
    left_speed_error_old = 0.0;
    left_speed_error_sum = 0.0;
    right_speed_error = 0.0;
    right_speed_error_old = 0.0;
    right_speed_error_sum = 0.0;
}

void pidDebug(){
  String debugString;
  debugString = "  speed_error is:  Left: "+String(left_speed_error)+  "   Right: "+String(right_speed_error);
  DebugOutput(debugString,0);  

  debugString = "  pidedMotorSpeed is:  Left: "+String(pidedLeftMotorSpeed)+  "   Right: "+String(pidedRightMotorSpeed);
  DebugOutput(debugString,0);  

  debugString = "  I is:  Left: "+String(left_speed_error_sum)+  "   Right: "+String(right_speed_error_sum);
  DebugOutput(debugString,0);  

  debugString = "  D is:  Left: "+String(left_speed_d)+  "   Right: "+String(right_speed_d);
  DebugOutput(debugString,0); 

  debugString = "  D is:  Left: "+String(left_speed_d)+  "   Right: "+String(right_speed_d);
  DebugOutput(debugString,0); 
}