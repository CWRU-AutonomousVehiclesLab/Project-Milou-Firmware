//take the desired speed calculated so far, use PID functions to calculate the actual speed
void PID()
{
  //get time 
  float currentTime = millis();  
  float dt = currentTime-PIDLastTime;
  //PID variables
  left_speed_error = (desLeftMotorSpeed - obsLeftMotorSpeed)*100.0;     //This PID magic number is tuned in cm/s
  right_speed_error = (desRightMotorSpeed - obsRightMotorSpeed)*100.0;  //This PID magic number is tuned in cm/s

  pidedLeftMotorSpeed = round(left_speed_error*kP_left + left_speed_error_sum*kI_left + (left_speed_error - left_speed_error_old)*kD_left);
  pidedRightMotorSpeed = round(right_speed_error*kP_right + right_speed_error_sum*kI_right + (right_speed_error - right_speed_error_old)*kD_right);

  //save previous error state
  left_speed_error_old = left_speed_error;
  right_speed_error_old = right_speed_error;

  //Add error to error sum for integral component
  left_speed_error_sum += left_speed_error;
  right_speed_error_sum += right_speed_error;

  //cap error to prevent huge windup, using arbitrary magic number
  left_speed_error_sum = min(max(left_speed_error_sum,pidErrorCap),pidErrorCap*-1.0);
  right_speed_error_sum = min(max(right_speed_error_sum,pidErrorCap),pidErrorCap*-1.0);

  //Direction Set: 
  leftMotorDirection=(pidedLeftMotorSpeed>=0) ? B00000001 : B00000000;
  rightMotorDirection=(pidedRightMotorSpeed>=0) ? B00000101 : B00000100;

  // Cap Motor Speed:
  cmdLeftMotorSpeed = max(abs(pidedLeftMotorSpeed),SABERTOOTHMAX);
  cmdRightMotorSpeed = max(abs(pidedRightMotorSpeed),SABERTOOTHMAX);

  PIDLastTime = currentTime;
}
