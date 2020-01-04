//take the desired speed calculated so far, use PID functions to calculate the actual speed
void PID()
{
  /*
  //get time 
  float currentTime = millis();  
  float timeElapsed = currentTime-PIDLastTime;
  //directions shouldn't change at all
  MotorSpeeds[LEFTDIRECTION] = DesiredSpeeds[LEFTDIRECTION];
  MotorSpeeds[RIGHTDIRECTION] = DesiredSpeeds[RIGHTDIRECTION];

  //take the pulse length from the encoder input, and turn it into current speed
  //calculate this by dividing the distance covered in one rotation by the time it takes to complete one rotation, the latter of which is the length of one encoder pulse times the number of pulses per cycle
  float currentSpeedInMetersPerSecond[2];

  currentSpeedInMetersPerSecond[LEFTENCODER] = ((WheelDiameter * PI) / (EncoderPulseData[LEFTENCODER] * ENCODERCYCLES));
  currentSpeedInMetersPerSecond[RIGHTENCODER] = ((WheelDiameter * PI) / (EncoderPulseData[RIGHTENCODER] * ENCODERCYCLES));

  //the final product will be a 0-127 value, just like the desired and final speeds
  int currentSpeeds[2];
  currentSpeeds[LEFTSPEED] = map(currentSpeedInMetersPerSecond[LEFTENCODER], 0, MaxDesiredSpeed, 0, 127);
  currentSpeeds[RIGHTSPEED] = map(currentSpeedInMetersPerSecond[RIGHTENCODER], 0, MaxDesiredSpeed, 0, 127);

  //PID calculations for left wheel
  PIDError[LEFTSPEED] = DesiredSpeeds[LEFTSPEED]-currentSpeeds[LEFTSPEED];
  PIDIntegralError[LEFTSPEED] += PIDError[LEFTSPEED]*timeElapsed;
  PIDDerivativeError[LEFTSPEED] = (PIDError[LEFTSPEED]-PIDPreviousError[LEFTSPEED])/timeElapsed;

  //PID calculations for right wheel
  PIDError[RIGHTSPEED] = DesiredSpeeds[RIGHTSPEED]-currentSpeeds[RIGHTSPEED];
  PIDIntegralError[RIGHTSPEED] += PIDError[RIGHTSPEED]*timeElapsed;
  PIDDerivativeError[RIGHTSPEED] = (PIDError[RIGHTSPEED]-PIDPreviousError[RIGHTSPEED])/timeElapsed;

  //using gain constants, calculate motor speed to output
  MotorSpeeds[LEFTSPEED] = KP*PIDError[LEFTSPEED] + KI*PIDIntegralError[LEFTSPEED] + KD*PIDDerivativeError[LEFTSPEED];
  MotorSpeeds[RIGHTSPEED] = KP*PIDError[RIGHTSPEED] + KI*PIDIntegralError[RIGHTSPEED] + KD*PIDDerivativeError[RIGHTSPEED];
  
  PIDLastTime = currentTime;
  */
}
/*
  //PID constants, move to start of code once PID verified working
  float kP_left = 0.22;
  float kI_left = 0.05;
  float kD_left = 0.001;
  float kP_right = 0.22;
  float kI_right = 0.05;
  float kD_right = 0.001;
  
  //PID variables
  left_speed_error = desiredleftspeed - LeftWheelSpeed;
  right_speed_error = desiredrightspeed - RightWheelSpeed;
  
  AutoLeftSpeed = round(left_speed_error*kP_left + left_speed_error_sum*kI_left + (left_speed_error - left_speed_error_old)*kD_left);
  AutoRightSpeed = round(right_speed_error*kP_right + right_speed_error_sum*kI_right + (right_speed_error - right_speed_error_old)*kD_right);

  //save previous error state
  left_speed_error_old = left_speed_error;
  right_speed_error_old = right_speed_error;
  
  //Add error to error sum for integral component
  left_speed_error_sum += left_speed_error;
  right_speed_error_sum += right_speed_error;
  
  //cap error to prevent huge windup, using arbitrary magic number
  if (left_speed_error_sum > 4000)
  {
    left_speed_error_sum = 4000;
  }
  if (left_speed_error_sum < -4000)
  {
    left_speed_error_sum = -4000;
  }
  
  if (right_speed_error_sum > 4000)
  {
    right_speed_error_sum = 4000;
  }
  if (right_speed_error_sum < -4000)
  {
    right_speed_error_sum = -4000;
  }  

//cap motor speeds and specify direction
  if (AutoLeftSpeed < 0)
  {
    LeftMotorDirection = B00000000;
    LeftMotorSpeed = -1 * AutoLeftSpeed;
  }
  else if (AutoLeftSpeed >= 0)
  {
    LeftMotorDirection = B00000001;
    LeftMotorSpeed = AutoLeftSpeed;
  }
  
  if (AutoRightSpeed < 0)
  {
    RightMotorDirection = B00000100;
    RightMotorSpeed = -1 * AutoRightSpeed;
  }
  else if (AutoRightSpeed >= 0)
  {
    RightMotorDirection = B00000101;
    RightMotorSpeed = AutoRightSpeed;
  }
  
  if (LeftMotorSpeed > 127)
  {
    LeftMotorSpeed = 127;
  }
  if (RightMotorSpeed > 127)
  {
    RightMotorSpeed = 127;
  }
  */