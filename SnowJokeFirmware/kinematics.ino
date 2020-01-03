void Auto_IK(uint32_t forwardPercent, uint32_t angularPercent) {
  float leftPercentage;
  float rightPercentage;
  long leftMotorSpeed;
  long rightMotorSpeed;
  long leftMotorDirection;
  long rightMotorDirection;
  long maxPossibleSpeed;
  float speedCapPercent;
  //maybe do some sort of validation, but shouldn't have to because interrupts
  //convert from percentage of linear and angular velocity to percentage of left and right wheel
  leftPercentage = (forwardPercent - ((WheelSpacing / 2) * angularPercent)) / (1 + (WheelSpacing / 2));
  rightPercentage = (forwardPercent + ((WheelSpacing / 2) * angularPercent)) / (1 + (WheelSpacing / 2));
  //now we have a percentage from -1 to 1 for how fast each wheel should be going

  //calculate the output to the motor controller, which is from -127 to 127
  //first, we'll get a multiplier so we don't command our wheels to go faster than the max desired speed
  //to do that we need to calculate the maximum possible speed of the robot
  maxPossibleSpeed = MaxMotorRPM * (1 / MotorRevsPerWheelRev) * WheelDiameter * PI;
  //now get the speed cap as a percent of the maximum
  //speedCapPercent = MaxDesiredSpeed/maxPossibleSpeed;
  //until we know the constants, just output motor commands at half of max possible speed
  speedCapPercent = 0.5;
  //finally, get the values we're outputting to the motor controller by multiplying these all together
  leftMotorSpeed = leftPercentage * SABERTOOTHMAX * speedCapPercent;
  rightMotorSpeed = leftPercentage * SABERTOOTHMAX * speedCapPercent;
  //encode signs in seperate direction variable; remove from speed
  if (leftMotorSpeed < 0)
  {
    leftMotorDirection = B00000000;
  }
  else if (leftMotorSpeed >= 0)
  {
    leftMotorDirection = B00000001;
  }
  if (rightMotorSpeed < 0)
  {
    rightMotorDirection = B00000100;
  }
  else if (rightMotorSpeed >= 0)
  {
    rightMotorDirection = B00000101;
  }

  //put into the array for accessing elsewhere
  DesiredSpeeds[LEFTSPEED] = abs(leftMotorSpeed);
  DesiredSpeeds[RIGHTSPEED] = abs(rightMotorSpeed);
  DesiredSpeeds[LEFTDIRECTION] = leftMotorDirection;
  DesiredSpeeds[RIGHTDIRECTION] = rightMotorDirection;

  return;
}

//Refering math to: http://robotsforroboticists.com/drive-kinematics/
//! Units are all in meters and rad
//! Vehicle Center is assumed at center of mass.
void ik(uint32_t linearVelocity,uint32_t angularVelocity){
  desLeftMotorSpeed = (linearVelocity - angularVelocity * WheelSpacing / 2.0)/WheelDiameter;
  desRightMotorSpeed = (linearVelocity + angularVelocity * WheelSpacing / 2.0)/WheelDiameter;
  }

void fk(uint32_t leftMotorSpeed,uint32_t rightMotorSpeed){

}