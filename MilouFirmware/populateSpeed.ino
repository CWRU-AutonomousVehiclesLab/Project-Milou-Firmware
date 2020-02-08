/**
 * rcPopulateSpeed populate speed using RC reciever pulses.
 * 
 * @param RCPulseData The RC Pulse Data that is populated by rcProcess.ino.
 * @param RCCenter, RCMin, RCMax Global parameter for how the RC is setup.
 * 
 * @return rcLinearSpeed,rcAngularSpeed writes globally
 * 
*/
void rcPopulateSpeed(){
    float ForwardPulse = RCPulseData[FWDPULSEDATA];
    float AngularPulse = RCPulseData[ANGPULSEDATA];
    
    rcLinearSpeed = (abs(ForwardPulse-RCCenter)<=30) ? 0.0 : map(ForwardPulse, RCMin, RCMax, -2.0, 2.0);
    rcAngularSpeed = (abs(AngularPulse-RCCenter)<=30) ? 0.0 :map(AngularPulse, RCMin, RCMax, -2.0, 2.0);
}


/**
 * actualObsSpeed populate the actual speed of the two side wheels robot
 * 
 * @param leftEncoderPos, rightEncoderPos The current observed left and right encoder tick.
 * @param lastLeftPos,lastRightPos The last observed left and right encoder tick.
 * @param ENCODERTICKPERREV, MotorRevsPerWheelRev, WheelDiameter The characteristics of the robot.
 * @param lastEncoderTime The time to obtain the dt value.
*/
void actualObsSpeed(){
    // Get the position first
    long currentLeftPos = leftEncoderPos;
    long currentRightPos = rightEncoderPos;

    // Get time
    long currentTime =  millis();
    
    obsLeftMotorSpeed = ((currentLeftPos-lastLeftPos)*1000.0/(ENCODERTICKPERREV*MotorRevsPerWheelRev*WheelDiameter))/(currentTime-lastEncoderTime);
    obsRightMotorSpeed = ((currentRightPos-lastRightPos)*1000.0/(ENCODERTICKPERREV*MotorRevsPerWheelRev*WheelDiameter))/(currentTime-lastEncoderTime);

    lastLeftPos = currentLeftPos;
    lastRightPos = currentRightPos;
    lastEncoderTime = currentTime;
}
