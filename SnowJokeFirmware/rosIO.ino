//! 2. Feedup the motor speed from encoder_rpm

//! 3. subscribe commanded velocity and velocity (This should be in populate speed)

//! 4. IMU interface

//! 5. GPS interface

//https://medium.com/@ericmaggard/building-an-autonomous-car-using-a-1-10th-scale-rc-car-part-3-6e1918813c75


char base_link[] = "/teensy";
char odom[] = "/base_link";

void ROSPublish(){
    ROSLastSent = millis();
    telemetryMessage.header.stamp = nh.now();
    telemetryMessage.header.frame_id = odom;
    // telemetryMessage.child_frame_id = base_link;
    // telemetryMessage.state = State;
    // telemetryMessage.RCSignal[0]=RCPulseData[FWDPULSEDATA];
    // telemetryMessage.RCSignal[1]=RCPulseData[ANGPULSEDATA];
    // telemetryMessage.RCSignal[2]=RCPulseData[ESTOPPULSEDATA];
    // telemetryMessage.rcInterpretedSpeed[0]=rcLinearSpeed;
    // telemetryMessage.rcInterpretedSpeed[1]=rcAngularSpeed;
    // telemetryMessage.autoInterpretedSpeed[0]=autoLinearVelocity;
    // telemetryMessage.autoInterpretedSpeed[1]=autoAngularVelocity;
    // telemetryMessage.desMotorSpeed[0]=desLeftMotorSpeed;
    // telemetryMessage.desMotorSpeed[1]=desRightMotorSpeed;
    // telemetryMessage.encoderRaw[0]=leftEncoderPos;
    // telemetryMessage.encoderRaw[1]=rightEncoderPos;
    // telemetryMessage.actualMotorSpeed[0]=obsLeftMotorSpeed;
    // telemetryMessage.actualMotorSpeed[1]=obsRightMotorSpeed;
    // telemetryMessage.pidedMotorSpeed[0]=pidedLeftMotorSpeed;
    // telemetryMessage.pidedMotorSpeed[1]=pidedRightMotorSpeed;
    // telemetryMessage.motorCommand[0]=cmdLeftMotorSpeed;
    // telemetryMessage.motorCommand[1]=cmdRightMotorSpeed;
    return;
}