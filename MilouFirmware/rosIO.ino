/**
 * TODO: 
 * 
*/


//! 3. subscribe commanded velocity and velocity (This should be in populate speed)

//! 4. IMU interface

//! 5. GPS interface

//https://medium.com/@ericmaggard/building-an-autonomous-car-using-a-1-10th-scale-rc-car-part-3-6e1918813c75

void ROSPublish(){
    ROSLastSent = millis();
    telemetryMessage.header.stamp = nh.now();
    telemetryMessage.state = State;
    telemetryMessage.RCSignal[0]=RCPulseData[FWDPULSEDATA];
    telemetryMessage.RCSignal[1]=RCPulseData[ANGPULSEDATA];
    telemetryMessage.RCSignal[2]=RCPulseData[ESTOPPULSEDATA];
    telemetryMessage.rcInterpretedSpeed[0]=rcLinearSpeed;
    telemetryMessage.rcInterpretedSpeed[1]=rcAngularSpeed;
    telemetryMessage.autoInterpretedSpeed[0]=autoLinearVelocity;
    telemetryMessage.autoInterpretedSpeed[1]=autoAngularVelocity;
    telemetryMessage.desMotorSpeed[0]=desLeftMotorSpeed;
    telemetryMessage.desMotorSpeed[1]=desRightMotorSpeed;
    telemetryMessage.encoderRaw[0]=leftEncoderPos;
    telemetryMessage.encoderRaw[1]=rightEncoderPos;
    telemetryMessage.actualMotorSpeed[0]=obsLeftMotorSpeed;
    telemetryMessage.actualMotorSpeed[1]=obsRightMotorSpeed;
    telemetryMessage.pidedMotorSpeed[0]=pidedLeftMotorSpeed;
    telemetryMessage.pidedMotorSpeed[1]=pidedRightMotorSpeed;
    telemetryMessage.motorCommand[0]=cmdLeftMotorSpeed;
    telemetryMessage.motorCommand[1]=cmdRightMotorSpeed;
    telemetryMessage.speedError[0]=left_speed_error;
    telemetryMessage.speedError[1]=right_speed_error;
    telemetryMessage.integralError[0]=left_speed_error_sum;
    telemetryMessage.integralError[1]=right_speed_error_sum;
    telemetryMessage.derivitiveError[0]=left_speed_d;
    telemetryMessage.derivitiveError[1]=right_speed_d;
    return;
}