//! 1. Communicate Estop Status to

//! 2. Feedup the motor speed from encoder_rpm

//! 3. subscribe commanded velocity and velocity (This should be in populate speed)

//! 4. IMU interface

//! 5. GPS interface

void ROSPublish(){
    ROSLastSent = millis();
    return;
}

//https://medium.com/@ericmaggard/building-an-autonomous-car-using-a-1-10th-scale-rc-car-part-3-6e1918813c75

void ROSESTOP(const std_msgs::int8_t& msg){
    if (msg.data==0){
        digitalWrite(ROSENABLEPIN,LOW);
        activateESTOP();
        State = STATE_ESTOP;
        debugOutput("ROS ESTOP REQUESTED!",2);
    }else{
        digitalWrite(ROSENABLEPIN,HIGH);
        State = NextState;
    }
}


void ROS_FEEDBACKESTOP(){
    return;
}