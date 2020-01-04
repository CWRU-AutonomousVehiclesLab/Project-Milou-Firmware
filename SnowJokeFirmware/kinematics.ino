//Refering math to: http://robotsforroboticists.com/drive-kinematics/
//! Units are all in meters and rad
//! Vehicle Center is assumed at center of mass.
void ik(float linearVelocity,float angularVelocity){
  /*
  # trim the desired commands such that they are within the limits:
        msg_car_cmd.v = self.trim(msg_car_cmd.v,
                                  low=-self.parameters['~v_max'], high=self.parameters['~v_max'])
        msg_car_cmd.omega = self.trim(msg_car_cmd.omega,
                                      low=-self.parameters['~omega_max'], high=self.parameters['~omega_max'])

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim']) / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim']) / k_l

        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.parameters['~baseline']) / self.parameters['~radius']
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.parameters['~baseline']) / self.parameters['~radius']

        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r, -self.parameters['~limit'], self.parameters['~limit'])
        u_l_limited = self.trim(u_l, -self.parameters['~limit'], self.parameters['~limit'])
  */
  float LeftMotorSpeed = (linearVelocity - angularVelocity * WheelSpacing / 2.0)/WheelDiameter;
  float RightMotorSpeed = (linearVelocity + angularVelocity * WheelSpacing / 2.0)/WheelDiameter;

  desLeftMotorSpeed = LeftMotorSpeed * k_l;
  desRightMotorSpeed = RightMotorSpeed * k_r;
  }

void fk(float leftMotorSpeed,float rightMotorSpeed){
/*
       # FORWARD KINEMATICS PART

        # Conversion from motor duty to motor rotation rate
        omega_r = msg_wheels_cmd.vel_right / k_r_inv
        omega_l = msg_wheels_cmd.vel_left / k_l_inv

        # Compute linear and angular velocity of the platform
        v = (self.parameters['~radius'] * omega_r + self.parameters['~radius'] * omega_l) / 2.0
        omega = (self.parameters['~radius'] * omega_r - self.parameters['~radius'] * omega_l) / \
                self.parameters['~baseline']
*/  
    leftMotorSpeed = leftMotorSpeed / k_l;
    rightMotorSpeed = rightMotorSpeed / k_r;

    obsLinearVelocity = (WheelDiameter * rightMotorSpeed +WheelDiameter * leftMotorSpeed) / 2.0;
    obsAngularVelocity =(WheelDiameter * rightMotorSpeed-WheelDiameter * leftMotorSpeed) / WheelSpacing;

}
