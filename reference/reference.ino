//Note to self: add error checking for cases where inputs are non numerical values (Raspberry pi returning NaN)
        //Implemented, now in testing
//Note to self: alignment may be critical when approaching waypoints. Pay close attention when testing.
//Note to self: bug currently exists that causes robot to consistenly overshoot angle when turning despite recognizing that it is alligned - bandaid was to slow down turning, but not appropriate long term
	//Bug potentially on line 689: if (heading_offset >= 0), but this theoretically should only be hit IFF the heading is outside the threshold
//Note to self: add timeouts to things to prevent runaway robot

/*CWRUCutter/SNOWJoke Papilio ZPUino Core
Consists of a scheduler and various sub-functions run by the scheduler

The scheduler checks how much time has passed since a function has run.
If the time is greater than the desired threshold, the function is run again and the counter is reset
*/

//Include math library and define values for pi and 2*pi to be used in localization
    #include <math.h>
    #define PI 3.1415926535897932384626433832795
    #define TWO_PI 6.283185307179586476925286766559


// Global constants

    //Constants defining how often functions should be run
    int k_RCPolling;
    int k_Control;
    int k_MotorCommandSpeed;
    int k_SwitchRead;   
    int k_CamRead;
    int k_debug;
    int k_RasPiRead;
    int k_RasPiTimeoutThreshold;

    //variables containing the time each function was last called
    
    int RCPollingLastTime;
    int ControlLoopLastTime = 0; // Must be set to zero for proper initialization - used every time the control loop runs for internal calculations.
    int MotorCommandLastSent;
    int LastSwitchReadTime;
    int LastDebug;
    int LastRasPiReadTime;

//RC Polling variables

    //Define communication on wishbone slot 6 for PWM inputs
    #define RCINPUT IO_SLOT(6)
    #define PWMIN(x) REGISTER(RCINPUT,x)

    //Wheel spacing constant
    float b = 0.77; //Wheel spacing (center to center) in meters
	
    //Constants defining RC maximum, minimum, center, time-out, and  Threshold (Positive pulse values in clock cycles)
    int k_RCCenter = 144500;
    int k_RCMax = 193000;
    int k_RCMin = 96000;
    int k_RCTimeout = 1930000;
    int k_EStopThreshold = 180000;
    
    //Variables pulled from the RC input registers
    int ForwardPulse;
    int ForwardCycle;
    int AngularPulse;
    int AngularCycle;
    int EStopPulse;
    int EStopCycle;
    
    //Time-out checking
    int ConsecutiveRCInvalidCheck = 0; //Used to disable RC control if we receive time-outs from any two consecutive inputs
    int ConsecutiveRCValidCheck = 0; //Checks to ensure we have five consecutive valid commands before resuming control

    int TimeLastRasPiCommandRecieved = 0; // Used to check the last time a command was received from the computer and disable auto control if past a threshold
    //Calculated motor commands and related variables
    
    int RCForwardCommand = 0;//negative is reverse, positive is forward
    int RCAngularCommand = 0;//negative is left, positive is right
    int SignedRCLeftSpeed = 0;
    int SignedRCRightSpeed = 0;
	
    //Used for RC Control
    long LeftTicksPerSecondDesiredRC;
    long RightTicksPerSecondDesiredRC;
    int k_MaxRCDesiredTicksPerSecond = 38500; //(1.5 m/s) * (1 rev/0.957557m) * (24 motor rev/wheel rev) * (1024 ticks/motor rev)
    
//Control variables
    //Current estop state variable
    boolean Estopstatus = false;
    
    //Define wishbone bus communication to read encoders
    #define EncoderInput (IO_SLOT (5)) //read inputs from wishbone slot 5
    #define EncoderRegister(x) REGISTER(EncoderInput,x) //set the inputs to a register

	//Control constants

        float k_TicksToMotorSpeedMultiplier = 0.00329870;// Max motor command of 127 divided by max desired ticks per second of 38500

	
	//Speed Control Variables
        long SpeedCheckTime = 0;
        long SpeedCheckTimeOld = 0;
	long LeftEncoderTicks = 0;
	long RightEncoderTicks = 0;
	long LeftEncoderTicksOld = 0;
	long RightEncoderTicksOld = 0;

	long LeftTicksPerSecondEncoder = 0;
	long RightTicksPerSecondEncoder = 0;
	long LeftTicksPerSecondEncoderOld = 0;
	long RightTicksPerSecondEncoderOld = 0;

        long LeftWheelSpeed = 0;
        long RightWheelSpeed = 0;
        //define constants used for converting wheel speed in ticks/second to cm/s
        float LeftSpeedConversion = 0.0042396; //(1/23587.15 ticks/rev) * 100 cm/rev = cm/tick
        float RightSpeedConversion = 0.0041714; //(1/24068.58 ticks/rev) * 100.4 cm/rev = cm/tick

        //PID variables
	long LeftControlTerm;
	long RightControlTerm;

        long lastPIDaction = 0;
        //PID variables
        float left_speed_error = 0;
        float right_speed_error = 0;
        float left_speed_error_old = 0;
        float right_speed_error_old = 0;
        float left_speed_error_sum = 0;
        float right_speed_error_sum = 0;

//PC communication Serial variables
        HardwareSerial RasPiSerial(9);
        //deprecated variables left in  to avoid breaking anything, shouldn't be used right now
        long lastxrequest = 0;
        long lastyrequest = 0;
        long lastphirequest = 0;
        long lastheadingcheck = 0;
        long lastnavigationaction = 0;
        int lastposrequest = 0;
        //left/right wheel speed request variables
        long lastleftspeedrequest = 0;
        long lastrightspeedrequest = 0;
        float desiredleftspeed = 0; //initialize desired left wheel speed for use in PID
        float desiredrightspeed = 0; //initialize desired right wheel speed for use in PID
        int lastspeedrequest = 0;
        int databuffer[8];
        int msgID;
        boolean ChecksumStatus = false;
        
        //Variable for checking path reset
        long lastresetcommand = 0;

// Autonomous mode variables and path plan

        //Position variables
        float x;
        float y;
        float phi;

        boolean isrobotweighted = true; //variable used to make robot more/less aggressive depending on how much weight is on it
        //Test path plans:
        //For use in lab (down and back, best used as RC debug)
        //float xtarget[2] = {0.8, 0.5};
        //float ytarget[2] = {2.7, 0.1};
        
        //For use in hallway (simple path)
        //float xtarget[6] = {1.25, 2.5,  2.25, 1, 2.25, 1};
        //float ytarget[6] = {2.25, 2.25, 0,    0, 2.25, 0};
        
        //For use with short single I path
        //float xtarget[3] = {2.00, 2.00, 2.00};
        //float ytarget[3] = {2.00, 8.00, 2.00};
        
        //For use with triple I path
        //float xtarget[13] = {2.50,  2.50,  4.50, 4.50, 3.50,  3.50,  4.50, 4.50, 2.50,  2.50,  4.50, 4.50, 2.50};
        //float ytarget[13] = {2.00, 14.00, 14.00, 2.00, 2.00, 14.00, 14.00, 2.00, 2.00, 14.00, 14.00, 2.00, 2.00};

        //For logging outdoor rectangle path
        float xtarget[5] = {2.50,  2.50,  4.50, 4.50, 2.50};
        float ytarget[5] = {2.00, 14.00, 14.00, 2.00, 2.00};
        
        //For moving to field center
        //float xtarget[2] = {3.50, 3.50};
        //float ytarget[2] = {2.00, 7.50};
        
        
        int waypointindex;
        boolean headingaligned;
        boolean routecompleted;
        const float k_AlignmentThresholdLower = 0.05; //known working at 0.075
        const float k_AlignmentThresholdUpper = 0.5;
        const float k_PositionThreshold = 0.6; // known working at 0.5
        boolean requestflag = false;
        
//Motor commands
	int LeftMotorSpeed;
	int RightMotorSpeed;
	int LeftMotorDirection;
	int RightMotorDirection;

//Sabertooth Motor controller command variables

    //Set up serial communication over the wishbone bus for the Sabertooth
    HardwareSerial SabertoothSerial(10);

    //Drive signal variables
    int SabertoothAddress;
    
    int Mask;
    int Checksum0;
    int Checksum1;
    int Checksum2;
    
    //zero value since serial write doesn't like zeros directly in
    int donothing = 0;
	
//Switch state variables
        boolean Switch_A;
        boolean Switch_B;
        boolean Switch_C;
        boolean Switch_D;
        boolean Physical_Estop;        
        
        boolean autonomous_enable;

// placeholder variables for forward speed in auto mode
      int RasPiDataReceived[64];
      int AutoLeftSpeed;
      int AutoRightSpeed;


// sloppy global variable to force things to work for side allignment
  int ForwardSpeed;

void setup()
{
  SabertoothSerial.begin(9600);
  RasPiSerial.begin(115200);
  Serial.begin(115200);

  //Initialize variables used for navigation/localization, and pad to false/zero values to avoid errors
  waypointindex = 0;
  headingaligned = false;
  routecompleted = false;
  x = 0;
  y = 0;
  phi = 0;

  //set constants to set number of times to run loops per second
  //values are equal to 1000/frequency to get the number of milliseconds to wait to run
  //in the case of loops that are intended to run more frequently than 100hz we take 1,000,000/frequency and use the micros instead of the millis function
  k_SwitchRead = 5; // 200 hz
  k_RCPolling = 20; // 50hz
  k_RasPiRead = 100; // 10hz
//k_Control and k_MotorCommandSpeed changed from 10 to 50 to move from 100 hz to 50 hz in a debug test - trying to fix overshoot on turns
  k_Control = 20; // 50 hz
  k_MotorCommandSpeed = 20; // 50 hz
  k_debug = 200; // 5 hz debug output time
  k_RasPiTimeoutThreshold = 100; // 100 ms timeout
  
  
  
  //Setting values we will need to repeatedly use for motor control
  SabertoothAddress = B10000010;                                   // set Address to 130
  LeftMotorDirection = B00000001;                                     // set Motor 1 backwards
  LeftMotorSpeed = B00000000;                                         // set Motor 1 speed to 0 to start
  RightMotorDirection = B00000100;                                     // set Motor 2 forwards
  RightMotorSpeed = B00000000;                                         // set Motor 2 speed to 0 to start
  Checksum1 = (SabertoothAddress + LeftMotorDirection + LeftMotorSpeed); // Check other Motor 1 commands against this
  Checksum2 = (SabertoothAddress + RightMotorDirection + RightMotorSpeed); // Check other Motor 2 commands against this
  Mask = B01111111; 
 
     // set baud rate to 9600
  SabertoothSerial.write(SabertoothAddress);
  SabertoothSerial.write(B00001111);
  SabertoothSerial.write(B00000010);
  Checksum0 = (SabertoothAddress + B00000010 + B00001111); // Check other Motor 1 commands against this
  SabertoothSerial.write(Checksum0 & Mask);
  
  // set timeout to 200ms
  SabertoothSerial.write(SabertoothAddress);
  SabertoothSerial.write(B00001110);
  SabertoothSerial.write(B00000010);
  Checksum0 = (SabertoothAddress + B00000010 + B00001110); // Check other Motor 1 commands against this
  SabertoothSerial.write(Checksum0 & Mask);
  
  pinMode(WCL7, OUTPUT); // set up pin for software enable as an output
  
  // set up pins connected to switches as inputs
  pinMode(WBH3, INPUT);
  pinMode(WBH4, INPUT);
  pinMode(WBH5, INPUT);
  pinMode(WBH6, INPUT);
  pinMode(WBH7, INPUT);
  
  //Set estop status pin as input
  pinMode(WCH2, INPUT);

  //Delay 10 seconds to allow for initialization of other components
  //Temp set to 2 seconds for testing
  delay(2000);
  
  //run all of the various sub-functions and establish an initial run time
  RCPolling();
  ControlLoop();
  SabertoothMotorCommandLoop();
  SwitchRead();

}

void loop()
{
  // check to see if enough time has passed to run the Safety State Machine loop
  if ((millis() - LastSwitchReadTime) >= (k_SwitchRead))
  {
    SwitchRead();
  }
  
  // check to see if enough time has passed to run the RC polling loop
  if ((millis() - RCPollingLastTime) >= (k_RCPolling))
  {
    RCPolling();
  }
  
//deprecated functions commented out, but left in for reference
/*
  //If enough time has elapsed since the last x request, no request flag has been raised, and current x data is older than y and phi data, request x 
  if ( (millis() - lastxrequest >= 10) && (requestflag == false) && ( (lastxrequest <= lastyrequest) && (lastxrequest <= lastphirequest) ) )
  {
    xrequest();
  }
  
  //If enough time has elapsed since the last y request, no request flag has been raised, and current y data is older than x and phi data, request y
  else if ( (millis() - lastyrequest >= 10) && (requestflag == false) && ( (lastyrequest < lastxrequest) && (lastyrequest <= lastphirequest) ) )
  {
    yrequest();
  }  
  
  //If enough time has elapsed since the last phi request, no request flag has been raised, and current phi data is older than x and y data, request phi
  else if ( (millis() - lastphirequest >= 10) && (requestflag == false) && ( (lastphirequest < lastxrequest) && (lastphirequest < lastyrequest) ) )
  {
    phirequest();
  }
  
  //If enough time has passed since the last position request was sent to the raspberry pi, attempt to read returned data from the buffer
  if ( (millis() - lastposrequest >= 20) && (requestflag == true) )
  {
    getposdata();
  }
*/

  //If mode reset is set to run:
  if (Switch_B == 1)
  {
      //attempt to get desired left/right wheel speeds for autonomous operation
    
      //If enough time has elapsed since the last x request, no request flag has been raised, and current x data is older than y and phi data, request x 
      if ( (millis() - lastleftspeedrequest >= 10) && (requestflag == false) && (lastleftspeedrequest <= lastrightspeedrequest ) )
      {
        leftspeedrequest();
      }
      
      //If enough time has elapsed since the last y request, no request flag has been raised, and current y data is older than x and phi data, request y
      else if ( (millis() - lastrightspeedrequest >= 10) && (requestflag == false) && (lastrightspeedrequest <= lastleftspeedrequest ) )
      {
        rightspeedrequest();
      }  
      //update with returned data
      if ( (millis() - lastspeedrequest >= 20) && (requestflag == true) )
      {
        getspeeddata();
      }
  }
  //If path reset switch is set to reset path, stop motors and send reset command computer
  else
  {
      if ( (millis() - lastresetcommand >= 20) )
      {
        pathreset();
      }
  }
  
  // check to see if enough time has passed to run the control loop
  if ((millis() - ControlLoopLastTime) >= (k_Control))
  {
    ControlLoop();
  }
  
  // check to see if enough time has passed to run motor command loop and if the robot is in
  if (((millis() - MotorCommandLastSent) >= (k_MotorCommandSpeed)) )
  {
    SabertoothMotorCommandLoop();    
  }


  if ((Switch_C == 0) && (millis() - LastDebug) >= (k_debug)) // require switch C to be in debug mode, as being in debug mode can interfere with operation timings
  {
       DebugPrint();
    //Serial.print("debug mode");
    //Serial.println();
  }

}

//--------------------------------------------------------------------------------------------------------------
//Subfunctions handling all of the control loops
//These should be retooled to use structs once the general scheduler structure is complete.
//--------------------------------------------------------------------------------------------------------------

//RCPolling checks the RC input signals at 10 hz.
//It returns a speed command to be sent to the control loop.
//-------------------------------------------------------
void RCPolling()
{
  RCPollingLastTime = millis();
  
  //get RC signal from registers
  ForwardPulse = (int)PWMIN(0);
  ForwardCycle = (int)PWMIN(1);
  AngularPulse = (int)PWMIN(2);
  AngularCycle = (int)PWMIN(3);
  EStopPulse = (int)PWMIN(4);
  EStopCycle = (int)PWMIN(5);
  
  //PWM input validity check. If signals are invalid 10 times in a row disable drive commands until they are valid 20 times in a row.

  //Check if any signals are invalid. If a signal is invalid increment the check by one. 
  //If two signals have been invalid in a row set the consecutive check to zero.
  if ((ForwardCycle == k_RCTimeout) || (AngularCycle == k_RCTimeout))
  {
    ConsecutiveRCInvalidCheck++;
    if (ConsecutiveRCInvalidCheck >= 10)
    {
      ConsecutiveRCValidCheck = 0; 
    }
  }
  
  //Check if the all current signals are valid. If all signals are valid 
  if ((ForwardCycle != k_RCTimeout) || (AngularCycle != k_RCTimeout)  ){
      ConsecutiveRCValidCheck++;
      ConsecutiveRCInvalidCheck = 0;
  }
  
  //Check to ensure current command is valid by checking that commands are within minimum and maximum values as well as ensuring we do not have a timeout case
  if (((ForwardCycle != k_RCTimeout) && (ForwardPulse >= k_RCMin) && (ForwardPulse <= k_RCMax)) 
  && ((AngularCycle != k_RCTimeout) && (AngularPulse >= k_RCMin) && (AngularPulse <= k_RCMax)) 
  )
  
  {
        
	//if the RC commands have been valid for more than five checks run the RC control loop
    if ((ConsecutiveRCValidCheck >= 20) )
    {
      //find the distance from the center for the forward and angular command and convert it into a signed left and right speed
      digitalWrite(WCL7, HIGH); // set software enable on
      RCForwardCommand = (k_RCCenter - ForwardPulse);
      RCAngularCommand = (k_RCCenter - AngularPulse);

// temp if for testing "safe" autonomous

      SignedRCLeftSpeed = (RCForwardCommand - 1.5 * b * RCAngularCommand);
      SignedRCRightSpeed = (RCForwardCommand + 1.5 * b * RCAngularCommand);

    
      //convert signed left and right wheel speeds into desired ticks per second
      
      //make adjustments for left motor/wheel
      // note that the multiplication by 2 and division by 2 are to handle an edge condition where simultaneously turning while going full speed causes a rollover turning the variable negative
      LeftTicksPerSecondDesiredRC = 2 * (((k_MaxRCDesiredTicksPerSecond/2) * SignedRCLeftSpeed) / (k_RCMax - k_RCCenter));
      
      //Make adjustments for the right motor/wheel
      RightTicksPerSecondDesiredRC = 2 * (((k_MaxRCDesiredTicksPerSecond/2) * SignedRCRightSpeed) / (k_RCMax - k_RCCenter));


    }
    
    // if rc commands have not been valid for five checks disable the software enable for the safety chain and set commands to 0
    else if(ConsecutiveRCValidCheck < 5)
    {
        digitalWrite(WCL7, LOW); // set software enable off
	LeftTicksPerSecondDesiredRC = 0;
	RightTicksPerSecondDesiredRC = 0;
    }  
  
  }
}

void ControlLoop()
{
    ControlLoopLastTime = millis(); // required for loop timing, do not touch. Must not be removed.

    if ((Switch_A == 0) && (Physical_Estop == 1)) // E-stop 1 means switch is down and robot is disabled
    {
      autonomous_enable = 1;
    }
    else if (Switch_A == 1)
    {
      autonomous_enable = 0;
    }

   
    // switch modes based on autonomous state
    if (autonomous_enable == 0)
    {
//Serial.println("RC Mode active");
      RCControl();    // parse futaba rc pwm commands from rcpolling and generates MotorDirection and MotorSpeed   

      //When not in autonomous PID, set error variables to 0 to prevent sudden jerk forward when moving to autonomous
      left_speed_error = 0;
      left_speed_error_old = 0;
      left_speed_error_sum = 0;
      right_speed_error = 0;
      right_speed_error_old = 0;
      right_speed_error_sum = 0;
  

    }
	
    else if (autonomous_enable == 1)
    {
//      Serial.println("Auto Mode active");
      //AutonomousNavigation();
      AutonomousPIDPassthrough();
    }
    
}  

void SabertoothMotorCommandLoop()
{
  MotorCommandLastSent = millis();
  
  //If robot is estopped, send a braking command to the motor controller as a secondary safety stop
  if (Estopstatus == false)
  {
    LeftMotorSpeed = 0;
    RightMotorSpeed = 0;
    //When E-stopped, set PID error variables to 0 to wipe accumulated error to prevent sudden acceleration on start
    left_speed_error = 0;
    left_speed_error_old = 0;
    left_speed_error_sum = 0;
    right_speed_error = 0;
    right_speed_error_old = 0;
    right_speed_error_sum = 0;
  }
 
 //Motor Control Code  
// send packet to motor 1
  if (LeftMotorSpeed <= 127){
    Checksum1 = (SabertoothAddress + LeftMotorDirection + LeftMotorSpeed);
    SabertoothSerial.write(SabertoothAddress);
    SabertoothSerial.write(LeftMotorDirection);
    SabertoothSerial.write(LeftMotorSpeed);
    SabertoothSerial.write(Checksum1 & Mask);
  }

// send packet to motor 2
  if (RightMotorSpeed <= 127){
  Checksum2 = (SabertoothAddress + RightMotorDirection + RightMotorSpeed);
  SabertoothSerial.write(SabertoothAddress);
  SabertoothSerial.write(RightMotorDirection);
  SabertoothSerial.write(RightMotorSpeed);
  SabertoothSerial.write(Checksum2 & Mask);
  }

}

void SwitchRead()
{
  LastSwitchReadTime = millis();
  Switch_A = digitalRead(WBH7);
  Switch_B = digitalRead(WBH6);
  Switch_C = digitalRead(WBH5);
  Switch_D = digitalRead(WBH4);
  Physical_Estop = digitalRead(WBH3);
  Estopstatus = digitalRead(WCH2); 
}


void RCControl()
{
    // rc control terms generated by RCPolling()    
    LeftControlTerm = k_TicksToMotorSpeedMultiplier * LeftTicksPerSecondDesiredRC;
    RightControlTerm = k_TicksToMotorSpeedMultiplier * RightTicksPerSecondDesiredRC;
    
    
    //Check signs for direction, remove sign from drive signal
    if (LeftControlTerm < 0)
    {
    LeftMotorDirection = B00000000;
    LeftMotorSpeed = -1 * LeftControlTerm;
    }
    else if (LeftControlTerm >= 0)
    {
    LeftMotorDirection = B00000001;
    LeftMotorSpeed = LeftControlTerm;
    }
    
    if (RightControlTerm < 0)
    {
    RightMotorDirection = B00000100;
    RightMotorSpeed = -1 * RightControlTerm;
    }
    else if (RightControlTerm >= 0)
    {
    RightMotorDirection = B00000101;
    RightMotorSpeed = RightControlTerm;
    }
    
    //Ensure motor speeds are equal to or below 127 and ensure turning is possible
    
    if ((LeftMotorSpeed > 127) || (RightMotorSpeed > 127))
    {
        if (LeftMotorSpeed > RightMotorSpeed)
        {
          RightMotorSpeed = ((RightMotorSpeed * 127)/LeftMotorSpeed); //Keep the same speed ratio but adjust maximum to 127
          LeftMotorSpeed = 127;
        }
        else if (LeftMotorSpeed < RightMotorSpeed)
        {
          LeftMotorSpeed = ((LeftMotorSpeed * 127) / RightMotorSpeed);
          RightMotorSpeed = 127;
        }
        else if (LeftMotorSpeed == RightMotorSpeed)
        {
          LeftMotorSpeed = 127;
          RightMotorSpeed = 127;
        }
    }
    
    // save current speed for next run
    // currently deprecated 2016/1/29
    LeftTicksPerSecondEncoderOld = LeftTicksPerSecondEncoder;
    RightTicksPerSecondEncoderOld = RightTicksPerSecondEncoder;  
}


void DebugPrint()
{
  LastDebug = millis();
  
  //Print current robot mode
  if (autonomous_enable == 0)
    {
      Serial.println("Robot emanuel");
     }       
  else if (autonomous_enable == 1)
     {
      Serial.println("Robert Otto");
    }
 /*   
  //Print last known robot coordinates
  Serial.print("[X,Y] = [");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println("]");
  
  //Print last known robot heading
  Serial.print("Phi = ");
  Serial.println(phi);
*/

  //Print left and right encoder values
  Serial.print("Right encoder: ");
  Serial.print(RightEncoderTicks);
  Serial.print(" Left encoder: ");
  Serial.println(LeftEncoderTicks);
  
  //Print left and right speed (cm per second)

  Serial.print("Right wheel speed: ");
  Serial.print(RightWheelSpeed);
  Serial.print(" Left Wheel speed: ");
  Serial.println(LeftWheelSpeed);  
  
  //Print left and right target speed (cm per second)

  Serial.print("Target right wheel speed: ");
  Serial.print(desiredrightspeed);
  Serial.print(" Target left wheel speed: ");
  Serial.println(desiredleftspeed);   
  
  //Print estop status
  if(Estopstatus == true)
  {
    Serial.println("Robot Live");
  }
  else
  {
    Serial.println("Robot Disabled");
  }
  
  Serial.print("Current time (millis) : ");
  Serial.println(millis());
  Serial.print("Current time (micros) : ");
  Serial.println(micros());  
  
  //Print an empty line to make it easier to differentiate between blocks of information
  Serial.println();
  

} 

//Test function for PID, assumes robot is getting input commands in the form of left and right wheel speeds
//Temporarily spoofing inputs by setting wheel speed to a constant
void AutonomousPIDPassthrough()
{
  lastPIDaction = millis();
  
  //debug values used for testing fixed speeds, comment out when not in use
  //desiredrightspeed = -30;
  //desiredleftspeed = 30;
  
  //update last time run and encoder counts for wheel speed estimation
  SpeedCheckTimeOld = SpeedCheckTime;
  SpeedCheckTime = millis();
  
  LeftEncoderTicksOld = LeftEncoderTicks;
  RightEncoderTicksOld = RightEncoderTicks;
  LeftEncoderTicks = EncoderRegister(1);
  RightEncoderTicks = EncoderRegister(0);
  
  int dt = SpeedCheckTime - SpeedCheckTimeOld;
  if (dt >= 0)
  {
    LeftTicksPerSecondEncoder = ((LeftEncoderTicks - LeftEncoderTicksOld) * 1000)/dt;
    RightTicksPerSecondEncoder = ((RightEncoderTicks - RightEncoderTicksOld) * 1000) / dt;
  
    LeftWheelSpeed = LeftTicksPerSecondEncoder * LeftSpeedConversion; //left wheel speed in cm/sec
    RightWheelSpeed = RightTicksPerSecondEncoder * RightSpeedConversion; //right wheel sped in cm/sec
  }
  
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
  
}

//Navigation function assuming connected computer is performing localization and returning X-Y coordinates and robot heading (phi)
void AutonomousNavigation()
{
  lastnavigationaction = millis();
  // Determine desired robot heading
  float phi_desired = atan2(ytarget[waypointindex]-y, xtarget[waypointindex]-x);
  //Determine heading deviation
  float heading_offset = phi_desired - phi;
  //Handle fringe conditions by correcting to be between pi and -pi
  if (heading_offset > PI)
  {
    heading_offset -= TWO_PI;
  }
  else if (heading_offset <= -PI)
  {
    heading_offset += TWO_PI;
  }
  
  //If the route has not been completed, we drive!
  if (routecompleted == false)
  {
//Debug text printed here 
//    Serial.println();  
//    Serial.print("Desired heading = ");
//    Serial.println(phi_desired);
//    Serial.print("Current heading = ");
//    Serial.println(phi);
//    Serial.print("Heading offset = ");
//    Serial.println(heading_offset);
//    Serial.print("headingaligned = ");
//    Serial.println(headingaligned);
    
    //If alignment flag raised, make the robot go straight using error as feedback until within a predefined threshold of target position
    if(headingaligned == true)
    {
      //Serial.println("Robot has been aligned");
      
      // If at desired target location, increment waypoint counter and pause robot operation
      if ( ( (xtarget[waypointindex]-x <= k_PositionThreshold) && (x - xtarget[waypointindex] <=  k_PositionThreshold) ) &&
         ( (ytarget[waypointindex]-y <= k_PositionThreshold) && (y - ytarget[waypointindex] <= k_PositionThreshold) ) )
      {
        
        //Serial.println("Robot within threshold of target position");
        //Since target position has been reached, change to the next target
        waypointindex++;
        headingaligned = false;
        //Temporarily pause operation to prevent overshoot
        LeftMotorSpeed = 0; 
        RightMotorSpeed = 0;
        
        //Check waypointindex here against length of xtarget or ytarget
        //If waypointindex is greater than the length, we have executed all steps
        //And should halt operation
        if ( (sizeof(xtarget)/sizeof(float)) <= waypointindex )
        {
          routecompleted = true;
          //Serial.println("Route completed");
          //Stop motion here
          LeftMotorSpeed = 0; 
          RightMotorSpeed = 0;
        }
                 
      }
      else
      {
        //Double check alignment, and correct heading if misaligned
        if( (heading_offset >= k_AlignmentThresholdUpper) || (heading_offset <= (-1 * k_AlignmentThresholdUpper)) )
        {
          headingaligned = false;
          //Temporarily stop motion here
          LeftMotorSpeed = 0; 
          RightMotorSpeed = 0;
        }
        else
        {
//Write forward navigation code here          
//          Serial.println("We're facing the right direction, go straight!");
          //go straight code here, speeds are magic numbers (known working at forward 40 and turn offsets at 50) without weights
          //check if the robot expects to have weights on it and adjust speed based on robot weight
          int forwardspeed;
          if(isrobotweighted)
          {
            forwardspeed = 64; //Test value for use with weights on robot
          }
          else
          {
            forwardspeed = 55; //Approximate number with no weights
          }

          int AutoLeftSpeed = forwardspeed - (heading_offset * 60);
          int AutoRightSpeed = forwardspeed + (heading_offset * 60);
          //Serial.print("leftspeed = ");
          //Serial.println(AutoLeftSpeed);
          //Serial.print("rightspeed = ");
          //Serial.println(AutoRightSpeed);
//Start copy-pasted code
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
          
          //Ensure motor speeds are equal to or below 127 and ensure turning is possible
          
          if ((LeftMotorSpeed > 127) || (RightMotorSpeed > 127))
          {
            if (LeftMotorSpeed > RightMotorSpeed)
            {
              RightMotorSpeed = ((RightMotorSpeed * 127)/LeftMotorSpeed); //Keep the same speed ratio but adjust maximum to 127
              LeftMotorSpeed = 127;
            }
            else if (LeftMotorSpeed < RightMotorSpeed)
            {
              LeftMotorSpeed = ((LeftMotorSpeed * 127) / RightMotorSpeed);
              RightMotorSpeed = 127;
            }
            else if (LeftMotorSpeed == RightMotorSpeed)
            {
              LeftMotorSpeed = 127;
              RightMotorSpeed = 127;
            }
        
          }                 
//End copy-pasted code
        }
  
      }
        
    }
    
    //If alignment flag lowered, check alignment and turn in place if not aligned
    else
    {
      //If magnitude of heading offset is under threshold, set alignment flag and temporarily stop
      if( (heading_offset <= k_AlignmentThresholdLower) && (heading_offset >= (-1 * k_AlignmentThresholdLower)) )
      {
        
        headingaligned = true;
        //Stop robot here before changing modes
        LeftMotorSpeed = 0; 
        RightMotorSpeed = 0;
//        Serial.println("Robot sucessfully aligned");

      }
      //If not thresholded, turn robot based on sign
      else
      {
        if (heading_offset >= 0)
        {
          //Turn left here, motor speeds hard coded
//          Serial.println("Robot misaligned, turn left");
//          LeftMotorSpeed = 30; 
//          RightMotorSpeed = 30;
//Attempting to change turn speed based on heading error instead of constant turn speed
          //
          if(isrobotweighted)
          {
            //magic numbers for robot turn speed with weight, subject to change
            LeftMotorSpeed = 40 + int(heading_offset * 28); 
            RightMotorSpeed = 40 + int(heading_offset * 28);            
          }
          else
          {
            //previously working values for turns without any weight
            LeftMotorSpeed = 24 + int(heading_offset * 18); 
            RightMotorSpeed = 24 + int(heading_offset * 18);            
          }


          LeftMotorDirection = B00000000; // left motor turns in reverse
          RightMotorDirection = B00000101; // right motor turns forward
        }
        else
        {
          //Turn right here, motor speeds hard coded
//          Serial.println("Robot misaligned, turn right");
//          LeftMotorSpeed = 30; 
//          RightMotorSpeed = 30;
//Attempting to change turn speed based on heading error instead of constant turn speed
          if(isrobotweighted)
          {
            //magic numbers for robot turn speed with weight, subject to change
            LeftMotorSpeed = 40 + int(heading_offset * (-28)); 
            RightMotorSpeed = 40 + int(heading_offset * (-28));  
          }
          else
          {
            //previously working values for turns without any weight            
            LeftMotorSpeed = 24 + int(heading_offset * (-18)); 
            RightMotorSpeed = 24 + int(heading_offset * (-18));            
          }

          
          LeftMotorDirection = B00000001; // left motor turns forward
          RightMotorDirection = B00000100; // right motor turns in reverse
        }
      }
      
    }
  
  }
  //If route is completed, stop robot
  else
  {
    //Stop motion here in case it wasn't stopped previously
    LeftMotorSpeed = 0; 
    RightMotorSpeed = 0;
    //Serial.println();
    //Serial.println("Route completed");
  }
}

void xrequest()
{
  lastxrequest = millis();
  lastposrequest = millis();
  requestflag = true;
  RasPiSerial.print("x");
}

void yrequest()
{
  lastyrequest = millis();
  lastposrequest = millis();
  requestflag = true;
  RasPiSerial.print("y");
}

void phirequest()
{
  lastphirequest = millis();
  lastposrequest = millis();
  requestflag = true;
  RasPiSerial.print("p");
}

void leftspeedrequest()
{
  lastleftspeedrequest = millis();
  lastspeedrequest = millis();
  requestflag = true;
  RasPiSerial.print("l");
}

void rightspeedrequest()
{
  lastrightspeedrequest = millis();
  lastspeedrequest = millis();
  requestflag = true;
  RasPiSerial.print("r");
}

void pathreset()
{
  lastresetcommand = millis();
  if (Switch_D == 1)
  {
    RasPiSerial.print("s"); //Reset single I
  }
  else
  {
    RasPiSerial.print("d"); //Reset double I
  }
  desiredleftspeed = 0;
  desiredrightspeed = 0;
}
  
void getposdata()
{
  // Lower request flag to indicate that the return packet is being read
  requestflag = false;
  // Read 8 bytes from the buffer
  for (int i = 0; i <8; i++)
  {
    databuffer[i] = RasPiSerial.read();
  }

  
  //Check that the first and second characters are the same, and are valid options (x,y, or p) for packet alignment
  if ( (databuffer[0] == databuffer[1]) && ( (databuffer[0] == 120) || (databuffer[0] == 121) || (databuffer[0] == 112) ) )
  {
    msgID = databuffer[0];
  }
  //If packet is misaligned enter while loop to attempt to realign up to 8 times
  else
  {
    msgID = 255;
    int shiftcounter = 0;
    //Serial.print("Desync present");
    //If not synchronized try to shift the buffer one to the left and read a new byte up to 8 times in case the fpga attempted to read the packet too early
    while ( (databuffer[0] != databuffer[1]) || ( (databuffer[0] != 120) || (databuffer[0] != 121) || (databuffer[0] != 112) ) )
    { 
      for (int i = 0; i <7; i++)
      {
        databuffer[i] = databuffer[i+1];
      }
      databuffer[7] = RasPiSerial.read();
      shiftcounter++;
      
      //If the buffer has been shifted 8 or more times break out and assume the entire paket has been wiped from the buffer
      if (shiftcounter > 7)
      {
        //Serial.print("Desync correction failed");
        break;
      }
    }
    //If sucessfully aligned in alignment detection, set msgID
    if ( (databuffer[0] == databuffer[1]) && ( (databuffer[0] == 120) || (databuffer[0] == 121) || (databuffer[0] == 112) ) )
    {
      msgID = databuffer[0];
    }  
  }
    
  float signmodifier = 0;
  if (databuffer[2] == 43)
  {
    signmodifier = 1;
  }
  else if (databuffer[2] == 45)
  {
    signmodifier = -1;
  }
  
  //Convert 4 digit number from ascii to an integer value then convert from centimeters/centiradians to meters/radians
  float tempvalue = signmodifier * (float((((((((databuffer[3] - 48) * 10) + (databuffer[4] - 48)) * 10) + (databuffer[5] - 48)) * 10) + (databuffer[6] - 48))) / 100);
  //Generate checksum from the second through 7th byte in the buffer
  int checksum = 0;
  for (int i = 1; i < 7; i++)
  {
    checksum += databuffer[i];
  }
  // Compare calculated checksum to expected checksum
  if ( (checksum%256) == databuffer[7])
  {
    //Serial.print("Checksum good. ");
    ChecksumStatus = true;
  }
  else
  {
    //Serial.println("Something here seems a bit wrong...  ");
    ChecksumStatus = false;
  }
  //Check to make sure all data recieved corresponds to an ascii 0-9 character (between decimal 48 and 57)
  //If any character is outside that range, set datavalid to false to protect against erroneous data from the raspberry pi
  boolean datavalid = true;
  for (int i = 3; i < 7; i++)
  {
    if ( (databuffer[i] < 48) || (databuffer[i] > 57) )
    {
      datavalid = false;
    }
    
  }
    
   if((ChecksumStatus == true) && (datavalid == true))
//if(ChecksumStatus == true)
   {
     switch (msgID)
    {
      case 120:
        x = tempvalue;
        //Serial.print("X = ");
        //Serial.println(x);
        break;
      case 121:
        y = tempvalue;
        //Serial.print("Y = ");
        //Serial.println(y);
        break;
      case 112:
        phi = tempvalue;
        //Serial.print("Phi = ");
        //Serial.println(phi);
        break;
      default:
        //Serial.print("ERROR CASE = ");
        //Serial.print(databuffer[0]);
        //Serial.print(databuffer[1]);
        //Serial.println(tempvalue);
        msgID = 255;
        break;
    }
 }
 else
 {
   //Serial.print("Checksum error. Checksum = ");
   //Serial.print(checksum%256);
   //Serial.print(" while the expected checksum = ");
   //Serial.println(databuffer[7]);
 }


  //Check for millis counter rollover and reset all prior request times if rollover has occured
  if ( ((millis() - lastxrequest) > 65535) || ((millis() - lastyrequest) > 65535) || ((millis() - lastphirequest) > 65535) )
  {
    lastxrequest = millis();
    lastyrequest = millis();
    lastphirequest = millis();
  }
  
    
}

//convert to new packet structure
void getspeeddata()
{
  // Lower request flag to indicate that the return packet is being read
  requestflag = false;
  // Read 8 bytes from the buffer
  for (int i = 0; i <7; i++)
  {
    databuffer[i] = RasPiSerial.read();
  }

  
  //Check that the first and second characters are the same, and are valid options (x,y, or p) for packet alignment
  if ( (databuffer[0] == databuffer[1]) && ( (databuffer[0] == 108) || (databuffer[0] == 114) )  )
  {
    msgID = databuffer[0];
  }
  //If packet is misaligned enter while loop to attempt to realign up to 8 times
  else
  {
    msgID = 255;
    int shiftcounter = 0;
    //Serial.print("Desync present");
    //If not synchronized try to shift the buffer one to the left and read a new byte up to 8 times in case the fpga attempted to read the packet too early
    while ( (databuffer[0] != databuffer[1]) || ( (databuffer[0] != 108) || (databuffer[0] != 114) ) )
    { 
      for (int i = 0; i <7; i++)
      {
        databuffer[i] = databuffer[i+1];
      }
      databuffer[7] = RasPiSerial.read();
      shiftcounter++;
      
      //If the buffer has been shifted 8 or more times break out and assume the entire paket has been wiped from the buffer
      if (shiftcounter > 7)
      {
        //Serial.print("Desync correction failed");
        break;
      }
    }
    //If sucessfully aligned in alignment detection, set msgID
    if ( (databuffer[0] == databuffer[1]) && ( (databuffer[0] == 108) || (databuffer[0] == 114)  ) )
    {
      msgID = databuffer[0];
    }  
  }
    
  float signmodifier = 0;
  if (databuffer[2] == 43)
  {
    signmodifier = 1;
  }
  else if (databuffer[2] == 45)
  {
    signmodifier = -1;
  }
  
  //Convert 3 digit number from ascii to an integer value
  float tempvalue = signmodifier * (float((((((((databuffer[3] - 48) * 10) + (databuffer[4] - 48)) * 10) + (databuffer[5] - 48)) ))));
  //Generate checksum from the second through 7th byte in the buffer
  int checksum = 0;
  for (int i = 1; i < 6; i++)
  {
    checksum += databuffer[i];
  }
  // Compare calculated checksum to expected checksum
  if ( (checksum%256) == databuffer[6])
  {
    //Serial.print("Checksum good. ");
    ChecksumStatus = true;
  }
  else
  {
    //Serial.println("Something here seems a bit wrong...  ");
    ChecksumStatus = false;
  }
  //Check to make sure all data recieved corresponds to an ascii 0-9 character (between decimal 48 and 57)
  //If any character is outside that range, set datavalid to false to protect against erroneous data from the raspberry pi
  boolean datavalid = true;
  for (int i = 3; i < 6; i++)
  {
    if ( (databuffer[i] < 48) || (databuffer[i] > 57) )
    {
      datavalid = false;
    }
    
  }
    
   if((ChecksumStatus == true) && (datavalid == true))
//if(ChecksumStatus == true)
   {
     switch (msgID)
     {
      case 108:
        desiredleftspeed = tempvalue;
        break;
      case 114:
        desiredrightspeed = tempvalue;
        /*
        //Comment block for debugging magical motor speed changing bug. 
        //Bug occurs from weak checksum and losing the first/last byte of data when the right motor is commanded to go -20 cm/sec.
        //Fix for bug is never actually telling the right motor to go -20 cm/sec
        for (int i = 1; i < 7; i++)
        {
          Serial.print(databuffer[i]-48);
        }
        Serial.println();
        Serial.print("checksum = ");
        Serial.println(checksum%256);
        Serial.print("expected checksum = ");
        Serial.println(databuffer[6]);
        Serial.print("Right speed command = ");
        Serial.println(desiredrightspeed);
        */
        break;
      default:
        //Serial.print("ERROR CASE = ");
        //Serial.print(databuffer[0]);
        //Serial.print(databuffer[1]);
        //Serial.println(tempvalue);
        msgID = 255;
        break;
    }
 }
 else
 {
   //Serial.print("Checksum error. Checksum = ");
   //Serial.print(checksum%256);
   //Serial.print(" while the expected checksum = ");
   //Serial.println(databuffer[7]);
 }


  //Check for millis counter rollover and reset all prior request times if rollover has occured
  if ( ((millis() - lastleftspeedrequest) > 65535) || ((millis() - lastrightspeedrequest) > 65535) )
  {
    lastleftspeedrequest = millis();
    lastrightspeedrequest = millis();
  }
  
    
}

