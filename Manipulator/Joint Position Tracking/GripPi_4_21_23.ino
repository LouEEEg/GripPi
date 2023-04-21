#include <LinkedList.h>
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <GripPi.h>
#define PI 3.14159

using namespace BLA;

// ----- Global Variables -----//
int grip_pos = 90;

// ----- Object Instantiation ----- //
AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);
AccelStepper ShoulderStepper(DRIVER_INTERFACE, SHOULDER_PUL, SHOULDER_DIR);
AccelStepper ElbowStepper(DRIVER_INTERFACE, ELBOW_PUL, ELBOW_DIR);
AccelStepper ForearmStepper(DRIVER_INTERFACE, FOREARM_PUL, FOREARM_DIR);
AccelStepper WristStepper(DRIVER_INTERFACE, WRIST_PUL, WRIST_DIR);

Servo Servo_Grip;

SerialInput GripPi_Serial; 

Manipulator GripPi;

// ----------------- //
// ----- Setup ----- //
// ----------------- //
void setup() {
  //SerialUSB.begin(115200);
  //SerialUSB.begin(115200);
  //ThetaCalc();
  //MatrixMath();
  
  Serial.begin(9600);  // Initialize Native USB port
  SerialUSB.begin(9600);

  driverInit();
  EnableStepperOutputs();
  //WakeUp();

  SerialUSB.println("This demo expects 4 pieces of data - an X, Y, Z and grip values");
  SerialUSB.println("Enter data in this style <X value, Y value, Z value Grip Value (90 open, 130 closed)>  ");
  SerialUSB.println();


}

// --------------------- //
// ----- Main Loop ----- //
// --------------------- //
  void loop() { 
    // Receive Goal position <x,y,z>
    RxGoalFromSerial(GripPi_Serial, GripPi);

    // Calculate Current Gripper Pose <x,y,z>
    //ForwardKinematics(GripPi);

    // Calculate Joint angles to to reach goal position
    //InverseKinematics(GripPi);

    // Convert joint angles to rotation distance in steps
    //AngleToSteps(GripPi);

    // Checks the Pose to Goal Trajectory
    // CalculateTrajectory(GripPi);

    // Updates the trajectory if nesecarry, this function could be called from the 
    // initial trajectory function. 
    // ObstacleAvoidance(GripPi);

    EnableStepperOutputs();
    SetStepperMoveTo(GripPi);
    RunSteppers();
    UpdatePosition(GripPi);
      
    // While runs until position is reached
    while (GripPi.position.base != 0 || GripPi.position.shoulder != 0 || GripPi.position.elbow != 0 || GripPi.position.wrist != 0) {
      EnableStepperOutputs();
      RunSteppers();
      UpdatePosition(GripPi);
    }

  }// end loop()

// ----------------------------- //
// ----- Stepper Functions ----- //
// ----------------------------- //
  void RunSteppers(void){
    // Increment each stepper one step, if possible
    BaseStepper.run();
    ShoulderStepper.run();
    ElbowStepper.run();
    WristStepper.run();
  }

  void SetStepperMoveTo(Manipulator GripPi){
    // Sets the #of steps each stepper needs to move to reach the Goal
    BaseStepper.moveTo(GripPi.steps.base);
    ShoulderStepper.moveTo(GripPi.steps.shoulder);
    ElbowStepper.moveTo(GripPi.steps.elbow);
    WristStepper.moveTo(GripPi.steps.wrist);
  }

  void EnableStepperOutputs(void){
    BaseStepper.enableOutputs();
    ShoulderStepper.enableOutputs();
    ElbowStepper.enableOutputs();
    ForearmStepper.enableOutputs();
    WristStepper.enableOutputs();
  }

  void DisableStepperOutputs(void){
    BaseStepper.disableOutputs();
    ShoulderStepper.disableOutputs();
    ElbowStepper.disableOutputs();
    ForearmStepper.disableOutputs();
    WristStepper.disableOutputs();
  }

  void driverInit(void) {
    // Base
    pinMode(BASE_LIMIT, INPUT);
    BaseStepper.setMaxSpeed(BASE_MAX_SPEED);
    BaseStepper.setEnablePin(BASE_ENA);
    BaseStepper.setPinsInverted(false, false, true);

    // Shoulder
    pinMode(SHOULDER_LIMIT, INPUT);
    ShoulderStepper.setMaxSpeed(SHOULDER_MAX_SPEED);
    ShoulderStepper.setEnablePin(SHOULDER_ENA);
    ShoulderStepper.setPinsInverted(false, false, true);

    // Elbow
    pinMode(ELBOW_LIMIT, INPUT);
    ElbowStepper.setMaxSpeed(ELBOW_MAX_SPEED);
    ElbowStepper.setEnablePin(ELBOW_ENA);
    ElbowStepper.setPinsInverted(false, false, true);

    // Forearm
    pinMode(FOREARM_LIMIT, INPUT);
    ForearmStepper.setMaxSpeed(FOREARM_MAX_SPEED);
    ForearmStepper.setEnablePin(FOREARM_ENA);
    ForearmStepper.setPinsInverted(false, false, true);

    //Wrist
    pinMode(WRIST_LIMIT, INPUT);
    WristStepper.setMaxSpeed(WRIST_MAX_SPEED);
    WristStepper.setEnablePin(WRIST_ENA);
    WristStepper.setPinsInverted(false, false, true);

    EnableStepperOutputs();

    Servo_Grip.attach(53);
    Servo_Grip.write(90);
  }

  void WakeUp(void){
    EnableStepperOutputs();
    
    ShoulderStepper.setAcceleration(200);
    ShoulderStepper.runToNewPosition(-2000); //test //shoulder moves back about 80-90 degrees
    //BaseStepper.setAcceleration(300);
    //BaseStepper.runToNewPosition(-4000); //test
    ElbowStepper.setAcceleration(800);
    ElbowStepper.runToNewPosition(-9000); //test //elbow moves back about 90 degrees

    homeShoulderAxis();
    homeElbowAxis();
    homeWristAxis();
    homeBaseAxis();
  }

void ReCalibrate(void){
    homeWristAxis();
    homeElbowAxis();    
    homeBaseAxis();
    BaseStepper.setAcceleration(300);
    BaseStepper.runToNewPosition(-4000); //test
    homeShoulderAxis();
    BaseStepper.setAcceleration(300);
    BaseStepper.runToNewPosition(0);
  }

// ------------------------------------- //
// ----- Manipulator Calculations  ----- //
// ------------------------------------- //
  void ForwardKinematics(Manipulator GripPi){
    // Caclulate the homogeneous transformations for the movement in reference to the starting position
    // Initializing the matricies  
    BLA::Matrix<4, 4> BASE =        {cos(GripPi.angle.base),0,-sin(GripPi.angle.base),0,0,1,0,0,sin(GripPi.angle.base),0,cos(GripPi.angle.base),0,0,0,0,1}; //rortate about the Y
    BLA::Matrix<4, 4> BASETR =      {1,0,0,0,0,1,0,GripPi.length.base_2_shoulder,0,0,1,0,0,0,0,1};//Shift along the Y
    BLA::Matrix<4, 4> SHOULDER =    {cos(GripPi.angle.shoulder),-sin(GripPi.angle.shoulder),0,0,sin(GripPi.angle.shoulder),cos(GripPi.angle.shoulder),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
    BLA::Matrix<4, 4> SHOULDERTR =  {1,0,0,0,0,1,0,GripPi.length.shoulder_2_elbow,0,0,1,0,0,0,0,1};//Shift along the Y
    BLA::Matrix<4, 4> ELBOW =       {cos(GripPi.angle.elbow),-sin(GripPi.angle.elbow),0,0,sin(GripPi.angle.elbow),cos(GripPi.angle.elbow),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
    BLA::Matrix<4, 4> ELBOWTR=      {1,0,0,0,0,1,0,GripPi.length.elbow_2_wrist,0,0,1,0,0,0,0,1};//Shift along the Y
    BLA::Matrix<4, 4> WRIST =       {cos(GripPi.angle.wrist),-sin(GripPi.angle.wrist),0,0,sin(GripPi.angle.wrist),cos(GripPi.angle.wrist),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
    BLA::Matrix<4, 4> WRISTTR=      {1,0,0,0,0,1,0,GripPi.length.wrist_2_gripper,0,0,1,0,0,0,0,1};//Shift along the Y
    
    // doing the matrix math   
    BLA::Matrix<4, 4> BASETOTAL =     BASE*BASETR;
    BLA::Matrix<4, 4> SHOULDERTOTAL = SHOULDER*SHOULDERTR;
    BLA::Matrix<4, 4> ELBOWTOTAL =    ELBOW*ELBOWTR;
    BLA::Matrix<4, 4> WRISTTOTAL =    WRIST*WRISTTR;

    BLA::Matrix<4, 4> TOTAL=BASETOTAL*SHOULDERTOTAL*ELBOWTOTAL*WRISTTOTAL;
  
    // calculate the new position matrix
    BLA::Matrix<4> POSF = {TOTAL(0,3),TOTAL(1,3),TOTAL(2,3),0}; //what is calculated each time 

    // Set the pose in the structure
    GripPi.pose.x = TOTAL(0,3);
    GripPi.pose.y = TOTAL(1,3);
    GripPi.pose.z = TOTAL(2,3);
  }

  void InverseKinematics(Manipulator GripPi){
    GripPi.goal.distance = sqrt(sq(GripPi.goal.z) + sq(GripPi.goal.x));         //calculate distance from Yaxis to Desired point 
    GripPi.angle.base = atan(GripPi.goal.z/GripPi.goal.x);                      //calculate theta 0 to align axis with the distance 
    GripPi.goal.x = GripPi.goal.distance - GripPi.length.wrist_2_gripper;       //adjusted x axis 
    GripPi.goal.y = GripPi.goal.y - GripPi.length.base_2_shoulder;              //adjusted y axis 

    GripPi.angle.elbow = acos((sq(GripPi.goal.x) + sq(GripPi.goal.y) - sq(GripPi.length.shoulder_2_elbow) - sq(GripPi.length.elbow_2_wrist)) 
                          / (2*GripPi.length.shoulder_2_elbow*GripPi.length.elbow_2_wrist));   //calculate theta 2 

    GripPi.angle.shoulder = atan(GripPi.goal.y/GripPi.goal.x) + atan((GripPi.length.elbow_2_wrist*sin(GripPi.angle.elbow)) 
                            / (GripPi.length.shoulder_2_elbow + GripPi.length.elbow_2_wrist*cos(GripPi.angle.elbow))); //calculate theta 1 

    // compensating for the orientation of our manipulator
    GripPi.angle.shoulder = -(PI/2 - GripPi.angle.shoulder ); 
    GripPi.angle.elbow = -GripPi.angle.elbow; 
    GripPi.angle.wrist = -(PI/2 + GripPi.angle.shoulder  + GripPi.angle.elbow);
  }

  void UpdatePosition(Manipulator GriPi){
    // Steps to go
    GripPi.position.base = BaseStepper.distanceToGo();
    GripPi.position.shoulder = ShoulderStepper.distanceToGo();
    GripPi.position.elbow = ElbowStepper.distanceToGo();
    GripPi.position.wrist = WristStepper.distanceToGo();
  }

  void AngleToSteps(Manipulator GripPi){
    GripPi.steps.base = 44.444*GripPi.angle.base*180/PI;
    GripPi.steps.shoulder = -24.4*GripPi.angle.shoulder*180/PI;
    GripPi.steps.elbow = -100*GripPi.angle.elbow*180/PI;
    GripPi.steps.wrist = 19.44*GripPi.angle.wrist*180/PI;
  }

  void StepsToAngle(Manipulator GripPi){
    GripPi.angle.base = GripPi.steps.base*PI / 180*44.444;
    GripPi.angle.shoulder = -GripPi.steps.shoulder*PI / 180*24.4;
    GripPi.angle.elbow = -GripPi.steps.elbow*PI / 100*180;
    GripPi.angle.wrist = GripPi.steps.wrist*PI / 19.44*180;
    
  }

  void CalculateTrajectory(Manipulator GripPi){
    //allocate the current position to the list 
    float xt=GripPi.goal.x - GripPi.pose.x; //difference in the x,y,z values from the current position to the goal 
    float yt=GripPi.goal.y - GripPi.pose.y; 
    float zt=GripPi.goal.z - GripPi.pose.z; 
    GripPi.goal.distance=sqrt(sq(xt)+sq(yt)+sq(zt));//total distance from the current position to the goal 
    float D = 20; //incrimental distance in mm we want each point interval to travel 

    for( D < GripPi.goal.distance; D >= GripPi.goal.distance;){

      float xd=D/GripPi.goal.distance*xt; //the changes in the x,y,z values to go to the next point of distance D away 
      float yd=D/GripPi.goal.distance*yt;
      float zd=D/GripPi.goal.distance*zt;

      // change these to the list or array structure 
      GripPi.pose.x=GripPi.pose.x+xd;//new current position 
      GripPi.pose.y=GripPi.pose.y+yd;
      GripPi.pose.z=GripPi.pose.z+zd;

        if(GripPi.pose.x > 450 && GripPi.pose.y > 180 ){ //coordinates to define the shelf  
          //give new safe x and y values for the next few points 
          //safe pos1
          //safe pos2 etc...

          //final safe position to re-enter the loop with 
          //GripPi.pose.x=;
          //GripPi.pose.y=;
          //GripPi.pose.z=;

        }

     //add point to the sequence of points
      float xt=GripPi.goal.x-GripPi.pose.x;//re-calculating the distances with the new point
      float yt=GripPi.goal.y-GripPi.pose.y;
      float zt=GripPi.goal.z-GripPi.pose.z;
      GripPi.goal.distance=sqrt(sq(xt)+sq(yt)+sq(zt));//re-calculate the distance to verify the for loop 

   }

  //next incriment in the list is just the goal itself


  }

// ----------------------------------------------- //
// ----- Axis Home Calibration Calculations  ----- //
// ----------------------------------------------- //
  /*----- Base Axis Calibration------*/
  void homeBaseAxis(void) {
    bool base_home_set = false;
    int base_limit_reached = 0;

    while (!base_home_set) {

      while (!base_limit_reached) {  //motor rotates CCW
        BaseStepper.setSpeed(500);
        BaseStepper.runSpeed();
        base_limit_reached = digitalRead(BASE_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (base_limit_reached) {  //this runs when the limit switch is being held
        BaseStepper.setSpeed(-100);
        BaseStepper.runSpeed();
        base_limit_reached = digitalRead(BASE_LIMIT);
      }

      BaseStepper.setCurrentPosition(GripPi.position.base);  //motor is set to zero and motor speed is zero
      BaseStepper.setAcceleration(300);
      BaseStepper.runToNewPosition(-6000);  //-6000 steps ~135deg from the limit switch gets us very close to the zero point on the arm
      //~44.44 steps per degree
      BaseStepper.setCurrentPosition(GripPi.position.base);  //zero position is when the two arrows are lined up
      base_home_set = true;
    }
  }

  /*----- Shoulder Axis Calibration------*/
  void homeShoulderAxis(void) {
    bool shoulder_home_set = false;
    int shoulder_limit_reached = 0;

    while (!shoulder_home_set) {
      while (!shoulder_limit_reached) {  //Joint rotates forwards
        ShoulderStepper.setSpeed(200);
        ShoulderStepper.runSpeed();
        shoulder_limit_reached = !digitalRead(SHOULDER_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (shoulder_limit_reached) {  //this runs when the limit switch is being held
        ShoulderStepper.setSpeed(-100);
        ShoulderStepper.runSpeed();
        shoulder_limit_reached = !digitalRead(SHOULDER_LIMIT);
      }

      ShoulderStepper.setCurrentPosition(GripPi.position.shoulder);  //motor is set to zero and motor speed is zero
      ShoulderStepper.setAcceleration(200);
      ShoulderStepper.runToNewPosition(-2200);//+ = forwards, - = backwards 2200 steps ~90 degrees
      //~24.44 steps per degree
      ShoulderStepper.setCurrentPosition(GripPi.position.shoulder);  //zero position is when the two arrows are lined up

      shoulder_home_set = true;
    }
  }

  //------Elbow Axis Calibration------//
  void homeElbowAxis(void) {
    bool elbow_home_set = false;
    int elbow_limit_reached = 0;

    while (!elbow_home_set) {
      while (!elbow_limit_reached) {
        ElbowStepper.setSpeed(1500);  //Arm rotates forwards
        ElbowStepper.runSpeed();
        elbow_limit_reached = !digitalRead(ELBOW_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (elbow_limit_reached) {  //this runs when the limit switch is being held
        ElbowStepper.setSpeed(-1000);
        ElbowStepper.runSpeed();
        elbow_limit_reached = !digitalRead(ELBOW_LIMIT);
      }

      ElbowStepper.setCurrentPosition(GripPi.position.elbow);  //motor is set to zero and motor speed is zero
      ElbowStepper.setAcceleration(800);
      ElbowStepper.runToNewPosition(-9000);//+ = forwards, - = backwards 9000 ~90deg
      ElbowStepper.setCurrentPosition(GripPi.position.elbow);  //zero position is when the two arrows are lined up

      elbow_home_set = true;
    }
  }

  //------Calibration Sequence for the Wrist------//
  void homeWristAxis(void) {
    bool wrist_home_set = false;
    int wrist_limit_reached = 0;

    while (!wrist_home_set) {
      while (!wrist_limit_reached) {
        WristStepper.setSpeed(-300);  //Arm rotates forwards
        WristStepper.runSpeed();
        wrist_limit_reached = !digitalRead(WRIST_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (wrist_limit_reached) {  //this runs when the limit switch is being held
        WristStepper.setSpeed(100);
        WristStepper.runSpeed();
        wrist_limit_reached = !digitalRead(WRIST_LIMIT);
      }

      WristStepper.setCurrentPosition(GripPi.position.wrist);  //motor is set to zero and motor speed is zero
      WristStepper.setAcceleration(300);
      WristStepper.runToNewPosition(1800);//- = forwards, + = backwards 1800 steps ~90 degrees
      WristStepper.setCurrentPosition(GripPi.position.wrist);  //zero position is when the two arrows are lined up
      wrist_home_set = true;
    }
  }

// ---------------------------- //
// ----- Serial Functions ----- //
// ---------------------------- //
  void RxGoalFromSerial(SerialInput GripPi_Serial, Manipulator GripPi) {
    static boolean rx_ready = false;
    static byte index = 0;
    char start_marker = '<';
    char end_marker = '>';
    char next_char;

    while (SerialUSB.available() > 0 && GripPi_Serial.new_data == false) {
      next_char = SerialUSB.read();

      if (rx_ready == true) {
        if (next_char != end_marker) {
          GripPi_Serial.receivedChars[index] = next_char;
          index++;
          if (index >= SERIAL_BUFFER_SIZE) {
            index = SERIAL_BUFFER_SIZE - 1;
          }
        }
        else {
          GripPi_Serial.receivedChars[index] = '\0'; // terminate the string
          rx_ready = false;
          index = 0;
          GripPi_Serial.new_data = true;
        }
      }

      else if (next_char == start_marker) {
        rx_ready = true;
      }
    }

    if (GripPi_Serial.new_data == true) {
      strcpy(GripPi_Serial.tempChars, GripPi_Serial.receivedChars);
      // this temporary copy is necessary to protect the original data
      // because strtok() used in parseData() replaces the commas with \0
      parseData(GripPi_Serial);
      //printParsedData(GripPi_Serial);
      GripPi.goal.x = GripPi_Serial.Rx_x; 
      GripPi.goal.y = GripPi_Serial.Rx_y; 
      GripPi.goal.z = GripPi_Serial.Rx_z;
      Servo_Grip.write(grip_pos);
      if(GripPi.goal.x=999){ 
        WakeUp();
        }
      else if(GripPi.goal.x=998){
        ReCalibrate();
      }

      else{
      InverseKinematics(GripPi);
      StepsToAngle(GripPi);      
      GripPi_Serial.new_data = false;
      }
    }
  }

  void parseData(SerialInput GripPi_serial) {               // split the data into its parts
    char *strtokIndx;                                       // this is used by strtok() as an index

    strtokIndx = strtok(GripPi_serial.tempChars,",");       // get the first part - the x value
    GripPi_serial.Rx_x = atof(strtokIndx);                  // copy it to Rx_x

    strtokIndx = strtok(NULL, ",");                         // this continues where the previous call left off
    GripPi_serial.Rx_y = atof(strtokIndx);                  // convert it to Rx_y

    strtokIndx = strtok(NULL, ",");                         // continues where the previous call left off
    GripPi_serial.Rx_z = atof(strtokIndx);                  // convert it to Rx_z

    strtokIndx = strtok(NULL, ",");                         // continues where previous call left off
    grip_pos = atoi(strtokIndx);                            // convert it to grip_pos
  }

  void printParsedData(SerialInput GripPi_serial) {
    SerialUSB.print("X ");
    SerialUSB.println(GripPi_serial.Rx_x);
    SerialUSB.print("Y ");
    SerialUSB.println(GripPi_serial.Rx_y);
    SerialUSB.print("Z ");
    SerialUSB.println(GripPi_serial.Rx_z);
    SerialUSB.print("Grip ");
    SerialUSB.println(grip_pos);
  }
