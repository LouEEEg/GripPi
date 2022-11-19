#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
//------Declarations------//
  // Program Constants
  //------base------//
    #define DRIVER_INTERFACE 1  // AccelStepper interface function. 1 = stepper driver with PUL/DIR pins
    #define BASE_MAX_SPEED 2000
    #define BASE_RUN_SPEED 500
    #define BASE_ACCELERATION 300
  //------shoulder------//
    #define SHOULDER_MAX_SPEED 1600
    #define SHOULDER_RUN_SPEED 500
    #define SHOULDER_ACCELERATION 200
  //------elbow------//
    #define ELBOW_MAX_SPEED 12000
    #define ELBOW_RUN_SPEED 800
    #define ELBOW_ACCELERATION 1800
  //------forarm------//
    #define FORARM_MAX_SPEED 2000
    #define FORARM_RUN_SPEED 500
    #define FORARM_ACCELERATION 200
  //------wrist------//
    #define WRIST_MAX_SPEED 2000
    #define WRIST_RUN_SPEED 500
    #define WRIST_ACCELERATION 200
//------Joint Declarations------//
  // Base Joint Declarations //Driver #1
    #define BASE_LIMIT 10  //standard pin  GREEN
    #define BASE_ENA 11    //standard pin  WHITE
    #define BASE_PUL 12    //standard pin  ORANGE
    #define BASE_DIR 13    //PWM pin YELLOW

  //Shoulder joint Declarations  //Driver #2&3
    #define SHOULDER_LIMIT 52  //standard pin  GREEN     (pull down circuit)
    #define SHOULDER_ENA 4     //standard pin (was pwm)  WHITE
    #define SHOULDER_PUL 5     //standard pin (was pwm)  ORANGE
    #define SHOULDER_DIR 6     //PWM pin YELLOW

  //Elbow joint Declarations //Driver #4
    #define ELBOW_LIMIT 51  //standard pin
    #define ELBOW_ENA 32    //standard pin
    #define ELBOW_PUL 33    //standard pin
    #define ELBOW_DIR 34    //PWM pin

  //Forarm Rotation Declarations //Driver #5
    #define FORARM_LIMIT 50
    #define FORARM_ENA 9 //White
    #define FORARM_PUL 8 //Orange
    #define FORARM_DIR 7 //Yellow

  //Wrist Joint Declarations //Driver #6
    #define WRIST_LIMIT 49  //standard pin
    #define WRIST_ENA 47    //standard pin
    #define WRIST_PUL 46    //standard pin
    #define WRIST_DIR 45    //PWM pin

//

//------Global Variables------//
  Servo Servo_Grip;
  static long base_motor_position = 0;
  static long shoulder_motor_position = 0;
  static long elbow_motor_position = 0;
  static long forarm_motor_position = 0;
  static long wrist_motor_position = 0;
  int base_steps = 0;
  int shoulder_steps = 0;
  int elbow_steps = 0;
  int forarm_steps = 0;
  int wrist_steps = 0;
  int grip_pos = 110;
  bool base_location_reached = 0;
  bool shoulder_location_reached = 0;
  bool elbow_location_reached = 0;
  bool forarm_location_reached = 0;
  bool wrist_location_reached = 0;

//

//------AccelStepper Object Instantiation------//
  AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);
  AccelStepper ShoulderStepper(DRIVER_INTERFACE, SHOULDER_PUL, SHOULDER_DIR);
  AccelStepper ElbowStepper(DRIVER_INTERFACE, ELBOW_PUL, ELBOW_DIR);
  AccelStepper ForarmStepper(DRIVER_INTERFACE, FORARM_PUL, FORARM_DIR);
  AccelStepper WristStepper(DRIVER_INTERFACE, WRIST_PUL, WRIST_DIR);
//

//------SETUP------//
void setup() {
  Serial.begin(9600);
  SerialUSB.begin(9600);  // Initialize Native USB port

  driverInit();
  //homeBaseAxis();
  //homeShoulderAxis();
  //homeElbowAxis();
  //homeForarmAxis();
  //homeWristAxis();
  Servo_Grip.write(grip_pos);
}
//

//------Main Loop------//
void loop() {
  if (SerialUSB.available() > 0) {
    String user_command = SerialUSB.readStringUntil('\n');
    switch (user_command.toInt()) {
      case 99:  //calibration
        homeBaseAxis();
        //homeShoulderAxis();
        //homeElbowAxis();
        //homeForarmAxis();
        //homeWristAxis();
        delay(1000);
        SerialUSB.println("GripI is Calibrated!");
        break;
      case 0:  //move to grab the second bin
       SerialUSB.println("Returning to zero...");
        base_steps = 0;
        shoulder_steps = 0;  //0deg
        elbow_steps = 0;     //0deg
        forarm_steps = 0;    //0deg
        wrist_steps = 0;     //0deg
        break;
    //------Bin 1------//
      case 10:  //lining up in front of the first bin (leftmost)
        //SerialUSB.println("You sent value: ");
        //SerialUSB.println(user_command);
        base_steps = 756;       //17deg
        shoulder_steps = 1145;  //46.85deg
        elbow_steps = 0;        //94.23deg
        forarm_steps = 0;       //0deg
        wrist_steps = 0;        //-51.08deg
        break;
      case 11:                  //move to grab the first bin
        base_steps = 756;       //17deg
        shoulder_steps = 1508;  //61.69deg
        elbow_steps = 0;        //62.68deg
        forarm_steps = 0;       //0
        wrist_steps = 0;        //-34.38deg
        break;
      case 12:  //grip bin
        for (grip_pos = 110; grip_pos <= 145; grip_pos += 2) {
          Servo_Grip.write(grip_pos);
          delay(20);
        }
        break;
      case 13:                  //lifting box
        base_steps = -756;      //arbitrary guess for the photo location
        shoulder_steps = 1324;  //54.18deg
        elbow_steps = 0;        //62.17deg
        forarm_steps = 0;       //0
        wrist_steps = 0;        //-26.35deg
        break;

      case 14:                 //removing box
        base_steps = -756;     //arbitrary guess for the photo location
        shoulder_steps = 912;  //37.33deg
        elbow_steps = 0;       //93.77deg
        forarm_steps = 0;      //0
        wrist_steps = 0;       //-41.10deg
        break;

      case 15:               //photo/user location
        base_steps = -1333;  //arbitrary guess for the photo location
        shoulder_steps = 0;
        elbow_steps = 0;
        forarm_steps = 0;
        wrist_steps = 0;
        break;
    //------Bin 2------//
      case 20:  //lining up in front of the second bin
       // SerialUSB.println("You sent value: ");
        //SerialUSB.println(user_command);
        //SerialUSB.println("Home GripPi");
        base_steps = 0;         //0deg
        shoulder_steps = 1021 ;  //deg
        elbow_steps = 10307;    //deg
        forarm_steps = 0;       //0deg
        wrist_steps = 1067;     //deg
        SerialUSB.println("moving...");
        break;
      case 21:  //move to grab the second bin
        base_steps = 0;      //0deg
        shoulder_steps = 1349;  //55.19deg
        elbow_steps = 7456;     //74.55deg
        forarm_steps = 0;    //0deg
        wrist_steps = 773;     //-39.75deg
        SerialUSB.println("moving...");
        break;
      case 22:  //grip bin
      SerialUSB.println("gripping...");
        grip_pos=110;
        for (grip_pos = 110; grip_pos <= 145; grip_pos += 2) {
          Servo_Grip.write(grip_pos);
          delay(20);
          
        }
        break;
      case 23:   //lifting box
        base_steps = 0;      //arbitrary guess for the photo location
        shoulder_steps = 1037;  //48.56deg
        elbow_steps = 8436;        //72.88deg
        forarm_steps = 0;       //0
        wrist_steps = 716;        //-31.45deg
        SerialUSB.println("moving...");
        break;
      case 24:                 //removing box
        base_steps = 0;     //arbitrary guess for the photo location
        shoulder_steps = 678;  //37.33deg
        elbow_steps = 11073;       //93.77deg
        forarm_steps = 0;      //0
        wrist_steps = 942;       //-41.10deg
        SerialUSB.println("moving...");
        break;
      case 25:               //photo/user location
        base_steps = -1333;  //
        shoulder_steps = 0;  //
        elbow_steps = 0;     //
        forarm_steps = 0;    //
        wrist_steps = 0;     //
        SerialUSB.println("moving...");
        break;

    //------Bin 3------//
      case 30:  //lining up in front of the third bin
        SerialUSB.println("You sent value: ");
        SerialUSB.println(user_command);
        SerialUSB.println("Home GripPi");
        base_steps = -756;
        shoulder_steps = 1145;  //46.85deg
        elbow_steps = 0;        //94.23deg
        forarm_steps = 0;       //0deg
        wrist_steps = 0;        //-51.08deg
        break;
      case 31:                  //move to grab the third bin
        base_steps = -756;      //17deg
        shoulder_steps = 1508;  //61.69deg
        elbow_steps = 0;        //62.68deg
        forarm_steps = 0;       //0
        wrist_steps = 0;        //-34.38deg
        break;
      case 32:  //grip bin
        for (grip_pos = 110; grip_pos >= 145; grip_pos += 2) {
          Servo_Grip.write(grip_pos);
          delay(20);
        }
        break;
      case 33:                  //lifting box
        base_steps = -756;      //-17deg
        shoulder_steps = 1324;  //54.18deg
        elbow_steps = 0;        //62.17deg
        forarm_steps = 0;       //0
        wrist_steps = 0;        //-26.35deg
        break;

      case 34:                 //removing box
        base_steps = -756;     //arbitrary guess for the photo location
        shoulder_steps = 912;  //37.33deg
        elbow_steps = 0;       //93.77deg
        forarm_steps = 0;      //0
        wrist_steps = 0;       //-41.10deg
        break;
      case 35:  //photo/user location
        base_steps = -1333;
        shoulder_steps = 0;
        elbow_steps = 0;
        forarm_steps = 0;
        wrist_steps = 0;
        break;

      default:
        SerialUSB.println("You sent value: ");
        SerialUSB.println(user_command);
        SerialUSB.println("No command Assigned");
        base_steps = 0;
        shoulder_steps = 0;
        elbow_steps = 0;
        forarm_steps = 0;
        wrist_steps = 0;
        break;
    }
  }


  //------loop for when the location has already been reached------//
    //outputs are disabled until a new posision is commanded
      //BaseStepper.disableOutputs();
      //ShoulderStepper.disableOutputs();
      //ElbowStepper.disableOutputs();
      //ForarmStepper.disableOutputs();
      //WristStepper.disableOutputs();
    BaseStepper.enableOutputs();
    ShoulderStepper.enableOutputs();
    ElbowStepper.enableOutputs();
    ForarmStepper.enableOutputs();
    WristStepper.enableOutputs();

    /*-----------base inputs----------*/
      BaseStepper.moveTo(base_steps);  //the pi will update this value base_steps
      BaseStepper.setAcceleration(BASE_ACCELERATION);
      BaseStepper.run();

    /*---------shoulder inputs-------*/
      ShoulderStepper.moveTo(shoulder_steps);  //the pi will update this value shoulder_steps
      ShoulderStepper.setAcceleration(SHOULDER_ACCELERATION);
      ShoulderStepper.run();

    //------Elbow Inputs------//
      ElbowStepper.moveTo(elbow_steps);  //the pi will update this value elbow_steps
      ElbowStepper.setAcceleration(ELBOW_ACCELERATION);
      ElbowStepper.run();

    //------Forarm Inputs------//
      ForarmStepper.moveTo(forarm_steps);
      ForarmStepper.setAcceleration(FORARM_ACCELERATION);
      ForarmStepper.run();

    //------Wrist Inputs------//
      WristStepper.moveTo(wrist_steps);  //the pi will update this value wrist_steps
      WristStepper.setAcceleration(WRIST_ACCELERATION);
      WristStepper.run();

    //------checking for input changes//
    base_location_reached = BaseStepper.distanceToGo();
    shoulder_location_reached = ShoulderStepper.distanceToGo();
    elbow_location_reached = ElbowStepper.distanceToGo();
    forarm_location_reached = ForarmStepper.distanceToGo();
    wrist_location_reached = WristStepper.distanceToGo();

  //-------While loop to run while joints are moving------//
  while (base_location_reached != 0 || shoulder_location_reached != 0 || elbow_location_reached != 0 || wrist_location_reached != 0) {
    //once the inputs are recieved, this loop plays traveling to the location
    BaseStepper.enableOutputs();
    ShoulderStepper.enableOutputs();
    ElbowStepper.enableOutputs();
    ForarmStepper.enableOutputs();
    WristStepper.enableOutputs();
    base_location_reached = BaseStepper.distanceToGo();
    BaseStepper.run();
    shoulder_location_reached = ShoulderStepper.distanceToGo();
    ShoulderStepper.run();
    elbow_location_reached = ElbowStepper.distanceToGo();
    ElbowStepper.run();
    forarm_location_reached = ForarmStepper.distanceToGo();
    ForarmStepper.run();
    wrist_location_reached = WristStepper.distanceToGo();
    WristStepper.run();
  }
}

//------Calibrations------//
/*----- Base Axis Calibration------*/
void homeBaseAxis(void) {
  bool base_home_set = false;
  int base_limit_reached = 0;

  BaseStepper.enableOutputs();

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

    BaseStepper.setCurrentPosition(base_motor_position);  //motor is set to zero and motor speed is zero
    BaseStepper.setAcceleration(300);
    BaseStepper.runToNewPosition(-6000);  //-6000 steps from the limit switch gets us very close to the zero point on the arm
    //~44.44 steps per degree
    BaseStepper.setCurrentPosition(base_motor_position);  //zero position is when the two arrows are lined up
    BaseStepper.disableOutputs();
    base_home_set = true;
  }
}

/*----- Shoulder Axis Calibration------*/
void homeShoulderAxis(void) {
  bool shoulder_home_set = false;
  int shoulder_limit_reached = 0;

  ShoulderStepper.enableOutputs();

  while (!shoulder_home_set) {

    while (!shoulder_limit_reached) {  //Joint rotates forwards
      ShoulderStepper.setSpeed(200);
      ShoulderStepper.runSpeed();
      shoulder_limit_reached = digitalRead(!SHOULDER_LIMIT);  //this pin stays low untill limit switch is hit
    }

    while (shoulder_limit_reached) {  //this runs when the limit switch is being held
      ShoulderStepper.setSpeed(-100);
      ShoulderStepper.runSpeed();
      shoulder_limit_reached = digitalRead(!SHOULDER_LIMIT);
    }

    ShoulderStepper.setCurrentPosition(shoulder_motor_position);  //motor is set to zero and motor speed is zero
    ShoulderStepper.setAcceleration(300);
    //ShoulderStepper.runToNewPosition(-6000);//??? steps from the limit switch gets us very close to the zero point on the arm
    //~24.44 steps per degree
    ShoulderStepper.setCurrentPosition(shoulder_motor_position);  //zero position is when the two arrows are lined up
    ShoulderStepper.disableOutputs();
    shoulder_home_set = true;
  }
}

//------Elbow Axis Calibration------//
void homeElbowAxis(void) {
  bool elbow_home_set = false;
  int elbow_limit_reached = 0;

  ElbowStepper.enableOutputs();

  while (!elbow_home_set) {

    while (!elbow_limit_reached) {

      ElbowStepper.setSpeed(200);  //Arm rotates forwards
      ElbowStepper.runSpeed();
      elbow_limit_reached = digitalRead(!ELBOW_LIMIT);  //this pin stays low untill limit switch is hit
    }

    while (elbow_limit_reached) {  //this runs when the limit switch is being held
      ElbowStepper.setSpeed(-100);
      ElbowStepper.runSpeed();
      elbow_limit_reached = digitalRead(!ELBOW_LIMIT);
    }

    ElbowStepper.setCurrentPosition(elbow_motor_position);  //motor is set to zero and motor speed is zero
    ElbowStepper.setAcceleration(300);
    //ElbowStepper.runToNewPosition();//Zero point after calibration

    ElbowStepper.setCurrentPosition(elbow_motor_position);  //zero position is when the two arrows are lined up
    ElbowStepper.disableOutputs();
    elbow_home_set = true;
  }
}

/*----- Forarm Axis Calibration------*/
void homeForarmAxis(void) {
  bool forarm_home_set = false;
  int forarm_limit_reached = 0;

  ForarmStepper.enableOutputs();

  while (!forarm_home_set) {

    while (!forarm_limit_reached) {  //motor rotates
      ForarmStepper.setSpeed(100);
      ForarmStepper.runSpeed();
      forarm_limit_reached = digitalRead(!FORARM_LIMIT);  //this pin stays low untill limit switch is hit
    }

    while (forarm_limit_reached) {  //this runs when the limit switch is being held
      ForarmStepper.setSpeed(-100);
      ForarmStepper.runSpeed();
      forarm_limit_reached = digitalRead(!FORARM_LIMIT);
    }

    ForarmStepper.setCurrentPosition(forarm_motor_position);  //motor is set to zero and motor speed is zero
    ForarmStepper.setAcceleration(300);
    //ForarmStepper.runToNewPosition();//needs to be tested
    ForarmStepper.setCurrentPosition(forarm_motor_position);  //zero position is when the two arrows are lined up
    ForarmStepper.disableOutputs();
    forarm_home_set = true;
  }
}

//------Calibration Sequence for the Wrist------//
void homeWristAxis(void) {
  bool wrist_home_set = false;
  int wrist_limit_reached = 0;

  WristStepper.enableOutputs();

  while (!wrist_home_set) {

    while (!wrist_limit_reached) {

      WristStepper.setSpeed(100);  //Arm rotates forwards
      WristStepper.runSpeed();
      wrist_limit_reached = digitalRead(!WRIST_LIMIT);  //this pin stays low untill limit switch is hit
    }

    while (wrist_limit_reached) {  //this runs when the limit switch is being held
      WristStepper.setSpeed(-100);
      WristStepper.runSpeed();
      wrist_limit_reached = digitalRead(!WRIST_LIMIT);
    }

    WristStepper.setCurrentPosition(wrist_motor_position);  //motor is set to zero and motor speed is zero
    WristStepper.setAcceleration(300);
    //WristStepper.runToNewPosition();//Zero point after calibration

    WristStepper.setCurrentPosition(wrist_motor_position);  //zero position is when the two arrows are lined up
    WristStepper.disableOutputs();
    wrist_home_set = true;
  }
}
//

/*----- AccelStepper Drivier Init ------*/
void driverInit(void) {
  // Base
  pinMode(BASE_LIMIT, INPUT);
  BaseStepper.setMaxSpeed(BASE_MAX_SPEED);
  BaseStepper.setEnablePin(BASE_ENA);
  BaseStepper.setPinsInverted(false, false, true);
  BaseStepper.disableOutputs();

  // Shoulder
  pinMode(SHOULDER_LIMIT, INPUT);
  ShoulderStepper.setMaxSpeed(SHOULDER_MAX_SPEED);
  ShoulderStepper.setEnablePin(SHOULDER_ENA);
  ShoulderStepper.setPinsInverted(false, false, true);
  ShoulderStepper.disableOutputs();

  // Elbow
  pinMode(ELBOW_LIMIT, INPUT);
  ElbowStepper.setMaxSpeed(ELBOW_MAX_SPEED);
  ElbowStepper.setEnablePin(ELBOW_ENA);
  ElbowStepper.setPinsInverted(false, false, true);
  //ElbowStepper.disableOutputs();

  // Forarm
  pinMode(FORARM_LIMIT, INPUT);
  ForarmStepper.setMaxSpeed(FORARM_MAX_SPEED);
  ForarmStepper.setEnablePin(FORARM_ENA);
  ForarmStepper.setPinsInverted(false, false, true);
  //ForarmStepper.disableOutputs();

  //Wrist
  pinMode(WRIST_LIMIT, INPUT);
  WristStepper.setMaxSpeed(WRIST_MAX_SPEED);
  WristStepper.setEnablePin(WRIST_ENA);
  WristStepper.setPinsInverted(false, false, true);
  //WristStepper.disableOutputs();

  Servo_Grip.attach(53);
  Servo_Grip.write(110);

}
