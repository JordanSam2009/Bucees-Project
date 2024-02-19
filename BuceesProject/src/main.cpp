/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       JDANNI                                                    */
/*    Created:      12/14/2023, 9:35:31 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
vex::brain Brain = vex::brain();
vex::controller Controller = vex::controller();

motor FrontLeft(PORT11, gearSetting::ratio6_1, true);
motor TopLeft(PORT19, gearSetting::ratio18_1, false);
motor BackLeft(PORT6, gearSetting::ratio18_1, true);

motor FrontRight(PORT1, gearSetting::ratio6_1, false);
motor TopRight(PORT2, gearSetting::ratio6_1, false);
motor BackRight(PORT3, gearSetting::ratio6_1, true);

inertial InertialSensor(PORT17);

motor_group LeftSide(FrontLeft, TopLeft, BackLeft);
motor_group RightSide(FrontRight, TopRight, BackRight);

drivetrain Drivetrain(LeftSide, RightSide);

rotation backTrackerSensor(PORT19);
rotation rightTrackerSensor(PORT20, false);

Bucees::PIDSettings LINEAR_SETTINGS {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN 
  0.75,
  // PROPORTIONAL GAIN
  1, 
  // INTEGRAL GAIN
  0.0, 
  // DERIVATIVE GAIN
  0.15, 
  // EXIT ERROR
  2.5, 
  // INTEGRAL THRESHOLD
  5, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings ANGULAR_90_SETTINGS {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.21,
  // INTEGRAL GAIN
  0.01,
  // DERIVATIVE GAIN
  0.166,
  // EXIT ERROR
  0.75, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::PIDSettings ANTIDRIFT_1TILE {
  // FEEDFORWARD GAIN
  0, 
  // ACCELERATION GAIN
  0.0,
  // PROPORTIONAL GAIN
  0.28,
  // INTEGRAL GAIN
  0.009,
  // DERIVATIVE GAIN
  0.25,
  // EXIT ERROR
  1, 
  // INTEGRAL THRESHOLD
  15, 
  // TIMEOUT TIME
  0, 
  // MAXIMUM VOLTAGES
  12
};

Bucees::FAPIDController Linear(LINEAR_SETTINGS);

Bucees::FAPIDController Angular(ANGULAR_90_SETTINGS);

Bucees::FAPIDController AntiDrift(ANTIDRIFT_1TILE);

// TRACKING WHEEL EXAMPLE:
/*Bucees::TrackingWheel RightTracker(
  // INSERT RIGHT TRACKER MOTOR GROUP IF NO TRACKING WHEEL, INSERT ROTATION SENSOR PORT IF TRACKING WHEEL:
  PORT16,

  // UNCOMMENT AND PUT REVERSED = TRUE/FALSE DEPENDING ON SETUP
  false,

  // INSERT WHEEL DIAMETER FOR THE TRACKING WHEEL [OR WHEEL DIAMETER FOR THE MOTOR GROUP]:
  2.9,

  // INSERT THE OFFSET FROM THE TRACKING CENTER:
  0,

  // INSERT THE GEAR RATIO OF THE TRACKING WHEEL [OR GEAR RATIO OF THE MOTOR GROUP]:
  1.f/1.f
);*/

// RIGHT TRACKER DRIVETRAIN MOTORS EXAMPLE:
Bucees::TrackingWheel RightTracker(
  &RightSide,

  3.1,

  7,

  36.f/60.f
);

// BACK TRACKER EXAMPLE:
Bucees::TrackingWheel BackTracker(
  PORT5,

  true,

  3.25,
  
  -5.5,

  1.f/1.f
);


Bucees::Robot Robot(
  // INSERT THE WHEEL DIAMETER OF YOUR DRIVETRAIN WHEELS:
  Bucees::Omniwheel::NEW_4,

  // INSERT THE GEAR RATIO OF YOUR DRIVETRAIN USING: X.f/X.f (REPLACE THE X WITH YOUR DRIVETRAIN GEAR RATIO):
  36.f/84.f,

  // INSERT THE TRACK WIDTH OF YOUR DRIVETRAIN IN INCHES:
  14.0,

  // INSERT THE LEFT MOTOR GROUP OF YOUR DRIVETRAIN:
  &LeftSide,

  // INSERT THE RIGHT MOTOR GROUP OF YOUR DRIVETRAIN:
  &RightSide,

  // INSERT THE PORT OF YOUR INERTIAL SENSOR, IF YOU DO NOT USE AN INERTIAL SENSOR, REPLACE WITH "nullptr"
  PORT17,

  // INSERT THE RIGHT TRACKING WHEEL OBJECT:
  &RightTracker,

  // INSERT THE BACK TRACKING WHEEL OBJECT [REPLACE WITH NULLPTR IF NO BACK TRACKER]:
  nullptr,

  // INSERT THE LINEAR PID SETTINGS OBJECT:
  &Linear,

  // INSERT THE ANGULAR PID SETTINGS OBJECT:
  &Angular,

  // INSERT THE ANTI DRIFT PID SETTINGS OBJECT:
  &AntiDrift
);

void pre_auton(void) {

  FrontLeft.setBrake(coast);
  TopLeft.setBrake(coast);
  BackLeft.setBrake(coast);
  FrontRight.setBrake(coast);
  TopLeft.setBrake(coast);
  BackRight.setBrake(coast);
  

  LeftSide.resetPosition();
  RightSide.resetPosition();

  InertialSensor.calibrate();
  waitUntil(InertialSensor.isCalibrating() == false);

  Robot.initOdom();

  Robot.setRobotCoordinates({5.552, -16.93, 0});
}

void autonomous(void) {

}

void usercontrol(void) {

  Brain.Screen.clearScreen();

  while (1) {
    
    double LeftJoystickPosition = Controller.Axis3.position();
    double RightJoystickPosition = Controller.Axis1.position();

    Robot.arcade(LeftJoystickPosition, RightJoystickPosition);

    Bucees::Coordinates normal = Robot.getRobotCoordinates();
    Bucees::Coordinates reversed = Robot.getRobotCoordinates(true, true);

    Brain.Screen.drawImageFromFile("Brain_Screen_Logo.png", 0, 0);

    //printf("x1: %f, y1: %f, theta: %f \n", normal.x, normal.y, normal.theta);
    //printf("x2: %f, y2: %f, theta: %f \n", reversed.x, reversed.y, reversed.theta);

    wait(20, msec); // Sleep the task for a short amount of time to
                  // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
