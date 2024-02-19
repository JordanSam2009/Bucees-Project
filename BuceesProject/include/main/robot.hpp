#pragma once
#include "../vex.h"

namespace Bucees {
    class Robot {
        private:

        float distanceTraveled;

        float defaultDeacceleration = 1;
        
        float drivetrainWheelDiameter;
        float drivetrainGearRatio;
        float drivetrainTrackWidth;
        float rightTrackerDistance;
        float backTrackerDistance;
        float rightTrackerDiameter;
        float backTrackerDiameter;

        vex::mutex mutex;

        public:

        Odometry odometry;
        Odometry reversedOdometry;

        Coordinates RobotPosition = {0, 0, 0};
        Coordinates reversedRobotPosition = {0, 0, 0};

        enum::ODOMETRY_CONFIG odomSettings;

        vex::motor_group* LeftSide;
        vex::motor_group* RightSide;
        vex::inertial InertialSensor;
        TrackingWheel* RightTracker;
        TrackingWheel* BackTracker;

        FAPIDController* Linear;
        FAPIDController* Angular;
        FAPIDController* AntiDrift;

        /**
         * @brief Initalize a new Robot.
         * 
         * @param drivetrainWheelDiameter The wheel diameter of the drivetrain wheels
         * @param drivetrainGearRatio The gear ratio for the drivetrain
         * @param drivetrainTrackWidth The trackwidth of the chassis
         * @param LeftSide The motor group object that contains the left side motors for the drivetrain
         * @param RightSide The motor group object that contains the right side motors for the drivetrain
         * @param InertialPort The PORT for the Inertial Sensor [SET TO "nullptr" IF NO INERTIAL SENSOR]
         * @param RightTracker The Tracking Wheel Object that contains the data for the right tracker
         * @param BackTracker The Tracking Wheel Object that contains the data for the back tracker [SET TO "nullptr" IF NO BACK TRACKER]
         * @param Linear The Linear PID Settings for the robot
         * @param Angular The Angular PID Settings for the robot
         * @param AntiDrift The Anti Drift PID Settings for the robot
        */
       Robot(float drivetrainWheelDiameter, float drivetrainGearRatio, float drivetrainTrackWidth, vex::motor_group* LeftSide, vex::motor_group* RightSide, int32_t InertialPort, TrackingWheel* RightTracker, TrackingWheel* BackTracker, FAPIDController* Linear, FAPIDController* Angular, FAPIDController* AntiDrift);
       Robot(float drivetrainWheelDiameter, float drivetrainGearRatio, float drivetrainTrackWidth, vex::motor_group* LeftSide, vex::motor_group* RightSide, int32_t InertialPort, TrackingWheel* RightTracker, std::nullptr_t BackTracker, FAPIDController* Linear, FAPIDController* Angular, FAPIDController* AntiDrift);

        /**
         * @brief Get the absolute heading of the inertial sensor using fmodf to constrain it to [0, 360]
        */
        float getAbsoluteHeading();

        /**
         * @brief Control the robot's drivetrain using arcade mode on the controller.
         * 
         * @param LeftJoystickPosition The position of the left joystick
         * @param RightJoystickPosition The position of the right joystick
        */
        void arcade(float LeftJoystickPosition, float RightJoystickPosition);

        /**
         * @brief Set the odometry offsets
        */
        void initOdom();

        /**
         * @brief Set the robots position to a specific coordinate
         * 
         * @param coordinates The coordinates the robot will start at the beginning of the autonomous
        */
        void setRobotCoordinates(Bucees::Coordinates coordinates);

        /**
         * @brief Returns the robot's coordinates calculated by odometry.
         * 
         * @param radians Whether or not to return the robot coordinates in radians/degrees. [true by default]
         * @param reversed Whether or not to return the robot coordinates as if the drivetrain was reversed. [false by default]
        */
        Bucees::Coordinates getRobotCoordinates(bool radians = true, bool reversed = false);

        /**
         * @brief Waits until the chassis is done with its movement
        */
        void waitChassis();

        /**
         * @brief Waits until the chassis has traveled a certain distance
         * 
         * @param distance The distance the chassis has traveled [ANY UNITS]
        */
        void waitChassis(float distance);

        /**
         * @brief Drive the robot for a certain amount of inches
         * 
         * @param target The amount of inches to drive.
         * @param antiDrift Whether or not to apply anti drift correction to movement. [default ]
         * @param timeout The amount of time the robot has to complete the action before moving on. [default to 0 meaning no timeout]
         * @param async Determine whether or not to run command in a separate thread. 
        */
        void DriveFor(float target, bool antiDrift = false, float timeout = 0, bool async = false);

        /**
         * @brief Turn the robot for a certain amount of degrees
         * 
         * @param target The amount of degrees to turn for
         * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
         * @param async Determine whether or not to run command in a separate thread. 
        */
        void TurnFor(float target, float timeout = 0, bool async = false);

        /**
         * @brief Move to a heading in a hook motion by powering one side of the drivetrain
         * 
         * @param target The amount of degrees to hook to
         * @param leftPower The amount of power to apply to the left side of the drivetrain to allow for longer hooks [0 - 12 range]
         * @param reversed True of false value that determines whether or not to hook going forward/backward
         * @param timeout The amount of time the robot has to complete the action before moving [default to 0 meaning no timeout]
         * @param async Determine whether or not to run command in a separate thread. 
        */
        void HookLeft(float target, float leftPower, bool reversed = false, float timeout = 0, bool async = false);

        /**
         * @brief Move to a heading in a hook motion by powering one side of the drivetrain
         * 
         * @param target The amount of degrees to hook to
         * @param rightPower The amount of power to apply to the right side of the drivetrain to allow for longer hooks [0 - 12 range]
         * @param reversed True of false value that determines whether or not to hook going forward/backward
         * @param timeout The amount of time the robot has to complete the action before moving [default to 0 meaning no timeout]
         * @param async Determine whether or not to run command in a separate thread. 
        */
        void HookRight(float target, float rightPower, bool reversed = false, float timeout = 0, bool async = false);

        /**
         * @brief Drive the Robot to a target point using distance formula
         * 
         * @param x x location to move to [inches]
         * @param y y location to move to [inches]
         * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
         * @param async Determine whether or not to run command in a separate thread.
         */
        void DriveToPoint(float x, float y, float timeout = 0, bool reversed = false, bool async = false); // simple drive to point

        /**
         * @brief Turn the Robot towards a target point using simple trigonometry
         * 
         * @param x x coordinate to face
         * @param y y coordinate to face
         * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
         * @param async Determine whether or not to run command in a separate thread. 
        */
        void TurnToPoint(float x, float y, float timeout = 0, bool async = false);

        /**
         * @brief Move the Robot towards a target coordinate using a boomerang controller [DISABLED]
         * 
         * @param x x location to move to [inches]
         * @param y y location to move to [inches]
         * @param theta The angle to face at the end of the movement [degrees]
         * @param chasePower Higher values = make robot faster but more overshoot on turns
         * @param lead Determines how curved the robot will move [default to 0.6,  can only be from 0 - 1]
         * @param maxVoltages Determines the maximum speed overall of the motors [default to 12 voltages]
         * @param timeout The amount of time the robot has to complete the action before moving on [default to 0 meaning no timeout, in miliseconds]
         * @param async Determine whether or not to run command in a separate thread. 
         */
        void DriveToCoordinates(float x, float y, float theta, float chasePower, float lead = 0.6, float maxVoltages = 12, float timeout = 0, bool async = false); // boomerang version

        /**
         * @brief Drive to a point using a ramsette controller, a more advanced version of drive to point using the BLRS wiki documentation [DISABLED]
         * 
         * @param x x location that was inputted
         * @param y y location that was inputted
         * @param theta angle in degrees that was inputted
         * @param beta A proportional term for the controller (0 < x)
         * @param zeta A damping term similar to derivative in PID (0 < x < 1)
         * @param linear Desired linear velocity (should come from a motion profile)
         * @param angular Desired angular velocity (should come from a motion profile)
         * @param async Determine whether or not to run command in a separate thread. 
        */
       void ramseteDriveTo(float x, float y, float theta, float beta, float zeta, float linear, float angular, bool async = false);

        /**
         * @brief Follow a set of coordinates using the pure pursuit motion algorithm and a path planner
         * 
         * @param path A vector containing coordinates to follow
         * @param maxSpeed The max speed the robot is allowed to go while following the path [default to 12 voltages]
         * @param lookaheadDistance The distance that the robot looks ahead of itself in the algorithm [in inches, smaller = more oscillation, larger = less oscillation]
        */
        void FollowPath(std::vector<Bucees::Coordinates> path, float lookaheadDistance, bool reversed = false, float maxSpeed = 12); // pure pursuit
    };
}