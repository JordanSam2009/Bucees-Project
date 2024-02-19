#pragma once
#include "../vex.h"

namespace Bucees 
{

    /**
     * @var kf Feedforward gain, sets voltage by using the target [kF * target]
     * @var ka Acceleration gain, sets voltage by using the acceleration of the robot [kA * acceleration]
     * @var kp Proportional gain, sets voltage by using the error/distance between current and target [kP * error]
     * @var ki Integral gain, sets voltage by using the total error built over time [kI * totalError]
     * @var kd Derivative gain, sets voltage by using the change of error [kD * changeInError]
     * @var exiterror The amount of error needed to considered "close enough" and settled
     * @var integralthreshold How much error is needed for the integral gain to start being calculated [default to 0, meaning integral does not take effect whatsoever]
     * @var timeout_time The amount of time on default each movement used by the PID Controller should take in general [default to 0 meaning no timeout will take effect, in miliseconds]
     * @var maxVoltages The maximum voltages the PID controller can output [0 to 12 voltages]
    */
    struct PIDSettings {
        float kF; // feedforward, calculates output based on the target (basically like guessing how much power to use without looking at external information like positions)
        float kA; // acceleration limit, limits the output to prevent very fast output changes, brings the output to the target in steps
        float kP; // proportional, calculates output based on error, directly proportional to error
        float kI; // integral, calculates output based on total error, overtime it can cause a very huge output
        float kD; // derivative, calculates output based off of the change of error, overtime it can slow down the output

        float exitError; // error it takes to exit pid
        float integralThreshold;
        float timeout; // time it takes to timeout

        float maxVoltages; // the maximum amount of voltages the pid can exert
    };

    class FAPIDController 
    {
        private:

        float error;
        float totalError;
        float previousError;
        float timePassed = 0;
        float timeSpentSettled = 0;
        float previousMotorPower;

        float slew(float output, float prevOutput, float maxChange);

        public:

        PIDSettings settings;

        /**
         * @brief Iniltaize a new FAPID Controller.
         * 
         * @param settings The settings structure passed into the FAPID Controller
        */
        FAPIDController(PIDSettings settings);

        /**
         * @brief Calculates the motor power using the target, current position, and gains/settings set
         * 
         * @param target What we are trying to reach
         * @param currentPosition Where we are at currently
        */
        float calculateMotorPower(const float error);

        /**
         * @brief Returns if the PID controller is settled by checking if it timed out or reached the exit error
        */
        bool isSettled();

        /**
         * @brief Resets the values in the PID controller to be used for the next movements
        */
        void reset();

        /**
         * @brief Sets the maxmimum amount of voltages the PID controller can output
         * 
         * @param The max voltages from 0 - 12
        */
        void setMaxVoltages(float maxVoltages);

        /**
         * @brief Sets the amount of time the PID controller can take to complete an action
         * 
         * @param timeout_time The amount of time in miliseconds
        */
        void setTimeoutTime(float timeout_time);

        /**
         * @brief Sets the exit error for the PID controller to be settled
         * 
         * @param exitError The amount of error, can be in any units, dependent on the PID controller
        */
        void setExitError(float exitError);

        /**
         * @brief Sets how much error is needed before calculating integral
         * 
         * @param integralThreshold The amount of error, higher numbers = integral starts sooner
        */
        void setIntegralThreshold(float integralThreshold);

        /**
         * @brief Sets the gains for the PID Controller
         * 
         * @param settings The PID Settings you want to override
        */
        void setGains(PIDSettings settings);

    };
}

