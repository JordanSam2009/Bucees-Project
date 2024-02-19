#include "vex.h"

/**
 * @brief Iniltaize a new FAPID Controller.
 * 
 * @param settings The settings structure passed into the FAPID Controller
*/
Bucees::FAPIDController::FAPIDController(PIDSettings settings) {
    this->settings = settings;
}

float Bucees::FAPIDController::slew(float output, float prevOutput, float maxChange) {
    float change = output - prevOutput;
    if (maxChange == 0) return output;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return output + change;
}

/**
 * @brief Calculates the motor power using the target, current position, and gains/settings set
 * 
 * @param target What we are trying to reach
 * @param currentPosition Where we are at currently
*/
float Bucees::FAPIDController::calculateMotorPower(const float error) {
    
    this->error = error;
    float derivative = this->error - this->previousError;

    if (this->error <= this->settings.integralThreshold && this->settings.kI != 0 && sgn(this->error) != -1) {
        this->totalError += this->error;
    } else {
        this->totalError = 0;
    }

    float motorPower = this->error * this->settings.kP + this->totalError * this->settings.kI + derivative * this->settings.kD;

    if (motorPower > this->settings.maxVoltages) motorPower = this->settings.maxVoltages;
    if (motorPower < -this->settings.maxVoltages) motorPower = -this->settings.maxVoltages;

    if (this->settings.kA != 0) motorPower = slew(motorPower, this->previousMotorPower, this->settings.kA);

    if (fabs(error) < this->settings.exitError) { 
        this->timeSpentSettled += 10;
    } else {
        this->timeSpentSettled = 0;
    }

    this->previousMotorPower = motorPower;

    this->previousError = error;

    this->timePassed += 10;

    return motorPower;
}

/**
 * @brief Returns if the PID controller is settled by checking if it timed out or reached the exit error
*/
bool Bucees::FAPIDController::isSettled() {
    if ((fabs(error) < this->settings.exitError) && (this->timeSpentSettled > 100)) {
        std::cout << "within error" << std::endl;
        return true;
    } else if ((this->timePassed > this->settings.timeout) && (this->settings.timeout != 0)) {
        std::cout << this->timePassed << " | " << this->settings.timeout << std::endl;
        std::cout << "timed out" << std::endl;
        return true;
    }
    return false;
}

/**
 * @brief Resets the values in the PID controller to be used for the next movements
*/
void Bucees::FAPIDController::reset() {
    this->error = 0;
    this->previousError = 0;
    this->totalError = 0;
    this->timePassed = 0;
    this->timeSpentSettled = 0;
    this->previousMotorPower = 0;
}

/**
 * @brief Sets the maxmimum amount of voltages the PID controller can output
 * 
 * @param The max voltages from 0 - 12
*/
void Bucees::FAPIDController::setMaxVoltages(float maxVoltages) {
    this->settings.maxVoltages = maxVoltages;
}

/**
 * @brief Sets the amount of time the PID controller can take to complete an action
 * 
 * @param timeout_time The amount of time in miliseconds
*/
void Bucees::FAPIDController::setTimeoutTime(float timeout_time) {
    this->settings.timeout = timeout_time;
}

/**
 * @brief Sets the exit error for the PID controller to be settled
 * 
 * @param exitError The amount of error, can be in any units, dependent on the PID controller
*/
void Bucees::FAPIDController::setExitError(float exitError) {
    this->settings.exitError = exitError;
}

/**
 * @brief Sets how much error is needed before calculating integral
 * 
 * @param integralThreshold The amount of error, higher numbers = integral starts sooner
*/
void Bucees::FAPIDController::setIntegralThreshold(float integralThreshold) {
    this->settings.integralThreshold = integralThreshold;
}

/**
 * @brief Sets the gains for the PID Controller
 * 
 * @param settings The PID Settings you want to override
*/
void Bucees::FAPIDController::setGains(PIDSettings settings) {
    this->settings = settings;
}