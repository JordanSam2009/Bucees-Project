#include "vex.h"

Bucees::TrackingWheel::TrackingWheel(vex::motor_group *motorGroup, float wheelDiameter, float offset, float gearRatio) :
    motorGroup(motorGroup)
{
    std::cout << "TRACKING WHEEL IS USING INTERNAL MOTOR ENCODERS." << std::endl;
    this->wheelDiameter = wheelDiameter;
    this->offset = offset;
    this->gearRatio = gearRatio;   
}

Bucees::TrackingWheel::TrackingWheel(int32_t rotationSensor, bool reversed, float wheelDiameter, float offset, float gearRatio)  : 
    rotationSensor(vex::rotation(rotationSensor, reversed))
{
    if (this->rotationSensor.installed() != true) {
        std::cout << "ROTATION SENSOR NOT INSTALLED. CHECK THE PORT." << std::endl;
    }

    std::cout << "TRACKING WHEEL IS USING ROTATION SENSOR." << std::endl;
    this->wheelDiameter = wheelDiameter;
    this->offset = offset;
    this->gearRatio = gearRatio;
    this->detectRotationSensor = &this->rotationSensor;
}

void Bucees::TrackingWheel::resetEncoders() {
    if (this->detectRotationSensor != nullptr) {
        this->rotationSensor.resetPosition();
    } else if (this->motorGroup != nullptr) {
        this->motorGroup->resetPosition();
    }
}

float Bucees::TrackingWheel::getOffset() {
    return this->offset;
}

float Bucees::TrackingWheel::getDistanceTraveled() {
    if (this->detectRotationSensor != nullptr) {
        return to_wheel_travel(this->rotationSensor.position(vex::rotationUnits::deg), this->wheelDiameter, this->gearRatio);
    } else if (this->motorGroup != nullptr) {
        return to_wheel_travel(this->motorGroup->position(vex::rotationUnits::deg), this->wheelDiameter, this->gearRatio);
    }

    return -999999;
}
