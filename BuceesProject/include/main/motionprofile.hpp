#pragma once
#include "../vex.h"

//https://www.youtube.com/watch?v=8319J1BEHwM

/*
Kinematic Variables:
Displacement: delta x
Inital Velocity: v0
Final Velocity: v
Time interval: t
Constant acceleration: a

Kinematics Equations:
v = v0 + at
deltaX = ( (v+ v0) / 2 ) * t
deltaX = v0 * t + 1/2(a * t^2)
v^2 = v0^2 + 2 * a * deltaX

*/

/*
1D Trapizodal Motion Profile:
First Step: Find the cruising velocity [maximum velocity for robot or highest velocity possible in given distance]
Second Step: Figure out how much distance you will cover from starting to cruising as well as cruising to deaccelerate
Equation would be: xf = xi + v * t + 1/2(a) * t^2
Third Step: Find how long you stay at the cruise velocity
Total distance = distance during accel + distance during deacel + distance during cruise

*/

namespace Bucees {
    class motionProfile {
        private:

        float startPosition;
        float endPosition;
        float maximumVelocity;

        uint32_t currentTime;

        float totalTime;

        float accelerationTime;
        float constantVelocityTime;
        float deaccelerationTime;
       
        float totalDistanceCovered;
        float accelDistanceCovered;
        float deAccelDistanceCovered;

        float velocityCalculation0();
        float velocityCalculation1();
        float displacementCalculation0();
        float displacementCalculation1();



    }
}