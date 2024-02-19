#pragma once
#include "../utility/coordinates.hpp"

class Odometry {
    private: 

    float rightTrackerDistance;
    float backTrackerDistance;

    float previousBackPosition = 0;
    float previousRightPosition = 0;
    float previousIMU = 0;

    public:

    Bucees::Coordinates updatePosition(Bucees::Coordinates previousCoordinates, float rightTrackerPosition, float backTrackerPosition, float currentIMU, bool reversed = false);

    void setOdometry(float rightTrackerDistance, float backTrackerDistance);

};