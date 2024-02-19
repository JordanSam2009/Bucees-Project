#pragma once
#include "../vex.h"

namespace Bucees {

    namespace Omniwheel {
        constexpr float NEW_275 = 2.75;
        constexpr float OLD_275 = 2.75;
        constexpr float NEW_275_HALF = 2.744;
        constexpr float OLD_275_HALF = 2.74;
        constexpr float NEW_325 = 3.25;
        constexpr float OLD_325 = 3.25;
        constexpr float NEW_325_HALF = 3.246;
        constexpr float OLD_325_HALF = 3.246;
        constexpr float NEW_4 = 4;
        constexpr float OLD_4 = 4.18;
        constexpr float NEW_4_HALF = 3.995;
        constexpr float OLD_4_HALF = 4.175;
    };

    class TrackingWheel {
        private:
        // sensors:
        vex::motor_group* motorGroup = nullptr;
        vex::rotation rotationSensor = NULL;
        vex::rotation* detectRotationSensor = nullptr;

        // settings:
        float wheelDiameter;
        float offset;
        float gearRatio;

        public:

        TrackingWheel(vex::motor_group* motorGroup, float wheelDiameter, float offset, float gearRatio);
        TrackingWheel(int32_t rotationSensor, bool reversed, float wheelDiameter, float offset, float gearRatio);

        void resetEncoders();
        float getOffset();
        float getDistanceTraveled();
    };
}