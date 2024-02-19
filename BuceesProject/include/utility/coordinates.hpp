#pragma once
#include "../vex.h"

namespace Bucees
{
    class Coordinates 
    {
        public:
        float x;
        float y;
        float theta;

        Coordinates(float x, float y, float theta = 0);

        Coordinates operator+(Coordinates& coordinates);
        Coordinates operator-(Coordinates& coordinates);
        Coordinates operator*(float scale);
        float operator*(Coordinates& coordinates);
        Coordinates operator/(Coordinates& coordinates);
        bool operator<(Coordinates& coordinates);
        bool operator>(Coordinates& coordinates);
        bool operator<=(Coordinates& coordinates);
        bool operator>=(Coordinates& coordinates);
        bool operator<(float number);
        bool operator>(float number);
        bool operator<=(float number);
        bool operator>=(float number);

        Coordinates lerp(Coordinates coordinates, float t);
        float distance(Coordinates coordinates);
        float angle(Coordinates coordinates);
    };
}