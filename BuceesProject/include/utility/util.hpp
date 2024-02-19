#include "coordinates.hpp"

int sgn(float num);
float to_wheel_travel(float input, float diameter, float gearRatio);
float to_rad(float angle_deg);
float to_deg(float angle_rad);
float findCurvature(Bucees::Coordinates currentPosition, Bucees::Coordinates point, float theta);