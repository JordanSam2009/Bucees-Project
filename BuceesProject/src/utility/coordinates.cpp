#include "vex.h"

Bucees::Coordinates::Coordinates(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Bucees::Coordinates Bucees::Coordinates::operator+(Bucees::Coordinates& coordinates) {
    return Bucees::Coordinates(this->x + coordinates.x, this->y + coordinates.y, this->theta);
}

Bucees::Coordinates Bucees::Coordinates::operator-(Bucees::Coordinates& coordinates) {
    return Bucees::Coordinates(this->x - coordinates.x, this->y - coordinates.y, this->theta);    
}

Bucees::Coordinates Bucees::Coordinates::operator*(float scale) {
    return Bucees::Coordinates(this->x * scale, this->y * scale, this->theta);    
}

float Bucees::Coordinates::operator*(Bucees::Coordinates& coordinates) {
    return (this->x * coordinates.x) + (this->y * coordinates.y);
}

Bucees::Coordinates Bucees::Coordinates::operator/(Bucees::Coordinates& coordinates) {
    return Bucees::Coordinates(this->x / coordinates.x, this->y / coordinates.y, this->theta);    
}

bool Bucees::Coordinates::operator<(Bucees::Coordinates& coordinates) {
    if 
    (
        this->x < coordinates.x &&
        this->y < coordinates.y &&
        this->theta < coordinates.theta
    ) return true;
     else return false;
}

bool Bucees::Coordinates::operator>(Bucees::Coordinates& coordinates) {
    if 
    (
        this->x > coordinates.x &&
        this->y > coordinates.y &&
        this->theta > coordinates.theta
    ) return true;
     else return false;
}

bool Bucees::Coordinates::operator<=(Bucees::Coordinates& coordinates) {
    if 
    (
        this->x <= coordinates.x &&
        this->y <= coordinates.y &&
        this->theta <= coordinates.theta
    ) return true;
     else return false;
}

bool Bucees::Coordinates::operator>=(Bucees::Coordinates& coordinates) {
    if 
    (
        this->x >= coordinates.x &&
        this->y >= coordinates.y &&
        this->theta >= coordinates.theta
    ) return true;
     else return false;
}

bool Bucees::Coordinates::operator<(float number) {
    if 
    (
        this->x < number &&
        this->y < number &&
        this->theta < number
    ) return true;
     else return false;
}

bool Bucees::Coordinates::operator>(float number) {
    if 
    (
        this->x > number &&
        this->y > number &&
        this->theta > number
    ) return true;
     else return false;
}

bool Bucees::Coordinates::operator<=(float number) {
    if 
    (
        this->x <= number &&
        this->y <= number &&
        this->theta <= number
    ) return true;
     else return false;
}

bool Bucees::Coordinates::operator>=(float number) {
    if 
    (
        this->x >= number &&
        this->y >= number &&
        this->theta >= number
    ) return true;
     else return false;
}

Bucees::Coordinates Bucees::Coordinates::lerp(Bucees::Coordinates coordinates, float t) {
    return Bucees::Coordinates(this->x + (coordinates.x - this->x) * t, this->y + (coordinates.y - this->y) * t, this->theta);
}

float Bucees::Coordinates::distance(Bucees::Coordinates coordinates) {
   // printf("xdif: %f, ydif: %f \n", this->x - coordinates.x, this->y - coordinates.y);
    return std::hypotf(this->x - coordinates.x, this->y - coordinates.y);
}

float Bucees::Coordinates::angle(Bucees::Coordinates coordinates) {
    return std::atan2(coordinates.x - this->x, coordinates.y - this->y);
    // this returns angles with 0 degrees starting on the positive y axis
}