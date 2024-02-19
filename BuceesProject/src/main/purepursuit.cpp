#include "vex.h"

float lookAheadDistance;
float distanceTraveled;

int findClosestPoint(Bucees::Coordinates point, std::vector<Bucees::Coordinates> path) {
    int closestPoint = 0;
    float closestDistance = 10000000; // set this so loop can run
    float dist;

    // index through all path points
    for (unsigned int i = 0; i < path.size(); i++) {
        dist = point.distance(path.at(i));
        if (dist < closestDistance) { // new closest point
            closestDistance = dist;
            closestPoint = i;
        }
    }

    //printf("Closest point found! \n");

    return closestPoint;
};

float circleIntersect(Bucees::Coordinates currentPosition, Bucees::Coordinates point1, Bucees::Coordinates point2) {
    // quadratic formula for circle intersect:
    Bucees::Coordinates d = point2 - point1;
    Bucees::Coordinates f = point1 - currentPosition;

    float a = d * d;
    float b = 2 * (f * d);
    float c = (f * f) - lookAheadDistance * lookAheadDistance;
    float discriminant = b * b - 4 * a * c;

    // a possible intersection was found
    if (discriminant >= 0) {
        discriminant = sqrtf(discriminant);

        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        //printf("Intersection was found! \n");

        // priortize further down the path:
        if (t2 >= 0 && t2 <= 1) return t2;
        else if (t1 >= 0 && t1 <= 1) return t1;
    }

    // no intersection found:
   // printf("No intersection was found! \n");
    return -1;
};

Bucees::Coordinates findLookAheadPoint(Bucees::Coordinates currentPosition, Bucees::Coordinates lastLookAhead, int closest, std::vector<Bucees::Coordinates> path) {

   // printf("%f, %f \n", path.size(), path.at(lastLookAhead.theta));

    for (int i = path.size() - 1; i > -1; i--) {
        // since we are searching in reverse, instead of getting
        // the current pose and the next one, we should get the
        // current pose and the last one
        printf("IN LOOP \n");
        Bucees::Coordinates lastPathCoordinates = path.at(i - 1);
        Bucees::Coordinates currentPathCoordinates = path.at(i);

        float t = circleIntersect(lastPathCoordinates, currentPathCoordinates, currentPosition);

        if (t != -1) {
            Bucees::Coordinates lookahead = lastPathCoordinates.lerp(currentPathCoordinates, t);
            lookahead.theta = i;
            printf("Look ahead point found. \n");
            return lookahead;
        }
    }

    // robot deviated from path, use last lookahead point
    printf("No lookahead point found. \n");
    return lastLookAhead;
};

/**
 * @brief Follow a set of coordinates using the pure pursuit motion algorithm and a path planner
 * 
 * @param path A vector containing coordinates to follow
 * @param lookaheadDistance The distance that the robot looks ahead of itself in the algorithm [in inches, smaller = more oscillation, larger = less oscillation]
 * @param maxSpeed The max speed the robot is allowed to go while following the path [default to 12 voltages]
*/
void Bucees::Robot::FollowPath(std::vector<Coordinates> path, float lookaheadDistance, bool reversed, float maxSpeed) {

    lookAheadDistance = lookaheadDistance;

    Coordinates initialCoordinates = this->getRobotCoordinates();
    Coordinates lookaheadCoordinates(0, 0, 0);
    Coordinates lastLookAhead = path.at(0);

    float curvature;

    int closestPoint;

    distanceTraveled = 0;

    while (1) {

        Bucees::Coordinates currentCoordinates = this->getRobotCoordinates(true, reversed);

        //if (reversed == true) currentCoordinates.theta -= M_PI;

        printf("x: %f, y: %f, theta: %f \n", currentCoordinates.x, currentCoordinates.y, currentCoordinates.theta);

        // Step 1: Finding the closest point
        closestPoint = findClosestPoint(currentCoordinates, path);

        // Step 2: Find the lookahead point
        lookaheadCoordinates = findLookAheadPoint(currentCoordinates, lastLookAhead, closestPoint, path);

        // Step 3: Calculating the curvature of the arc to the lookahead point
        float curvatureHeading = M_PI / 2 - currentCoordinates.theta; // basically it's the heading perpendicular to it's current heading so that paths can be followed smoothly if its curvy
        curvature = findCurvature(currentCoordinates, lookaheadCoordinates, curvatureHeading);

        // Step 4: Calculating the left/right wheel velocities
        float targetPointVelocity = path.at(closestPoint).theta;
        targetPointVelocity *= (12.f/127.f); // stupid c++ being gay, basically converting from PROS byte voltage to vex voltages (smh)

        float targetLeftVelocity = targetPointVelocity * (2 + curvature * this->drivetrainTrackWidth) / 2;
        float targetRightVelocity = targetPointVelocity * (2 - curvature * this->drivetrainTrackWidth) / 2;

        float ratio = std::max(std::fabs(targetLeftVelocity), std::fabs(targetRightVelocity)) / maxSpeed;

        if (ratio > 1) { // makes it so it doesn't go over the max speed
            targetLeftVelocity /= ratio;
            targetRightVelocity /= ratio;
        }

        //printf("left: %f \n", targetLeftVelocity);
        //printf("right: %f \n", targetRightVelocity);

        if (reversed != true) {
            LeftSide->spin(vex::directionType::fwd, targetLeftVelocity, vex::voltageUnits::volt);
            RightSide->spin(vex::directionType::fwd, targetRightVelocity, vex::voltageUnits::volt);
        } else {
            LeftSide->spin(vex::directionType::fwd, -targetRightVelocity, vex::voltageUnits::volt);
            RightSide->spin(vex::directionType::fwd, -targetLeftVelocity, vex::voltageUnits::volt);
        }

        // extra: 
        distanceTraveled = initialCoordinates.distance(currentCoordinates);
        lastLookAhead = lookaheadCoordinates;

        printf("velocity: %f \n", path.at(closestPoint).theta);

        // exit condition:
        if (path.at(closestPoint).theta == 0) break;

        vex::wait(10, vex::msec);
    }

    LeftSide->stop(vex::brakeType::coast);
    RightSide->stop(vex::brakeType::coast);

    printf("STOPPED FOLLOWING PATH.");
  }