//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_MACROS_H
#define BOIDS_MACROS_H

#include <cmath>

#define EPSILON 0.000001

#define SEPARATION_DISTANCE 50
#define OBSTACLE_DISTANCE 50
#define VISION_DISTANCE 150
#define MAX_LOCAL_FLOCKMATES 10


#define MAX_NUMBER_BOIDS 1000
#define MAX_FPS 60
#define MAX_SIM_SECONDS 30


// Converts degrees to radians.
#define DEG_TO_RAD(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define RAD_TO_DEG(angleRadians) (angleRadians * 180.0 / M_PI)

#endif //BOIDS_MACROS_H
