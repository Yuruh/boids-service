//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_MACROS_H
#define BOIDS_MACROS_H

#include <cmath>

#define EPSILON 0.000001

#define SEPARATION_DISTANCE 50

// Most of those values could / should be in the protocol, so client can adapt with canvas size

#define OBSTACLE_DISTANCE 200
#define VISION_DISTANCE 150
#define MAX_LOCAL_FLOCKMATES 20


#define MAX_NUMBER_BOIDS 1000
#define MAX_FPS 60
#define MAX_SIM_SECONDS 30

#define BOIDS_DECISION_RATE 1 / MAX_FPS

#define SEPARATION_COEFF 1.2
#define COHESION_COEFF 1.1
#define ALIGNMENT_COEFF 1
#define OBSTACLES_COEFF 0.7

// base acceleration
#define STANDARD_ACCELERATION 0.03

// Converts degrees to radians.
#define DEG_TO_RAD(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define RAD_TO_DEG(angleRadians) (angleRadians * 180.0 / M_PI)

#endif //BOIDS_MACROS_H
