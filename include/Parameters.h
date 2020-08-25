//
// Created by antoine on 25/08/20.
//

#ifndef BOIDS_PARAMETERS_H
#define BOIDS_PARAMETERS_H


#include "map.pb.h"

struct Parameters {
    Parameters();
    float alignmentScale;
    float separationScale;
    float cohesionScale;
    float avoidanceScale;

    uint32_t visionDistance;
    uint32_t obstacleDistance;
    uint32_t separationDistance;

    uint32_t maxLocalFlockmates;


    friend Parameters& operator<<(Parameters &out, const Protobuf::Parameters &protobufParams);

};


#endif //BOIDS_PARAMETERS_H
