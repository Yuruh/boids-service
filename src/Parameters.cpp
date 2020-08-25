//
// Created by antoine on 25/08/20.
//

#include "../include/Parameters.h"

Parameters::Parameters() = default;

/*
 * TODO bornes and default values
 */
Parameters& operator<<(Parameters &out, const Protobuf::Parameters &protobufParams) {
    out.alignmentScale = protobufParams.alignmentscale();
    out.separationScale = protobufParams.separationscale();
    out.cohesionScale = protobufParams.cohesionscale();
    out.avoidanceScale = protobufParams.avoidancescale();

    out.visionDistance = protobufParams.visiondistance();
    out.obstacleDistance = protobufParams.obstacledistance();
    out.separationDistance = protobufParams.separationdistance();

    out.maxLocalFlockmates = protobufParams.maxlocalflockmates();

    return out;
}

