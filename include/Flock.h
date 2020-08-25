//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_FLOCK_H
#define BOIDS_FLOCK_H


#include <vector>
#include "Boid.h"
#include "Map.h"
#include "Parameters.h"

class Flock {
private:
    std::vector<Boid> boids;

    Parameters params;

public:
//    explicit Flock(const Map &map);
    void addBoid(const Boid &boid);
    const std::vector<Boid> getBoids() const;
    void update(float elapsedTimeSec, const Map &map);

    void setParams(const Parameters &params);

    friend Flock& operator<<(Flock &out, const Protobuf::Flock &protobufFlock);
    friend Protobuf::Flock& operator>>(const Flock &out, Protobuf::Flock &protobufFlock);

    std::pair<std::vector<Pos2D>, std::vector<Pos2D>> getCloseObstaclesNormalVectors(const Map &map) const;
};


#endif //BOIDS_FLOCK_H
