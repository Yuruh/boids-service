//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_FLOCK_H
#define BOIDS_FLOCK_H


#include <vector>
#include "Boid.h"
#include "Map.h"

class Flock {
private:
    std::vector<Boid> boids;

public:
//    explicit Flock(const Map &map);
    void addBoid(const Boid &boid);
    const std::vector<Boid> getBoids() const;
    void update(float elapsedTimeSec, const Map &map);

    friend Flock& operator<<(Flock &out, const Protobuf::Flock &protobufFlock);
    friend Protobuf::Flock& operator>>(const Flock &out, Protobuf::Flock &protobufFlock);

    std::vector<Pos2D> getCloseObstaclesNormalVectors(const Map &map) const;
};


#endif //BOIDS_FLOCK_H
