//
// Created by antoine on 10/07/20.
//

#include <iostream>
#include "../include/Flock.h"
#include "../include/Macros.h"

void Flock::addBoid(const Boid &boid) {
    this->boids.push_back(boid);
}

const std::vector<Boid> Flock::getBoids() const {
    return this->boids;
}

// TODO take map as input or smth
void Flock::update(float elapsedTimeSec, const Map &map) {

    for (Boid &boid : this->boids) {
        Pos2D dir = boid.getDirection();
        dir.normalize();
        boid.setDirection(dir);

        // steer to move towards the average position (center of mass) of local flockmates
        Pos2D cohesion = boid.getCohesion(boids) * 0.1;

        boid.addAcceleration(cohesion);

        // steer to avoid crowding local flockmates
        Pos2D separation = boid.getSeparation(boids) * 0.13;
        boid.addAcceleration(separation);

        // steer towards the average heading of local flockmates
        Pos2D alignment = boid.getAlignment(boids) * 0.1;
        boid.addAcceleration(alignment);

        std::vector<Line> closeObstacles = map.closeObstacles(boid.getPosition());
        Pos2D avoidObstacle = boid.getSteerFromObstacles(closeObstacles) * 0.15;

        //std::cout << "avoid direction: " << avoidObstacle << std::endl;
        boid.addAcceleration(avoidObstacle);

        boid.update(elapsedTimeSec, map.getDimensions());
    }
}

Flock &operator<<(Flock &out, const Protobuf::Flock &protobufFlock) {
    protobufFlock.boids().size();

    for (int i = 0; i < protobufFlock.boids().size(); ++i) {
        Boid boid;
        boid << protobufFlock.boids(i);
        out.boids.push_back(boid);
    }
    return out;
}

Protobuf::Flock &operator>>(const Flock &out, Protobuf::Flock &protobufFlock) {
    protobufFlock.clear_boids();

    for (int i = 0; i < out.boids.size(); ++i) {
        auto *boid = protobufFlock.add_boids();

        out.boids[i] >> *boid;
    }
    return protobufFlock;
}
