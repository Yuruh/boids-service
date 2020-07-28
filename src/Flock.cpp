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

void Flock::update(float elapsedTimeSec, const Map &map) {


    for (Boid &boid : this->boids) {
        Pos2D dir = boid.getDirection();
        dir.normalize();
        boid.setDirection(dir);

        auto closeBoids = boid.getCloseBoids(boids);


        std::vector<Line> closeObstacles = map.closeObstacles(boid.getPosition());
        // steer to avoid obstacle
        Pos2D avoidObstacle = boid.getSteerFromObstacles(closeObstacles) * 0.2;
        boid.addAcceleration(avoidObstacle);




        // If we meet an obstacle, it becomes priority to avoid it
        // steer to move towards the average position (center of mass) of local flockmates
        Pos2D cohesion = boid.getCohesion(closeBoids) * 0.15;
        boid.addAcceleration(cohesion);

        // steer to avoid crowding local flockmates
        Pos2D separation = boid.getSeparation(closeBoids) * 0.15;
        boid.addAcceleration(separation);


        // steer towards the average heading of local flockmates
        Pos2D alignment = boid.getAlignment(closeBoids) * 0.1;
        boid.addAcceleration(alignment);

        boid.update(elapsedTimeSec, closeObstacles);
    }
}

Flock &operator<<(Flock &out, const Protobuf::Flock &protobufFlock) {
    protobufFlock.boids().size();

    for (int i = 0; i < protobufFlock.boids().size() && i < MAX_NUMBER_BOIDS; ++i) {
        Boid boid;
        boid << protobufFlock.boids(i);
        out.boids.push_back(boid);
    }
    return out;
}

Protobuf::Flock &operator>>(const Flock &out, Protobuf::Flock &protobufFlock) {
    protobufFlock.clear_boids();

    for (const auto &i : out.boids) {
        auto *boid = protobufFlock.add_boids();

        i >> *boid;
    }
    return protobufFlock;
}
