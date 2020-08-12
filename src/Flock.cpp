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
        std::vector<Line> closeObstacles = map.closeObstacles(boid.getPosition());
/*        Pos2D dir = boid.getDirection();
        dir.normalize();
        boid.setDirection(dir);*/

        auto closeBoids = boid.getClosestBoids(boids, VISION_DISTANCE, MAX_LOCAL_FLOCKMATES);


        // steer to avoid obstacle
        Pos2D avoidObstacle = boid.getSteerFromObstacles(closeObstacles) * OBSTACLES_COEFF;
        boid.addAcceleration(avoidObstacle);


        // If we meet an obstacle, it becomes priority to avoid it
        // steer to move towards the average position (center of mass) of local flockmates
        Pos2D cohesion = boid.getCohesion(closeBoids) * COHESION_COEFF;
        boid.addAcceleration(cohesion);

        auto closeBoidsToAvoid = boid.getClosestBoids(boids, VISION_DISTANCE / 2, MAX_LOCAL_FLOCKMATES);
        // steer to avoid crowding local flockmates
        Pos2D separation = boid.getSeparation(closeBoidsToAvoid) * SEPARATION_COEFF;
        boid.addAcceleration(separation);


        // steer towards the average heading of local flockmates
        Pos2D alignment = boid.getAlignment(closeBoids) * ALIGNMENT_COEFF;
        boid.addAcceleration(alignment);

        // Boids always try to speed up to their max speed
        auto dir = boid.getDirection();
        dir.normalize();
        boid.addAcceleration(dir * STANDARD_ACCELERATION);

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
