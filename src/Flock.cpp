//
// Created by antoine on 10/07/20.
//

#include <iostream>
#include "../include/Flock.h"
#include "../include/Macros.h"

Pos2D Flock::centreOfMass() const {
    if (this->boids.empty()) {
        return Pos2D();
    }
    Pos2D accumulator(0, 0);


    for (const Boid &boid : this->boids) {
        accumulator += boid.getPosition();
    }

    return accumulator / this->boids.size();
}


Pos2D Flock::centreOfDirection() const {
    Pos2D accumulator(0, 0);

    for (const Boid &boid : this->boids) {
        accumulator += boid.getDirection();
    }

//    accumulator.normalize();

    return accumulator;
}

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

Pos2D Flock::avoidVector(const Boid &boid) {
    Pos2D ret;
    int count = 0;

    int distanceMin = SEPARATION_DISTANCE;

    for (const Boid &boid_it : this->boids) {
        float distance = boid_it.getPosition().distanceWith(boid.getPosition());
        if ((distance > EPSILON) && (distance < distanceMin)) {
            Pos2D oppositeWay = ret - (boid_it.getPosition() - boid.getPosition());

            oppositeWay.normalize();
            oppositeWay = oppositeWay / distance; // The closer the other boid is, the more we want to steer
            ret += oppositeWay;

            count++;
        }

    }
    ret.normalize();
    if (count > 0) {
        ret = ret / count;
    }
    return ret;
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
