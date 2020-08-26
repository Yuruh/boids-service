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
        std::vector<Line> closeObstacles = map.closeObstacles(boid.getPosition(), this->params.obstacleDistance);

        auto closeBoids = boid.getClosestBoids(boids, this->params.visionDistance, this->params.maxLocalFlockmates);


        // steer to avoid obstacle
        Pos2D avoidObstacle = boid.getSteerFromObstacles(closeObstacles) * OBSTACLES_COEFF * this->params.avoidanceScale;
        boid.addAcceleration(avoidObstacle);




        // If we meet an obstacle, it becomes priority to avoid it
        // steer to move towards the average position (center of mass) of local flockmates
        Pos2D cohesion = boid.getCohesion(closeBoids) * COHESION_COEFF * this->params.cohesionScale;
        boid.addAcceleration(cohesion);

        auto closeBoidsToAvoid = boid.getClosestBoids(boids, this->params.separationDistance, this->params.maxLocalFlockmates);
        // steer to avoid crowding local flockmates
        Pos2D separation = boid.getSeparation(closeBoidsToAvoid) * SEPARATION_COEFF * this->params.separationScale;
        boid.addAcceleration(separation);



        // steer towards the average heading of local flockmates
        Pos2D alignment = boid.getAlignment(closeBoids) * ALIGNMENT_COEFF * this->params.alignmentScale;
        boid.addAcceleration(alignment);

        boid.setRulesResult(cohesion, alignment, separation, avoidObstacle);

        // Boids always try to speed up to their max speed
        auto dir = boid.getDirection();
        dir.normalize();
        boid.addAcceleration(dir * STANDARD_ACCELERATION);

        boid.update(elapsedTimeSec, closeObstacles);
    }
}        // If We are avoiding an obstacle, other rules should matter less


Flock &operator<<(Flock &out, const Protobuf::Flock &protobufFlock) {
    protobufFlock.boids().size();

    for (int i = 0; i < protobufFlock.boids().size() && i < MAX_NUMBER_BOIDS; ++i) {
        Boid boid;
        boid << protobufFlock.boids(i);
        out.boids.push_back(boid);
    }
    return out;
}

Protobuf::Flock &operator>>(const Flock &in, Protobuf::Flock &protobufFlock) {
    protobufFlock.clear_boids();

    for (const auto &i : in.boids) {
        auto *boid = protobufFlock.add_boids();

        i >> *boid;
    }
    return protobufFlock;
}

std::pair<std::vector<Pos2D>, std::vector<Pos2D>> Flock::getCloseObstaclesNormalVectors(const Map &map) const {
//    std::vector<Pos2D> ret;
    std::vector<Pos2D> positions;

    std::pair<std::vector<Pos2D>, std::vector<Pos2D>> ret;

    for (const Boid &boid : this->boids) {
        std::vector<Line> closeObstacles = map.closeObstacles(boid.getPosition(), this->params.obstacleDistance);

        for (const auto obstacle: closeObstacles) {

            // TODO got to also send the point in the middle of the line
            Pos2D normalVector = obstacle.getNormalVector(boid.getPosition());



/*            auto vectors = obstacle.getVectors();

            if (std::abs(vectors.first.angleWithVector(boid.getDirection())) > std::abs(vectors.second.angleWithVector(boid.getDirection()))) {
                normalVector = vectors.first + normalVector;
            } else {
                normalVector = vectors.second + normalVector;
            }*/



            normalVector = normalVector / std::sqrt(std::sqrt(obstacle.distanceToPoint(boid.getPosition()))); // The closer the obstacle is, the more we want to steer


            ret.first.push_back(normalVector);
            ret.second.push_back(obstacle.getHalfPoint());
        }
    }

    return ret;
}

void Flock::setParams(const Parameters &params) {
    this->params = params;

}
