//
// Created by antoine on 10/07/20.
//

#include <cstdlib>
#include "../include/Boid.h"
#include "../include/Macros.h"

// All signatures explained on Boid.h

Pos2D Boid::getPosition() const {
    return this->position;
}

Boid::Boid(): direction(Pos2D(1, 1)), position(0, 0)
{
    this->maxForce = 0.02;
    this->maxSpeed = 1.5;
    this->minSpeed = 0.1;
}

bool Boid::operator==(const Boid &boid) const {
    return static_cast<int>(boid.position.x) == static_cast<int>(this->position.x) &&
            static_cast<int>(boid.position.y) == static_cast<int>(this->position.y);
}

void Boid::setPosition(Pos2D pos) {
    this->position = pos;
}

Pos2D Boid::getDirection() const {
    return this->direction;
}

void Boid::setDirection(const Pos2D &dir) {
    this->direction = dir;
}

bool Boid::operator!=(const Boid &boid) const {
    return !(*this == boid);
}

Boid &operator<<(Boid &out, const Protobuf::Boid &protobufBoid) {
    out.direction << protobufBoid.direction();
    out.position << protobufBoid.position();

    return out;
}

Protobuf::Boid &operator>>(const Boid &in, Protobuf::Boid &protobufBoid) {
    auto *direction = new Protobuf::Pos2D;
    auto *position = new Protobuf::Pos2D;

    in.direction >> *direction;
    in.position >> *position;

    protobufBoid.set_allocated_direction(direction);
    protobufBoid.set_allocated_position(position);

/*    auto *cohesion = new Protobuf::Pos2D;
    auto *alignment = new Protobuf::Pos2D;
    auto *separation = new Protobuf::Pos2D;
    auto *avoidance = new Protobuf::Pos2D;

    in.currentCohesion >> *cohesion;
    in.currentAlignment >> *alignment;
    in.currentSeparation >> *separation;
    in.currentAvoidance >> *avoidance;

    protobufBoid.set_allocated_cohesion(cohesion);
    protobufBoid.set_allocated_alignment(alignment);
    protobufBoid.set_allocated_separation(separation);
    protobufBoid.set_allocated_avoidance(avoidance);*/

    return protobufBoid;
}


Pos2D Boid::getCohesion(const std::vector<Boid*> &boids) const {
    Pos2D res;
    for (const Boid *boid : boids) {
        res = res + boid->position;
    }
    if (!boids.empty()) {
        res = res / boids.size();

        // Vector from location to target
        Pos2D goal = res - position;

        return this->steerToDirection(goal);
    }
    return Pos2D();
}

Pos2D Boid::getAlignment(const std::vector<Boid*> &boids) const {
    Pos2D res;
    for (const Boid *boid : boids) {
        res = res + boid->direction;
    }

    if (!boids.empty()) {
        res = res / boids.size();

        return this->steerToDirection(res);
    }
    return res;
}

Pos2D computeSeparation(Pos2D boidPos, Pos2D otherBoidPs, float distance) {
    Pos2D diff = boidPos - otherBoidPs;

    if (distance > EPSILON) {
        diff = diff / std::sqrt(std::sqrt(distance));
    }
    return diff;
}

Pos2D Boid::getSeparation(const std::vector<Boid*> &boids) const {
    Pos2D ret;

    for (const Boid *boid : boids) {
        float distance = boid->getPosition().distanceWith(this->getPosition());
        ret += computeSeparation(this->position, boid->getPosition(), distance);

    }

    if (!boids.empty()) {

        ret = ret / boids.size();

        return this->steerToDirection(ret);
    }

    return ret;
}


Pos2D Boid::getSeparation(const std::vector<std::pair<Boid *, float>> &boids) const {
    Pos2D ret;
    for (const auto elem: boids) {
        ret += computeSeparation(this->position, elem.first->position, elem.second);
    }
    if (!boids.empty()) {

        ret = ret / boids.size();

        return this->steerToDirection(ret);
    }
    return Pos2D();
}


void Boid::update(float elapsedTimeSec, const std::vector<Line> &obstacles) {
    //acceleration = acceleration * 0.4;


    // FIXME should i use acceleration here ?
    Pos2D posBefore = position;
    Pos2D posAfter = position + direction * elapsedTimeSec * 200;

    direction = direction + acceleration;

    // We don't reset acceleration as we want to stay on course
//    acceleration = acceleration * 0;
    direction.limitToMaxMagnitude(maxSpeed);

    // TODO ? have a max rotation instead
    direction.limitToMinMagnitude(minSpeed);

    Line mvt(posBefore, posAfter);
    for (const auto obstacle: obstacles) {
        // If we intersect with any obstacle, we stay still
        if (mvt.intersectsWith(obstacle)) {
            direction = direction * 0.0001;
            return;
        }
    }

    position = posAfter;
}

void Boid::setAcceleration(Pos2D acc) {
    this->acceleration = acc;

}

void Boid::addAcceleration(const Pos2D &acc) {
    acceleration = acceleration + acc;
}

Pos2D Boid::getSteerFromObstacles(const std::vector<Line> &obstacles) const {
    Pos2D ret;


    int count = 0;

    for (const auto obstacle: obstacles) {
         count++;

            Pos2D normalVector = obstacle.getNormalVector(position);

        // To go along the wall instead of directly avoid it
        // Sort of works but requires more work, and may not be needed
/*            auto vectors = obstacle.getVectors();

            auto vec1 = vectors.first + normalVector;
            auto vec2 = vectors.second + normalVector;

            if (std::abs(vectors.first.angleWithVector(this->direction)) > std::abs(vectors.second.angleWithVector(this->direction))) {
                normalVector = vec1;
            } else {
                normalVector = vec2;
            }*/


            normalVector = normalVector / std::sqrt(std::sqrt(
                    obstacle.distanceToPoint(position))); // The closer the obstacle is, the more we want to steer

            ret = ret + normalVector;
    }

    if (count > 0) {
        ret = ret / count;

        return this->steerToDirection(ret);
    }
    return ret;

}

Pos2D Boid::steerToDirection(Pos2D desiredDirection) const {

    float mag = desiredDirection.getMagnitude();

    // We compute steering on normalized vectors, we don't want magnitude to influence direction
    desiredDirection.normalize();
    Pos2D dir = this->direction;
    dir.normalize();

    Pos2D steer;

    steer = desiredDirection - direction;

    // We use the magnitude of the desired direction for the steering
    steer.setMagnitude(mag);
    steer.limitToMaxMagnitude(maxForce); // Limit to max steering force
    return steer;
}

const std::vector<Boid> Boid::getClosestBoids(const std::vector<Boid> &boids, float maxDistance, float maxQty) const {

    std::vector<Boid> closeBoids;
    std::map<float, Boid> distanceToBoids;

    for (const Boid &boid : boids) {
        float distance = boid.getPosition().distanceWith(this->getPosition());

        // Exclude current boid and only accept boids closer than maxDistance
        if ((distance > EPSILON) && (distance < maxDistance)) {
            distanceToBoids[distance] = boid;
        }
    }


    // Items in map are sorted
    for (int i = 0; i < maxQty && i < distanceToBoids.size(); ++i) {
        closeBoids.push_back(distanceToBoids.begin()->second);
        distanceToBoids.erase(distanceToBoids.begin());
    }
    return closeBoids;

}

void Boid::setRulesResult(const Pos2D &cohesion, const Pos2D &alignment, const Pos2D &separation, const Pos2D &avoidance) {
    this->currentCohesion = cohesion;
    this->currentAlignment = alignment;
    this->currentSeparation = separation;
    this->currentAvoidance = avoidance;
}

void Boid::storeCourse() {
    this->course = this->acceleration;
}

