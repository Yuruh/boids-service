//
// Created by antoine on 10/07/20.
//

#include <cstdlib>
#include "../include/Boid.h"
#include "../include/Macros.h"


Pos2D Boid::getPosition() const {
    return this->position;
}

Boid::Boid(): direction(Pos2D(std::rand() % 2 - 1, std::rand() % 2 - 1)), position(std::rand() % WIDTH, std::rand() % HEIGHT)
{
    this->speed = 100;
    this->maxForce = 0.5;
    this->maxSpeed = 3.5;

    this->display = 'a' + std::rand() % 26;
}

char Boid::getDisplay() const {
    return this->display;
}

/*
 * We consider two boids equal if they have the same location (they should never overlap)
 */
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
//    this->direction.normalize();
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
    return protobufBoid;
}

float Boid::getSpeed() const {
    return speed;
}

Pos2D Boid::getCohesion(const std::vector<Boid> &boids) const {
    Pos2D res;
    int count = 0;
    for (const Boid &boid : boids) {
        float distance = boid.getPosition().distanceWith(this->getPosition());

        if ((distance > EPSILON) && (distance < VISION_DISTANCE)) {
            res = res + boid.position;
            count++;
        }
    }
    if (count > 0) {
        res = res / count;

        // Vector from location to target
        Pos2D goal = res - position;

        // Scale to max speed
        goal.normalize();
        goal = goal * maxSpeed;

        Pos2D steer(goal.x, goal.y);
        steer = goal - direction;
        steer.limitToMaxMagnitude(maxForce); // Limit to max steering force
        return steer;
    }
    return Pos2D();
}

Pos2D Boid::getAlignment(const std::vector<Boid> &boids) const {
    Pos2D res;
    int count = 0;
    for (const Boid &boid : boids) {
        float distance = boid.getPosition().distanceWith(this->getPosition());

        if ((distance > EPSILON) && (distance < VISION_DISTANCE)) {
            res = res + boid.direction;
            count++;
        }
    }
    if (count > 0) {
        res = res / count;

        // Scale to max speed
        res.normalize();
        res = res * maxSpeed;

        Pos2D steer(res.x, res.y);
        steer = res - direction;
        steer.limitToMaxMagnitude(maxForce); // Limit to max steering force
        return steer;
    }
    return res;
}

Pos2D Boid::getSeparation(const std::vector<Boid> &boids) const {
    Pos2D ret;
    int count = 0;
    for (const Boid &boid : boids) {
        float distance = boid.getPosition().distanceWith(this->getPosition());

        if ((distance > EPSILON) && (distance < SEPARATION_DISTANCE)) {
            Pos2D oppositeWay = ret - (boid.getPosition() - this->getPosition());

            oppositeWay.normalize();
            oppositeWay = oppositeWay / distance; // The closer the other boid is, the more we want to steer
            ret += oppositeWay;

            count++;

        }
    }
    if (count > 0) {
        ret = ret / count;

        // Scale to max speed
        ret.normalize();
        ret = ret * maxSpeed;

        Pos2D steer(ret.x, ret.y);
        steer = ret - direction;
        steer.limitToMaxMagnitude(maxForce); // Limit to max steering force
    }
    return ret;
}

void Boid::update(float elapsedTimeSec) {
    //acceleration = acceleration * 0.4;

    direction = direction + acceleration;
    direction.limitToMaxMagnitude(maxSpeed);
    position = position + direction * elapsedTimeSec * 100;
    acceleration = acceleration * 0;
}

void Boid::addAcceleration(const Pos2D &acc) {
    acceleration = acceleration + acc;
}
