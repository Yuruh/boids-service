//
// Created by antoine on 10/07/20.
//

#include <cstdlib>
#include "../include/Boid.h"
#include "../include/Macros.h"


Pos2D Boid::getPosition() const {
    return this->position;
}

Boid::Boid(): direction(Pos2D(std::rand() % 2 - 1, std::rand() % 2 - 1)), position(0, 0)
{
    this->speed = 100;
    this->maxForce = 0.7;
    this->maxSpeed = 3.5;
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

        return this->steerToGoal(goal);
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
        return this->steerToGoal(res);
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

        return this->steerToGoal(ret);
    }
    return ret;
}

void Boid::update(float elapsedTimeSec, const std::vector<Line> &obstacles) {
    //acceleration = acceleration * 0.4;

    direction = direction + acceleration;
    acceleration = acceleration * 0;
    direction.limitToMaxMagnitude(maxSpeed);

    Pos2D posBefore = position;
    Pos2D posAfter = position + direction * elapsedTimeSec * 250;

    Line mvt(posBefore, posAfter);
    for (const auto obstacle: obstacles) {
        // If we intersect with any obstacle, we stay still
        if (mvt.intersectsWith(obstacle)) {
            return;
        }
    }

    position = posAfter;
}

void Boid::addAcceleration(const Pos2D &acc) {
    acceleration = acceleration + acc;
}

Pos2D Boid::getSteerFromObstacles(const std::vector<Line> &obstacles) const {
    Pos2D ret;

//    std::cout << "Steering from obstacle" << std::endl;
    for (const auto obstacle: obstacles) {
        Pos2D reflected = obstacle.reflectedVector(direction);

        // We'll use the normal vector instead of the reflexion if we are parallel to the obstacle
        if (!(reflected == Pos2D())) {
            reflected = obstacle.getNormalVector(position) * 0.5;
        }

        reflected.normalize();
//        reflected = reflected + obstacle.getNormalVector(direction) * 0.5; // So boids don't lean against the fence
        //reflected.normalize();
        reflected = reflected / obstacle.distanceToPoint(position); // The closer the obstacle is, the more we want to steer

//        Pos2D steer = this->steerToGoal(reflected);

/*        std::cout << "obstacle = " << obstacle << std::endl;
        std::cout << "direction = " << direction << std::endl;
        std::cout << "angle = " << static_cast<int>(RAD_TO_DEG(obstacle.angleWithVector(direction))) << std::endl;
        std::cout << "reflect vec = " << reflected << std::endl;*/
  //      std::cout << "steer = " << steer << std::endl;


        ret = ret + reflected;

        // We steer away from each obstacle
    //    ret = ret + steer;
    }
    if (!obstacles.empty()) {
        ret = ret / obstacles.size();
/*        std::cout << "Overall reflect = " << ret << std::endl;
        std::cout << "final steer = " << this->steerToGoal(ret) << std::endl << std::endl;*/
        return this->steerToGoal(ret);
    }
    return ret;

}

Pos2D Boid::steerToGoal(Pos2D goal) const {
// Scale to max speed
    goal.normalize();
    goal = goal * maxSpeed;

//    std::cout << "Goal normalized = " << goal << std::endl;
    Pos2D steer;
    // Steering = Desired minus Velocity
    steer = goal - direction;

    steer.limitToMaxMagnitude(maxForce); // Limit to max steering force
    return steer;
}
