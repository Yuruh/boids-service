//
// Created by antoine on 10/07/20.
//

#include <cstdlib>
#include "../include/Boid.h"
#include "../include/Macros.h"


/*
 * todo
 * quadtree optimization: chaque case de la taille de la distance de vision, et on ne check que notre case et les cases aux alentours.
 * radius vision, and don't see across obstacles
 *
 */
Pos2D Boid::getPosition() const {
    return this->position;
}

Boid::Boid(): direction(Pos2D(1, 1)), position(0, 0)
{
    this->maxForce = 0.2;

    this->maxSteerAngle = M_PI / 4; // 45°

    this->maxSpeed = 1;
    this->minSpeed = 0.4;
    this->weight = 1;
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
 //   std::cout << "Boid pos " << out.position << std::endl;

    return out;
}


Protobuf::Boid &operator>>(const Boid &in, Protobuf::Boid &protobufBoid) {
    auto *direction = new Protobuf::Pos2D;
    auto *position = new Protobuf::Pos2D;

    in.direction >> *direction;
    in.position >> *position;

    protobufBoid.set_allocated_direction(direction);
    protobufBoid.set_allocated_position(position);

    auto *cohesion = new Protobuf::Pos2D;
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
    protobufBoid.set_allocated_avoidance(avoidance);

    return protobufBoid;
}


Pos2D Boid::getCohesion(const std::vector<Boid> &boids) const {
    Pos2D res;
    for (const Boid &boid : boids) {
        res = res + boid.position;
    }
    if (!boids.empty()) {
        res = res / boids.size();

        // Vector from location to target
        Pos2D goal = res - position;

        return this->steerToGoal(goal);
    }
    return Pos2D();
}

Pos2D Boid::getAlignment(const std::vector<Boid> &boids) const {
    Pos2D res;
    for (const Boid &boid : boids) {
        res = res + boid.direction;
    }

    if (!boids.empty()) {
        res = res / boids.size();

        return this->steerToGoal(res);
    }
    return res;
}

Pos2D Boid::getSeparation(const std::vector<Boid> &boids) const {
    Pos2D ret;

    for (const Boid &boid : boids) {
        float distance = boid.getPosition().distanceWith(this->getPosition());

        Pos2D diff = this->position - boid.getPosition();

        if (distance > EPSILON) {
            diff = diff / distance;
        }
        ret += diff;
//        Pos2D oppositeWay = ret - (boid.getPosition() - this->getPosition());

//        oppositeWay.normalize();
//        oppositeWay = oppositeWay / distance; // The closer the other boid is, the more we want to steer
 //       ret += oppositeWay;

    }

    if (!boids.empty()) {

        ret = ret / boids.size();

        return this->steerToGoal(ret);
    }

    return ret;
}

void Boid::update(float elapsedTimeSec, const std::vector<Line> &obstacles) {
    //acceleration = acceleration * 0.4;


    Pos2D posBefore = position;
    Pos2D posAfter = position + direction * elapsedTimeSec * 250;

    direction = direction + acceleration;
    acceleration = acceleration * 0;
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

void Boid::addAcceleration(const Pos2D &acc) {
    acceleration = acceleration + acc;
}

/*
 * Ok j'ai une idée: le vecteur doit être a moitié le vecteur normal et a moitié longer la ligne avec la quelle on a l'angle le plus petit
 */
Pos2D Boid::getSteerFromObstacles(const std::vector<Line> &obstacles) const {
    Pos2D ret;

    Line currentMovement(this->position, this->position + this->direction * OBSTACLE_DISTANCE);
    int count = 0;

    for (const auto obstacle: obstacles) {

        /*
 * We trace a virutal line from the position to the direction + the distance to find obstacles
 * If it collides with an obstacles, we try to steer away from it
 * This code could be in the close obstacles function
 */
      //  if (currentMovement.intersectsWith(obstacle)) {
            count++;
            Pos2D normalVector = obstacle.getNormalVector(position);

/*            auto vectors = obstacle.getVectors();

            auto vec1 = vectors.first + normalVector;
            auto vec2 = vectors.second + normalVector;

            if (std::abs(vectors.first.angleWithVector(this->direction)) > std::abs(vectors.second.angleWithVector(this->direction))) {
                normalVector = vec1;
            } else {
                normalVector = vec2;
            }*/


            normalVector = normalVector / std::sqrt(
                    obstacle.distanceToPoint(position)); // The closer the obstacle is, the more we want to steer
            //  normalVector = normalVector / (obstacle.distanceToPoint(position)); // The closer the obstacle is, the more we want to steer

            ret = ret + normalVector;
        }

//    }
    if (count > 0) {
        ret = ret / count;

        return this->steerToGoal(ret);
    }
    return ret;

}

// We send the desired direction. Goal is a vector
Pos2D Boid::steerToGoal(Pos2D goal) const {
    // Scale to max speed fixme i think its dumb
    // Fixme should be limited to maxSpeed
    goal.setMagnitude(this->maxSpeed);

//    goal.limitToMaxMagnitude(this->maxSpeed);


  //  goal.limitToMinMagnitude(direction.getMagnitude());

    Pos2D steer;

    // Steering = Desired minus Velocity
    steer = goal - direction;

    steer.limitToMaxMagnitude(maxForce); // Limit to max steering force
    return steer;
}

const std::vector<Boid> Boid::getClosestBoids(const std::vector<Boid> &boids, float maxDistance, float maxQty) const {

    std::vector<Boid> closeBoids;
    std::map<float, Boid> distanceToBoids;

    for (const Boid &boid : boids) {
        float distance = boid.getPosition().distanceWith(this->getPosition());

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

