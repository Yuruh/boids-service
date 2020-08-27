//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_BOID_H
#define BOIDS_BOID_H


#include "Pos2D.h"
#include "Line.h"

// resources:

// http://www.red3d.com/cwr/boids/
// http://www.kfish.org/boids/pseudocode.html
// https://processing.org/examples/flocking.html

class Boid {
private:
    // The current position of the boid
    Pos2D position;

    // The current direction. The magnitude of this vector is the boid's velocity
    Pos2D direction;

    // Vector added to direction on each update.
    Pos2D acceleration;

    // Min magnitude of direction vector
    float minSpeed;

    // Max magnitude of direction vector
    float maxSpeed;

    // The maximum magnitude of steering vectors
    float maxForce;

    // [NOT IMPLEMENTED YET] The weight of the boids. A heavy boid has a harder time steering
    float weight;

    // [NOT IMPLEMENTED YET] How much can a boid steer when updating
    double maxSteerAngle;


    // We store result for boid rules so we can send them to the client on each simulation frame, for debugging / demo purpose
    Pos2D currentCohesion;
    Pos2D currentAlignment;
    Pos2D currentSeparation;
    Pos2D currentAvoidance;

public:
    Boid();

    Pos2D getPosition() const;
    void setPosition(Pos2D pos);

    Pos2D getDirection() const;
    void setDirection(const Pos2D &dir);

    void addAcceleration(const Pos2D &acc);

    // Boid rule: Boids should try to steer to move toward the average position of local flockmates
    Pos2D getCohesion(const std::vector<Boid*> &boids) const;

    // Boid rule: Boids should try to steer towards the average heading of local flockmates
    Pos2D getAlignment(const std::vector<Boid*> &boids) const;

    // Boid rule: Boids should try to steer to avoid crowding local flockmates
    Pos2D getSeparation(const std::vector<Boid*> &boids) const;

    // Same prev but with distance as param
    Pos2D getSeparation(const std::vector<std::pair<Boid*, float>> &boids) const;

    // Custom boid rule: boids should try to steer away from obstacles
    Pos2D getSteerFromObstacles(const std::vector<Line> &obstacles) const;

    void setRulesResult(const Pos2D &cohesion, const Pos2D &alignment, const Pos2D &separation, const Pos2D &avoidance);

    // Retrieve maxQty number of boids around current boid.
    const std::vector<Boid> getClosestBoids(const std::vector<Boid> &boids, float maxDistance, float maxQty) const;

    // We consider two boids equal if they have the same location
    bool operator==(const Boid &boid) const;
    bool operator!=(const Boid &boid) const;


    // Update boid position according to decisions made, elapsed time and map obstacles
    void update(float elapsedTimeSec, const std::vector<Line> &obstacles);

    // Deserialize from protobuf
    friend Boid& operator<<(Boid &out, const Protobuf::Boid &protobufBoid);
    // Serialize to protobuf
    friend Protobuf::Boid& operator>>(const Boid &out, Protobuf::Boid &protobufBoid);

    // Computes the steering vector to join desired direction
    Pos2D steerToDirection(Pos2D direction) const;

};

#endif //BOIDS_BOID_H
