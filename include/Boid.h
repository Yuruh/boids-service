//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_BOID_H
#define BOIDS_BOID_H


#include "Pos2D.h"
#include "Line.h"

// resources:

// http://www.kfish.org/boids/pseudocode.html
// https://processing.org/examples/flocking.html

// serve.proto over http, or use different method like json https://www.npmjs.com/package/protobufjs#examples

// To handle walls, use reflection: plus je vais droit vers le mur, plus la force me pousse vers le coté opposé

class Boid {
private:
    Pos2D position;

    // the magnitude of this vector is the boid's velocity
    Pos2D direction;

    Pos2D acceleration;

    float minSpeed;

    float maxSpeed;

    // The maximum magnitude of steering vectors
    float maxForce;

    // The weight of the boids. A heavy boid has a harder time steering
    float weight;

    // How much can a boid steer during one frame
    double maxSteerAngle;


    // We store result for boid rules so we can send them to the client on each simulation frame
    Pos2D currentCohesion;
    Pos2D currentAlignment;
    Pos2D currentSeparation;
    Pos2D currentAvoidance;

public:
    Boid();
    Pos2D getPosition() const;
    Pos2D getDirection() const;

    Pos2D getCohesion(const std::vector<Boid> &boids) const;
    Pos2D getAlignment(const std::vector<Boid> &boids) const;
    Pos2D getSeparation(const std::vector<Boid> &boids) const;
    Pos2D getSteerFromObstacles(const std::vector<Line> &obstacles) const;

    void setRulesResult(const Pos2D &cohesion, const Pos2D &alignment, const Pos2D &separation, const Pos2D &avoidance);

//    const std::vector<Boid> getCloseBoids(const std::vector<Boid> &boids) const;
    const std::vector<Boid> getClosestBoids(const std::vector<Boid> &boids, float maxDistance, float maxQty) const;

    bool operator==(const Boid &boid) const;
    bool operator!=(const Boid &boid) const;

    void setDirection(const Pos2D &dir);
    void addAcceleration(const Pos2D &acc);
    void setPosition(Pos2D pos);

    void update(float elapsedTimeSec, const std::vector<Line> &obstacles);


    friend Boid& operator<<(Boid &out, const Protobuf::Boid &protobufBoid);
    friend Protobuf::Boid& operator>>(const Boid &out, Protobuf::Boid &protobufBoid);

    Pos2D steerToGoal(Pos2D goal) const;

};


#endif //BOIDS_BOID_H
