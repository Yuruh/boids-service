//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_FLOCK_H
#define BOIDS_FLOCK_H


#include <vector>
#include "Boid.h"
#include "Map.h"
#include "Parameters.h"
#include "QuadTreeNode.h"

class Flock {
private:

    // We store a vector of pointer to all boids, as we regularly need to go through them
    std::vector<Boid*> allBoids;

    QuadTreeNode<Boid> boids;

    Parameters params;

public:
    Flock(const Map &map);

    ~Flock();

//    explicit Flock(const Map &map);
    void addBoid(const Boid &boid);
    const std::vector<Boid> getBoids() const;
    void update(float elapsedTimeSec, const Map &map);

    void setParams(const Parameters &params);

    friend Flock& operator<<(Flock &out, const Protobuf::Flock &protobufFlock);
    friend Protobuf::Flock& operator>>(const Flock &out, Protobuf::Flock &protobufFlock);

    std::pair<std::vector<Pos2D>, std::vector<Pos2D>> getCloseObstaclesNormalVectors(const Map &map) const;

    std::vector<Boid*> restructureQuadtree();

    void updateStayOnCourse(float elapsedTimeSec, const Map &map);
};


#endif //BOIDS_FLOCK_H
