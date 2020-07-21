//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_FLOCK_H
#define BOIDS_FLOCK_H


#include <vector>
#include "Boid.h"
#include "Map.h"

    /*
 * For now we take "all boids" in the computation, but it should be only the ones that are near
 */

class Flock {
private:
    std::vector<Boid> boids;

//    const Map &map;


    // could take a boid as param to exclude it
    /*
     * Compute the center position of all boids
     */
    Pos2D centreOfMass() const;
    Pos2D centreOfDirection() const;
    Pos2D avoidVector(const Boid &boid);

public:
//    explicit Flock(const Map &map);
    void addBoid(const Boid &boid);
    const std::vector<Boid> getBoids() const;
    void update(float elapsedTimeSec, const Map &map);

    friend Flock& operator<<(Flock &out, const Protobuf::Flock &protobufFlock);
    friend Protobuf::Flock& operator>>(const Flock &out, Protobuf::Flock &protobufFlock);

};


#endif //BOIDS_FLOCK_H
