//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_MAP_H
#define BOIDS_MAP_H

#include <vector>
#include "Pos2D.h"
#include "map.pb.h"
#include "Line.h"

/*
 * Comme input on va avoir:
 *
 * largeur / hauteur canvas
 * obstacles
 * boids param
 *
 */

class Map {
private:
    Pos2D dimensions;
    std::vector<Line> obstacles;

    bool isBoid(int x, int y) const;
    int getBoidIndex(int x, int y) const;
public:
    Map();
    explicit Map(Pos2D dimensions);

    /*
     * Print the map to stdout
     */
    void display() const;


    Pos2D getDimensions() const;
    const std::vector<Line> &getObstacles() const;
    const Line &closestObstacle(const Pos2D& pos) const;
    const std::vector<Line> closeObstacles(const Pos2D& pos, unsigned int obstacleDistance) const;
    friend Map& operator<<(Map &out, const Protobuf::Map &protobufMap);
};


#endif //BOIDS_MAP_H
