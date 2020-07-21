//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_MAP_H
#define BOIDS_MAP_H

#include <vector>
#include "Pos2D.h"
#include "../map.pb.h"

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
    friend Map& operator<<(Map &out, const Protobuf::Map &protobufMap);
};


#endif //BOIDS_MAP_H
