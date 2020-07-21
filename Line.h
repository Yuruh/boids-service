//
// Created by antoine on 21/07/20.
//

#ifndef BOIDS_LINE_H
#define BOIDS_LINE_H


#include "include/Pos2D.h"

class Line {
private:
    Pos2D a;
    Pos2D b;
public:
    float distanceToPoint(const Pos2D& point);
};


#endif //BOIDS_LINE_H
