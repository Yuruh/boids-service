//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_CORE_H
#define BOIDS_CORE_H

#include "Map.h"

/*
 * Should listen to something and reply
 */
class Core {
private:
    Map map;
public:
    void displayNextMovements(int n, float refreshRateSec);
};


#endif //BOIDS_CORE_H
