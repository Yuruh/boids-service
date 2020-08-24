//
// Created by antoine on 16/08/20.
//

#include "catch.hpp"
#include "../include/Boid.h"

TEST_CASE( "Boid Class", "[plop]" ) {

    Boid boid;

    boid.setPosition(Pos2D(50, 50));
    boid.setDirection(Pos2D(0.1, 0.1));


    REQUIRE(boid.getPosition() == Pos2D(50, 50));

    REQUIRE(boid.steerToGoal(Pos2D(50, 100)) == Pos2D(1, 1));
}