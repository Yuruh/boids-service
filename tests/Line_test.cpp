//
// Created by antoine on 21/07/20.
//

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include "catch.hpp"
#include "../include/Line.h"

unsigned int Factorial( unsigned int number ) {
    return number <= 1 ? number : Factorial(number-1)*number;
}

// Can be compiled with g++ tests/Line_test.cpp src/Line.cpp src/Pos2D.cpp -o unit_tests
// Would need

TEST_CASE( "Line Class", "[plop]" ) {
    Line line(Pos2D(300, 300), Pos2D(600, 300));

    REQUIRE(line.distanceToPoint(Pos2D(100, 100)) == 200.0f);
}