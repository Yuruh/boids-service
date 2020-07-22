//
// Created by antoine on 21/07/20.
//

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include "catch.hpp"
#include "../include/Line.h"

unsigned int Factorial( unsigned int number ) {
    return number <= 1 ? number : Factorial(number-1)*number;
}


TEST_CASE( "Line Class", "[plop]" ) {

    // From left to right
    Line line(Pos2D(300, 300), Pos2D(600, 300));
    // From right to left
    Line line2(Pos2D(300, 100), Pos2D(100, 100));
    // From top to bottom
    Line line3(Pos2D(200, 80), Pos2D(200, 120));
    // From top left to bottom right
    Line line4(Pos2D(100, 100), Pos2D(200, 200));

    REQUIRE(line.distanceToPoint(Pos2D(100, 100)) == 200.0f);


    REQUIRE(line.reflectedVector(Pos2D(0.5, 0.5)) == Pos2D(0.5, -0.5));
    REQUIRE(line.reflectedVector(Pos2D(0.5, -0.5)) == Pos2D(0.5, 0.5));
    REQUIRE(line.reflectedVector(Pos2D(-0.5, -0.5)) == Pos2D(-0.5, 0.5));
    REQUIRE(line.reflectedVector(Pos2D(-0.5, 0.5)) == Pos2D(-0.5, -0.5));


    REQUIRE(line.reflectedVector(Pos2D(0, 1)) == Pos2D(0, -1));


    REQUIRE(!line.intersectsWith(line2));
    REQUIRE(line3.intersectsWith(line2));


}