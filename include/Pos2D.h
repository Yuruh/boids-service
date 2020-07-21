//
// Created by antoine on 10/07/20.
//

#ifndef BOIDS_POS2D_H
#define BOIDS_POS2D_H


#include <iostream>
#include "../map.pb.h"

struct Pos2D {
    Pos2D();
    Pos2D(float x, float y);
    float x;
    float y;

    bool operator==(const Pos2D& pos) const;

    Pos2D operator+(const Pos2D& pos) const;
    Pos2D operator-(const Pos2D& pos) const;
    Pos2D& operator+=(const Pos2D& pos);
    Pos2D operator/(float n) const;
    Pos2D operator*(float n) const;

    void normalize();
    float getMagnitude() const;
    float distanceWith(const Pos2D& other) const;

    void limitToMaxMagnitude(float max);
};

std::ostream& operator<<(std::ostream& os, const Pos2D& pos);
Pos2D& operator<<(Pos2D& out, const Protobuf::Pos2D& pos);
Protobuf::Pos2D& operator>>(const Pos2D& in, Protobuf::Pos2D& pos);


#endif //BOIDS_POS2D_H
