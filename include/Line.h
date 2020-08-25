//
// Created by antoine on 21/07/20.
//

#ifndef BOIDS_LINE_H
#define BOIDS_LINE_H


#include "Pos2D.h"

// Represents a line going from a to b
class Line {
private:
    Pos2D a;
    Pos2D b;
public:
    Line(Pos2D a, Pos2D b);
    float distanceToPoint(const Pos2D& point) const;
    double angleWithVector(const Pos2D& vector) const;
    Pos2D getNormalVector(const Pos2D &src) const;
    Pos2D reflectedVector(const Pos2D& vector) const;

    std::pair<Pos2D, Pos2D> getVectors() const;

    bool intersectsWith(const Line& other) const;

    friend std::ostream& operator<<(std::ostream& os, const Line& line);


    Pos2D getHalfPoint() const;
    float length() const;
    float lengthSquared() const;
};


#endif //BOIDS_LINE_H
