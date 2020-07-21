//
// Created by antoine on 21/07/20.
//

#include <cmath>
#include "Line.h"
#include "include/Macros.h"

float Line::distanceToPoint(const Pos2D &point) const {

    Pos2D ab = b - a;
    Pos2D ap = point - a;


    float area = ab.getCrossProduct(ap);
    return std::abs(area / ab.getMagnitude());
}

double Line::angleWithVector(const Pos2D &vector) const {
    Pos2D ab = b - a;
    double dot = ab * vector;
    double det = ab.getCrossProduct(vector);
    return std::atan2(det, dot);
}

Line::Line(Pos2D a, Pos2D b): a(a), b(b) {

}

std::ostream &operator<<(std::ostream &os, const Line &line) {
    os << "A = " << line.a << " B = " << line.b;
    return os;
}

Pos2D Line::reflectedVector(const Pos2D &vector) const {
    Pos2D norm = getNormalVector();

    float dot = 2 * (vector * norm);
    return vector - norm * dot;
}

// todo handle two possible direction (pass arg ?)
Pos2D Line::getNormalVector() const {
    float dx = b.x - a.x;
    float dy = b.y - a.y;

    Pos2D ret(-dy, dx);
    ret.normalize();
    return ret;
}
