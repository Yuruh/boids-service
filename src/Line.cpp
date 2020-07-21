//
// Created by antoine on 21/07/20.
//

#include <cmath>
#include "../include/Line.h"
#include "../include/Macros.h"

float Line::distanceToPoint(const Pos2D &point) const {

    Pos2D ab = b - a;
    Pos2D ap = point - a;


    float area = ab.getCrossProduct(ap);
    return std::abs(area / ab.getMagnitude());
}

double Line::angleWithVector(const Pos2D &vector) const {
    Pos2D ab = b - a;
    return ab.angleWithVector(vector);
}

Line::Line(Pos2D a, Pos2D b): a(a), b(b) {

}

std::ostream &operator<<(std::ostream &os, const Line &line) {
    os << "A = " << line.a << " B = " << line.b;
    return os;
}

Pos2D Line::reflectedVector(const Pos2D &vector) const {
    Pos2D norm = getNormalVector(vector);

    float dot = 2 * (vector * norm);
    return vector - norm * dot;
}

// if we define dx=x2-x1 and dy=y2-y1, then the normals are (-dy, dx) and (dy, -dx).
// todo handle two possible direction (pass arg ?)
Pos2D Line::getNormalVector(const Pos2D &src) const {
    float dx = b.x - a.x;
    float dy = b.y - a.y;

    Pos2D ret1(-dy, dx);
    Pos2D ret2(dy, -dx);
    ret1.normalize();
    ret2.normalize();

    double angle1 = ret1.angleWithVector(src);
    double angle2 = ret2.angleWithVector(src);

    if (angle1 > angle2) {
        return ret1;
    } else {
        return ret2;
    }
}
