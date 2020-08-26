//
// Created by antoine on 21/07/20.
//

#include <cmath>
#include "../include/Line.h"
#include "../include/Macros.h"
#include <algorithm>

/*
 * Implem from https://stackoverflow.com/a/1501725/13167478
 */
float Line::distanceToPoint(const Pos2D &point) const {

    // Return minimum distance between line segment vw and point p
    const float l2 = this->lengthSquared();  // i.e. |w-v|^2 -  avoid a sqrt
    if (l2 == 0.0) return this->a.distanceWith(this->b);   // v == w case
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.


    Pos2D lineVector = this->b - this->a;
    Pos2D aToPoint = point - this->a;

    const float t = std::max(0.f, std::min(1.f, aToPoint * lineVector / l2));
    auto projection = this->a + lineVector * t;  // Projection falls on the segment
    return point.distanceWith(projection);
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
    double angle = this->angleWithVector(vector);
    // If the vectors and line are parallel, no reflection
    if ((angle > -EPSILON && angle < EPSILON) ||
        (angle > M_PI - EPSILON && angle < M_PI + EPSILON) ||
        (angle > -M_PI - EPSILON && angle < -M_PI + EPSILON)) {
        return Pos2D();
    }
    // The direction of the normal vector doesn't affect reflection
    // TODO or does it ?
    Pos2D norm = getNormalVector(Pos2D());

    float dot = 2 * (vector * norm);
    return vector - norm * dot;
}

// if we define dx=x2-x1 and dy=y2-y1, then the normals are (-dy, dx) and (dy, -dx).
// todo handle two possible direction (pass arg ?)
Pos2D Line::getNormalVector(const Pos2D &srcPosition) const {
    float dx = b.x - a.x;
    float dy = b.y - a.y;

    Pos2D lineToPoint = b - srcPosition;

    Pos2D ret1(-dy, dx);
    Pos2D ret2(dy, -dx);
    ret1.normalize();
    ret2.normalize();

    double angle1 = ret1.angleWithVector(lineToPoint);
    double angle2 = ret2.angleWithVector(lineToPoint);

    if (std::abs(angle1) > std::abs(angle2)) {
        return ret1;
    } else {
        return ret2;
    }
}

// From https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

// Given three colinear Pos2Ds p, q, r, the function checks if 
// Pos2D q lies on line segment 'pr' 
bool onSegment(Pos2D p, Pos2D q, Pos2D r)
{
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
    q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);

}

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
float orientation(Pos2D p, Pos2D q, Pos2D r)
{
    // See https://www.geeksforgeeks.org/orientation-3-ordered-Pos2Ds/ 
    // for details of below formula. 
    float val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear 

    return (val > 0)? 1: 2; // clock or counterclock wise 
}

// The main function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Pos2D p1, Pos2D q1, Pos2D p2, Pos2D q2)
{
    // Find the four orientations needed for general and 
    // special cases 
    float o1 = orientation(p1, q1, p2);
    float o2 = orientation(p1, q1, q2);
    float o3 = orientation(p2, q2, p1);
    float o4 = orientation(p2, q2, q1);

    // General case 
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases 
}

bool Line::intersectsWith(const Line &other) const {
    return doIntersect(a, b, other.a, other.b);
}

std::pair<Pos2D, Pos2D> Line::getVectors() const {
    std::pair<Pos2D, Pos2D> ret;

    ret.first = this->b - this->a;
    ret.second = this->a - this->b;

    ret.first.normalize();
    ret.second.normalize();

    return ret;
}

Pos2D Line::getHalfPoint() const {
    return Pos2D((a.x + b.x) / 2, (a.y + b.y) / 2);
}

float Line::length() const {
    return std::sqrt(this->lengthSquared());
}

float square(float value) {
    return value * value;
}

float Line::lengthSquared() const {
    return square(this->b.x - this->a.x) + square(this->b.y - this->a.y);
}

Protobuf::Line &operator>>(const Line &in, Protobuf::Line &protobufLine) {
    auto *a = new Protobuf::Pos2D();
    in.a >> *a;
    protobufLine.set_allocated_a(a);

    auto *b = new Protobuf::Pos2D();
    in.b >> *b;
    protobufLine.set_allocated_b(b);

    return protobufLine;
}

