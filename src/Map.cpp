//
// Created by antoine on 10/07/20.
//

#include <iostream>
#include <algorithm>
#include "../include/Map.h"
#include "../include/Macros.h"

/*
 * Default constructor generates a random map
 */
/*Map::Map(): dimensions(Pos2D(WIDTH, HEIGHT)) {
    int nbOfBoids = std::rand() % 30 + 10;
    for (int i = 0; i < nbOfBoids; ++i) {
        Boid boid;
        this->flock.addBoid(boid);
    }
}*/



void Map::display() const {


/*    struct Predicate {
        int x;
        int y;

        explicit Predicate(int x, int y): x(x), y(y) {}

        bool operator()(Boid boid) const {
            std::cout<<"allo\n";
            return boid.getPosition().x == x && boid.getPosition().y == y;
        }
    };*/
    for (int i = 0; i < this->dimensions.x + 2; ++i) {
        std::cout << "-";
    }
    std::cout << std::endl;
    for (int y = 0; y < this->dimensions.y; y++) {
        std::cout << "|";
        for (int x = 0; x < this->dimensions.x; ++x) {
            int idx = this->getBoidIndex(x, y);
            if (idx > -1) {
//                std::cout << this->flock.getBoids()[idx].getDisplay();
            } else {
                std::cout << " ";
            }
/*            if (std::any_of(this->flock.getBoids().cbegin(), this->flock.getBoids().cend(), Predicate(i, j))) {
                std::cout << "o";
            } else {
                std::cout << "x";
            }*/
        }
        std::cout  <<  "|" << std::endl;
    }
    for (int i = 0; i < this->dimensions.x + 2; ++i) {
        std::cout << "-";
    }
    std::cout << std::endl;

}

Map::Map(Pos2D dimensions): dimensions(dimensions) {

}

bool Map::isBoid(int x, int y) const {
/*    for (int i = 0; i < this->flock.getBoids().size(); ++i) {

        const Boid boid = this->flock.getBoids()[i];


        if (static_cast<int>(boid.getPosition().x) == x && static_cast<int>(boid.getPosition().y) == y) {
            return true;
        }
    }*/
    return false;
}

int Map::getBoidIndex(int x, int y) const {
/*    for (int i = 0; i < this->flock.getBoids().size(); ++i) {

        const Boid boid = this->flock.getBoids()[i];


        if (static_cast<int>(boid.getPosition().x) == x && static_cast<int>(boid.getPosition().y) == y) {
            return i;
        }
    }*/
    return -1;
}

Map &operator<<(Map &out, const Protobuf::Map &protobufMap) {
    out.dimensions << protobufMap.dimensions();

    Line left(Pos2D(0, 0), Pos2D(0, out.dimensions.y));
    Line right(Pos2D(out.dimensions.x, 0), Pos2D(out.dimensions.x, out.dimensions.y));
    Line top(Pos2D(0, 0), Pos2D(out.dimensions.x, 0));
    Line bottom(Pos2D(0, out.dimensions.y), Pos2D(out.dimensions.x, out.dimensions.y));


    for (const auto &obstacle: protobufMap.obstacles()) {
        Line line(Pos2D(obstacle.a().x(), obstacle.a().y()), Pos2D(obstacle.b().x(), obstacle.b().y()));
        out.obstacles.push_back(line);
    }

    Line corner1(Pos2D(-1, 0), Pos2D(0, -1));
    Line corner2(Pos2D(out.dimensions.x + 1, 0), Pos2D(out.dimensions.x, -1));
    Line corner3(Pos2D(-1, out.dimensions.y), Pos2D(0, out.dimensions.y + 1));
    Line corner4(Pos2D(out.dimensions.x  + 1, out.dimensions.y), Pos2D(out.dimensions.x, out.dimensions.y + 1));

    Line square1(Pos2D(300, 300), Pos2D(600, 300));
    Line square2(Pos2D(600, 300), Pos2D(600, 600));
    Line square3(Pos2D(600, 600), Pos2D(300, 600));
    Line square4(Pos2D(300, 600), Pos2D(300, 300));

/*    out.obstacles.push_back(square1);
    out.obstacles.push_back(square2);
    out.obstacles.push_back(square3);
    out.obstacles.push_back(square4);*/

    out.obstacles.push_back(left);
    out.obstacles.push_back(right);
    out.obstacles.push_back(top);
    out.obstacles.push_back(bottom);

/*    out.obstacles.push_back(corner1);
    out.obstacles.push_back(corner2);
    out.obstacles.push_back(corner3);
    out.obstacles.push_back(corner4);*/
    return out;
}

Pos2D Map::getDimensions() const {
    return this->dimensions;
}

Map::Map(): dimensions(Pos2D()) {

}

const std::vector<Line> &Map::getObstacles() const {
    return obstacles;
}

const Line &Map::closestObstacle(const Pos2D &pos) const {
    int idx = -1;
    float closest = -1;
    for (int i = 0; i < obstacles.size(); ++i) {
        float distance = obstacles[i].distanceToPoint(pos);
        if (distance < closest || closest < -EPSILON) {
            closest = distance;
            idx = i;
        }
    }
    if (idx == -1) {
        throw std::runtime_error("No obstacle in map");
    }
    return obstacles[idx];
}

const std::vector<Line> Map::closeObstacles(const Pos2D &pos, unsigned int obstacleDistance) const {
    std::vector<Line> ret;

    for (auto obstacle : obstacles) {
        float distance = obstacle.distanceToPoint(pos);
        if (distance < obstacleDistance) {
            ret.push_back(obstacle);
        }
    }
    return ret;
}
