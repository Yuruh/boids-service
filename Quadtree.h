//
// Created by antoine on 26/07/20.
//

#ifndef BOIDS_QUADTREE_H
#define BOIDS_QUADTREE_H

// Adapted from https://www.geeksforgeeks.org/quad-tree/
// and https://editor.p5js.org/TheTastefulToastie/sketches/BJNaRITyE

#include "include/Pos2D.h"


template <class T>
struct Node {
    Pos2D   pos;
    T data;
};

template <class T>
class Quadtree {
private:
    Node<T> *node;

    Quadtree *topLeft;
    Quadtree *topRight;
    Quadtree *botLeft;
    Quadtree *botRight;

//    Pos2D topLeft;


public:
    void insert(Node<T> *node);
    std::vector<Node<T>*> searchInRadius(Pos2D center, float radius) const;
    bool inBoundary(Pos2D point);
};

template<class T>
void Quadtree<T>::insert(Node<T> *node) {

}

template<class T>
std::vector<Node<T> *> Quadtree<T>::searchInRadius(Pos2D center, float radius) const {
    return std::vector<Node<T> *>();
}

template<class T>
bool Quadtree<T>::inBoundary(Pos2D point) {
    return false;
}


#endif //BOIDS_QUADTREE_H
