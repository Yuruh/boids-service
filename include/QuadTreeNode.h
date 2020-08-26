//
// Created by antoine on 26/07/20.
//

#ifndef BOIDS_QUADTREE_H
#define BOIDS_QUADTREE_H

// Adapted from https://www.geeksforgeeks.org/quad-tree/

#include "Pos2D.h"
#include "Line.h"
#include "Macros.h"

// TODO export to protobuf for debug / showcase

template <class T>
struct NodeData {
    Pos2D   pos;
    T *data;

    NodeData(Pos2D pos, T *data): pos(pos), data(data) {}
};

template <class T>
class QuadTreeNode {
private:
    NodeData<T> *node;

    QuadTreeNode *topLeftNode;
    QuadTreeNode *topRightNode;
    QuadTreeNode *botLeftNode;
    QuadTreeNode *botRightNode;

    Pos2D topLeft;
    Pos2D botRight;



public:
    void insert(NodeData<T> *node);
    std::vector<NodeData<T>*> searchInRadius(Pos2D center, float radius) const;
    bool inBoundary(Pos2D point);


    // Serialize to protobuf
    template<class U>
    friend Protobuf::Flock &operator>>(const QuadTreeNode<U> &in, Protobuf::Flock &protobufFlock);

    // Returns the 4 lines that make the box
    std::array<Line, 4> toLines() const;

    QuadTreeNode(Pos2D topL, Pos2D botR)
    {
        node = NULL;
        topLeftNode  = NULL;
        topRightNode = NULL;
        botLeftNode  = NULL;
        botRightNode = NULL;
        topLeft = topL;
        botRight = botR;
    }
};



template<class T>
void QuadTreeNode<T>::insert(NodeData<T> *newNode) {
    if (!newNode)
        return;

    // Current quad cannot contain it
    if (!inBoundary(newNode->pos))
        return;

    // We are at a quad of unit area
    // We cannot subdivide this quad further
    // TODO replace 1 by max width ?

    if (std::abs(this->topLeft.x - this->botRight.x) <= MIN_QUADNODE_SIZE &&
        std::abs(this->topLeft.y - this->botRight.y) <= MIN_QUADNODE_SIZE) {
        if (this->node == NULL)
            this->node = newNode;
        // FIXME and else ?? We should store a list of boids int he node i think
        return;
    }

    if ((this->topLeft.x + this->botRight.x) / 2 >= newNode->pos.x)
    {
        // Indicates topLeftNode
        if ((this->topLeft.y + this->botRight.y) / 2 >= newNode->pos.y)
        {
            if (this->topLeftNode == NULL)
                this->topLeftNode = new QuadTreeNode(
                        Pos2D(this->topLeft.x, this->topLeft.y),
                        Pos2D((this->topLeft.x + this->botRight.x) / 2,
                              (this->topLeft.y + this->botRight.y) / 2));
            this->topLeftNode->insert(newNode);
        }

            // Indicates botLeftNode
        else
        {
            if (botLeftNode == NULL)
                botLeftNode = new QuadTreeNode(
                        Pos2D(topLeft.x,
                              (topLeft.y + botRight.y) / 2),
                        Pos2D((topLeft.x + botRight.x) / 2,
                              botRight.y));
            botLeftNode->insert(newNode);
        }
    }
    else
    {
        // Indicates topRightNode
        if ((topLeft.y + botRight.y) / 2 >= newNode->pos.y)
        {
            if (topRightNode == NULL)
                topRightNode = new QuadTreeNode(
                        Pos2D((topLeft.x + botRight.x) / 2,
                              topLeft.y),
                        Pos2D(botRight.x,
                              (topLeft.y + botRight.y) / 2));
            topRightNode->insert(newNode);
        }

            // Indicates botRightNode
        else
        {
            if (botRightNode == NULL)
                botRightNode = new QuadTreeNode(
                        Pos2D((topLeft.x + botRight.x) / 2,
                              (topLeft.y + botRight.y) / 2),
                        Pos2D(botRight.x, botRight.y));
            botRightNode->insert(newNode);
        }
    }

}

template<class T>
std::vector<NodeData<T> *> QuadTreeNode<T>::searchInRadius(Pos2D center, float radius) const {
    return std::vector<NodeData<T> *>();
}

template<class T>
bool QuadTreeNode<T>::inBoundary(Pos2D p) {
    return (p.x >= this->topLeft.x &&
            p.x <= this->botRight.x &&
            p.y >= this->topLeft.y &&
            p.y <= this->botRight.y);
}

template <class T>
Protobuf::Flock &operator>>(const QuadTreeNode<T> &in, Protobuf::Flock &protobufFlock) {
    auto lines = in.toLines();

    for (int i = 0; i < lines.size(); ++i) {
        auto *protoLine = protobufFlock.add_quadtree();

        lines[i] >> *protoLine;
    }
    if (in.botLeftNode) {
        *(in.botLeftNode) >> protobufFlock;
    }
    if (in.topLeftNode) {
        *(in.topLeftNode) >> protobufFlock;
    }
    if (in.topRightNode) {
        *(in.topRightNode) >> protobufFlock;
    }
    if (in.botRightNode) {
        *(in.botRightNode) >> protobufFlock;
    }

    return protobufFlock;

}

template<class T>
std::array<Line, 4> QuadTreeNode<T>::toLines() const {
    Pos2D topRight(this->botRight.x, this->topLeft.y);
    Pos2D botLeft(this->topLeft.x, this->botRight.y);

    std::array<Line, 4> ret = {
            Line(this->topLeft, topRight), // top
            Line(topRight, this->botRight), // right
            Line(this->botRight, botLeft), // bottom
            Line(botLeft, this->topLeft), // left
    };
    return ret;
}


#endif //BOIDS_QUADTREE_H
