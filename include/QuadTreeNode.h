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
    std::vector<NodeData<T> *> nodesData;
//    NodeData<T> *node;

    QuadTreeNode *topLeftNode;
    QuadTreeNode *topRightNode;
    QuadTreeNode *botLeftNode;
    QuadTreeNode *botRightNode;

    Pos2D topLeft;
    Pos2D botRight;

    // TODO free old nodeData;


public:
    void insert(NodeData<T> *node);

    std::vector<T*> searchInRadius(Pos2D center, float radius) const;

    bool inBoundary(Pos2D point);

    // Assert if a point is within the QuadTreeNode
    bool withinRadius(Pos2D center, float radius = 0);

    std::vector<T*> toItemsVector() const;

    // Reset everything, frees nodes and return all items;
    std::vector<T*> clear();


    // Serialize to protobuf
    template<class U>
    friend Protobuf::Flock &operator>>(const QuadTreeNode<U> &in, Protobuf::Flock &protobufFlock);

    // Returns the 4 lines that make the box
    std::array<Line, 4> toLines() const;

    QuadTreeNode(Pos2D topL, Pos2D botR)
    {
        //node = NULL;
        topLeftNode  = NULL;
        topRightNode = NULL;
        botLeftNode  = NULL;
        botRightNode = NULL;
        topLeft = topL;
        botRight = botR;
    }

    ~QuadTreeNode() {
        this->clear();

        // todo delete boids ?
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
    if (std::abs(this->topLeft.x - this->botRight.x) <= MIN_QUADNODE_SIZE &&
        std::abs(this->topLeft.y - this->botRight.y) <= MIN_QUADNODE_SIZE) {
        this->nodesData.push_back(newNode);
//        if (this->node == NULL)
  //          this->node = newNode;
        // FIXME We should store a list of boids int the node
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

template<class T>
std::vector<T*> childSearch(QuadTreeNode<T> *node, Pos2D center, float radius) {
    auto ret = std::vector<T*>();

    if (node && node->withinRadius(center, radius)) {
        return node->searchInRadius(center, radius);
    }
    return ret;
}

template<class T>
std::vector<T*> QuadTreeNode<T>::searchInRadius(Pos2D center, float radius) const {
    auto ret = std::vector<T*>();

    for (int i = 0; i < this->nodesData.size(); ++i) {
        // TODO check within extent of radius for each node + check distance > EPSILON
        ret.push_back(this->nodesData[i]->data);
    }

    auto childRes = childSearch(this->botRightNode, center, radius);
    ret.insert(ret.end(), childRes.begin(), childRes.end());

    childRes = childSearch(this->botLeftNode, center, radius);
    ret.insert(ret.end(), childRes.begin(), childRes.end());

    childRes = childSearch(this->topRightNode, center, radius);
    ret.insert(ret.end(), childRes.begin(), childRes.end());

    childRes = childSearch(this->topLeftNode, center, radius);
    ret.insert(ret.end(), childRes.begin(), childRes.end());

    return ret;
}

template<class T>
bool QuadTreeNode<T>::withinRadius(Pos2D center, float radius) {
    return center.x >= this->topLeft.x - radius && center.x < this->botRight.x + radius &&
            center.y >= this->topLeft.y - radius && center.y < this->botRight.y + radius;
}

template<class T>
std::vector<T *> QuadTreeNode<T>::toItemsVector() const {
    // Quick and dirty
    return this->searchInRadius(Pos2D(0, 0), INT32_MAX / 2);

    auto ret = std::vector<T *>();

    return ret;
}

template<class T>
std::vector<T *> QuadTreeNode<T>::clear() {
    auto ret = std::vector<T*>();

    for (int i = 0; i < this->nodesData.size(); ++i) {
        ret.push_back(this->nodesData[i]->data);
        delete this->nodesData[i];
    }


    if (this->botRightNode) {
        auto childRes = this->botRightNode->clear();
        ret.insert(ret.end(), childRes.begin(), childRes.end());
    }

    if (this->topRightNode) {
        auto childRes = this->topRightNode->clear();
        ret.insert(ret.end(), childRes.begin(), childRes.end());
    }

    if (this->topLeftNode) {
        auto childRes = this->topLeftNode->clear();
        ret.insert(ret.end(), childRes.begin(), childRes.end());
    }

    if (this->botLeftNode) {
        auto childRes = this->botLeftNode->clear();
        ret.insert(ret.end(), childRes.begin(), childRes.end());
    }

    this->nodesData.clear();
    this->botLeftNode = NULL;
    this->topLeftNode = NULL;
    this->topRightNode = NULL;
    this->botRightNode = NULL;

    return ret;
}


#endif //BOIDS_QUADTREE_H
