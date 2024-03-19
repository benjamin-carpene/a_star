#include "node.hpp"

#include <iostream>
#include <algorithm>

// --------------- Vector2D ---------------

a_star::Vector2D::Vector2D(uint16_t x, uint16_t y):
    x(x), y(y) 
{}

bool a_star::Vector2D::operator==(Vector2D other)
{
    return x == other.x && y == other.y;
}

std::vector<a_star::Vector2D> a_star::Vector2D::getNeighbors(Vector2D mapSize)
{
    std::vector<Vector2D> container;
    container.reserve(8);

    for(int8_t i : {-1, 0, 1} ){
        for(int8_t j : {-1, 0, 1}){
            if(i == 0 && j == 0)
                continue; // isnt a neighbor
            
            uint16_t currX = x+i;
            uint16_t currY = y+j;

            if( currX < mapSize.x && currY < mapSize.y ) // no need to compare >= 0 because uint
                container.push_back({currX, currY});
        }
    }

    return container;
}

// --------------- Node ---------------
a_star::Node::Node(Vector2D position, Node* predecessor):
    position(position), predecessor(predecessor)
{}

std::vector<a_star::Vector2D> a_star::Node::constructPath()
{
    std::vector<Vector2D> container;
    container.reserve(50); // default
    
    auto currNode = this;
    while(currNode != nullptr) {
        container.push_back(currNode->position);
        currNode = currNode->predecessor;
    }
    
    //put it back in order
    std::reverse(container.begin(), container.end()); 
    return container;
}

// --------------- NodeSet ---------------

// TODO : change the approach for using std::make_heap etc 
void a_star::NodeSet::insert(Node *node)
{
    nodeQueue.push_back(node);
}

a_star::Node* a_star::NodeSet::pop()
{
    // search for the least value
    auto minNodeIter = std::min_element(nodeQueue.begin(), nodeQueue.end(), [](Node* a, Node* b){ return a->score < b->score; });
    Node* minNode = *minNodeIter;

    // remove from vector eand returns the value
    nodeQueue.erase(minNodeIter);
    return minNode;
}

void a_star::NodeSet::clear()
{
    for(auto ptr: nodeQueue){
        delete ptr;
    }
    nodeQueue.clear();
}

bool a_star::NodeSet::isEmpty()
{
    return nodeQueue.empty();
}

bool a_star::NodeSet::includes(Vector2D position)
{
    return static_cast<bool>(getNodeFromPosition(position));
}

a_star::Node* a_star::NodeSet::getNodeFromPosition(Vector2D position)
{
    auto pred = [position](Node* a){
        return a->position == position;
    };
    auto iter = std::find_if(nodeQueue.begin(), nodeQueue.end(), pred);
    if(iter == nodeQueue.end())
        return nullptr;
    else
        return *iter;    
}

a_star::NodeSet::~NodeSet()
{
    for(auto ptr: nodeQueue){
        delete ptr;
    }
}


