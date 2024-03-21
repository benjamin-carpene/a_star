#include "node.hpp"

#include <algorithm>
#include <math.h>

// --------------- Vector2D ---------------

a_star::Vector2D::Vector2D(uint16_t x, uint16_t y):
    x(x), y(y) 
{}

bool a_star::Vector2D::operator==(Vector2D other) const
{
    return x == other.x && y == other.y;
}

float a_star::Vector2D::length()
{
    const float sqrt2 = static_cast<float>(sqrt(2));

    if(x+y == 1) // one is 1 other is 0
        return 1;
    if(x==1 && y==1) // hardcode the value for quick retrieval
        return sqrt2;
    else
        return static_cast<float>(sqrt(x*x + y*y));
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

a_star::Vector2D a_star::Vector2D::delta(Vector2D a, Vector2D b)
{
    return Vector2D(
        (a.x > b.x) ? (a.x - b.x) : (b.x - a.x),
        (a.y > b.y) ? (a.y - b.y) : (b.y - a.y)
    );
}

float a_star::Vector2D::distance(Vector2D a, Vector2D b)
{
    return delta(a, b).length();
}

size_t a_star::Vector2D::HashFunction::operator()(const Vector2D &vector2d) const
{
    size_t xHash = std::hash<int>()(vector2d.x);
    size_t yHash = std::hash<int>()(vector2d.y) << 1;
    return xHash ^ yHash;
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



