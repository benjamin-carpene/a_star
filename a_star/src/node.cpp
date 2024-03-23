#include "node.hpp"

#include <algorithm>
#include <math.h>
#include <sstream>

// --------------- Vector2D ---------------

a_star::Vector2D::Vector2D(uint16_t x, uint16_t y):
    x(x), y(y) 
{}

bool a_star::Vector2D::operator==(Vector2D other) const
{
    return x == other.x && y == other.y;
}

a_star::Vector2D::operator std::string() const
{
    std::stringstream ss;
    ss << "{" << x << ", " << y << "}";
    return ss.str();
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

a_star::PointSet a_star::Vector2D::getNeighbors(Vector2D mapSize)
{
    PointSet container;
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

std::size_t a_star::Vector2D::HashFunction::operator()(const Vector2D &vector2d) const
{
    std::size_t xHash = std::hash<int>()(vector2d.x);
    std::size_t yHash = std::hash<int>()(vector2d.y) << 1;
    return xHash ^ yHash;
}


// --------------- Node ---------------
a_star::Node::Node(Vector2D position, Node* predecessor):
    position(position), predecessor(predecessor)
{}

a_star::PointSet a_star::Node::constructPath()
{
    PointSet container;
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

bool a_star::Node::operator==(const Node &other) const
{
    return score == other.score;
}

bool a_star::Node::operator<(const Node &other) const
{
    return score < other.score;
}

bool a_star::Node::operator>(const Node &other) const
{
    return score > other.score;
}

// --------------- NodeSet ---------------

void a_star::NodeSet::insert(Node *node)
{
    mNodeQueue.push_back(node);
    // min heap => 'operator >' (as value not ptr)
    std::push_heap(mNodeQueue.begin(), mNodeQueue.end(), [](Node* a, Node* b){return *a > *b;});
}

a_star::Node* a_star::NodeSet::pop()
{
    if(isEmpty())
        return nullptr;
    
    std::pop_heap(mNodeQueue.begin(), mNodeQueue.end(), [](Node* a, Node* b){return *a > *b;});
    Node* minNode = mNodeQueue.back();
    mNodeQueue.pop_back(); // rm the value

    return minNode;
}

void a_star::NodeSet::clear()
{
    for(auto ptr: mNodeQueue){
        delete ptr;
    }
    mNodeQueue.clear();
}

bool a_star::NodeSet::isEmpty()
{
    return mNodeQueue.empty();
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
    auto iter = std::find_if(mNodeQueue.begin(), mNodeQueue.end(), pred);
    if(iter == mNodeQueue.end())
        return nullptr;
    else
        return *iter;
}

a_star::NodeSet::const_iterator a_star::NodeSet::begin() const
{
    return mNodeQueue.begin();
}

a_star::NodeSet::const_iterator a_star::NodeSet::end() const
{
    return mNodeQueue.end();
}

a_star::NodeSet::~NodeSet()
{
    for(auto ptr: mNodeQueue){
        delete ptr;
    }
}



