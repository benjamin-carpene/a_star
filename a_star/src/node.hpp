#ifndef NODE_HPP_
#define NODE_HPP_

#include <vector>
#include <inttypes.h>
#include <iostream>


namespace a_star{
    
    // Can represent a point or a direction
    struct Vector2D{
        Vector2D() = default;
        Vector2D(uint16_t x, uint16_t y);
        bool operator==(Vector2D other);
        std::vector<Vector2D> getNeighbors(Vector2D mapSize);

        uint16_t x{}; 
        uint16_t y{};
    };


    // For the linked list of elements 
    struct Node{
        
        Node()=delete;
        Node(Vector2D position, Node* predecessor);
 
        std::vector<Vector2D> constructPath(); 
        
        Vector2D position{}; //The position represented by the node
        Node* predecessor{nullptr}; //non owning pointer to previous node 
        
        uint16_t pathCost{}; // The cost from the beginning to position 
        uint16_t heuristicCost{}; // The heuristical cost from Vector2D to 
        uint16_t score{};
    };


    // Manages Nodes for the openSet and closedSet
    class NodeSet{
    public:
        NodeSet()=default;
        ~NodeSet();
        void insert(Node* node);
        Node* pop();
        void clear();
        bool isEmpty();
        bool includes(Vector2D position);
        Node* getNodeFromPosition(Vector2D position);

    private:
        // we use a vector with the std heap functions because we also 
        // need to know if a coordinate is already in the container
        std::vector<Node*> nodeQueue; 

    };
}

#endif // NODE_HPP_