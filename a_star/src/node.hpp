#ifndef NODE_HPP_
#define NODE_HPP_

#include <vector>

namespace a_star{
    
    // Can represent a point or a direction
    struct Vector2D{
        Vector2D() = default;
        Vector2D(uint16_t x, uint16_t y);
        bool operator==(Vector2D other) const;
        float length();
        std::vector<Vector2D> getNeighbors(Vector2D mapSize);

        static Vector2D delta(Vector2D a, Vector2D b);
        static float distance(Vector2D a, Vector2D b);
        
        // attributes
        uint16_t x{}; 
        uint16_t y{};

        // hash function for unordered_set
        struct HashFunction
        {
            size_t operator()(const Vector2D& vector2d) const;
        };

    };


    // For the linked list of elements 
    struct Node{
        
        Node()=delete;
        Node(Vector2D position, Node* predecessor);
 
        std::vector<Vector2D> constructPath(); 
        
        Vector2D position{}; //The position represented by the node
        Node* predecessor{nullptr}; //non owning pointer to previous node 
        
        float pathCost{}; // The cost from the beginning to position 
        float heuristicCost{}; // The heuristical cost from Vector2D to 
        float score{};
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
