#ifndef NODE_HPP_
#define NODE_HPP_

#include <vector>
#include <inttypes.h>
#include <string>

namespace a_star{

    // Can represent a point or a direction
    
    struct Vector2D{
        // Ctors
        Vector2D() = default;
        Vector2D(uint16_t x, uint16_t y);

        // operators
        bool operator==(Vector2D other) const;
        operator std::string() const;

        // methods
        float length() const;
        std::vector<Vector2D> getNeighbors(Vector2D mapSize, bool diagonals) const;

        // static methods
        static Vector2D delta(Vector2D a, Vector2D b);
        static float distance(Vector2D a, Vector2D b);
        
        // attributes
        uint16_t x{}; 
        uint16_t y{};

        // hash function for unordered_set
        struct HashFunction
        {
            std::size_t operator()(const Vector2D& vector2d) const;
        };

    };
    using PointSet = std::vector<Vector2D>;

    // For the linked list of elements 
    struct Node{
        
        Node()=delete;
        Node(Vector2D position, Node* predecessor = nullptr);
 
        PointSet constructPath() const;
        
        // equality operators (as per their score)
        bool operator==(const Node& other) const;
        bool operator<(const Node& other) const;
        bool operator>(const Node& other) const;
        
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
        bool isEmpty() const;
        bool includes(Vector2D position) const;
        Node* getNodeFromPosition(Vector2D position) const;

        using const_iterator = std::vector<Node*>::const_iterator;
        const_iterator begin() const;
        const_iterator end() const;

    private:
        // we use a vector with the std heap functions because we also 
        // need to know if a coordinate is already in the container
        std::vector<Node*> mNodeQueue;
    };


}

#endif // NODE_HPP_
