#ifndef PATH_FINDER_HPP_
#define PATH_FINDER_HPP_

#include <vector>
#include <inttypes.h>
#include <iostream>

#include "node.hpp"

namespace a_star{

    template<typename HeuristicPolicy>
    class PathFinder{
    public:
        // Environment functions
        void setMapSize(Vector2D mapSize){
            mMapSize = mapSize;
        }
        
        PathFinder& addObstacle(Vector2D obstacle){
            mObstacles.push_back(obstacle);    
            return *this;
        }
    
        PathFinder& addObstacles(std::vector<Vector2D> obstacles){
            for(auto obstacle : obstacles)
                mObstacles.push_back(obstacle); 
            return *this;
        }

        bool isInsideObstacle(Vector2D toCheck){
            for(auto obstacleCell : mObstacles){
                if(obstacleCell == toCheck){
                    return true;
                }
            }
            return false;
        }

        void clearObstacles(){
            mObstacles.clear();   
        }

        // TODO : getters for the NodeSets

        auto getFoundPath(){return mFoundPath;}

        void clearComputationData(){
            mFrom = mTo = Vector2D{};
            mOpenSet.clear();
            mClosedSet.clear();
            mFoundPath.clear();
        }

        void init(Vector2D from, Vector2D to){
            mFrom = from;
            mTo = to;
            mOpenSet.clear();
            mClosedSet.clear();
        }


        bool findPath(Vector2D from, Vector2D to){
            init(from, to);
            Node* initNode = new Node(mFrom, nullptr);
            mOpenSet.insert(initNode);

            while(!mOpenSet.isEmpty()){
                Node* current = mOpenSet.pop(); // Get the node with the least score and remove it from the open set
                
                // If the node is the one we search => we can return
                if(current->position == mTo){
                    mFoundPath = current->constructPath();
                    return true;
                }

                // checked => add to closed set and see neighbors
                mClosedSet.insert(current);
                for(Vector2D neighbor: current->position.getNeighbors(mMapSize)){  
                    if(isInsideObstacle(neighbor) || mClosedSet.includes(neighbor))
                        continue; 
                    
                    // compute the new scores
                    uint16_t pahtCost = current->pathCost + 1; // TODO better distance approximation, using float ?
                    uint16_t heuristicCost = mHeuristic.calc(neighbor, mTo);
                    uint16_t score = pahtCost + heuristicCost;

                    if(!mOpenSet.includes(neighbor)){ 
                        // not inside so we create it
                        Node* nextNode = new Node(neighbor, current);

                        nextNode->pathCost = pahtCost;
                        nextNode->heuristicCost = heuristicCost;
                        nextNode->score = score;

                        mOpenSet.insert(nextNode);
                    }
                    else {
                        //inside so we update
                        Node* existingNode = mOpenSet.getNodeFromPosition(neighbor);
                        if(existingNode->score > score){
                            existingNode->pathCost = pahtCost;
                            existingNode->heuristicCost = heuristicCost;
                            existingNode->score = score;

                            existingNode->predecessor = current;
                        }
                    }        
                }
            }
            return false; // A* didnt find a path
        }

        // TODO : program an "iterative" version so that we can see the results with the open set, closed set ...
        void step(){
            
        }

        void findPath(){
            
        }

    private:
        const HeuristicPolicy mHeuristic{}; // a heuristic callable object
        Vector2D mMapSize{};
        // TODO refactor for using sets
        std::vector<Vector2D> mObstacles{}; 
        
        Vector2D mFrom;
        Vector2D mTo;
        NodeSet mOpenSet;
        NodeSet mClosedSet;
        std::vector<Vector2D> mFoundPath{};
    };
    
}

#endif // PATH_FINDER_HPP_