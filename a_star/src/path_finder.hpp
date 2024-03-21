#ifndef PATH_FINDER_HPP_
#define PATH_FINDER_HPP_

#include <vector>
#include <unordered_set>

#include "node.hpp"

namespace a_star{

    template<typename HeuristicPolicy>
    class PathFinder{
    public:
        // Environment functions
        void setMapSize(Vector2D mapSize){
            mMapSize = mapSize;
        }

        auto getObstacles(){
            return mObstacles;
        }
        
        PathFinder& addObstacle(Vector2D obstacle){
            mObstacles.insert(obstacle);    
            return *this;
        }
    
        PathFinder& addObstacles(std::vector<Vector2D> obstacles){
            mObstacles.insert(obstacles.begin(), obstacles.end());
            return *this;
        }

        void clearObstacles(){
            mObstacles.clear();
        }

        bool isInsideObstacle(Vector2D toCheck){

            return mObstacles.find(toCheck) != mObstacles.end();
        }

        // getters
        Vector2D getFromPoint(){return mFrom;}
        Vector2D getToPoint(){return mTo;}
        NodeSet getOpenSet(){return mOpenSet;}
        NodeSet getClosedSet(){return mClosedSet;}
        std::vector<Vector2D> getFoundPath(){return mFoundPath;}
        

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
                mCurrentNode = mOpenSet.pop(); // Get the node with the least score and remove it from the open set
                
                // If the node is the one we search => we can return
                if(mCurrentNode->position == mTo){
                    mFoundPath = mCurrentNode->constructPath();
                    return true;
                }

                // checked => add to closed set and see neighbors
                mClosedSet.insert(mCurrentNode);
                for(Vector2D neighbor: mCurrentNode->position.getNeighbors(mMapSize)){  
                    if(isInsideObstacle(neighbor) || mClosedSet.includes(neighbor))
                        continue; 
                    
                    // compute the new scores
                    float pahtCost = mCurrentNode->pathCost + Vector2D::distance(mCurrentNode->position, neighbor);
                    float heuristicCost = mHeuristic.calc(neighbor, mTo);
                    float score = pahtCost + heuristicCost;

                    if(!mOpenSet.includes(neighbor)){ 
                        // not inside so we create it
                        Node* nextNode = new Node(neighbor, mCurrentNode);

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

                            existingNode->predecessor = mCurrentNode;
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
        std::unordered_set<Vector2D, Vector2D::HashFunction> mObstacles{};
        
        Vector2D mFrom;
        Vector2D mTo;
        NodeSet mOpenSet;
        NodeSet mClosedSet;
        std::vector<Vector2D> mFoundPath{};
        Node* mCurrentNode;
    };
    
}

#endif // PATH_FINDER_HPP_
