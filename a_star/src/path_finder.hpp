#ifndef PATH_FINDER_HPP_
#define PATH_FINDER_HPP_

#include <vector>
#include <unordered_set>
#include "node.hpp"

// Template - Header only class PathFinder<HeuristicPolicy> for better performances
// Class designed to be usable in a "step() by step()" approach for educational purposes (could get the result for each step)
//   Can be used with a single findPath() function call

namespace a_star{

    template<typename HeuristicPolicy>
    class PathFinder{
    public:
        // Environment functions
        void setMapSize(Vector2D mapSize){
            mMapSize = mapSize;
        }

        auto getObstacles()const {
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

        bool isInsideObstacle(Vector2D toCheck) const{
            return mObstacles.find(toCheck) != mObstacles.end();
        }

        // getters
        bool isSuccessfull()const{return mSuccess;};
        Vector2D getFromPoint()const{return mFrom;}
        Vector2D getToPoint()const{return mTo;}
        NodeSet getOpenSet()const{return mOpenSet;}
        NodeSet getClosedSet()const{return mClosedSet;}
        std::vector<Vector2D> getFoundPath()const{return mFoundPath;}
        

        void clearComputationData(){
            mFrom = mTo = Vector2D{};
            mOpenSet.clear();
            mClosedSet.clear();
            mFoundPath.clear();
        }

        bool init(Vector2D from, Vector2D to){
            // Validation of from and to
            if( mFrom.x < 0 || mFrom.y < 0
                || mFrom.x >= mMapSize.x || mFrom.y >= mMapSize.y){
                return false;
            }

            // initialisation of variables
            mSuccess = false;
            mFrom = from;
            mTo = to;
            mOpenSet.clear();
            mClosedSet.clear();

            // initialisation of the openSet
            Node* initNode = new Node(mFrom, nullptr);
            mOpenSet.insert(initNode);

            return true;
        }

        bool step(){
            mCurrentNode = mOpenSet.pop(); // Get the node with the least score and remove it from the open set
            if(!mCurrentNode){
                mSuccess = false; // There is no node in open set : no path found
                return false; // algo finished failure
            }
            // If the node is the one we search => we can return
            if(mCurrentNode->position == mTo){
                mFoundPath = mCurrentNode->constructPath();
                mSuccess = true;
                return false; // Algo finished NO next step
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

            return true; // next iteration
        }

        bool findPath(Vector2D from, Vector2D to){
            // 1. Init the values
            if(!init(from, to)){
                mSuccess = false;
                return mSuccess;
            }
            // 2. Compute each step of the algorithms until its finished (success or failure)
            while(step());
            return mSuccess;
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

        bool mSuccess{false};
    };
}

#endif // PATH_FINDER_HPP_
