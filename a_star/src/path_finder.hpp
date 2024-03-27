#ifndef PATH_FINDER_HPP_
#define PATH_FINDER_HPP_

#include <unordered_set>
#include "node.hpp"

// Template - Header only class PathFinder<HeuristicPolicy> for better performances
// Class designed to be usable in a "step() by step()" approach for educational purposes (could get the result for each step)
//   Can be used with a single findPath() function call

namespace a_star{
    struct Snapshot{ // an "image" of the current state of the PathFinder
        Vector2D size;
        Vector2D from;
        Vector2D to;
        PointSet obstacles;
        PointSet seenNodes;
        PointSet currentPath;
    };

    enum class State{
        unstarted,
        started,
        finished
    };

    template<typename HeuristicPolicy>
    class PathFinder{
    public:
        PathFinder() = default;
        PathFinder(Vector2D mapSize):mMapSize{mapSize} {};

        // Environment functions
        void setMapSize(Vector2D mapSize){
            mMapSize = mapSize;
            reinitialize(); // environment changed => algo must be re run
        }
       
        PathFinder& addObstacle(Vector2D obstacle){
            mObstacles.insert(obstacle);    
            reinitialize();
            return *this;
        }

        PathFinder& addObstacles(const PointSet& obstacles){
            mObstacles.insert(obstacles.begin(), obstacles.end());
            reinitialize();
            return *this;
        }

        void removeObstacle(Vector2D obstacle){
            mObstacles.erase(obstacle);
        }

        void clearObstacles(){
            mObstacles.clear();
            reinitialize();
        }

        PointSet getObstacles() const {
            PointSet vec(mObstacles.begin(), mObstacles.end());
            return vec;
        }

        bool isInsideObstacle(Vector2D toCheck) const{
            return mObstacles.find(toCheck) != mObstacles.end();
        }

        void enableDiagonal(){
            mDiagonalEnabled = true;
            reinitialize();
        }
        void disableDiagonal(){
            mDiagonalEnabled = false;
            reinitialize();
        }
        bool isDiagonalEnabled() const {
            return mDiagonalEnabled;
        }
        
        // simples getters & setters
        bool isSuccessfull()const{return mSuccess;};
        Vector2D getFromPoint()const{return mFrom;}

        void setFromPoint(Vector2D p){
            mFrom = p;
            reinitialize();
        }
        void setToPoint(Vector2D p){
            mTo= p;
            reinitialize();
        }

        Vector2D getToPoint() const{return mTo;}
        Vector2D getMapSize() const{return mMapSize;};
        const State& getState() const{return mState;}

        PointSet getOpenSet(){
            PointSet vec;
            for(auto elt: mOpenSet)
                vec.push_back(elt->position);
            return vec;
        }
        PointSet getClosedSet() const{
            PointSet vec;
            for(auto elt: mClosedSet)
                vec.push_back(elt->position);
            return vec;
        }
        PointSet getFoundPath() const{return mFoundPath;}
        
        void reinitialize(){
            mState = State::unstarted;
            mSuccess = false;
            mCurrentNode = nullptr;
            mOpenSet.clear();
            mClosedSet.clear();
            mFoundPath.clear();
        }

        bool init(Vector2D from, Vector2D to){
            // Validation of from and to
            if( mFrom.x < 0 || mFrom.y < 0
                || mFrom.x >= mMapSize.x || mFrom.y >= mMapSize.y){
                return false; //do nothing
            }

            // initialisation of variables
            reinitialize();
            mState = State::started;
            mFrom = from;
            mTo = to;

            // initialisation of the openSet
            Node* initNode = new Node(mFrom, nullptr);
            mOpenSet.insert(initNode);

            return true;
        }

        bool step(){
            if(mState != State::started)
                return false; //dont compute

            mCurrentNode = mOpenSet.pop(); // Get the node with the least score and remove it from the open set
            if(!mCurrentNode){
                mSuccess = false; // There is no node in open set : no path found
                mState = State::finished;
                return false; // algo finished failure
            }
            // If the node is the one we search => we can return
            if(mCurrentNode->position == mTo){
                mFoundPath = mCurrentNode->constructPath();
                mSuccess = true;
                mState = State::finished;
                return false; // Algo finished NO next step()
            }

            // checked => add to closed set and see neighbors
            mClosedSet.insert(mCurrentNode);
            for(Vector2D neighbor: mCurrentNode->position.getNeighbors(mMapSize, mDiagonalEnabled)){
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

        Snapshot computeSnapShot() const {
            Snapshot snap{
                .size = getMapSize(),
                .from = getFromPoint(),
                .to = getToPoint(),
                .obstacles = getObstacles(),
                .seenNodes = getClosedSet(),
            };
            snap.currentPath = mSuccess ? getFoundPath() : mCurrentNode->constructPath();
            return snap;
        }

    private:
        const HeuristicPolicy mHeuristic{}; // a heuristic callable object
        Vector2D mMapSize{};
        std::unordered_set<Vector2D, Vector2D::HashFunction> mObstacles{};
        
        Vector2D mFrom;
        Vector2D mTo;
        NodeSet mOpenSet;
        NodeSet mClosedSet;
        PointSet mFoundPath{};
        Node* mCurrentNode;

        State mState{State::unstarted};
        bool mSuccess{false};
        bool mDiagonalEnabled{true};
    };
}

#endif // PATH_FINDER_HPP_
