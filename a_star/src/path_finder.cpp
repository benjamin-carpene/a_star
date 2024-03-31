#include "path_finder.hpp"

void a_star::PathFinder::setMapSize(Vector2D mapSize){
    mMapSize = mapSize;
    reinitialize(); // environment changed => algo must be re run
}

a_star::PathFinder &a_star::PathFinder::addObstacle(Vector2D obstacle){
    mObstacles.insert(obstacle);
    reinitialize();
    return *this;
}

a_star::PathFinder &a_star::PathFinder::addObstacles(const PointSet &obstacles){
    mObstacles.insert(obstacles.begin(), obstacles.end());
    reinitialize();
    return *this;
}

void a_star::PathFinder::removeObstacle(Vector2D obstacle){
    mObstacles.erase(obstacle);
}

void a_star::PathFinder::clearObstacles(){
    mObstacles.clear();
    reinitialize();
}

a_star::PointSet a_star::PathFinder::getObstacles() const {
    PointSet vec(mObstacles.begin(), mObstacles.end());
    return vec;
}

bool a_star::PathFinder::isInsideObstacle(Vector2D toCheck) const{
    return mObstacles.find(toCheck) != mObstacles.end();
}

void a_star::PathFinder::enableDiagonal(){
    mDiagonalEnabled = true;
    reinitialize();
}

void a_star::PathFinder::disableDiagonal(){
    mDiagonalEnabled = false;
    reinitialize();
}

bool a_star::PathFinder::isDiagonalEnabled() const {
    return mDiagonalEnabled;
}

bool a_star::PathFinder::isSuccessfull() const{return mSuccess;}

a_star::Vector2D a_star::PathFinder::getFromPoint() const{return mFrom;}

void a_star::PathFinder::setFromPoint(Vector2D p){
    mFrom = p;
    reinitialize();
}

void a_star::PathFinder::setToPoint(Vector2D p){
    mTo= p;
    reinitialize();
}

a_star::Vector2D a_star::PathFinder::getToPoint() const{return mTo;}

a_star::Vector2D a_star::PathFinder::getMapSize() const{return mMapSize;}

const a_star::State &a_star::PathFinder::getState() const{return mState;}

a_star::PointSet a_star::PathFinder::getOpenSet(){
    PointSet vec;
    for(auto elt: mOpenSet)
        vec.push_back(elt->position);
    return vec;
}

a_star::PointSet a_star::PathFinder::getClosedSet() const{
    PointSet vec;
    for(auto elt: mClosedSet)
        vec.push_back(elt->position);
    return vec;
}

a_star::PointSet a_star::PathFinder::getFoundPath() const{return mFoundPath;}

void a_star::PathFinder::reinitialize(){
    mState = State::unstarted;
    mSuccess = false;
    mCurrentNode = nullptr;
    mOpenSet.clear();
    mClosedSet.clear();
    mFoundPath.clear();
}

bool a_star::PathFinder::init(){
    reinitialize();
    mState = State::started;
    Node* initNode = new Node(mFrom, nullptr);
    mOpenSet.insert(initNode);
    if(!mHeuristic)
        mHeuristic = std::make_unique<a_star::heuristics::EuclidianHeuristic>(); //default heuristic
    return true;
}

bool a_star::PathFinder::init(Vector2D from, Vector2D to){
    // Validation of from and to
    if(
        mFrom.x < 0 || mFrom.y < 0 || mFrom.x >= mMapSize.x || mFrom.y >= mMapSize.y
        || mTo.x < 0 || mTo.y < 0 || mTo.x >= mMapSize.x || mTo.y >= mMapSize.y
        ){
        return false; //do nothing
        mState = State::finished;
    }
    // initialisation of variables
    mFrom = from;
    mTo = to;

    return init();
}

bool a_star::PathFinder::step(){
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
        float heuristicCost = mHeuristic->calc(neighbor, mTo);
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

bool a_star::PathFinder::findPath(Vector2D from, Vector2D to){
    // 1. Init the values
    if(!init(from, to)){
        mSuccess = false;
        return mSuccess;
    }
    // 2. Compute each step of the algorithms until its finished (success or failure)
    while(step());
    return mSuccess;
}

a_star::Snapshot a_star::PathFinder::computeSnapShot() const {
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

const a_star::heuristics::HeuristicInterface* a_star::PathFinder::heuristic() const
{
    return mHeuristic.get();
}

void a_star::PathFinder::setHeuristic(std::unique_ptr<const heuristics::HeuristicInterface> newHeuristic)
{
    mHeuristic = std::move(newHeuristic);
}

