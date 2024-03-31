#ifndef PATH_FINDER_HPP_
#define PATH_FINDER_HPP_

#include <memory>
#include <unordered_set>
#include "node.hpp"
#include "heuristics.hpp"

// Class designed to be usable in a "step() by step()" approach for educational purposes (could get the result for each step)
// Can be used with a single findPath() function call

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

    class PathFinder{
    public:
        PathFinder() = default;
        PathFinder(Vector2D mapSize):mMapSize{mapSize} {}

        // Environment functions
        void setMapSize(Vector2D mapSize);
        PathFinder& addObstacle(Vector2D obstacle);
        PathFinder& addObstacles(const PointSet& obstacles);
        void removeObstacle(Vector2D obstacle);
        void clearObstacles();
        PointSet getObstacles() const;
        bool isInsideObstacle(Vector2D toCheck) const;
        void enableDiagonal();
        void disableDiagonal();
        bool isDiagonalEnabled() const;
        
        // simples getters & setters
        bool isSuccessfull()const;
        Vector2D getFromPoint()const;
        void setFromPoint(Vector2D p);
        void setToPoint(Vector2D p);
        Vector2D getToPoint() const;
        Vector2D getMapSize() const;
        const State& getState() const;
        PointSet getOpenSet();
        PointSet getClosedSet() const;
        PointSet getFoundPath() const;

        void reinitialize();
        bool init();
        bool init(Vector2D from, Vector2D to);
        bool step();
        bool findPath(Vector2D from, Vector2D to);
        Snapshot computeSnapShot() const;

        const heuristics::HeuristicInterface*heuristic() const;
        void setHeuristic(std::unique_ptr<const heuristics::HeuristicInterface> newHeuristic);

    private:
        std::unique_ptr<const heuristics::HeuristicInterface> mHeuristic{nullptr}; // a heuristic callable object
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
