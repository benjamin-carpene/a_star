#ifndef HEURISTICS_HEURISTICS_HPP_
#define HEURISTICS_HEURISTICS_HPP_

#include "node.hpp"

namespace a_star::heuristics{      

    // An Interface for the heuristic policies, uses the C++ policy design pattern
    struct HeuristicInterface{
        virtual float calc(Vector2D a, Vector2D b) const = 0;
        virtual ~HeuristicInterface(){}
    };

    struct EuclidianHeuristic : public HeuristicInterface{
        float calc(Vector2D a, Vector2D b) const override;
    };

    struct ManhattanHeuristic : public HeuristicInterface{
        float calc(Vector2D a, Vector2D b) const override;
    };

    struct ChebyshevHeuristic : public HeuristicInterface{
        float calc(Vector2D a, Vector2D b) const override;
    };

    struct OctileHeuristic : public HeuristicInterface{
        float calc(Vector2D a, Vector2D b) const override;
    };
}


#endif // HEURISTICS_HEURISTICS_HPP_
