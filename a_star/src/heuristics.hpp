#ifndef HEURISTICS_HEURISTICS_HPP_
#define HEURISTICS_HEURISTICS_HPP_

#include "node.hpp"

namespace a_star::heuristics{      

    // An Interface for the heuristic policies, uses the C++ policy design pattern
    struct HeuristicPolicyInterface{
        HeuristicPolicyInterface() = default;
        virtual uint16_t calc(Vector2D a, Vector2D b) const = 0;
    };


    struct EuclidianHeuristic : public HeuristicPolicyInterface{
        uint16_t calc(Vector2D a, Vector2D b) const override;
    };
}


#endif // HEURISTICS_HEURISTICS_HPP_