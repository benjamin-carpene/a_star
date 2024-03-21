#include "heuristics.hpp"

#include <math.h>

const float sqrt2 = static_cast<float>(sqrt(2));

float a_star::heuristics::EuclidianHeuristic::calc(Vector2D a, Vector2D b) const
{
    return a_star::Vector2D::distance(a, b);
}

float a_star::heuristics::ManhattanHeuristic::calc(Vector2D a, Vector2D b) const
{
    auto delta = Vector2D::delta(a, b);
    return static_cast<float>(delta.x + delta.y);
}

float a_star::heuristics::ChebyshevHeuristic::calc(Vector2D a, Vector2D b) const
{   
    auto delta = Vector2D::delta(a, b);
    return std::max(delta.x, delta.y);
}

float a_star::heuristics::OctileHeuristic::calc(Vector2D a, Vector2D b) const
{
    auto delta = Vector2D::delta(a, b);
    int max = std::max(delta.x, delta.y);
    int min = std::min(delta.x, delta.y);

    return sqrt2 * min + max - min;
}
