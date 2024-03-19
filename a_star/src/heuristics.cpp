#include "heuristics.hpp"

#include <math.h>

uint16_t a_star::heuristics::EuclidianHeuristic::calc(Vector2D a, Vector2D b) const
{
    int x = a.x - b.x;
    int y = a.y - b.y;
    return sqrt(x*x + y*y);
}
