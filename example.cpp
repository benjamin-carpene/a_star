#include <iostream>
#include "a_star.hpp"

int main(){ 
    // pathfinder using EuclidianHeuristic as its policy
    a_star::PathFinder<a_star::heuristics::EuclidianHeuristic> pathFinder;
    pathFinder.setMapSize({50, 50});

    // "Build" the obstacle
    pathFinder 
        .addObstacle({10,20})
        .addObstacles({ {12, 20}, {13, 20} }) // or as a list
        .addObstacles({{1,1}, {1,0}, {0,1}}); // Won't work with these obstacles
    
    for(uint16_t i{14}; i< 40; ++i)
        pathFinder.addObstacle({i, 20});
    
    if(pathFinder.findPath({0,0}, {49, 49}))
        for(auto point : pathFinder.getFoundPath())
            std::cout << "Path : x y = " << point.x << ' ' << point.y << '\n';
    else
        std::cout << "pathFinder couldn't find a path" << std::endl;

    return 0;
}



