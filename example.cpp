#include <iostream>
#include <string>
#include "a_star.hpp"

int main(){
    // 1. Construct the PathFinder object using a given Heuristic as policy
    a_star::PathFinder<a_star::heuristics::EuclidianHeuristic> pathFinder;

    // 2. Set environment data
    pathFinder.setMapSize({50, 50});
    pathFinder
        .addObstacle({10,20})
        .addObstacles({ {12, 20}, {13, 20} });

    // 3. Use the a star algorithm
    bool succeeded = pathFinder.findPath({0,0}, {49, 49});
    if(succeeded){
        // Do someting with the path
        a_star::PointSet path = pathFinder.getFoundPath();
        for(const auto &point: path)
            std::cout << static_cast<std::string>(point) << std::endl;
    }else{
        std::cout << "Didnt succeed" << std::endl;
    }

    // 3 bis : use the program iteratively
    // 3.1 init the pathfinder with the "from" and "to" points
    if(pathFinder.init({0,0}, {49, 49})){

        //3.2 Call .step method in a while loop until it stops
        while(pathFinder.step()){
            // Do whatever you want between 2 steps
        }

        // 3.3 Do whatever you want with the final result
        if(pathFinder.isSuccessfull()){
            std::cout << "it worked";
        }
    }

    return 0;
}



