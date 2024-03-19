cmake_minimum_required(VERSION 3.10)

set(SOURCES
    src/heuristics.cpp
    src/node.cpp)
    
add_library(a_star_lib STATIC ${SOURCES})
target_include_directories(a_star_lib PUBLIC src)