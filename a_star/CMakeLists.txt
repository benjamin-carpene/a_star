cmake_minimum_required(VERSION 3.10)

set(SOURCES
    src/heuristics.cpp
    src/node.cpp
    src/path_finder.cpp)

set(HEADERS
    src/path_finder.hpp
    src/a_star.hpp
    src/heuristics.hpp
    src/node.hpp)
    
add_library(a_star_lib STATIC ${SOURCES} ${HEADERS})
target_include_directories(a_star_lib PUBLIC src)
target_compile_features(a_star_lib PRIVATE cxx_std_17)
