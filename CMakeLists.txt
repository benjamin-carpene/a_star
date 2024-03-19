cmake_minimum_required(VERSION 3.10)
project(a_star_example)

add_executable(test_a_star example.cpp example2.cpp)
target_compile_features(test_a_star PRIVATE cxx_std_17)

add_subdirectory(a_star) # a_star_lib
target_link_libraries(test_a_star PRIVATE a_star_lib)