#! /bin/bash
# Quick script for compiling and launching the test executable
cmake -S . -B build && 
cmake --build build && 
./build/test_a_star &&
echo ''