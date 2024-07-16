cmake_minimum_required(VERSION 3.21)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

# Include Eigen directories
include_directories(../../eigen)

add_executable(HelloWorld src/HelloWorld.cpp)
add_executable(EigenAdd src/EigenAdd.cpp)