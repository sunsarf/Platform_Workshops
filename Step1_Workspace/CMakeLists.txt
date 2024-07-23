cmake_minimum_required(VERSION 3.21)
project(HelloWorld)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

# Include header files
include_directories(../../eigen)
include_directories(include/HelloWorld)

# Create a library
add_library(HelloWorldLib src/HelloWorld.cpp)

add_executable(HelloWorld src/run_helloworld.cpp)
target_link_libraries(HelloWorld HelloWorldLib)

add_executable(EigenAdd src/EigenAdd.cpp)

# Enable testing
enable_testing()

# Dowload and unpack Google Test at configure time
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.0
)
FetchContent_MakeAvailable(googletest)

# Create a test executable
add_executable(HelloWorldTest tests/HelloWorldTest.cpp)
target_link_libraries(HelloWorldTest gtest HelloWorldLib)
target_include_directories(HelloWorldTest PRIVATE include/HelloWorld)

# Add the tests to CMake's testing system
add_test(NAME HelloWorldTest COMMAND HelloWorldTest)