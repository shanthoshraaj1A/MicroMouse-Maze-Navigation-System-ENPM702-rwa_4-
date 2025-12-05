cmake_minimum_required(VERSION 3.20)
project(rwa4 VERSION 1.0 LANGUAGES C CXX)

add_executable(
    rwa4_cpp 
    src/main.cpp 
    src/maze_api.cpp
    src/dfs.cpp
    src/robot.cpp
    src/bfs.cpp
    )

target_include_directories(rwa4_cpp PRIVATE include)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-Wall -Wextra -Werror -pedantic-errors)

# Set C++17 standard for the targets
set_property(TARGET rwa4_cpp PROPERTY CXX_STANDARD 17)
set_property(TARGET rwa4_cpp PROPERTY CXX_STANDARD_REQUIRED ON)