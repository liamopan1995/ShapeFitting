cmake_minimum_required(VERSION 3.10)
project(simple_math_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(simple_math src/simple_math.cpp)

enable_testing()
add_subdirectory(test)
