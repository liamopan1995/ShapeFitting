cmake_minimum_required(VERSION 3.10)

project(FitCircle2D)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Eigen library
find_package(Eigen3 REQUIRED)
# Include the Eigen header directories
include_directories(${EIGEN3_INCLUDE_DIR})


# Add the executable for the fit_circle_2d function
add_library(CircleFitByLeastSquare_library STATIC CircleFitByLeastSquare.cpp)
add_library(CircleFitByHyper_library STATIC CircleFitByHyper.cpp)
add_library(CircleFitByKasa_library STATIC CircleFitByKasa.cpp)
add_library(CircleFitByPratt_library STATIC CircleFitByPratt.cpp)
add_library(CircleFitByTaubin_library STATIC CircleFitByTaubin.cpp)
# Add the executable
add_executable(Test_vector_input Test_vector_input.cpp)
add_executable(TestAlgebraicCircleFits TestAlgebraicCircleFits.cpp)

# Link against Eigen
target_link_libraries(Test_vector_input Eigen3::Eigen CircleFitByLeastSquare_library)

target_link_libraries(TestAlgebraicCircleFits CircleFitByLeastSquare_library
CircleFitByHyper_library
CircleFitByKasa_library
CircleFitByPratt_library
CircleFitByTaubin_library
)


# # Add the executable
# add_executable( Test_vector_input 
# Test_vector_input.cpp
# CircleFitByLeastSquare.cpp )

# add_executable( TestAlgebraicCircleFits 
# TestAlgebraicCircleFits.cpp 
# CircleFitByLeastSquare.cpp
# CircleFitByHyper.cpp
# CircleFitByKasa.cpp 
# CircleFitByPratt.cpp
# CircleFitByTaubin.cpp
# Utilities.cpp)

# # Link against Eigen
# target_link_libraries(Test_vector_input Eigen3::Eigen )

# target_link_libraries(TestAlgebraicCircleFits Eigen3::Eigen)