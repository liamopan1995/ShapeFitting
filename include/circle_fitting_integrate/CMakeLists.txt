cmake_minimum_required(VERSION 3.10)

project(FitCircle_3D_package)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Eigen library
find_package(Eigen3 REQUIRED)
# Include the Eigen header directories
include_directories(${EIGEN3_INCLUDE_DIR})



# Add the executable
add_executable(Test_vector_Integrate Test_vector_Integrate.cpp )
# add_executable(TestAlgebraicCircleFits TestAlgebraicCircleFits.cpp)

# Link against Eigen
# target_link_libraries(Test_vector_input Eigen3::Eigen CircleFitByLeastSquare_library)

target_link_libraries(Test_vector_Integrate Eigen3::Eigen )
