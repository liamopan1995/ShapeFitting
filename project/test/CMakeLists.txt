find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

add_executable(simple_math_test test_simple_math.cpp)

target_include_directories(simple_math_test PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(simple_math_test PRIVATE simple_math Threads::Threads)

if (TARGET GTest::gtest_main)
    target_link_libraries(simple_math_test PRIVATE GTest::gtest_main)
else()
    target_link_libraries(simple_math_test PRIVATE GTest::GTest GTest::Main)
endif()

# Ensure the test executable can find the conda-installed GTest shared libraries at runtime.
set_target_properties(simple_math_test PROPERTIES
    BUILD_RPATH "/opt/conda/lib"
    INSTALL_RPATH "/opt/conda/lib"
)

add_test(NAME simple_math_test COMMAND simple_math_test)
