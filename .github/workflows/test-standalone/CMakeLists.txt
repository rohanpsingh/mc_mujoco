cmake_minimum_required(VERSION 3.1)

project(TestMcMuJoCo LANGUAGES CXX VERSION 1.0.0)

include(CTest)
enable_testing()

find_package(mc_mujoco REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC mc_mujoco::mc_mujoco)

add_test(NAME TestMcMuJoCo COMMAND main)
