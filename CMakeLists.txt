cmake_minimum_required(VERSION 3.1)

# These variables have to be defined before running setup_project
set(PROJECT_NAME mc_mujoco)
set(PROJECT_DESCRIPTION "Mujoco interface for mc-rtc")

set(CMAKE_CXX_STANDARD 11)

project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.0.0)

set(USE_GL 1)
set(MUJOCO_SRC_DIR "$ENV{HOME}/.mujoco/mujoco200/bin" CACHE String "Path to mujoco bin directory.")
set(MUJOCO_INCLUDE_DIR "$ENV{HOME}/.mujoco/mujoco200/include" CACHE String "Path to mujoco include directory.")
message(STATUS "MuJoCo path: " ${MUJOCO_SRC_DIR})

link_directories(${MUJOCO_SRC_DIR})

# find glfw library
find_library(GLFW
  NAMES libglfw.so.3
  PATHS ${MUJOCO_SRC_DIR}
  NO_DEFAULT_PATH)
message(STATUS "GLFW lib found at: " ${GLFW})

# find mujoco library
if(${USE_GL})
  file(GLOB LIB_MUJOCO ${MUJOCO_SRC_DIR}/libmujoco[0-9][0-9][0-9].so)
else()
  file(GLOB LIB_MUJOCO ${MUJOCO_SRC_DIR}/libmujoco[0-9][0-9][0-9]nogl.so)
endif()
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

find_package(mc_rtc 1.4.0 REQUIRED)

add_subdirectory(src)

enable_testing()
