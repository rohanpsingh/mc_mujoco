cmake_minimum_required(VERSION 3.10)

# These variables have to be defined before running setup_project
set(PROJECT_NAME mc_mujoco)
set(PROJECT_DESCRIPTION "Mujoco interface for mc-rtc")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(
  ${PROJECT_NAME}
  LANGUAGES C CXX
  VERSION 1.0.0)

if(NOT DEFINED MUJOCO_ROOT_DIR)
  # FetchContent from CMake >= 3.11
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11")
    set(MUJOCO_DOWNLOAD_DIR "$ENV{HOME}/.mujoco/mujoco301")
    message(
      STATUS
        "MUJOCO_ROOT_DIR is not provided. Will be automatically downloaded to ${MUJOCO_DOWNLOAD_DIR}."
    )
    include(FetchContent)
    FetchContent_Declare(
      mujoco
      URL https://github.com/google-deepmind/mujoco/releases/download/3.0.1/mujoco-3.0.1-linux-x86_64.tar.gz
          SOURCE_DIR
          ${MUJOCO_DOWNLOAD_DIR})
    FetchContent_MakeAvailable(mujoco)
    set(MUJOCO_ROOT_DIR "${MUJOCO_DOWNLOAD_DIR}")
  else()
    message(
      FATAL_ERROR
        "MUJOCO_ROOT_DIR is not provided. Download MuJoCo from mujoco.org and provide MUJOCO_ROOT_DIR."
    )
  endif()
endif()

find_package(mc_rtc REQUIRED)

option(USE_GL "Use Mujoco with OpenGL" ON)
set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco.h")
  set(MUJOCO_ROOT_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}")
  set(MUJOCO_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}/mujoco")
endif()
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/lib")
set(MUJOCO_SAMPLE_DIR "${MUJOCO_ROOT_DIR}/sample")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT_DIR}/simulate")
message("${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo root dir: " ${MUJOCO_ROOT_DIR})

set(COMBINED_MC_MUJOCO_LIB OFF)
# find glfw library
find_library(
  GLFW
  NAMES libglfw.so.3
  PATHS ${MUJOCO_BIN_DIR}
  NO_DEFAULT_PATH)
if(NOT GLFW)
  if(UNIX)
    set(COMBINED_MC_MUJOCO_LIB ON)
  endif()
  set(BUILD_STATIC
      ON
      CACHE BOOL "" FORCE)
  add_subdirectory(ext/glfw EXCLUDE_FROM_ALL)
else()
  message(STATUS "GLFW lib found at: " ${GLFW})
  file(COPY "${MUJOCO_INCLUDE_DIR}/glfw3.h"
       DESTINATION "${PROJECT_BINARY_DIR}/src/include/GLFW/")
endif()

set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# find mujoco library
if(USE_GL)
  file(GLOB LIB_MUJOCO ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9].so
       ${MUJOCO_LIB_DIR}/libmujoco.so.*)
else()
  file(GLOB LIB_MUJOCO ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9]nogl.so
       ${MUJOCO_LIB_DIR}/libmujoco_nogl.so.*)
endif()
message(STATUS "MuJoCo lib found at: " ${LIB_MUJOCO})

set(MC_MUJOCO_SHARE_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco")
if(WIN32)
  set(MC_MUJOCO_USER_DESTINATION "$ENV{APPDATA}/mc_rtc/mc_mujoco")
else()
  set(MC_MUJOCO_USER_DESTINATION "$ENV{HOME}/.config/mc_rtc/mc_mujoco")
endif()

add_subdirectory(cmake)
add_subdirectory(ext/pugixml)
add_subdirectory(src)
add_subdirectory(robots)

enable_testing()
