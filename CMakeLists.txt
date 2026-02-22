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

# Read MuJoCo version from single source of truth
file(STRINGS "${CMAKE_SOURCE_DIR}/MUJOCO_VERSION" MUJOCO_VERSION)
string(STRIP "${MUJOCO_VERSION}" MUJOCO_VERSION)

if(NOT DEFINED MUJOCO_ROOT_DIR)
  # FetchContent from CMake >= 3.11
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11")
    set(MUJOCO_DOWNLOAD_DIR "$ENV{HOME}/.mujoco/mujoco-${MUJOCO_VERSION}")
    message(
      STATUS
        "MUJOCO_ROOT_DIR is not provided. Will be automatically downloaded to ${MUJOCO_DOWNLOAD_DIR}."
    )
    include(FetchContent)
    FetchContent_Declare(
      mujoco
      URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz
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

# find torch
# Prefer an explicit LIBTORCH_ROOT (e.g. set by Docker), then ext/libtorch,
# then auto-download via FetchContent.
set(LIBTORCH_ROOT "" CACHE PATH "Path to a pre-installed libtorch (overrides auto-download)")
if(LIBTORCH_ROOT)
  message(STATUS "Using provided libtorch at: ${LIBTORCH_ROOT}")
  set(MC_MUJOCO_LIBTORCH_PATH ${LIBTORCH_ROOT})
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${LIBTORCH_ROOT})
elseif(EXISTS ${PROJECT_SOURCE_DIR}/ext/libtorch)
  message(STATUS "Using local ext/libtorch")
  set(MC_MUJOCO_LIBTORCH_PATH ${PROJECT_SOURCE_DIR}/ext/libtorch)
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/ext/libtorch)
else()
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11")
    message(STATUS "libtorch not found. Downloading nightly CPU build...")
    include(FetchContent)
    FetchContent_Declare(libtorch
      URL https://download.pytorch.org/libtorch/nightly/cpu/libtorch-cxx11-abi-shared-with-deps-latest.zip
      SOURCE_DIR ${PROJECT_SOURCE_DIR}/ext/libtorch)
    FetchContent_Populate(libtorch)
    set(MC_MUJOCO_LIBTORCH_PATH ${PROJECT_SOURCE_DIR}/ext/libtorch)
    set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/ext/libtorch)
  else()
    message(FATAL_ERROR "libtorch not found. Set LIBTORCH_ROOT or provide ext/libtorch. CMake >= 3.11 required for auto-download.")
  endif()
endif()
find_package(Torch REQUIRED)

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

add_executable(test_torch tests/test_torch.cpp)
target_link_libraries(test_torch PRIVATE "${TORCH_LIBRARIES}")
target_compile_options(test_torch PRIVATE ${TORCH_CXX_FLAGS})
add_test(NAME test_torch COMMAND test_torch)
