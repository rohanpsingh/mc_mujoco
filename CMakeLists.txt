cmake_minimum_required(VERSION 3.22)

# These variables have to be defined before running setup_project
set(PROJECT_NAME mc_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

project(
  mc_mujoco
  LANGUAGES C CXX
  VERSION 1.0.0
  DESCRIPTION "Mujoco interface for mc-rtc"
  HOMEPAGE_URL "https://github.com/rohanpsingh/mc_mujoco")

include(cmake/get-jrl-cmakemodules.cmake)

jrl_configure_defaults()

# jrl_option(USE_GL "Use Mujoco with OpenGL" ON)

jrl_find_package(mc_rtc REQUIRED)
jrl_find_package(mujoco 3.3.6 REQUIRED)
# ok
get_target_property(MUJOCO_INCLUDE_DIR mujoco::mujoco
                    INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "MuJoCo include directory: ${MUJOCO_INCLUDE_DIR}")
# remove the last component (`include/`) from the path
get_filename_component(MUJOCO_ROOT_DIR "${MUJOCO_INCLUDE_DIR}" DIRECTORY)
set(MUJOCO_ROOT_DIR "${MUJOCO_INCLUDE_DIR}")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT_DIR}/simulate")
message(STATUS "MuJoCo root directory: ${MUJOCO_ROOT_DIR}")
set(MC_MUJOCO_SHARE_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco")
set(MC_MUJOCO_USER_DESTINATION "FIXME")

jrl_find_package(glfw3 REQUIRED)
set(COMBINED_MC_MUJOCO_LIB OFF)

set(OpenGL_GL_PREFERENCE "GLVND")
jrl_find_package(OpenGL REQUIRED)
jrl_find_package(GLEW REQUIRED)

# set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin") set(MUJOCO_INCLUDE_DIR
# "${MUJOCO_ROOT_DIR}/include") if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco.h")
# set(MUJOCO_ROOT_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}") set(MUJOCO_INCLUDE_DIR
# "${MUJOCO_INCLUDE_DIR}/mujoco") endif() set(MUJOCO_LIB_DIR
# "${MUJOCO_ROOT_DIR}/lib") set(MUJOCO_SAMPLE_DIR "${MUJOCO_ROOT_DIR}/sample")
# message("${MUJOCO_INCLUDE_DIR}") message(STATUS "MuJoCo root dir: "
# ${MUJOCO_ROOT_DIR})

# set(COMBINED_MC_MUJOCO_LIB OFF) # find glfw library find_library( GLFW NAMES
# libglfw.so.3 PATHS ${MUJOCO_BIN_DIR} NO_DEFAULT_PATH) if(NOT GLFW) if(UNIX)
# set(COMBINED_MC_MUJOCO_LIB ON) endif() set(BUILD_STATIC ON CACHE BOOL ""
# FORCE) add_subdirectory(ext/glfw EXCLUDE_FROM_ALL) else() message(STATUS "GLFW
# lib found at: " ${GLFW}) file(COPY "${MUJOCO_INCLUDE_DIR}/glfw3.h" DESTINATION
# "${PROJECT_BINARY_DIR}/src/include/GLFW/") endif()

# find mujoco library if(USE_GL) file(GLOB LIB_MUJOCO
# ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9].so
# ${MUJOCO_LIB_DIR}/libmujoco.so.*) else() file(GLOB LIB_MUJOCO
# ${MUJOCO_BIN_DIR}/libmujoco[0-9][0-9][0-9]nogl.so
# ${MUJOCO_LIB_DIR}/libmujoco_nogl.so.*) endif() message(STATUS "MuJoCo lib
# found at: " ${LIB_MUJOCO})
#
# set(MC_MUJOCO_SHARE_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco")
# if(WIN32) set(MC_MUJOCO_USER_DESTINATION "$ENV{APPDATA}/mc_rtc/mc_mujoco")
# else() set(MC_MUJOCO_USER_DESTINATION "$ENV{HOME}/.config/mc_rtc/mc_mujoco")
# endif()

add_subdirectory(ext/pugixml)
add_subdirectory(src)
add_subdirectory(robots)

add_subdirectory(examples)

# enable_testing()

# jrl_print_dependencies_summary()
jrl_print_options_summary()
