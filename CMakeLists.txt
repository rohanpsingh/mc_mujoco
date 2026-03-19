cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME mc_mujoco)
set(CMAKE_CXX_STANDARD 17)

project(
  mc_mujoco
  LANGUAGES C CXX
  VERSION 1.0.0
  DESCRIPTION "Mujoco interface for mc-rtc"
  HOMEPAGE_URL "https://github.com/rohanpsingh/mc_mujoco")

include(cmake/get-jrl-cmakemodules.cmake)

jrl_configure_defaults()

# This is kept as an option to allow testing building against MuJoCo's official
# simulate/ folder, should there be updates there However, for consistency we
# intend to use the bundled ext/mujoco_simulate/simulate folder, that is -
# currently - a copy of MuJoCo's official relase. We need to bundle our own as
# MuJoCo's cmake install does not export it, and relying on building it from
# MuJoCo's source tree is brittle (e.g would require tricks in Nix) When
# updating MuJoCo, we should check that this bundled version remains up-to-date.
jrl_option(USE_BUNDLED_SIMULATE_ADAPTER
           "Whether to use the GLFW adapter bundled with mc_mujoco" ON)

# jrl_option(USE_GL "Use Mujoco with OpenGL" ON)

jrl_find_package(mc_rtc REQUIRED)
jrl_find_package(mujoco 3.3.6 REQUIRED)
# TODO: replace with cli11
jrl_find_package(Boost REQUIRED COMPONENTS program_options)

get_target_property(MUJOCO_INCLUDE_DIR mujoco::mujoco
                    INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "MuJoCo include directory: ${MUJOCO_INCLUDE_DIR}")
# remove the last component (`include/`) from the path to get MuJoCo's root path
# NOTE: this may be a bit brittle should MuJoCo decide to change it's include
# paths.
get_filename_component(MUJOCO_ROOT_DIR "${MUJOCO_INCLUDE_DIR}" DIRECTORY)
set(MUJOCO_ROOT_DIR "${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo detected in MUJOCO_ROOT_DIR: ${MUJOCO_ROOT_DIR}")
set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT_DIR}/simulate")
set(MUJOCO_BIN_DIR "${MUJOCO_ROOT_DIR}/bin")
set(MC_MUJOCO_SHARE_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_mujoco")
# FIXME this won't work under nix as the home path is not available at build
# time in the sandbox
if(WIN32)
  set(MC_MUJOCO_USER_DESTINATION "$ENV{APPDATA}/mc_rtc/mc_mujoco")
else()
  set(MC_MUJOCO_USER_DESTINATION "$ENV{HOME}/.config/mc_rtc/mc_mujoco")
endif()

# TODO check if paths here make sense/are really needed
configure_file(include/mc_mujoco/config.in.h
               "${CMAKE_CURRENT_BINARY_DIR}/include/mc_mujoco/config.h")

jrl_find_package(glfw3 REQUIRED)
set(COMBINED_MC_MUJOCO_LIB OFF)

set(OpenGL_GL_PREFERENCE "GLVND")
jrl_find_package(OpenGL REQUIRED)
jrl_find_package(GLEW REQUIRED)

add_subdirectory(ext/pugixml)
add_subdirectory(src)
add_subdirectory(robots)

add_subdirectory(examples)

# Exporting targets and installing
jrl_export_package()

# jrl_print_dependencies_summary()
jrl_print_options_summary()
