cmake_minimum_required(VERSION 3.14)
project(SampleNeckPolicy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Auto-download libtorch if not already available
find_package(Torch QUIET)
if(NOT Torch_FOUND)
  include(FetchContent)
  set(LIBTORCH_URL "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcpu.zip")
  message(STATUS "[SampleNeckPolicy] Downloading libtorch from ${LIBTORCH_URL}")
  FetchContent_Declare(
    libtorch
    URL ${LIBTORCH_URL}
  )
  FetchContent_MakeAvailable(libtorch)
  set(CMAKE_PREFIX_PATH "${libtorch_SOURCE_DIR}" ${CMAKE_PREFIX_PATH})
  find_package(Torch REQUIRED)
endif()

find_package(mc_rtc REQUIRED)

mc_rtc_set_prefix(CONTROLLER mc_controller)

add_subdirectory(src)

configure_file(etc/SampleNeckPolicy.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/SampleNeckPolicy.yaml")
install_controller_configuration("${CMAKE_CURRENT_BINARY_DIR}/etc/SampleNeckPolicy.yaml")
