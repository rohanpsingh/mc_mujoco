# Try to find system CLI11 first
find_package(CLI11 CONFIG QUIET)

if(NOT CLI11_FOUND)
  message(STATUS "CLI11 not found, let's fetch it")
  include(FetchContent)
  FetchContent_Declare(
    cli11_proj
    QUIET
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.5.0)
  FetchContent_MakeAvailable(cli11_proj)
  # Alias target for compatibility if needed
  if(NOT TARGET CLI11::CLI11 AND TARGET cli11)
    add_library(CLI11::CLI11 ALIAS cli11)
  endif()
endif()
