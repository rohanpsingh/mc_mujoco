option(BUILD_EXAMPLES "Build example controllers" OFF)
if(NOT BUILD_EXAMPLES)
  return()
endif()

# Auto-discover examples: any subdirectory containing a CMakeLists.txt is built.
# Un-init'd submodule directories are silently skipped (no CMakeLists.txt
# present), so a fresh clone without `git submodule update --init` still
# configures cleanly.
file(
  GLOB example_dirs
  LIST_DIRECTORIES true
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(d ${example_dirs})
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${d}"
     AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${d}/CMakeLists.txt")
    message(STATUS "Adding example ${d}")
    add_subdirectory(${d})
  endif()
endforeach()
