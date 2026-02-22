option(BUILD_EXAMPLES "Build example controllers" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(neck_policy)
endif()
