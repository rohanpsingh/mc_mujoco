set(controller_SRC
  controller.cpp
  controller.h
  api.h
  lib.cpp
)

add_controller(SampleNeckPolicy "${controller_SRC}" "")
target_link_libraries(SampleNeckPolicy PUBLIC mc_rtc::mc_control "${TORCH_LIBRARIES}")
target_compile_definitions(SampleNeckPolicy PRIVATE SampleNeckPolicy_EXPORTS)

# Ensure libtorch headers are available
target_include_directories(SampleNeckPolicy SYSTEM PUBLIC "${TORCH_INCLUDE_DIRS}")

# Set RPATH so libtorch .so files are found at runtime
set_target_properties(SampleNeckPolicy PROPERTIES
  BUILD_RPATH "${TORCH_INSTALL_PREFIX}/lib"
  INSTALL_RPATH "${TORCH_INSTALL_PREFIX}/lib"
)
