set(mc_mujoco_SRC
  main.cpp
  mj_sim.cpp
  mj_utils.cpp
)

configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_executable(mc_mujoco ${mc_mujoco_SRC})
target_include_directories(mc_mujoco PUBLIC ${MUJOCO_INCLUDE_DIR})
target_include_directories(mc_mujoco PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(mc_mujoco PUBLIC ${LIB_MUJOCO} ${GLFW} GLEW::GLEW mc_rtc::mc_control)

install(TARGETS mc_mujoco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

