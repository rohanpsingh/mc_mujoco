set(mc_mujoco_SRC
  main.cpp
  mj_sim.cpp
  mj_utils.cpp
)

set(assets_DIR "${PROJECT_SOURCE_DIR}/assets")

set(imgui_DIR "${PROJECT_SOURCE_DIR}/ext/imgui")
set(imgui_SRC
  ${imgui_DIR}/imgui.h
  ${imgui_DIR}/imgui.cpp
  ${imgui_DIR}/imgui_demo.cpp
  ${imgui_DIR}/imgui_draw.cpp
  ${imgui_DIR}/imgui_tables.cpp
  ${imgui_DIR}/imgui_widgets.cpp
  ${imgui_DIR}/backends/imgui_impl_glfw.h
  ${imgui_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_DIR}/backends/imgui_impl_opengl3.h
  ${imgui_DIR}/backends/imgui_impl_opengl3.cpp
)

set(implot_DIR "${PROJECT_SOURCE_DIR}/ext/implot")
set(implot_SRC
  ${implot_DIR}/implot.h
  ${implot_DIR}/implot.cpp
  ${implot_DIR}/implot_items.cpp
  ${implot_DIR}/implot_demo.cpp
)

set(mc_rtc-imgui_DIR "${PROJECT_SOURCE_DIR}/ext/mc_rtc-imgui")
add_subdirectory("${mc_rtc-imgui_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc-imgui/")

configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

file(COPY "${MUJOCO_INCLUDE_DIR}/glfw3.h" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/include/GLFW/")

add_executable(mc_mujoco ${mc_mujoco_SRC} ${imgui_SRC} ${implot_SRC} ${mc_rtc-imgui-SRC} ${mc_rtc-imgui-HDR})
target_include_directories(mc_mujoco PUBLIC ${MUJOCO_INCLUDE_DIR})
target_include_directories(mc_mujoco PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_include_directories(mc_mujoco PRIVATE "${imgui_DIR}" "${implot_DIR}" "${mc_rtc-imgui_DIR}" "${assets_DIR}")
target_link_libraries(mc_mujoco PUBLIC ${LIB_MUJOCO} ${GLFW} GLEW::GLEW mc_rtc::mc_control mc_rtc::mc_control_client ${CMAKE_DL_LIBS})

install(TARGETS mc_mujoco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

