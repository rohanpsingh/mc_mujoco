set(mc_mujoco_lib_SRC
  mj_configuration.h
  mj_sim.cpp
  mj_utils.cpp
  mj_utils_merge_mujoco_models.cpp
  mj_sim.h
  mj_sim_impl.h
  mj_utils.h
  MujocoClient.cpp
  MujocoClient.h
  widgets/Arrow.h
  widgets/Force.h
  widgets/MujocoWidget.h
  widgets/Point3D.h
  widgets/Polygon.h
  widgets/Rotation.h
  widgets/Trajectory.h
  widgets/Transform.h
  widgets/Visual.h
  widgets/Visual.cpp
  widgets/XYTheta.h
  widgets/details/ControlAxis.h
  widgets/details/InteractiveMarker.h
  widgets/details/InteractiveMarker.cpp
  widgets/details/TransformBase.h
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

set(ImGuizmo_DIR ${PROJECT_SOURCE_DIR}/ext/ImGuizmo)
set(ImGuizmo_SRC ${ImGuizmo_DIR}/ImGuizmo.h ${ImGuizmo_DIR}/ImGuizmo.cpp)

set(mc_rtc-imgui_DIR "${PROJECT_SOURCE_DIR}/ext/mc_rtc-imgui")
add_subdirectory("${mc_rtc-imgui_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc-imgui/")

configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

file(COPY "${MUJOCO_INCLUDE_DIR}/glfw3.h" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/include/GLFW/")

add_library(mc_mujoco_lib STATIC ${mc_mujoco_lib_SRC} ${imgui_SRC} ${implot_SRC} ${ImGuizmo_SRC} ${mc_rtc-imgui-SRC} ${mc_rtc-imgui-HDR})
target_include_directories(mc_mujoco_lib PUBLIC $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>$<INSTALL_INTERFACE:include>)
target_include_directories(mc_mujoco_lib PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_include_directories(mc_mujoco_lib PRIVATE "${imgui_DIR}" "${implot_DIR}" "${mc_rtc-imgui_DIR}" "${assets_DIR}" "${ImGuizmo_DIR}")
target_link_libraries(mc_mujoco_lib PUBLIC ${LIB_MUJOCO} ${GLFW} GLEW::GLEW OpenGL::GL mc_rtc::mc_control mc_rtc::mc_control_client ${CMAKE_DL_LIBS})
target_link_libraries(mc_mujoco_lib PRIVATE pugixml)

find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(mc_mujoco main.cpp)
target_link_libraries(mc_mujoco PRIVATE mc_mujoco_lib Boost::program_options Boost::disable_autolinking)

install(TARGETS mc_mujoco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

