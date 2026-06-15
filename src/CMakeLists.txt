set(OUR_GLFW_ADAPTER_HDR
    ${CMAKE_BINARY_DIR}/include/mc_mujoco/our_glfw_adapter.h)
if(USE_BUNDLED_SIMULATE_ADAPTER)
  message(STATUS "Using bundled GLFW adapter from mc_mujoco")
  # MuJoCo simulate folder imported from MuJoCo We need to do this as it is not
  # installed by MuJoCo but provides a sensible default adapter for displaying
  # the scene with GLFW, but MuJoCo's cmake install does not install it
  set(MUJOCO_SIMULATE_DIR "${PROJECT_SOURCE_DIR}/ext/mujoco_simulate/simulate")
else()
  set(MUJOCO_SIMULATE_DIR "${MUJOCO_ROOT_DIR}/simulate")
endif()
set(platform_adapter_HDR
    ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.h
    ${MUJOCO_SIMULATE_DIR}/glfw_adapter.h
    ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.h ${OUR_GLFW_ADAPTER_HDR})
set(platform_adapter_SRC
    ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc
    ${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc
    ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc)
# NOTE: Make private members public so we can access the window_ field
file(READ "${MUJOCO_SIMULATE_DIR}/glfw_adapter.h" GLFW_ADAPTER_SRC)
string(REPLACE "private:" "public:" GLFW_ADAPTER_SRC "${GLFW_ADAPTER_SRC}")
file(
  GENERATE
  OUTPUT "${OUR_GLFW_ADAPTER_HDR}"
  CONTENT "${GLFW_ADAPTER_SRC}")

set(mc_mujoco_lib_SRC
    mj_sim.cpp
    mj_utils.cpp
    mj_utils_merge_mujoco_models.cpp
    ${platform_adapter_SRC}
    MujocoClient.cpp
    ../include/mc_mujoco/widgets/Visual.cpp
    ../include/mc_mujoco/widgets/details/InteractiveMarker.cpp)

set(mc_mujoco_lib_HDR
    ${platform_adapter_HDR}
    include/mc_mujoco/MujocoClient.h
    include/mc_mujoco/mj_configuration.h
    include/mc_mujoco/mj_sim_impl.h
    include/mc_mujoco/mj_sim.h
    include/mc_mujoco/mj_utils.h
    include/mc_mujoco/widgets/Arrow.h
    include/mc_mujoco/widgets/Force.h
    include/mc_mujoco/widgets/MujocoWidget.h
    include/mc_mujoco/widgets/Point3D.h
    include/mc_mujoco/widgets/Polygon.h
    include/mc_mujoco/widgets/Polyhedron.h
    include/mc_mujoco/widgets/Rotation.h
    include/mc_mujoco/widgets/Trajectory.h
    include/mc_mujoco/widgets/Transform.h
    include/mc_mujoco/widgets/Visual.h
    include/mc_mujoco/widgets/XYTheta.h
    include/mc_mujoco/widgets/details/ControlAxis.h
    include/mc_mujoco/widgets/details/InteractiveMarker.h
    include/mc_mujoco/widgets/details/TransformBase.h)

set(assets_DIR "${PROJECT_SOURCE_DIR}/assets")

add_library(mc_mujoco_lib SHARED ${mc_mujoco_lib_SRC})
jrl_target_headers(mc_mujoco_lib PUBLIC HEADERS ${mc_mujoco_lib_HDR} BASE_DIRS
                   include)
target_link_libraries(
  mc_mujoco_lib PUBLIC mujoco::mujoco mc_rtc-imgui::mc_rtc-imgui
                       ImGuizmo::ImGuizmo pugixml::pugixml)
target_include_directories(
  mc_mujoco_lib
  PUBLIC $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_include_directories(mc_mujoco_lib
                           PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_include_directories(
  mc_mujoco_lib PRIVATE "${assets_DIR}" "${PROJECT_SOURCE_DIR}/ext/pugixml")
target_link_libraries(
  mc_mujoco_lib PUBLIC mc_rtc::mc_control mc_rtc::mc_control_client
                       ${CMAKE_DL_LIBS})
target_link_libraries(mc_mujoco_lib PUBLIC GLEW::GLEW OpenGL::GL)
target_link_libraries(mc_mujoco_lib PUBLIC glfw)
set_target_properties(mc_mujoco_lib PROPERTIES EXPORT_NAME mc_mujoco)

add_executable(mc_mujoco main.cpp)
target_compile_definitions(mc_mujoco PUBLIC MUJOCO_BIN_DIR="${MUJOCO_BIN_DIR}")
target_link_libraries(mc_mujoco PUBLIC mc_mujoco_lib)
target_link_libraries(mc_mujoco PRIVATE CLI11::CLI11)

install(
  TARGETS mc_mujoco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

jrl_add_export_component(NAME mc_mujoco_lib TARGETS mc_mujoco_lib)
