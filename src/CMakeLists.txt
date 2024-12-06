set(uitools_is_platform_ui_adapter OFF)
if(EXISTS ${MUJOCO_INCLUDE_DIR}/uitools.c)
  set(uitools_SRC ${MUJOCO_INCLUDE_DIR}/uitools.c
                  ${MUJOCO_INCLUDE_DIR}/uitools.h)
elseif(EXISTS ${MUJOCO_ROOT_DIR}/simulate/uitools.cc)
  set(uitools_SRC
      ${MUJOCO_ROOT_DIR}/simulate/glfw_dispatch.cc
      ${MUJOCO_ROOT_DIR}/simulate/glfw_dispatch.h
      ${MUJOCO_ROOT_DIR}/simulate/uitools.cc
      ${MUJOCO_ROOT_DIR}/simulate/uitools.h)
elseif(EXISTS ${MUJOCO_SAMPLE_DIR}/uitools.c)
  set(uitools_SRC ${MUJOCO_SAMPLE_DIR}/uitools.c ${MUJOCO_SAMPLE_DIR}/uitools.h)
else()
  set(uitools_is_platform_ui_adapter ON)
  set(OUR_GLFW_ADAPTER_HDR
      ${CMAKE_CURRENT_BINARY_DIR}/include/our_glfw_adapter.h)
  set(uitools_SRC
      ${MUJOCO_ROOT_DIR}/simulate/glfw_dispatch.cc
      ${MUJOCO_ROOT_DIR}/simulate/glfw_dispatch.h
      ${MUJOCO_ROOT_DIR}/simulate/glfw_adapter.cc
      ${MUJOCO_ROOT_DIR}/simulate/platform_ui_adapter.h
      ${MUJOCO_ROOT_DIR}/simulate/platform_ui_adapter.cc
      ${OUR_GLFW_ADAPTER_HDR})
  # Make private members public so we can access the window_ field
  file(READ "${MUJOCO_ROOT_DIR}/simulate/glfw_adapter.h" GLFW_ADAPTER_SRC)
  string(REPLACE "private:" "public:" GLFW_ADAPTER_SRC "${GLFW_ADAPTER_SRC}")
  file(
    GENERATE
    OUTPUT "${OUR_GLFW_ADAPTER_HDR}"
    CONTENT "${GLFW_ADAPTER_SRC}")
endif()

set(mc_mujoco_lib_SRC
    mj_configuration.h
    mj_sim.cpp
    mj_utils.cpp
    mj_utils_merge_mujoco_models.cpp
    mj_sim.h
    mj_sim_impl.h
    mj_utils.h
    ${uitools_SRC}
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
    widgets/details/TransformBase.h)

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
    ${imgui_DIR}/backends/imgui_impl_opengl3.cpp)

set(implot_DIR "${PROJECT_SOURCE_DIR}/ext/implot")
set(implot_SRC ${implot_DIR}/implot.h ${implot_DIR}/implot.cpp
               ${implot_DIR}/implot_items.cpp ${implot_DIR}/implot_demo.cpp)

set(ImGuizmo_DIR ${PROJECT_SOURCE_DIR}/ext/ImGuizmo)
set(ImGuizmo_SRC ${ImGuizmo_DIR}/ImGuizmo.h ${ImGuizmo_DIR}/ImGuizmo.cpp)

set(mc_rtc-imgui_DIR "${PROJECT_SOURCE_DIR}/ext/mc_rtc-imgui")
add_subdirectory("${mc_rtc-imgui_DIR}"
                 "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc-imgui/")

configure_file(config.in.h "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_library(
  mc_mujoco_lib STATIC
  ${mc_mujoco_lib_SRC}
  ${imgui_SRC}
  ${implot_SRC}
  ${ImGuizmo_SRC}
  ${mc_rtc-imgui-SRC}
  ${mc_rtc-imgui-HDR}
  $<TARGET_OBJECTS:pugixml>)
if(uitools_is_platform_ui_adapter)
  target_compile_definitions(mc_mujoco_lib PRIVATE USE_UI_ADAPTER)
endif()
target_include_directories(
  mc_mujoco_lib
  PUBLIC ${MUJOCO_INCLUDE_DIR} $<BUILD_INTERFACE:${MUJOCO_SAMPLE_DIR}>
         $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}> $<INSTALL_INTERFACE:include>)
if(DEFINED MUJOCO_ROOT_INCLUDE_DIR)
  target_include_directories(mc_mujoco_lib PUBLIC ${MUJOCO_ROOT_INCLUDE_DIR})
endif()
target_include_directories(mc_mujoco_lib
                           PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_include_directories(
  mc_mujoco_lib
  PRIVATE "${imgui_DIR}" "${implot_DIR}" "${mc_rtc-imgui_DIR}" "${assets_DIR}"
          "${ImGuizmo_DIR}" "${PROJECT_SOURCE_DIR}/ext/pugixml")
target_link_libraries(
  mc_mujoco_lib PUBLIC mc_rtc::mc_control mc_rtc::mc_control_client
                       ${CMAKE_DL_LIBS})
if(UNIX)
  set(NO_AS_NEEDED "-Wl,--no-as-needed")
  set(AS_NEEDED "-Wl,--as-needed")
endif()
target_link_libraries(
  mc_mujoco_lib PRIVATE ${NO_AS_NEEDED} GLEW::GLEW OpenGL::GL ${AS_NEEDED}
                        ${LIB_MUJOCO})
if(GLFW)
  target_link_libraries(mc_mujoco_lib PRIVATE ${GLFW})
else()
  target_include_directories(
    mc_mujoco_lib PRIVATE "${PROJECT_SOURCE_DIR}/ext/glfw/include/GLFW")
  target_link_libraries(mc_mujoco_lib PRIVATE $<BUILD_INTERFACE:glfw>)
  if(UNIX)
    set(mc_mujoco_lib_combined_OUT
        ${CMAKE_CURRENT_BINARY_DIR}/combined/libmc_mujoco_lib_combined.a)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/combined)
    add_custom_command(
      OUTPUT ${mc_mujoco_lib_combined_OUT}
      COMMAND ar -x $<TARGET_FILE:mc_mujoco_lib>
      COMMAND ar -x $<TARGET_FILE:glfw>
      COMMAND ar -qcs ${mc_mujoco_lib_combined_OUT} *.o
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/combined
      DEPENDS mc_mujoco_lib glfw)
    add_custom_target(mc_mujoco_lib_combined ALL
                      DEPENDS ${mc_mujoco_lib_combined_OUT})
    install(FILES ${mc_mujoco_lib_combined_OUT} DESTINATION lib)
  endif()
endif()
if(NOT COMBINED_MC_MUJOCO_LIB)
  set_target_properties(mc_mujoco_lib PROPERTIES EXPORT_NAME mc_mujoco)
endif()

find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(mc_mujoco main.cpp)
target_compile_definitions(mc_mujoco PUBLIC MUJOCO_BIN_DIR="${MUJOCO_BIN_DIR}")
target_link_libraries(mc_mujoco PRIVATE mc_mujoco_lib Boost::program_options
                                        Boost::disable_autolinking)

install(
  TARGETS mc_mujoco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES mj_sim.h mj_configuration.h DESTINATION include/mc_mujoco)

install(
  TARGETS mc_mujoco_lib
  EXPORT mc_mujocoTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  EXPORT mc_mujocoTargets
  FILE mc_mujocoTargets.cmake
  NAMESPACE mc_mujoco::
  DESTINATION lib/cmake/mc_mujoco)
