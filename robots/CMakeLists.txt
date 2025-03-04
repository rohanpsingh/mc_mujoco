if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/jvrc_mj_description/CMakeLists.txt")
  message(
    FATAL_ERROR
      "No CMakeLists.txt in jvrc_mj_description folder, did you run `git submodule update --init`?"
  )
endif()

file(GLOB robot_dirs CONFIGURE_DEPENDS "*")
foreach(robot_dir ${robot_dirs})
  if(EXISTS "${robot_dir}/CMakeLists.txt")
    message(STATUS "Adding robot: ${robot_dir}")
    add_subdirectory(${robot_dir})
  endif()
endforeach()

function(setup_env_object NAME ENV_OBJECT)
  set(YAML_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}.yaml")
  configure_file(${NAME}.in.yaml "${YAML_OUT}")
  install(FILES "${YAML_OUT}" DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}")
  install(FILES "${NAME}.xml"
          DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}/${ENV_OBJECT}")
  install(DIRECTORY assets
          DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}/${ENV_OBJECT}")
endfunction()

setup_env_object(ground env)
setup_env_object(box object)
setup_env_object(longtable object)
setup_env_object(table object)
