include(CMakePackageConfigHelpers)
set(VERSION_CONFIG
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
set(CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
write_basic_package_version_file(
  ${VERSION_CONFIG}
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in" "${PROJECT_CONFIG}"
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}")
install(FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
        DESTINATION "${CONFIG_INSTALL_DIR}")
