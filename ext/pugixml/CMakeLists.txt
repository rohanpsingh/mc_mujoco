add_library(pugixml OBJECT pugixml.cpp)
target_include_directories(pugixml PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/pugixml")
target_include_directories(pugixml PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
