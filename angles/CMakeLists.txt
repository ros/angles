cmake_minimum_required(VERSION 3.0.2)
project(angles)

find_package(catkin REQUIRED)
catkin_package(INCLUDE_DIRS include)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

include_directories(include)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
catkin_python_setup()