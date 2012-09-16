project(angles)
catkin_project(${PROJECT_NAME} INCLUDE_DIRS include)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

include_directories(include)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  add_subdirectory(test)
endif()
