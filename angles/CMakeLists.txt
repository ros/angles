install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

# for compatibility with old build system
install(FILES manifest.xml
        DESTINATION share/${PROJECT_NAME})

include_directories(include)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  add_subdirectory(test)
endif()
