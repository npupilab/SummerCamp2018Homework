cmake_minimum_required(VERSION 2.8)
project(cmake)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Hello)

include_directories(${Hello_INCLUDES})
include_directories(./include)

add_library(${PROJECT_NAME} SHARED
        src/SharedLibDemo/SharedLibDemo.cpp
)

add_library(StaticLibDemo STATIC
        src/StaticLibDemo/StaticLibDemo.cpp
)

target_link_libraries(${PROJECT_NAME}
        StaticLibDemo
        ${Hello_LIBS}
)

add_executable(example src/AppDemo/main.cpp)
target_link_libraries(example ${PROJECT_NAME})

add_executable(example_find_package src/AppDemo/test_for_findpackage.cpp)
target_link_libraries(example_find_package ${PROJECT_NAME})

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
