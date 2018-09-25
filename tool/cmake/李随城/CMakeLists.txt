cmake_minimum_required(VERSION 2.8)

project(cmake_learning)

include_directories(${PROJECT_SOURCE_DIR}/../src/SharedLibDemo)
set(SRC_SHARED ${PROJECT_SOURCE_DIR}/../src/SharedLibDemo/SharedLibDemo.cpp)
add_library(SharedLibDemo  SHARED  ${SRC_SHARED})

include_directories(${PROJECT_SOURCE_DIR}/../src/StaticLibDemo)
set(SRC_STATIC ${PROJECT_SOURCE_DIR}/../src/StaticLibDemo/StaticLibDemo.cpp)
add_library(StaticLibDemo ${SRC_STATIC})

include_directories(${PROJECT_SOURCE_DIR}/../src/AppDemo)
set(SRC_MAIN ${PROJECT_SOURCE_DIR}/../src/AppDemo/main.cpp)
add_executable(AppDemo ${SRC_MAIN})
target_link_libraries(AppDemo SharedLibDemo StaticLibDemo)



SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
FIND_PACKAGE(Hello)
INCLUDE_DIRECTORIES(${HELLO_INCLUDE_DIR})


set(SRC_TEST ${PROJECT_SOURCE_DIR}/../src/AppDemo/test_for_findpackage.cpp)
add_executable(HelloDemo ${SRC_TEST})
target_link_libraries(HelloDemo ${HELLO_LIBRARIES})

INSTALL(TARGETS SharedLibDemo StaticLibDemo AppDemo	HelloDemo
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
)	