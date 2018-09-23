project(cmake_learning)
cmake_minimum_required(VERSION 3.5)

set(EXE_SRC      ../src/AppDemo/main.cpp)
set(EXE_INCLUDES ../src/StaticLibDemo ../src/SharedLibDemo)
set(STATIC_SRC   ../src/StaticLibDemo/StaticLibDemo.cpp)
set(SHARED_SRC   ../src/SharedLibDemo/SharedLibDemo.cpp)

#Cause in main.cpp ,it use include "../XX" to find head file ,so no need to INCLUDE_DIRCTORIES
add_library(SharedLibDemo SHARED ${SHARED_SRC})
add_library(StaticLibDemo STATIC ${STATIC_SRC})

add_executable(AppDemo ${EXE_SRC})
target_link_libraries(AppDemo SharedLibDemo StaticLibDemo)
#Hello part
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Hello REQUIRED)

set(FIND_SRC ../src/AppDemo/test_for_findpackage.cpp)

add_executable(HelloDemo ${FIND_SRC})
include_directories(${Hello_INCLUDES})
target_link_libraries(HelloDemo ${Hello_LIBRARIES})

install(
    TARGETS AppDemo SharedLibDemo StaticLibDemo HelloDemo
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)