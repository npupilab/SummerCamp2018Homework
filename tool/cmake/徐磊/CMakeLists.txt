project(cmake_learning)
cmake_minimum_required(VERSION 3.5)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
set(EXE_SRC  AppDemo/main.cpp)
set(STATIC_SRC StaticLibDemo/StaticLibDemo.cpp)
set(SHARED_SRC SharedLibDemo/SharedLibDemo.cpp)
set(FIND_SRC AppDemo/test_for_findpackage.cpp)

find_package(Hello REQUIRED)

include_directories( ./StaticLibDemo ./SharedLibDemo)
include_directories(${Hello_INCLUDE_DIR})

add_library(shared_lib SHARED ${SHARED_SRC})
add_library(static_lib ${STATIC_SRC})

add_executable(main ${EXE_SRC})
target_link_libraries(main shared_lib static_lib)



add_executable(HelloDemo ${FIND_SRC})
target_link_libraries(HelloDemo ${Hello_LIBRARIES})




install(
TARGETS main shared_lib static_lib
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
)



