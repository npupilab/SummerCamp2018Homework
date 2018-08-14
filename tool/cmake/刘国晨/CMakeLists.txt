project(cmake_learning)
cmake_minimum_required(VERSION 3.5)

set(EXE_SRC  AppDemo/main.cpp)
set(STATIC_SRC StaticLibDemo/StaticLibDemo.cpp)
set(SHARED_SRC SharedLibDemo/SharedLibDemo.cpp)

include_directories( ./StaticLibDemo ./SharedLibDemo)

add_library(shared_lib SHARED ${SHARED_SRC})
add_library(static_lib ${STATIC_SRC})

add_executable(main ${EXE_SRC})

target_link_libraries(main shared_lib static_lib)

install(
TARGETS main shared_lib static_lib
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
)



