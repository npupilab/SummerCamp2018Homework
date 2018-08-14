cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ../../cmake)

include(PICMake)

include_directories(src)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(NOT USER_NAME)
    set(USER_NAME demo)
endif()

pi_add_target(Matcher BIN src ../${USER_NAME} REQUIRED GSLAM OpenCV OpenGL GLEW GLUT System)
pi_report_target()
pi_install(TARGETS Matcher)
