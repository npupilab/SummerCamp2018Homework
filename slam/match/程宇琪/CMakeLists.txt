cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)

include(PICMake)

include_directories(src)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(NOT USER_NAME)
    set(USER_NAME 程宇琪)
endif()

pi_add_target(Matcher BIN src ../${USER_NAME} REQUIRED GSLAM OpenCV GLEW GLUT OpenGL System DEPENDENCY -lX11)
pi_report_target()
pi_install(TARGETS Matcher)
