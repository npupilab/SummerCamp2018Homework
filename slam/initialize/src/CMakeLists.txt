cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)

include(PICMake)

include_directories(.)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(NOT USER_NAME)
    set(USER_NAME 赵勇)
endif()

pi_add_target(Initializer BIN main.cpp ../${USER_NAME} REQUIRED GSLAM Eigen3 System)
pi_report_target()
pi_install(TARGETS Initializer)
