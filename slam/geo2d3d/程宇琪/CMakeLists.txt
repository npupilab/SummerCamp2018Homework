cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)

include(PICMake)

if(NOT USER_NAME)
  set(USER_NAME 程宇琪)
endif()

INCLUDE_DIRECTORIES(.)

set(CMAKE_CXX_FLAGS -std=c++11)

pi_add_target(geo2d3d BIN main.cpp ../${USER_NAME}/Geometry.cpp REQUIRED GSLAM)
pi_report_targets()
