cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../cmake)
include(PICMake)

INCLUDE_DIRECTORIES(../src)

set(CMAKE_CXX_FLAGS -std=c++11)

pi_add_target(geo2d3d BIN ../src/main.cpp Geometry.cpp REQUIRED GSLAM)
pi_report_targets()
