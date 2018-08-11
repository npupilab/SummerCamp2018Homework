cmake_minimum_required(VERSION 2.8)
project(demo)

include(${CMAKE_CURRENT_LIST_DIR}/PICMake.cmake)# Use PICMake

pi_add_target(StaticLibDemo STATIC ../src/StaticLibDemo)
pi_add_target(SharedLibDemo SHARED ../src/SharedLibDemo)
pi_add_target(AppDemo BIN ../src/AppDemo DEPENDENCY StaticLibDemo SharedLibDemo)
pi_report_target()

# Install things
pi_install(TARGETS StaticLibDemo SharedLibDemo AppDemo)
