cmake_minimum_required(VERSION 3.5)
project(demo)
include_directories(../src/SharedLibDemo ../src/StaticLibDemo)
ADD_LIBRARY(libSharedLibDemo SHARED ../src/SharedLibDemo/SharedLibDemo.cpp)	#添加动态库
ADD_LIBRARY(libStaticLibDemo STATIC ../src/StaticLibDemo/StaticLibDemo.cpp) 	#添加静态库
ADD_EXECUTABLE(hello_demo ../src/AppDemo/main.cpp)
target_LINK_LIBRARIES(hello_demo libStaticLibDemo libSharedLibDemo)
install(TARGETS hello_demo libStaticLibDemo libSharedLibDemo
       RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
#export LD_LIBRARY_PATH=($LD_LIBRARY_PATH:${CMAKE_INSTALL_PREFIX}/lib)
#export LD_LIBRARY_PATH=(/home/chengyuqi/learn/summercamp/SummerCamp2018Homework/tool/cmake/程宇琪/src/lib)
