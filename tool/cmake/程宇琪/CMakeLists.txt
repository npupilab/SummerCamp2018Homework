cmake_minimum_required(VERSION 3.5)
project(demo)
include_directories(../src/SharedLibDemo ../src/StaticLibDemo)
ADD_LIBRARY(SharedLibDemo SHARED ../src/SharedLibDemo/SharedLibDemo.cpp)	#添加动态库
ADD_LIBRARY(StaticLibDemo STATIC ../src/StaticLibDemo/StaticLibDemo.cpp) 	#添加静态库
ADD_EXECUTABLE(AppDemo ../src/AppDemo/main.cpp)
target_LINK_LIBRARIES(AppDemo StaticLibDemo SharedLibDemo)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
FIND_PACKAGE(Hello)

INCLUDE_DIRECTORIES(${HELLO_INCLUDE})

ADD_EXECUTABLE(HelloDemo ../src/AppDemo/test_for_findpackage.cpp)
target_LINK_LIBRARIES(HelloDemo ${Hello_LIBRARIES})

install(TARGETS AppDemo StaticLibDemo SharedLibDemo HelloDemo
       RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)


#export LD_LIBRARY_PATH=($LD_LIBRARY_PATH:${CMAKE_INSTALL_PREFIX}/lib)
#export LD_LIBRARY_PATH=(/home/chengyuqi/learn/summercamp/SummerCamp2018Homework/tool/cmake/程宇琪/src/lib)
