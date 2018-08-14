cmake_minimum_required(VERSION 3.5)
project(demo)
include_directories(../src/SharedLibDemo ../src/StaticLibDemo)
ADD_LIBRARY(libSharedLibDemo SHARED ../src/SharedLibDemo/SharedLibDemo.cpp)	#添加动态库
ADD_LIBRARY(libStaticLibDemo STATIC ../src/StaticLibDemo/StaticLibDemo.cpp) 	#添加静态库
ADD_EXECUTABLE(hello_demo ../src/AppDemo/main.cpp)
target_LINK_LIBRARIES(hello_demo libStaticLibDemo libSharedLibDemo)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
FIND_PACKAGE(Hello)

INCLUDE_DIRECTORIES(${HELLO_INCLUDE})

ADD_EXECUTABLE(findhello ../src/AppDemo/test_for_findpackage.cpp)
target_LINK_LIBRARIES(findhello ${Hello_LIBRARIES})

install(TARGETS hello_demo libStaticLibDemo libSharedLibDemo findhello
       RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)


#export LD_LIBRARY_PATH=($LD_LIBRARY_PATH:${CMAKE_INSTALL_PREFIX}/lib)
#export LD_LIBRARY_PATH=(/home/chengyuqi/learn/summercamp/SummerCamp2018Homework/tool/cmake/程宇琪/src/lib)
