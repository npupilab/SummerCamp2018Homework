CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(homework)

SET(SOURCES_FILES  src/AppDemo/main.cpp)

ADD_LIBRARY(SharedLibDemo SHARED src/SharedLibDemo/SharedLibDemo.cpp)
ADD_LIBRARY(StaticLibDemo STATIC src/StaticLibDemo/StaticLibDemo.cpp)

ADD_EXECUTABLE(AppDemo ${SOURCES_FILES})
TARGET_LINK_LIBRARIES(AppDemo SharedLibDemo StaticLibDemo)
