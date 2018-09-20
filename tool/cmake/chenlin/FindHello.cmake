find_path(Hello_INCLUDES hello/hello.h 
	../include
	)
find_library(Hello_LIBRARIES hello 
	../lib
	)

if(Hello_INCLUDES AND  Hello_LIBRARIES)
    message(STATUS "Looking for Hello lib-found")
    message(STATUS "Hello lib path:${Hello_LIBRARIES}")
	message(STATUS "Hello include path: ${Hello_INCLUDES}")
	set(Hello_FOUND 1)
else(Hello_INCLUDE_DIR AND  Hello_LIBRARIES)
    message(FATAL_ERROR "Can not Find Hello lib")
    set(Hello_FOUND 0)
endif(Hello_INCLUDES AND  Hello_LIBRARIES)