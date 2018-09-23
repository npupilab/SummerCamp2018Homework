find_path(Hello_INCLUDE_DIR hello/hello.h ${PROJECT_SOURCE_DIR}/../include)
find_library(Hello_LIBRARIES hello  ${PROJECT_SOURCE_DIR}/../lib)
if(Hello_INCLUDE_DIR AND  Hello_LIBRARIES)
    message(STATUS "Looking for Hello lib  - found ${Hello_LIBRARIES}")
	message(STATUS "Hello lib include path: ${Hello_INCLUDE_DIR}")
	set(Hello_FOUND 1)
else(Hello_INCLUDE_DIR AND  Hello_LIBRARIES)
    message(STATUS "Can not Find Hello lib")
    set(Hello_FOUND 0)
endif(Hello_INCLUDE_DIR AND  Hello_LIBRARIES)

message("test")



	
	
