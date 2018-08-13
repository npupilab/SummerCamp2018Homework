find_path(Hello_INCLUDE_DIR Hello.h ${PROJECT_SOURCE_DIR} 
	 \usr\include
	 \usr\local\include)

 if(Hello_INCLUDE_DIR and  Hello_LIBRARIES )
	message(STATUS "Looking for Hello lib  - found")
	message(STATUS "Hello lib include path: ${Hello_INCLUDE_DIR}")
	set(Hello_FOUND 1)
else

	
	
