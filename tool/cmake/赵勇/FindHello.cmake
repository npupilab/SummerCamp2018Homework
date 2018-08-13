FIND_PATH( HELLO_INCLUDE_DIR hello/hello.h
    ${PROJECT_SOURCE_DIR}/../include
    # system placed in /usr/local/include
    /usr/local/include
    # system placed in /usr/include
    /usr/include
    )


FIND_LIBRARY(HELLO_LIBRARIES  hello
    ${PROJECT_SOURCE_DIR}/../lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    /opt/opencv-2.4.9/lib
    )

if( HELLO_INCLUDE_DIR AND HELLO_LIBRARIES)
    MESSAGE( STATUS "Looking for Hello - found")
    MESSAGE( STATUS "Hello include path: ${HELLO_INCLUDE_DIR}" )
    MESSAGE( STATUS "Hello library path: ${HELLO_LIBRARIES}" )
    SET ( HELLO_FOUND 1 )
else(  HELLO_INCLUDE_DIR AND HELLO_LIBRARIES)
    message( STATUS "Looking for Hello  - not found" )
    SET ( HELLO_FOUND 0 )
endif(HELLO_INCLUDE_DIR AND HELLO_LIBRARIES)

IF(HELLO_FOUND)
    set(Hello_INCLUDES ${HELLO_INCLUDE_DIR})
    set(Hello_LIBRARIES ${HELLO_LIBRARIES})
ENDIF(HELLO_FOUND)
