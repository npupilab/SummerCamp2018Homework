FIND_PATH(HELLO_INCLUDE_DIR hello/hello.h
        ${PROJECT_SOURCE_DIR}/../include
        /usr/local/include/
        /usr/include/
)

FIND_LIBRARY(HELLO_LIBS hello
        ${PROJECT_SOURCE_DIR}/../lib
        /usr/local/lib
        /usr/lib
        /opt/lib
)

if(HELLO_INCLUDE_DIR AND HELLO_LIBS)
    MESSAGE( STATUS "Looking for Hello - found")
    SET(HELLO_FOUND 1)
else(HELLO_INCLUDE_DIR AND HELLO_LIB)
    message( STATUS "Looking for Hello  - not found" )
    SET(HELLO_FOUND 0)
endif(HELLO_INCLUDE_DIR AND HELLO_LIB)

if (HELLO_FOUND)
    SET(Hello_INCLUDES ${HELLO_INCLUDE_DIR})
    SET(Hello_LIBS ${HELLO_LIBS})
endif (HELLO_FOUND)
