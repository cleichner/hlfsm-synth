set( CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -g" )

file( GLOB SRCS *.cpp *.h )

add_executable( hlsyn ${SRCS} )
