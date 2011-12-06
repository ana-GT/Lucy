cmake_minimum_required ( VERSION 2.6 )
project (test)

include_directories( . ../../../ )

set( SRC_FILES test.cpp )

   add_executable ( test ${SRC_FILES} )

