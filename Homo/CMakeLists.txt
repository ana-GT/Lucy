# Homotopic Paths 2D

cmake_minimum_required( VERSION 2.8 )
project ( HP2D )

message ( STATUS "Creating Makefile for Homotopic Paths 2D" )

file ( GLOB srcs "*.cpp" )
file ( GLOB hdrs "*.h" )


add_executable ( test ${srcs} ${hdrs} )
# target_link_libraries ( test )
