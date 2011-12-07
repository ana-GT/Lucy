cmake_minimum_required (VERSION 2.6)
project (JacobianTests)

message (STATUS "Creating Makefile for Jacobian Tests")

set (CMAKE_INSTALL_PREFIX /usr/local )
set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")

include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)

# Set my include directories
#include_directories( . )

# Search all header and source files
file (GLOB srcs "*.cpp")
file (GLOB hdrs "*.h")
					
set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS} )

  add_executable ( JacobianTests ${srcs} ${hdrs} ) 

  target_link_libraries ( JacobianTests ${GRIP_LIBRARIES} 
										${DART_LIBRARIES} 
										${wxWidgets_LIBRARIES} )
   
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
