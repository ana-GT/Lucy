cmake_minimum_required (VERSION 2.6)
project (BenchmarkTests)

message (STATUS "Creating Makefile for the BenchmarkTests")

# This include is used only if you have installed Grip
if(WIN32)
	set (CMAKE_INSTALL_PREFIX "C:/Golems" )
    message (STATUS "Setup Visual Studio Specific Flags")
	set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1")
	set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1")
	set (CMAKE_DEFAULT_STARTUP_PROJECT BenchmarkTests)
else ()
   set (CMAKE_INSTALL_PREFIX /usr/local )
   set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")
endif()
include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)
# End include


# Search all header and source files
file (GLOB srcs "*.cpp")
file (GLOB hdrs "*.h")

# Source for planners
set ( planners_srcs RRT/RRT.cpp
							  		RRT/RRTPlanner.cpp
                    B1Planner/B1Planner.cpp
                    B1Planner/B1RRT.cpp
                    B1Planner/B1Ranking.cpp
                    kdtree/kdtree.c
                    Hope/HopePlanner.cpp )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
	add_executable ( BenchmarkTests WIN32 ${srcs} ${planners_srcs} ${hdrs} )
	set_target_properties (BenchmarkTests PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
   else()
    add_executable (BenchmarkTests ${srcs} ${planners_srcs} ${hdrs} )
   endif()

   target_link_libraries (BenchmarkTests ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${wxWidgets_LIBRARIES} )
   
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
