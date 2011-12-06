cmake_minimum_required (VERSION 2.6)
project (BenchmarkTests)

message (STATUS "Creating Makefile for the BenchmarkTests")

set (CMAKE_INSTALL_PREFIX /usr/local )
set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")

include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)

find_package( PCL 1.1 REQUIRED )

# Set my include directories
include_directories( .
                     Hope/rosFunctions/include
                     ${PCL_INCLUDE_DIRS} )

add_definitions( ${PCL_DEFINITIONS} )

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
                    Hope/HopePlanner.cpp
					Hope/HopePlanner_Search.cpp
					Hope/HopePlanner_Plotting.cpp )

set ( myFunctions_hdrs Hope/myFunctions/CheckObject.h
		               Hope/myFunctions/CheckProcess.h
	                   Hope/myFunctions/Dijkstra.h )

set ( myFunctions_srcs Hope/myFunctions/CheckObject.cpp
		               Hope/myFunctions/CheckProcess.cpp
                       Hope/myFunctions/Dijkstra.cpp
	                    )	

set ( rosFunctions_srcs Hope/rosFunctions/src/bfs_3d.cpp
						Hope/rosFunctions/src/pf_distance_field.cpp )
					
# Hope/myFunctions/ArmState.cpp

# Hope/myFunctions/HopePreprocessor.cpp

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} 				  ${WXWIDGETS_SHARED_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

  add_executable (BenchmarkTests ${srcs} 
                                 ${planners_srcs} 
                                 ${myFunctions_srcs} 
								 ${rosFunctions_srcs}
                                 ${hdrs} 
                                 ${myFunctions_hdrs} ) 

  target_link_libraries (BenchmarkTests ${GRIP_LIBRARIES} 
										${DART_LIBRARIES} 											${wxWidgets_LIBRARIES}
										${PCL_COMMON_LIBRARIES} 
										${PCL_IO_LIBRARIES} 
                                        ${PCL_LIBRARIES} )
   
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
