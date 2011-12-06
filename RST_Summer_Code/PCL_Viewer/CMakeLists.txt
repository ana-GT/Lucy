cmake_minimum_required ( VERSION 2.8 FATAL_ERROR )

project( pcd_write )

find_package( PCL 1.1 REQUIRED )

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( pcd_write pcd_write.cpp )
add_executable( simple_viewer simple_viewer.cpp )

target_link_libraries( pcd_write ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )
target_link_libraries( simple_viewer ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} )
