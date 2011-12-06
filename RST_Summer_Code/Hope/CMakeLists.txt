cmake_minimum_required( VERSION 2.8 )
project( Hope )

include_directories( .
                     ../../
		     ../../librst
		     ../../humanoids
                     rosFunctions/include
                     myFunctions/include )

set( CMAKE_CXX_FLAGS "-L ../../librst -L ../../humanoids/robina_kin -L ../../humanoids/amino/build" )

set( SRC_TAB_SIMULATION  HopeTab
			 Hope 
                         search
                         dijkstra
		         armState
                         HopePlanner
		         myFunctions/src/CheckProcess
                         myFunctions/src/CheckObject
                         rosFunctions/src/bfs_3d
	                 rosFunctions/src/pf_distance_field
                         rosFunctions/src/propagation_distance_field )


set( wxWidgets_USE_LIBS base core gl )

find_package ( wxWidgets )

if ( wxWidgets_FOUND )
   include ( ${wxWidgets_USE_FILE} )
   add_executable ( HopeTab ${SRC_TAB_SIMULATION} )
   target_link_libraries ( HopeTab rst robina_kin amino ${wxWidgets_LIBRARIES} )
else ( wxWidgets_FOUND )
   message ( "--(!) wxWidgets not found!" )
endif ( wxWidgets_FOUND )

