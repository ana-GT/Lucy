cmake_minimum_required ( VERSION 2.6 )
project (MyTesterTab)

include_directories( ../../ 
		     ../../librst
		     ../../humanoids )

set( CMAKE_CXX_FLAGS "-L ../../librst -L ../../humanoids/robina_kin -L ../../humanoids/amino/build" )

set( SRC_TAB_SIMULATION 
	MyTesterTab 
	MyTester
	MyTesterPlanner
        ParSmoother
        Planner1 
        MyRSC
        JT_RRT
        JG_RRT
        M_RRT
        WS_RRT
        goWSPos
        goWSOrient
        utilities
        A3d
        voxelSuite/BinaryVoxel
        voxelSuite/EDT
        voxelSuite/CheckObject
        voxelSuite/CheckProcess
        voxelSuite/lineTraversal
   )

set( wxWidgets_USE_LIBS base core gl )

find_package ( wxWidgets )

if ( wxWidgets_FOUND )
   include ( ${wxWidgets_USE_FILE} )
   add_executable ( MyTesterTab ${SRC_TAB_SIMULATION} )
   target_link_libraries ( MyTesterTab rst robina_kin amino ${wxWidgets_LIBRARIES} )
else ( wxWidgets_FOUND )
   message ( "wxWidgets not found!" )
endif ( wxWidgets_FOUND )
