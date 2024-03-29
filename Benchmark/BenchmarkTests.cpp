/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "BenchmarkTests.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>

#include <iostream>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

// PLANNERS
#include "RRT/RRTPlanner.h"
#include "B1Planner/B1Planner.h"
#include "Hope/HopePlanner.h"

using namespace std;


/* Quick intro to adding tabs:
 * 1- Copy template cpp and header files and replace with new class name
 * 2- include classname.h in AllTabs.h, and use the ADD_TAB macro to create it
 */

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum BenchmarkTestsEvents {
	button_SetStartConf = 50,
	button_ShowStartConf,
	button_Empty1,
	button_SetTargetConf,
	button_ShowTargetConf,
	button_SetTargetPose,
  button_Test1,
  button_Test2,
  button_Test3,
  button_PlotJoints,
  button_PlotCollisions,
  button_PlotFree,
	button_ResetPlanner,
	button_Plan,
	button_Stop,
	button_UpdateTime,
	button_ExportSequence,
	button_ShowPath,
	slider_Time
};

// sizer for whole tab
wxBoxSizer* sizerFull;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE( BenchmarkTests, wxPanel )
EVT_COMMAND ( wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, BenchmarkTests::OnSlider )
EVT_COMMAND ( wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, BenchmarkTests::OnButton )
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(BenchmarkTests, GRIPTab)

/**
 * @function BenchmarkTests
 * @brief Constructor
 */
BenchmarkTests::BenchmarkTests( wxWindow *parent, const wxWindowID id,
		                            const wxPoint& pos, const wxSize& size, long style) :
	                              GRIPTab(parent, id, pos, size, style) {

    startConf.resize(0);
    targetConf.resize(0);
    targetPose.resize(0);

    robotId = 0;
    links.resize(0);

    sizerFull = new wxBoxSizer( wxHORIZONTAL );
 
    // ** Create left static box for configuring the planner **

    // Create StaticBox container for all items
    wxStaticBox* configureBox = new wxStaticBox(this, -1, wxT("Configure"));

    // Create sizer for this box with horizontal layout
    wxStaticBoxSizer* configureBoxSizer = new wxStaticBoxSizer(configureBox, wxHORIZONTAL);

    // Create sizer for start buttons in 1st column
    wxBoxSizer *col1Sizer = new wxBoxSizer(wxVERTICAL);
    col1Sizer->Add( new wxButton(this, button_SetStartConf, wxT("Set &Start Conf")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col1Sizer->Add( new wxButton(this, button_ShowStartConf, wxT("Show S&tart Conf")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col1Sizer->Add( new wxButton(this, button_Empty1, wxT("Empty 1")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    // Add col1Sizer to the configuration box
    configureBoxSizer->Add( col1Sizer,
			    1, // takes half the space of the configure box
			    wxALIGN_NOT ); // no border and center horizontally


    // Create sizer for goal buttons in 2nd column
    wxBoxSizer *col2Sizer = new wxBoxSizer(wxVERTICAL);
    col2Sizer->Add( new wxButton(this, button_SetTargetConf, wxT("Set &Target Conf")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    col2Sizer->Add( new wxButton(this, button_ShowTargetConf, wxT("Show T&arget Conf")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col2Sizer->Add( new wxButton(this, button_SetTargetPose, wxT("Set Target Pose")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    // Add col2Sizer to the configuration box
    configureBoxSizer->Add( col2Sizer,
			    1, // size evenly with radio box and checkboxes
			    wxALIGN_NOT ); // no border and center horizontally



    // Create sizer for planner buttons in 3rd column
    wxBoxSizer *col3Sizer = new wxBoxSizer(wxVERTICAL);
    col3Sizer->Add( new wxButton(this, button_Test1, wxT("Test 1")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    col3Sizer->Add( new wxButton(this, button_Test2, wxT("Test 2")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col3Sizer->Add( new wxButton(this, button_Test3, wxT("Test 3")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    // Add col2Sizer to the configuration box
    configureBoxSizer->Add( col3Sizer,
			    1, // size evenly with radio box and checkboxes
			    wxALIGN_NOT ); // no border and center horizontally


    // Create sizer for planner buttons in 4th column
    wxBoxSizer *col4Sizer = new wxBoxSizer(wxVERTICAL);
    col4Sizer->Add( new wxButton(this, button_PlotJoints, wxT("Plot joints")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    col4Sizer->Add( new wxButton(this, button_PlotCollisions, wxT("Plot joint collisions")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together
    col4Sizer->Add( new wxButton(this, button_PlotFree, wxT("Show Pointcloud")),
		    0, // make horizontally unstretchable
		    wxALL, // make border all around (implicit top alignment)
		    1 ); // set border width to 1, so start buttons are close together

    // Add col2Sizer to the configuration box
    configureBoxSizer->Add( col4Sizer,
			    1, // size evenly with radio box and checkboxes
			    wxALIGN_NOT ); // no border and center horizontally

    // Add this box to parent sizer
    sizerFull->Add( configureBoxSizer,
		    4, // 4-to-1 ratio with execute sizer, since it has 4 buttons
		    wxEXPAND | wxALL,
		    6 );

    // ** Create right static box for running the planner **
    wxStaticBox* executeBox = new wxStaticBox(this, -1, wxT("Execute Planner"));

    // Create sizer for this box
    wxStaticBoxSizer* executeBoxSizer = new wxStaticBoxSizer(executeBox, wxVERTICAL);

    // Add buttons for "plan", "save movie", and "show path"
    executeBoxSizer->Add( new wxButton(this, button_Plan, wxT("&Run")),
	 		  1, // stretch to fit horizontally
			  wxGROW ); // let it hog all the space in it's column

    executeBoxSizer->Add( new wxButton(this, button_Stop, wxT("&Stop")),
			  1, // stretch to fit horizontally
			  wxGROW );


    wxBoxSizer *timeSizer = new wxBoxSizer(wxHORIZONTAL);
    timeText = new wxTextCtrl(this,1008,wxT("5.0"),wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
    timeSizer->Add(timeText,2,wxALL,1);
    timeSizer->Add(new wxButton(this, button_UpdateTime, wxT("Set T(s)")),2,wxALL,1);
    executeBoxSizer->Add(timeSizer,1,wxALL,2);

    executeBoxSizer->Add( new wxButton(this, button_ShowPath, wxT("&Print")),
			  1, // stretch to fit horizontally
			  wxGROW );

    sizerFull->Add(executeBoxSizer, 1, wxEXPAND | wxALL, 6);

    SetSizer(sizerFull);

}

/**
 * @function getLeftArmIds
 * @brief Get DOF's IDs for ROBINA's left arm
 */
Eigen::VectorXi BenchmarkTests::GetLeftArmIds() {

  string LINK_NAMES[7] = {"LJ0", "LJ1", "LJ2", "LJ3", "LJ4", "LJ5", "LJ6" };
   
  Eigen::VectorXi linksAll = mWorld->mRobots[robotId]->getQuickDofsIndices(); 

  Eigen::VectorXi linksLeftArm(7);
  for( unsigned int i = 0; i < 7; i++ ) {
      for( unsigned int j = 0; j < linksAll.size(); j++ ) {      
          if( mWorld->mRobots[robotId]->getDof( linksAll[j] )->getJoint()->getChildNode()->getName() == LINK_NAMES[i] ) {
              linksLeftArm[i] = linksAll[j]; 
              break;   
          }
      }
  }
  
  return linksLeftArm;
}


/**
 * @function OnButton
 * @brief Handle Button Events
 */
void BenchmarkTests::OnButton(wxCommandEvent &evt) {

    int button_num = evt.GetId();
    links = GetLeftArmIds();

    switch (button_num) {

        /** Set Start Configuration */
        case button_SetStartConf:
	          if ( mWorld != NULL ) {
	              if( mWorld->mRobots.size() < 1) {
            	      cout << "--(!) Must have a world with a robot to set a Start state (!)--" << endl;
		                break;
		            }
		            std::cout << "--(i) Setting Start state for " << mWorld->mRobots[robotId]->getName() << ":" << std::endl;
                
                startConf = mWorld->mRobots[robotId]->getDofs( links );

		            for( unsigned int i = 0; i < startConf.size(); i++ )
                {  std::cout << startConf(i) << " ";  } 
		            std::cout << endl;
	          } else {
	              std::cout << "--(!) Must have a world loaded to set a Start state.(!)--" << std::endl;
	          }
	          break;

        /** Set Target Configuration */
	      case button_SetTargetConf:
	          if ( mWorld != NULL ) {
	              if( mWorld->mRobots.size() < 1){
		                std::cout << "--(!) Must have a world with a robot to set a Goal state.(!)--" << endl;
		                break;
		            }
		            std::cout << "--(i) Setting Goal state for " << mWorld->mRobots[robotId]->getName() << ":" << std::endl;

                targetConf = mWorld->mRobots[robotId]->getDofs( links );

		            for( unsigned int i = 0; i < targetConf.size(); i++ )
                {  cout << targetConf(i) << " "; } 
		            std::cout << std::endl;
	          } else {
	              cout << "--(!) Must have a world loaded to set a Goal state (!)--" << endl;
	          }
	          break;

        /** Set Target Pose */
	      case button_SetTargetPose:

	          if ( mWorld != NULL ) {
	              if( mWorld->mRobots.size() < 1){
		                std::cout << "--(!) Must have a world with a robot to set a Goal state.(!)--" << endl;
		                break;
		            }

                if( selectedObject != NULL ) {

                    double x; double y; double z;
                    double roll; double pitch; double yaw;

                    selectedObject->getPositionXYZ( x, y, z );
                    selectedObject->getRotationRPY( roll, pitch, yaw );
 
		                std::cout << "--(i) Setting Goal state for " << mWorld->mRobots[robotId]->getName() << " from "<<selectedObject->getName()<<":"<< std::endl;

                    targetPose.resize(6);
                    targetPose(0) = x; targetPose(1) = y; targetPose(2) = z; 
                    targetPose(3) = roll; targetPose(4) = pitch; targetPose(5) = yaw; 

		                for( unsigned int i = 0; i < targetPose.size(); i++ )
                    {  cout << targetPose(i) << " "; } 
		                   std::cout << std::endl;

                } else { 
                    std::cout<<"--(x) Please, select an object in the Viewer Tree and try again (x)--"<<std::endl; 
                }                    


	          } else {
	              std::cout << "--(!) Must have a world loaded to set a Goal state (!)--" << std::endl;
	          }          

	          break;

        /** Show Start */
	      case button_ShowStartConf:

	          if( startConf.size() < 1 ) {
	              std::cout << "--(x) First, set a start configuration (x)--" << std::endl;
		            break;
	          } 

            mWorld->mRobots[robotId]->setDofs( startConf, links );

	          for( unsigned int i = 0; i< startConf.size(); i++ )
            {  cout << startConf(i) << " "; }
	          std::cout << std::endl;

	          mWorld->mRobots[robotId]->update();
	          viewer->UpdateCamera();
 
      	    break;

        /** Show Target */
	      case button_ShowTargetConf:
	          if( targetConf.size() < 1) {
	              std::cout << "--(x) First, set a goal configuration (x)--" << std::endl;
		            break;
	          }

            mWorld->mRobots[robotId]->setDofs( targetConf, links );

	          for( unsigned int i = 0; i< targetConf.size(); i++ )
            {  std::cout << targetConf[i] << " ";  }
	          std::cout << std::endl;

	          mWorld->mRobots[robotId]->update();
	          viewer->UpdateCamera(); 
	          break;

        /** Reset Planner */ 
	      case button_ResetPlanner:
	          if ( mWorld != NULL) {               
		            std::cout << "Creating a new planner" << std::endl;
	          } else {
	              std::cout << "--(!) Must have a world loaded to make a planner (!)--" << std::endl;
	          }
	          break;

        /** Empty button 1 */
	      case button_Empty1:
          {
	          std::cout << "-- (0) Checking Collisions (0)--" << endl;

            bool st = mCollision->CheckCollisions();
            if( st == true )
            { printf("Collisions \n");}
            else
            { printf("No Collisions \n");}
          }
	          break;

        /** Execute Plan */
	      case button_Plan:
          {         
            
            Eigen::VectorXd conf = mWorld->mRobots[robotId]->getDofs( links );
            mWorld->mRobots[robotId]->setDofs( conf, links );
            Eigen::VectorXd pose;
            mWorld->mRobots[robotId]->getPose( pose );
            std::cout<< "pose: "<< std::endl << pose <<std::endl;
            mWorld->mRobots[robotId]->setPose( pose, true, true );
            mWorld->mRobots[robotId]->getNode("LJ4")->evalJacLin();
            Eigen::MatrixXd jaclin = mWorld->mRobots[robotId]->getNode("LJ4")->getJacobianLinear(); 
            std::cout << "Linear jac for LJ4:" << std::endl;
            std::cout << jaclin << std::endl; 
          }
	          break;

        /** Test 1: Bertram RRT */
        case button_Test1:
          {
	          if( targetPose.size() != 6 ){ std::cout << "--(x) Must set a target pose (x)--" << std::endl; break; }
	          if( startConf.size() < 0 ){ std::cout << "--(x) Must set a start configuration (x)--" << std::endl; break; }
	          if( mWorld == NULL ){ std::cout << "--(x) Must load a world (x)--" << std::endl; break; }
	          if( mWorld->mRobots.size() < 1 ){ std::cout << "--(x) Must load a world with a robot(x)--" << std::endl; break; }

            double stepSize = 0.02;


            std::vector<Eigen::VectorXd> path;

          }
            break;

        /** Test 2: Hope */
        case button_Test2:
          {
	          if( targetPose.size() != 6 ){ std::cout << "--(x) Must set a target pose (x)--" << std::endl; break; }
	          if( startConf.size() < 0 ){ std::cout << "--(x) Must set a start configuration (x)--" << std::endl; break; }
	          if( mWorld == NULL ){ std::cout << "--(x) Must load a world (x)--" << std::endl; break; }
	          if( mWorld->mRobots.size() < 1){ std::cout << "--(x) Must load a world with a robot(x)--" << std::endl; break; }

            double stepSize = 0.02;

	          //wxThread planThread;
	          //planThread.Create();
            printf("Send planning command \n");
            int maxNodes = 8000;
            HopePlanner planner( *mWorld, mCollision, false, stepSize );
    	    bool result = planner.planPath( robotId, 
                                            links, 
                                            startConf, 
                                            targetPose,
                                            "LJ6", 
                                            0, // no smooth 
                                            maxNodes );
             
            
            if( result )
            {  mPath = planner.mPath;
               planner.GetCollisionWaypoints( planner.mPath, mCollisionPath );       
               SetTimeline( planner.mPath ); }            
          }
            break;

        /** Test 3: Plain RRT */
        case button_Test3:
          {
	          if( targetConf.size() < 0 ){ std::cout << "--(x) Must set a goal (x)--" << std::endl; break; }
	          if( startConf.size() < 0 ){ std::cout << "--(x) Must set a start (x)--" << std::endl; break; }
	          if( mWorld == NULL ){ std::cout << "--(x) Must load a world (x)--" << std::endl; break; }
	          if( mWorld->mRobots.size() < 1){ std::cout << "--(x) Must load a world with a robot(x)--" << std::endl; break; }

            double stepSize = 0.02;

	          //wxThread planThread;
	          //planThread.Create();

            int maxNodes = 8000;
            RRTPlanner planner( *mWorld, mCollision, false, stepSize );
    	      bool result = planner.planPath( robotId, 
                                            links, 
                                            startConf, 
                                            targetConf, 
                                            1, // bidirectional  
                                            1, // connect mode
                                            1, // greedy
                                            0, // no smooth 
                                            maxNodes );

            if( result )
            {  SetTimeline( planner.path ); }
          }
            break; 

        /** Plot joints */
	      case button_PlotJoints:
          {
             // Print a .dat file with the info
			  	   printf("Generating Joint Plots \n");
  					 FILE *jointsFile;
  					 jointsFile = fopen ("Joints.dat","w");

             for( std::list<Eigen::VectorXd>::iterator it = mPath.begin(); it != mPath.end(); it++ ) {
               Eigen::VectorXd config  = *it;
               for ( int i = 0 ; i < config.size(); i++ ) {
                 fprintf ( jointsFile, " %.3f ", config[i] ); 
               }
               fprintf( jointsFile, "\n" );
             }

            fclose ( jointsFile ); 
          }		
	          break;

        /** Plot collisions */
	      case button_PlotCollisions:
          {
             // Print a .dat file with the info
			  	   printf("Generating Joint Collision Plots \n");
  					 FILE *collisionFile;
  					 collisionFile = fopen ("CollidingJoints.dat","w");

             for( std::list<Eigen::VectorXd>::iterator it = mCollisionPath.begin(); it != mCollisionPath.end(); it++ ) {
               Eigen::VectorXd config  = *it;
               for ( int i = 0 ; i < config.size(); i++ ) {
                 fprintf ( collisionFile, " %.3f ", config[i] ); 
               }
               fprintf( collisionFile, "\n" );
             }

            fclose ( collisionFile ); 
          }		
	          break;

        /** Plot free */
	      case button_PlotFree:
          {
			 HopePlanner planner;
             string pcl_name;
             pcl_name = planner.PointcloudWriter( "PointCloudData.txt", 0 );
             std::vector<string> clouds(1);
             clouds[0] = pcl_name;
             planner.PointcloudViewer( clouds );
          }		
	          break;

        

        /** UpdateTime (?) */
	      case button_UpdateTime:
          {
	          /// Update the time span of the movie timeline
	          //SetTimeline();
          }		
	          break;

        /** Show Path */
	      case button_ShowPath:            
	          if( mWorld == NULL ) {
	              cout << "--(!) Must create a valid plan before printing. (!)--" << endl;
		            return;
	          } else {
                printf("--(i) Printing (i)-- \n");
            }        
	          break;

    } // end of switch
}

/**
 * @function setTimeLine
 * @brief 
 */
void BenchmarkTests::SetTimeline( std::list<Eigen::VectorXd> _path ) {
    
    if( mWorld == NULL || _path.size() == 0 ) {
        std::cout << "--(!) Must create a valid plan before updating its duration (!)--" << std::endl;
	      return;
    }
    
    double T;
    timeText->GetValue().ToDouble(&T);
    
    int numsteps = _path.size();
    double increment = T/(double)numsteps;

    cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << T << " Steps: " << numsteps << endl;

    frame->InitTimer( string("RRT_Plan"),increment );

    Eigen::VectorXd vals( links.size() );

    for( std::list<Eigen::VectorXd>::iterator it = _path.begin(); it != _path.end(); it++ ) {

        mWorld->mRobots[robotId]->setDofs( *it, links );
	      mWorld->mRobots[robotId]->update();

        frame->AddWorld( mWorld );
    }
   
}


/**
 * @function OnSlider
 * @brief Handle slider changes
 */
void BenchmarkTests::OnSlider(wxCommandEvent &evt) {

    if ( selectedTreeNode == NULL ) {
        return;
    }

    int slnum = evt.GetId();
    double pos = *(double*) evt.GetClientData();
    char numBuf[1000];

    switch (slnum) {

        case slider_Time:
	          sprintf(numBuf, "X Change: %7.4f", pos);
	          std::cout << "-->(i) Timeline slider output: " << numBuf << std::endl;
	          //handleTimeSlider(); // uses slider position to query plan state
	          break;

      	default:
	          return;
    }

    //world->updateCollision(o);
    //viewer->UpdateCamera();

    if (frame != NULL)
        frame->SetStatusText(wxString(numBuf, wxConvUTF8));
}

/**
 * @function GRIPStateChange -- Keep using this name as it is a virtual function
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void BenchmarkTests::GRIPStateChange() {
    if ( selectedTreeNode == NULL ) {

        return;
    }

    string statusBuf;
    string buf, buf2;

    switch (selectedTreeNode->dType) {

        case Return_Type_Object:
	          selectedObject = (planning::Object*) ( selectedTreeNode->data );
	          statusBuf = " Selected Object: " + selectedObject->getName();
	          buf = "You clicked on object: " + selectedObject->getName();
	          // Enter action for object select events here:
	          break;

	      case Return_Type_Robot:
	          selectedRobot = (planning::Robot*) ( selectedTreeNode->data );
	          statusBuf = " Selected Robot: " + selectedRobot->getName();
	          buf = " You clicked on robot: " + selectedRobot->getName();
      	    // Enter action for Robot select events here:
	          break;
	      case Return_Type_Node:
	          selectedNode = (kinematics::BodyNode*) ( selectedTreeNode->data );
	          statusBuf = " Selected Body Node: " + string(selectedNode->getName()) + " of Robot: "
			      + ( (planning::Robot*) selectedNode->getSkel() )->getName();
	          buf = " Node: " + string(selectedNode->getName()) + " of Robot: " + ( (planning::Robot*) selectedNode->getSkel() )->getName();
	          // Enter action for link select events here:
      	    break;
        default:
            fprintf(stderr, "--( :D ) Someone else's problem!\n");
            assert(0);
            exit(1);
    }

    //cout << buf << endl;
    frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
    sizerFull->Layout();
}
