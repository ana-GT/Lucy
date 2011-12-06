//---------------------------------------------------------------------
//  Copyright (c) 2009 Mike Stilman
//  All Rights Reserved.
//
//  Permission to duplicate or use this software in whole or in part
//  is only granted by consultation with the author.
//
//    Mike Stilman              mstilman@cc.gatech.edu
//
//	  Robotics and Intelligent Machines
//    Georgia Tech
//--------------------------------------------------------------------

#ifndef MY_TESTER_TAB
#define MY_TESTER_TAB

#include <set>
#include <list>
#include <vector>
#include <Eigen/Core>
#include <Tabs/RSTTab.h>
#include <Tools/World.h>
#include <Tools/Robot.h>
#include <Tools/Link.h>
#include <Tools/Object.h>
#include <Tools/Constants.h>
#include <Tools/PathPlanner.h>
#include <Tools/IK.h>

#include "WS_RRT.h"
#include "MyRSC.h"
#include "goWSOrient.h"
#include "A3d.h"
#include "Planner1.h"
#include "ParSmoother.h"


class MyTesterTab : public RSTTab
{
public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MyTesterTab(){};
        MyTesterTab( wxWindow * parent, wxWindowID id = -1,
                     const wxPoint & pos = wxDefaultPosition,
                     const wxSize & size = wxDefaultSize,
                     long style = wxTAB_TRAVERSAL);
	virtual ~MyTesterTab(){}

	wxTextCtrl *timeText;

        Planner1 *planner;
        MyRSC *rsc;

        //-- Robot
	int robot_ID;
        const string static robot_name;

        //-- Start data
	Eigen::VectorXd startConfig;

        //-- Goal data
        Eigen::Transform<double, 3, Eigen::Affine> goal_pose; 
        Eigen::VectorXd goal_position;
        string goal_name;
        int target_ID;
        
        //-- Links  
        int static const leftArm_numLinks = 7;
        static const string leftArm_LastLink_name;
        int num_links;
        std::vector<int> links_ID;
	std::vector<string> links_names;
        int EE_ID;

        //-- World objects 
        std::vector<int> movableObjects;

	//-- Transformations
        Eigen::Transform<double, 3, Eigen::Affine> T;
        Eigen::Transform<double, 3, Eigen::Affine> TWBase;
        Eigen::Transform<double, 3, Eigen::Affine> Tee;

        //-- Jacobian 
        Eigen::MatrixXd J;
        Eigen::MatrixXd Jt;

	//-- public vars to capture external selection stuff (should move these higher sometime)
	Object* selectedObject;
	Robot* selectedRobot;
	Link* selectedLink;

        /** FUNCTIONS */
	void OnSlider(wxCommandEvent &evt);
	void OnButton(wxCommandEvent &evt);
	void SetTimeline(int robot, std::vector<int> links, std::vector<Eigen::VectorXd> path);
        void SetTimeline(int robot, std::vector<int> links, std::list<Eigen::VectorXd> path);
	void RSTStateChange();

        int getRobotID( string robotName );
        void getLinks();
        void reset_all();
        Eigen::VectorXd getCurrentConfig();
        void initTrans();

	DECLARE_DYNAMIC_CLASS(MyTesterTab)
	DECLARE_EVENT_TABLE()
};

#endif
