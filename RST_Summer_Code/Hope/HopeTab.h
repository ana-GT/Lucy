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

#ifndef _HOPE_TAB_
#define _HOPE_TAB_

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
#include <robina_kin/robina_kin.h>

#include "HopePlanner.h"

class HopeTab : public RSTTab
{
   public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	HopeTab(){};
        HopeTab( wxWindow * parent, wxWindowID id = -1,
                     const wxPoint & pos = wxDefaultPosition,
                     const wxSize & size = wxDefaultSize,
                     long style = wxTAB_TRAVERSAL);
	virtual ~HopeTab(){}

	wxTextCtrl *timeText;

        //-- Robot
	int robot_ID_;
        const string static robot_name_;

        //-- Start data
	Eigen::VectorXd start_config_;

        //-- Goal data
        Eigen::Transform<double, 3, Eigen::Affine> goal_pose_; 
        Eigen::VectorXd goal_position_;
        string goal_name_;
        int goal_ID_;
        
        //-- Links  
        int static const leftArm_numLinks_ = 7;
        static const string leftArm_LastLink_name_;
        int num_links_;
        std::vector<int> links_ID_;
	std::vector<string> links_names_;
        int EE_ID_;

        //-- World objects 
        std::vector<int> movableObjects_;

	//-- Transformations
        Eigen::Transform<double, 3, Eigen::Affine> T_;
        Eigen::Transform<double, 3, Eigen::Affine> TWBase_;
        Eigen::Transform<double, 3, Eigen::Affine> Tee_;

        //-- Jacobian 
        Eigen::MatrixXd J_;
        Eigen::MatrixXd Jt_;

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

	DECLARE_DYNAMIC_CLASS(HopeTab)
	DECLARE_EVENT_TABLE()
};

#endif /** _HOPE_TAB_ */
