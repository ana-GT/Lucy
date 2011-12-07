/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef _JACOBIAN_TESTS_
#define _JACOBIAN_TESTS_

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <planning/Robot.h>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include <iostream>
#include <list>

using namespace std;

/**
 * @class JacobianTests
 * @brief Tab with Tester Planners
 */
class JacobianTests : public GRIPTab
{
public:

    /// Variables to save info for the planners
    Eigen::VectorXd startConf;
    Eigen::VectorXd targetConf;
    Eigen::VectorXd targetPose; /**< Always dimension 6, otherwise you are toasted */

    int robotId;
    Eigen::VectorXi links;

    std::list< Eigen::VectorXd > mPath;
    std::list< Eigen::VectorXd > mCollisionPath;

    /// For Jacobian testing
    bool mFixXYZ;
    bool mFixOrient;

    /// Miscellaneous stuff ( no idea why this is here )
    wxTextCtrl *timeText;

    /// Public vars to capture external selection stuff 
    planning::Object* selectedObject;
    planning::Robot* selectedRobot;
    kinematics::BodyNode* selectedNode;

    /// Functions about Robina's left arm specifically
    Eigen::VectorXi GetLeftArmIds();

    /// Functions related to Tab
    JacobianTests(){};
    JacobianTests( wxWindow * parent, wxWindowID id = -1,
                    const wxPoint & pos = wxDefaultPosition,
                    const wxSize & size = wxDefaultSize,
                    long style = wxTAB_TRAVERSAL);
    virtual ~JacobianTests(){}

    void OnSlider(wxCommandEvent &evt);
    void OnRadio(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnCheckBox(wxCommandEvent &evt);

    GRIPSlider* mSlider_Nullspace;

    void SetTimeline( std::list<Eigen::VectorXd> _path );
    void GRIPStateChange();

    double NullspaceTest( double _ang );

    // Thread specific
    // GRIPThread* thread;

    // Your Thread routine
    // call GRIPThread::CheckPoint() regularly
    // void Thread();
    // void onCompleteThread();

    DECLARE_DYNAMIC_CLASS( JacobianTests )
    DECLARE_EVENT_TABLE()
};

#endif /** _JACOBIAN_TESTS_ */

