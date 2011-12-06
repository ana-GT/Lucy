/**
 * @file HopePlanner.h
 * @brief Awwww!
 */

#ifndef _HOPE_PLANNER_
#define _HOPE_PLANNER_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <distance_field/distance_field.h>
#include <distance_field/pf_distance_field.h>
#include <CheckProcess.h>
#include <robina_kin/robina_kin.h>
#include <amino/include/amino.h>

#include <assert.h>
#include <stdio.h>
#include <unistd.h>

#include "search.h"
#include "dijkstra.h"

using namespace distance_field;

/**
 * @class HopePlanner
 * @brief My testing planner. No IK, no RRT (that should be)
 */

class HopePlanner 
{
  public: 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor / Destructor
    HopePlanner();
    ~HopePlanner();
   
    //-- Environment
    World* mWorld;
    int mRobotID;
    std::vector<int> mLinksID;
    double mSizeX; double mSizeY; double mSizeZ;
    double mOrigX; double mOrigY; double mOrigZ;

    //-- 3D information - ALL
    double mResolution;
    
    //-- Bodies left arm information
    int mNumBodiesLeft;    
    std::vector< int > mBodiesLeftID;

    //-- Kinematics stuff
    Eigen::Transform<double, 3, Eigen::Affine> mTWBase;
    Eigen::Transform<double, 3, Eigen::Affine> mTee;

    //-- Start
    Eigen::VectorXd mStartConfig;
    Eigen::Vector3d mStartPosition;
    Eigen::Vector3i mStartCell;

    //-- Goal
    Eigen::VectorXd mGoalPosition;
    Eigen::Transform<double, 3, Eigen::Affine> mGoalPose;
    Eigen::Vector3i mGoalCell;

    //-- Voxelization and Distance Transforms 
    VoxelGrid<int>* mVoxel; int mDefaultVoxelObject;
    PFDistanceField* mPf;

    //-- Search
    Dijkstra3D mDijkstraEE;
  
    //-- Additional cool stuff
   time_t ts; time_t tf; double dt;         

    /// Functions
    void fill_voxel();
    void initialize( World &_world,
                     const int &_robot_ID, 
		     const std::vector<int> &_links_ID,
                     const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase, 
		     const Eigen::Transform<double, 3, Eigen::Affine> &_Tee );

    bool findPath( const Eigen::VectorXd &_start_config, 
		   const Eigen::Transform<double, 3, Eigen::Affine> &_goal_pose, 
                   std::vector<Eigen::VectorXd> &_path );

    void fromJoints2Position( Eigen::VectorXd q, Eigen::Vector3d &p );
    void calculateInfo3D( VoxelGrid<int> &_voxel, PFDistanceField &_df );

};

#endif /** _HOPE_PLANNER_ */
