/**
 * @file Planner1.h
 * @brief Holy
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "voxelSuite/CheckProcess.h"
#include "A3d.h"
#include <robina_kin/robina_kin.h>
#include <amino/include/amino.h>

#include <assert.h>
#include <stdio.h>
#include <unistd.h>

#include <Tools/RRT.h>
#include <Tools/Robot.h>

#include "goWSPos.h"
#include "JG_RRT.h"

#ifndef PLANNER_1
#define PLANNER_1

/**
 * @class Planner1
 * @brief My testing planner. No IK, no RRT (that should be)
 */
class Planner1 
{
  public: 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor / Destructor
    Planner1();
    ~Planner1();
   
    //-- Environment
    World* world;
    int robot_ID;
    std::vector<int> links_ID;

    //-- Bodies left arm information
    int num_bodies_left_;    
    std::vector< int > bodies_left_ID_;

    //-- Kinematics stuff
    Eigen::VectorXd q;
    Eigen::Transform<double, 3, Eigen::Affine> TWBase;
    Eigen::Transform<double, 3, Eigen::Affine> Tee;

    //-- Start
    Eigen::VectorXd startConfig;
    Eigen::VectorXd startPosition;
    Eigen::Vector3i startVoxelCell;

    //-- Goal
    Eigen::VectorXd goalPosition;
    Eigen::Transform<double, 3, Eigen::Affine> goalPose;
    Eigen::Vector3i goalVoxelCell;

    Eigen::VectorXd delta_x;
    Eigen::VectorXd delta_q;
    vector<Eigen::VectorXd> path;
         
   //-- Obstacles information
   int num_iter;

   //-- A* Planner
   A_3D a;
   std::vector< Eigen::VectorXi > blackList;
   goWSPos directPoint;
 
   //-- To save data in files
   FILE * pPath3D;

    /// Testing stuff (to be erased probably)
    static int const max_trials = 1000;
    static double const workspace_threshold = 0.025;
    double stepSize;

    /// Functions
    void initialize( World &_world,
                     const int &_robot_ID, 
		     const std::vector<int> &_links_ID,
                     const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase, 
		     const Eigen::Transform<double, 3, Eigen::Affine> &_Tee );

    bool findPath( const Eigen::VectorXd &_startConfig, 
		   const Eigen::VectorXd &_goalPosition, 
                   std::vector<Eigen::VectorXd> &_path );
    bool checkPath( std::vector< Eigen::VectorXi > path, CheckProcess &cp, int &num_col, std::vector< Eigen::VectorXd > &collision_free );

};

#endif
