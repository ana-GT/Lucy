/**
 * @file MyRSC.h
 * @brief Resolve Spatial Constraints ( RSC )
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "JT_RRT.h"
#include "MyTesterPlanner.h"

#ifndef MY_RSC
#define MY_RSC

class MyRSC
{
  public:

   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   /// Functions
   MyRSC();
   MyRSC( World& _world, 
	  int _robot_ID, 
	  std::vector<int> _links_ID,
	  Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
	  Eigen::Transform<double, 3, Eigen::Affine> _Tee );
   ~MyRSC();

   /// Variables
   World* world; 
   int robot_ID;
   std::vector<int> links_ID;
   Eigen::Transform<double, 3, Eigen::Affine> TWBase;
   Eigen::Transform<double, 3, Eigen::Affine> Tee;
   MyTesterPlanner *planner;

   //-- Start
   Eigen::VectorXd startConfig;
   Eigen::Transform<double, 3, Eigen::Affine> startPose;
   //-- Target
   int target_ID;

   /// Functions
   bool run( int _target_ID, Eigen::VectorXd _startConfig, vector<Eigen::VectorXd> &_path );
   bool PLAN_GRASP( int object_ID, Eigen::VectorXd baseConfig, Eigen::Transform<double, 3, Eigen::Affine> rt_0, vector<Eigen::VectorXd> &graspPlan );
   bool PLAN_MANIPULATION( int object_ID, Eigen::VectorXd rt0Config,  Eigen::Transform<double, 3, Eigen::Affine> rt1, vector<Eigen::VectorXd> &manipPlan );
   bool PLAN_NAVIGATION( Eigen::VectorXd rt1Config, Eigen::Transform<double, 3, Eigen::Affine> rt2, vector<Eigen::VectorXd> &navigPlan );

};

#endif

