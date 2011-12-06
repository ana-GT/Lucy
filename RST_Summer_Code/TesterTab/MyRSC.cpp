/**
 * @file MyRSC.cpp
 * @brief Resolve Spatial Constraints ( RSC )
 */

#include <iostream>
#include <Tools/Robot.h>
#include "MyRSC.h"

using namespace std;
using namespace Eigen;

/**
 * @function MyRSC
 * @brief Constructor (empty)
 */
MyRSC::MyRSC()
{
}

/**
 * @function MyRSC
 * @brief Constructor
 */
MyRSC::MyRSC( World& _world, 
	      int _robot_ID, 
	      std::vector<int> _links_ID,
	      Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
	      Eigen::Transform<double, 3, Eigen::Affine> _Tee )
{
  this->world = new World( _world );
  this->robot_ID = _robot_ID;
  this->links_ID = _links_ID;
  this->TWBase = _TWBase	;
  this->Tee = _Tee;
}

/**
 * @function run
 * @brief Run RSC
 */
bool MyRSC::run( int _target_ID, Eigen::VectorXd _startConfig, vector<Eigen::VectorXd> &_path )
{
  target_ID = _target_ID;
  startConfig = _startConfig;    
  robinaLeftArm_fk( startConfig, TWBase, Tee, startPose );

  Eigen::Transform<double, 3, Eigen::Affine> rt0, rt1, rt2;    
  vector<Eigen::VectorXd> graspPlan, manipPlan, navigPlan;

  //-- Plan Grasp 
  PLAN_GRASP( target_ID, startConfig, rt0, graspPlan );
  printf("Grasping plan done with a path of %d \n", graspPlan.size() );

  //-- Plan Manipulation 
  rt1.setIdentity(); rt1.translation()<<0, 0.8, 0.46;

  Eigen::VectorXd rt0Config; 
  rt0Config = graspPlan[graspPlan.size() - 1];
  
  PLAN_MANIPULATION( target_ID, rt0Config, rt1, manipPlan );
  printf("Manipulating plan done with a path of %d \n", manipPlan.size() );

  //-- Plan Navigation 
  rt2.setIdentity(); rt2 = startPose;

  Eigen::VectorXd rt1Config; 
  rt1Config = manipPlan[manipPlan.size() - 1];
  
  PLAN_NAVIGATION( rt1Config, rt2, navigPlan );
  printf("Navigation plan done with a path of %d \n", navigPlan.size() );

  //-- Stitch them together
  for( int i = 0; i < graspPlan.size(); i++ )
     { _path.push_back( graspPlan[i] ); }
  for( int i = 0; i < manipPlan.size(); i++ )
     { _path.push_back( manipPlan[i] ); }
  for( int i = 0; i < navigPlan.size(); i++ )
     { _path.push_back( navigPlan[i] ); }
         
}

/**
 * @function PLAN_GRASP
 */
bool MyRSC::PLAN_GRASP( int object_ID, Eigen::VectorXd baseConfig, Eigen::Transform<double, 3, Eigen::Affine> rt_0, vector<Eigen::VectorXd> &graspPlan )
{
  rt_0.setIdentity();
  rt_0 = world->objects[ object_ID ]->absPose;
  /*
  MyTesterPlanner *planner = new MyTesterPlanner( *world ); 
  graspPlan.clear();

  planner->findPath( robot_ID, links_ID, baseConfig, rt_0.translation(), graspPlan, TWBase, Tee ); 
  */
  return true;
}

/**
 * @function PLAN_MANIPULATION
 */
bool MyRSC::PLAN_MANIPULATION( int object_ID, Eigen::VectorXd rt0Config,  Eigen::Transform<double, 3, Eigen::Affine> rt1, vector<Eigen::VectorXd> &manipPlan )
{
  /*
  MyTesterPlanner *planner = new MyTesterPlanner( *world );
  manipPlan.clear();

  planner->findPath( robot_ID, links_ID, rt0Config, rt1.translation(), manipPlan, TWBase, Tee ); 
  */
  return true;

}

/**
 * @function PLAN_NAVIGATION
 */
bool MyRSC::PLAN_NAVIGATION( Eigen::VectorXd rt1Config, Eigen::Transform<double, 3, Eigen::Affine> rt2, vector<Eigen::VectorXd> &navigPlan )
{
  /*
  MyTesterPlanner *planner = new MyTesterPlanner( *world );
  navigPlan.clear();

  planner->findPath( robot_ID, links_ID, rt1Config, rt2.translation(), navigPlan, TWBase, Tee ); 
  */
  return true;
}
