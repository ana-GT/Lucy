/**
 * @file goWSPos.h
 * @brief Straight path to the goal, if feasible, not taking into account obstacles
 */

#ifndef GO_WS_ORIENT_H
#define GO_WS_ORIENT_H

#include <vector>
#include <list>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <Eigen/Core>

#include <Tools/World.h>
#include <Tools/Link.h>
#include <Tools/kdtree/kdtree.h>

#include <robina_kin/robina_kin.h>
#include "utilities.h"


/**
 * @class goWSOrient
 */
class goWSOrient {

public: 

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  goWSOrient();
  void initialize( World* _world,
                   int _robot_ID,
                   std::vector<int> _links_ID,
                   Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                   Eigen::Transform<double, 3, Eigen::Affine> _Tee );

  World* world;
  int robot_ID;
  std::vector<int> links_ID;
  int ndim;
  double stepSize;
  int activeNode;

  std::vector<int>configVector;
  std::vector<int>parentVector;
 
  Eigen::VectorXd startConfig;
  Eigen::VectorXd goalPos;
  Eigen::Transform<double, 3, Eigen::Affine> goalPose;
  Eigen::Transform<double, 3, Eigen::Affine> TWBase;
  Eigen::Transform<double, 3, Eigen::Affine> Tee;
  
  double goalThresh;
  int maxTrials;

  /** Functions */
  void cleanup();
  double wsDistance( Eigen::VectorXd q );
  bool straightPath( const Eigen::VectorXd &startConfig, 
		     const Eigen::Transform<double, 3, Eigen::Affine> &goalPose,
		     std::vector<Eigen::VectorXd> &path );
  bool balancePath( const Eigen::VectorXd &_startConfig, 
		    const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose,
	            std::vector<Eigen::VectorXd> &path );

};

#endif /** GO_WS_POS_H*/

