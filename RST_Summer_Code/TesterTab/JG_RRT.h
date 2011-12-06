/**
 * @file JG_RRT.h
 * @author achq
 */

#ifndef JG_RRT_H
#define JG_RRT_H

#include <vector>
#include <list>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <Eigen/Core>

#include <Tools/World.h>
#include <Tools/Link.h>
#include <Tools/Robot.h>
#include <Tools/kdtree/kdtree.h>

#include <robina_kin/robina_kin.h>
#include "utilities.h"

#define RANDNM(N,M) N + ((M-N) * ((double)rand() / ((double)RAND_MAX + 1)))

/**
 * @class JG_RRT
 * @brief 
 */
class JG_RRT {

  public: 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    JG_RRT();
    ~JG_RRT();
    typedef enum {
      STEP_COLLISION,
      STEP_REACHED,
      STEP_PROGRESS,
      STEP_NO_NEARER
    } StepResult;  
   
   //-- Variables
   World* world;
   int robot_ID;
   std::vector<int> links_ID;
   int ndim;
   double stepSize;
   int activeNode;
   std::vector<int> parentVector;
   std::vector<Eigen::VectorXd> configVector;
   struct kdtree *kdTree;

  /** Planner variables */
  Eigen::VectorXd startConfig;
  Eigen::Transform<double, 3, Eigen::Affine> goalPose;
  Eigen::VectorXd goalPosition;
  Eigen::Transform<double, 3, Eigen::Affine> TWBase;
  Eigen::Transform<double, 3, Eigen::Affine> Tee;  

  double p_goal;
  double distanceThresh;  
  int maxNodes;
  //-- rankingVector saves an ordered binary heap of the nodes to be extended by Jacobian app. You pop them out.
  std::vector<int> rankingVector;
  std::vector<double> goalDistVector;


   //-- Leftover functions from RRT (untouched)
   bool connect();
   bool connect( const Eigen::VectorXd &target );

   StepResult tryStep();
   StepResult tryStep( const Eigen::VectorXd &qtry );
   virtual StepResult tryStep( const Eigen::VectorXd &qtry, int NNidx );
   
   int addNode( const Eigen::VectorXd &qnew, int parent_ID );
   
   virtual Eigen::VectorXd getRandomConfig();
   int getNearestNeighbor( const Eigen::VectorXd &qsample );
   
   void tracePath( int node_ID, std::vector<Eigen::VectorXd> &path, bool reverse = false );
   virtual bool checkCollisions( const Eigen::VectorXd &config );

   //-- Initialize function
   void initialize( World* _world, 
		    	const int &_robot_ID, 
                    	const std::vector<int> &_links_ID,
                        const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase,
                        const Eigen::Transform<double, 3, Eigen::Affine> &_Tee );

   //-- Plan function
   bool plan( const Eigen::VectorXd &startConfig, 
	      const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose, 
              const std::vector< Eigen::VectorXd > &_guidingNodes,
              std::vector<Eigen::VectorXd> &path );

   bool checkPathSegment( Eigen::VectorXd config1, Eigen::VectorXd config2) const ;
   void smoothPath( std::vector<Eigen::VectorXd> &path ) const;

   virtual void cleanup();
   void addGuidingNodes( std::vector<Eigen::VectorXd> guidingNodes );
   void add_Ranking( int node_ID );
   int pop_Ranking();
   bool extendGoal();
   bool extendRandom();
   bool connectGoal();
   double wsDistance( Eigen::VectorXd q );
   Eigen::VectorXd wsDiff( Eigen::VectorXd q );
};

#endif /** JG_RRT_H */
