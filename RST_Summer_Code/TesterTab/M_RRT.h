/**
 * @file M_RRT.h
 * @author achq
 * @brief Variation of Bertram using Jacobian for all stuff
 */

#ifndef M_RRT_H
#define M_RRT_H

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

#define RANDNM(N,M) N + ((M-N) * ((double)rand() / ((double)RAND_MAX + 1)))

/**
 * @class M_RRT
 * @brief Implements the Bertram RRT variation :)
 */
class M_RRT {

  public: 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    //-- Constructor
    M_RRT();

    //-- Destructor 
    ~M_RRT();
    typedef enum {
      STEP_COLLISION,
      STEP_REACHED,
      STEP_PROGRESS,
      STEP_NO_NEARER
    } StepResult;  

   //-- Initializing function
   void initialize( World* _world, 
		    int _robot_ID, 
                    std::vector<int> _links_ID, 
                    Eigen::VectorXd &_root ); 

   virtual void cleanup();
   
   //-- Environment variables
   World* world;

   //-- Robot variables
   int robot_ID;
   std::vector<int> links_ID;
   int ndim;

   //-- RRT Tree variables
   double stepSize;
   int activeNode;
   std::vector<int> parentVector;
   std::vector<Eigen::VectorXd> configVector;
   struct kdtree *kdTree;
   int maxNodes;

  //-- Planner variables 
  Eigen::VectorXd startConfig;

  Eigen::Transform<double, 3, Eigen::Affine> goalPose;
  Eigen::VectorXd goalPosition;

  Eigen::Transform<double, 3, Eigen::Affine> TWBase;
  Eigen::Transform<double, 3, Eigen::Affine> Tee;  

  double pGoal;

  //-- Heuristic functions
  double heuristicThresh;  
  double distThresh;
  double xAlignThresh;
  double yAlignThresh;
  double w1; double w2; double w3;

  int failureThresh;

  //-- Containers
  std::vector<int> rankingVector;
  std::vector<double> heuristicVector;
  std::vector<int>failureVector;


   /** Functions */
   bool connect();
   bool connect( const Eigen::VectorXd &target );
   bool connectGoal();

   StepResult tryStep();
   StepResult tryStep( const Eigen::VectorXd &qtry );
   virtual StepResult tryStep( const Eigen::VectorXd &qtry, int NNidx );
   
   int addNode( const Eigen::VectorXd &qnew, int parent_ID );
   
   virtual Eigen::VectorXd getRandomConfig();
   int getNearestNeighbor( const Eigen::VectorXd &qsample );
   
   void tracePath( int node_ID, std::vector<Eigen::VectorXd> &path, bool reverse = false );
   virtual bool checkCollisions( const Eigen::VectorXd &config );

   /** M_RRT Planner */  
   bool BasicPlan( std::vector<Eigen::VectorXd> &path, 
		   Eigen::VectorXd &startConfig, 
		   Eigen::Transform<double, 3, Eigen::Affine> &_goalPose, 
                   Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                   Eigen::Transform<double, 3, Eigen::Affine> _Tee );

   void add_Ranking( int node_ID );
   int pop_Ranking();

   Eigen::VectorXd workspaceDist( Eigen::VectorXd node, Eigen::VectorXd ws_target );

   M_RRT::StepResult extendGoal();
   M_RRT::StepResult extendRandom();

   double heuristicCost( Eigen::VectorXd node );
   M_RRT::StepResult tryStepGoal( const Eigen::VectorXd &qtry, int NNidx );
   int minHeuristicIndex();
};

#endif /** M_RRT_H */
