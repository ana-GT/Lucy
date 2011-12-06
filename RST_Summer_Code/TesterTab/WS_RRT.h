/**
 * @file WS_RRT.h
 * @author achq
 */

#ifndef WS_RRT_H
#define WS_RRT_H

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

#define RANDNM(N,M) N + ((M-N) * ((double)rand() / ((double)RAND_MAX + 1)))

/**
 * @class WS_RRT
 * @brief 
 */
class WS_RRT {

  public: 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //-- Constructor 
    WS_RRT();
    //-- Destructor 
    ~WS_RRT();

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
    double js_stepSize;
    double ws_distThresh;
    Eigen::VectorXd ws_win_low;
    Eigen::VectorXd ws_win_high;
    int activeNode;
    std::vector<int> parentVector;
    std::vector<Eigen::VectorXd> configVector;
    std::vector<Eigen::VectorXd> wsVector;
    std::vector<double> distGoalVector;
    std::vector<int> rankingVector;
    struct kdtree *kdTree;

    Eigen::VectorXd startConfig;
    Eigen::Transform<double, 3, Eigen::Affine> goalPose;
    Eigen::VectorXd goalPosition;
    Eigen::Transform<double, 3, Eigen::Affine> TWBase; 
    Eigen::Transform<double, 3, Eigen::Affine> Tee;

    double p_goal;
    int maxNodes;
    int max_calls;
   
    bool connect( );
    bool connect( const Eigen::VectorXd &ws_target );
    bool connect( const Eigen::VectorXd &ws_target, int NN_ID );
    bool connect_goal();

    StepResult tryStep( const Eigen::VectorXd &ws_target, int NN_ID );

    int addNode( const Eigen::VectorXd &js_node, int parent_ID );
    virtual Eigen::VectorXd getRandom_wsPos();
    virtual Eigen::VectorXd getRandom_wsPos( int center, double delta );
    int getNearestNeighbor( const Eigen::VectorXd &ws_sample );

    void tracePath( int node_ID, std::vector<Eigen::VectorXd> &path, bool reverse = false );
    virtual bool checkCollisions( const Eigen::VectorXd &q );

    int pop_Ranking();
    void add_Ranking( int node_ID );

   //-- Initialize function
   void initialize( World* _world, 
		    const int &_robot_ID, 
                    const std::vector<int> &_links_ID,
                    const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase,
                    const Eigen::Transform<double, 3, Eigen::Affine> &_Tee );

   //-- Plan function
   bool plan( const Eigen::VectorXd &_startConfig, 
	      const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose, 
              std::vector<Eigen::VectorXd> &path );

   //-- Utility functions
   virtual void cleanup();
   double wsDist( int ID, Eigen::VectorXd ws_pos );
   Eigen::VectorXd wsDiff( int ID, Eigen::VectorXd ws_pos );
   Eigen::VectorXd js_wsPos( Eigen::VectorXd js_node );

};

#endif /** WS_RRT_H */
