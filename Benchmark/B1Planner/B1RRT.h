/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 * @file RRT.cpp
 *
 * Authors:  Ana Huaman <ahuaman3@gatech.edu>, Tobias Kunz <tobias@gatech.edu>
 * Date: 10/2011
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 */

#ifndef _B1_RRT_H_
#define _B1_RRT_H_

#include <vector>
#include <list>
#include <time.h>
#include <Eigen/Core>
#include <Tools/Collision.h>
#include <planning/World.h>
#include "../kdtree/kdtree.h"


/**
 * @class RRT
 */
class B1RRT {

public:

    typedef enum {
      STEP_COLLISION, /**< Collided with obstacle. No added */
      STEP_REACHED,    /**< The configuration that we grow to is less than stepSize away from node we grow from. No node added */
      STEP_PROGRESS    /**< One node added */
    }  StepResult;

    /// Member variables
    planning::World* world;
    Collision *collision;
    int robotId;
    Eigen::VectorXi links;
    int EEId; /**< End Effector ID */

    int ndim;
    double stepSize;
  
    int activeNode;
    std::vector<int> parentVector; /**< Vector of indices to relate configs in RRT */
    std::vector<Eigen::VectorXd> configVector; /**< Vector of all visited configs */

    std::vector<Eigen::VectorXd> ranking;          
    Eigen::VectorXd targetPose; /**< Target pose for the End Effector */

    struct kdtree *kdTree;

    /// Constructor
    B1RRT( planning::World* _world,
           Collision *_collision, 
           int _robotId, 
           const Eigen::VectorXi &_links, 
           const Eigen::VectorXd &_root, 
           const Eigen::VectorXd &_targetPose,
           double _stepSize = 0.02 );

    /// Destructor
    virtual ~B1RRT();

    /// Added functions 
    bool ExtendHeuristic();
    double GoalDist( int _nodeId, Eigen::VectorXd _targetPose );
    double GoalDist( Eigen::VectorXd _nodeConfig, Eigen::VectorXd _targetPose );

    void removeRanking( int _ind );
    int popRanking();
    void pushRanking( const Eigen::Vector3d &_entry );
    double HeuristicCost( int _id, Eigen::VectorXd _targetPose );
    bool connectHeuristic( int _nearId, const Eigen::VectorXd &_target );

    /// Connect a random target with the closest tree node (until reaching or colliding)
    bool connect();
    /// Connect the target with the closest tree node, (until reaching or colliding )
    bool connect( const Eigen::VectorXd &_target );

    /// Try to advance one stepSize towards a random target
    StepResult tryStep();
    /// Try to advance one stepSize towards qtry
    StepResult tryStep( const Eigen::VectorXd &_qtry );

    /// Tries to extend tree towards provided sample (must be overridden for MBP ) 
    virtual StepResult tryStepFromNode( const Eigen::VectorXd &_qtry, int _NNIdx );

    /// Add qnew to tree with parent _parentId
    int addNode( const Eigen::VectorXd &_qnew, int _parentId );  

    /// Returns a random config
    virtual Eigen::VectorXd getRandomConfig();

    /// Returns Nearest Neighbor index to query point
    int getNearestNeighbor( const Eigen::VectorXd &_qsamp );

    /// Get the gap (Distance) between the closest node in the tree to the _target
    double getGap( const Eigen::VectorXd &_target );

    /// Traces the path from some node to the initConfig node
    void tracePath( int _node, std::list<Eigen::VectorXd> &_path, bool _reverse = false );

    /// Implementation-specific function for checking collisions (must be overridden for MBP )
    virtual bool checkCollisions( const Eigen::VectorXd &c );

    unsigned int getSize();

protected:
   
   /// Get random number between min and max
   double randomInRange( double _min, double _max );
     
};

#endif /** _B1_RRT_H_ */
