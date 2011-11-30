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

#ifndef _RRT_PLANNER_
#define _RRT_PLANNER_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <planning/World.h>
#include <Tools/Collision.h>
#include "RRT.h"

/**
 * @class RRTPlanner
 * @brief Basic Planner RRT-based
 */
class RRTPlanner {

public:

    /// Member variables
    double stepSize;
    planning::World* world;
    Collision * collision;
    std::list<Eigen::VectorXd> path;
    

    /// Constructor
    RRTPlanner();
    RRTPlanner( planning::World &_world,
                 Collision *_collision, 
                 bool _copyWorld = false, 
                 double _stepSize = 0.02 );
    /// Destructor
    ~RRTPlanner();  

    /// planPath
    bool planPath( int _robotId, 
                   const Eigen::VectorXi &_links, 
                   const Eigen::VectorXd &_start, 
                   const Eigen::VectorXd &_goal, 
                   bool _bidirectional = true, 
                   bool _connect = true, 
                   bool _greedy = false,
                   bool _smooth = true, 
                   unsigned int _maxNodes = 0 );

    bool checkPathSegment( int _robotId, 
                           const Eigen::VectorXi &_links, 
                           const Eigen::VectorXd &_config1, 
                           const Eigen::VectorXd &_config2 ) const;

    void smoothPath( int _robotId, 
                     const Eigen::VectorXi &_links,
                     std::list<Eigen::VectorXd> &_path );
    void smoothPath2( int _robotId, 
                      const Eigen::VectorXi &_links, 
                      std::list<Eigen::VectorXd> &_path );

private:
    /// Member variables
    bool copyWorld;

    /// Finds a plan using a standard RRT
    bool planSingleTreeRrt( int _robotId, 
                            const Eigen::VectorXi &_links, 
                            const Eigen::VectorXd &_start, 
                            const Eigen::VectorXd &_goal, 
                            bool _connect, 
                            bool _greedy,
                            unsigned int _maxNodes );

    /// Grows 2 RRT (Start and Goal)
    bool planBidirectionalRrt( int _robotId, 
                               const Eigen::VectorXi &_links, 
                               const Eigen::VectorXd &_start, 
                               const Eigen::VectorXd &_goal,  
                               bool _connect, 
		               						 bool _greedy,
                               unsigned int _maxNodes );

    /// Random value between two given min and max
    inline int randomInRange( int _min, int _max) {
        return (int) ( (double)_min + (double) ( (_max - _min) * ( (double)rand() / ((double)RAND_MAX + 1) ) ) );
    }

};

#endif /** _RRT_PLANNER */
