/**
 * @file B1Planner.h
 * @brief Header for Bertram's RRT modification
 * @author A.H.Q.
 * @date November 14th, 2011
 */

/*
@inproceedings{Bertram_2006_5436,
   author = "Dominik Bertram and James Kuffner and Ruediger Dillmann and Tamim Asfour",
   title = "An Integrated Approach to Inverse Kinematics and Path Planning for Redundant Manipulators",
   booktitle = "Proceedings of the IEEE International Conference on Robotics and Automation",
   pages = "1874-1879",
   publisher = "IEEE",
   month = "May",
   year = "2006",
}
*/

#ifndef _B1_PLANNER_H_
#define _B1_PLANNER_H_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <planning/World.h>
#include <Tools/Collision.h>
#include "B1RRT.h"

/**
 * @class B1Planner
 * @brief Class that implements Bertram's RRT planner
 */
class B1Planner {

public:

    /// Member variables
    double mStepSize;
    planning::World *mWorld;
    Collision *mCollision;
    std::list<Eigen::VectorXd> mPath;
    B1RRT *mRrt;
    int mRobotId;
    Eigen::VectorXi mLinks;
    double mThreshold;

    /// Constructor
    B1Planner();
    B1Planner( planning::World &_world,
						   Collision *_collision,
               bool _copyWorld = false,
               double _stepSize = 0.02 );

    /// Destructor
    ~B1Planner();

    /// Planner itself
    bool planPath( int _robotId,
                   const Eigen::VectorXi &_links,
								   const Eigen::VectorXd &_start,  // Configuration
                   const Eigen::VectorXd &_target, // Pose
                   bool _smooth = false,
                   unsigned int _maxNodes = 0 );               

    bool ExtendHeuristic( Eigen::VectorXd _targetPose );
    double GoalDist( Eigen::VectorXd _nodeConfig, Eigen::VectorXd _targetPose );

    bool checkPathSegment( int _robotId, 
                           const Eigen::VectorXi &_links, 
                           const Eigen::VectorXd &_config1, 
                           const Eigen::VectorXd &_config2 ) const;

    void smoothPath( int _robotId, 
                     const Eigen::VectorXi &_links,
                     std::list<Eigen::VectorXd> &_path );

private:
    /// Member variables
    bool mCopyWorld;

    /// Random value between two given min and max
    inline int randomInRange( int _min, int _max) {
        return (int) ( (double)_min + (double) ( (_max - _min) * ( (double)rand() / ((double)RAND_MAX + 1) ) ) );
    }

};

#endif /** _B1_PLANNER_H_ */

