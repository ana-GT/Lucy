/**
 * @file HopePlanner.h
 * @brief Header for Bertram's RRT modification
 * @author A.H.Q.
 * @date November 14th, 2011
 */

#ifndef _HOPE_PLANNER_H_
#define _HOPE_PLANNER_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <vector>
#include <list>
#include <planning/World.h>
#include <kinematics/BodyNode.h>
#include <Tools/Collision.h>

/**
 * @class HopePlanner
 * @brief Class that implements Bertram's RRT planner
 */
class HopePlanner {

public:

    /// Member variables
    double mStepSize;
    planning::World *mWorld;
    Collision *mCollision;
    std::list<Eigen::VectorXd> mPath;
    int mRobotId;
    int mEEId; /**< End Effector ID */
    kinematics::BodyNode *mEENode;

    Eigen::VectorXi mLinks;
    double mThreshold;

    double mStep_XYZ;
    double mThresh_XYZ;
    double mThresh_RPY;
    double mStep_RPY; 
    int mMaxIter;

    /// Constructor
    HopePlanner();
    HopePlanner( planning::World &_world,
						   Collision *_collision,
               bool _copyWorld = false,
               double _stepSize = 0.02 );

    /// Destructor
    ~HopePlanner();

    /// Planner itself
    bool planPath( int _robotId,
                   const Eigen::VectorXi &_links,
								   const Eigen::VectorXd &_start,  // Configuration
                   const Eigen::VectorXd &_target, // Pose
                   const string &_EEName, // Name of the End Effector Body Node
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

    /// Functions added
    Eigen::VectorXd GetEE_XYZ( const Eigen::VectorXd &_q );
    Eigen::VectorXd GetEE_RPY( const Eigen::VectorXd &_q );
    Eigen::VectorXd GetEE_Pose( const Eigen::VectorXd &_q );

    Eigen::VectorXd GetDiff_XYZ( const Eigen::VectorXd &_xyz1, const Eigen::VectorXd &_xyz2 );
    Eigen::VectorXd GetDiff_RPY( const Eigen::VectorXd &_rpy1, const Eigen::VectorXd &_rpy2 ); 
    Eigen::VectorXd GetDiff_Pose( const Eigen::VectorXd &_pose1, const Eigen::VectorXd &_pose2 );

    void GetTestPath( const Eigen::VectorXd &_startConfig, const Eigen::VectorXd &_targetPose, std::list< Eigen::VectorXd > &_path );
    void GetDirectPath( const Eigen::VectorXd &_startConfig, const Eigen::VectorXd &_targetPose, std::list< Eigen::VectorXd > &_path ); 
    void GetOrientedPath( const Eigen::VectorXd &_startConfig, const Eigen::VectorXd &_targetPose, std::list< Eigen::VectorXd > &_path );
    Eigen::MatrixXd GetEELinearJ();
    Eigen::MatrixXd GetEEAngularJ();
    Eigen::MatrixXd GetEECompleteJ();

    void GetCollisionWaypoints( std::list<Eigen::VectorXd> _path, std::list<Eigen::VectorXd> &_collisionPath );
    bool CheckCollisions( const Eigen::VectorXd &_config ); 

private:
    /// Member variables
    bool mCopyWorld;

    /// Random value between two given min and max
    inline int randomInRange( int _min, int _max) {
        return (int) ( (double)_min + (double) ( (_max - _min) * ( (double)rand() / ((double)RAND_MAX + 1) ) ) );
    }

};

#endif /** _HOPE_PLANNER_H_ */
