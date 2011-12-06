/**
 * @file HopePlanner.h
 * @brief 
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

#include <distance_field/voxel_grid.h>
#include <distance_field/pf_distance_field.h>
#include "myFunctions/Dijkstra.h"

using namespace distance_field;

/**
 * @class HopePlanner
 * @brief My testing planner.
 */
class HopePlanner {

public: 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //-- Environment info
    planning::World *mWorld;
    Collision *mCollision;

    //-- Robot specific info
    int mRobotId;
    int mEEId; /**< End Effector ID */
    kinematics::BodyNode *mEENode;
    Eigen::VectorXi mLinks;

	//-- Planner info
    double mStepSize;
    double mThreshold;
    double mStep_XYZ;
    double mThresh_XYZ;
    double mThresh_RPY;
    double mStep_RPY; 
    int mMaxIter;	

    //-- Output
    std::list<Eigen::VectorXd> mPath;
 
    //-- Constructor
    HopePlanner();
    HopePlanner( planning::World &_world,
				 Collision *_collision,
                 bool _copyWorld = false,
                 double _stepSize = 0.02 );
    //-- Destructor
    ~HopePlanner();
   
    //-- 3D Construction
    double mResolution;
    int mDefaultVoxelObject;
    double mOrigX; double mOrigY; double mOrigZ;
    double mSizeX; double mSizeY; double mSizeZ;

    //-- Kinematics stuff
    Eigen::Transform<double, 3, Eigen::Affine> msTWBase;
    Eigen::Transform<double, 3, Eigen::Affine> msTee;

    //-- Start
    Eigen::VectorXd mStartConfig;
    Eigen::VectorXd mStartPose;
	Eigen::Vector3d mStartXYZ;
	Eigen::VectorXd mStartOrient;
    Eigen::Vector3i mStartCell;

    //-- Target
    Eigen::Vector3d mTargetXYZ;
    Eigen::VectorXd mTargetPose;
    Eigen::Vector3i mTargetCell;

    //-- Time measurement
    time_t ts; time_t tf; double dt;         

    //-- Voxel variables
    VoxelGrid<int> *mVoxel;
    PFDistanceField *mPf;

    //-- Search
    Dijkstra3D mDijkstraEE;

    //-- Planner
    bool planPath( int _robotId,
                   const Eigen::VectorXi &_links,
				   const Eigen::VectorXd &_start,  // Configuration
                   const Eigen::VectorXd &_target, // Pose
                   const string &_EEName, // Name of the End Effector Body Node
                   bool _smooth = false,
                   unsigned int _maxNodes = 0 );               

	//-- Auxiliar functions
    bool ExtendHeuristic( Eigen::VectorXd _targetPose );
    double GoalDist( Eigen::VectorXd _nodeConfig, Eigen::VectorXd _targetPose );

    bool checkPathSegment( int _robotId, 
                           const Eigen::VectorXi &_links, 
                           const Eigen::VectorXd &_config1, 
                           const Eigen::VectorXd &_config2 ) const;

    void smoothPath( int _robotId, 
                     const Eigen::VectorXi &_links,
                     std::list<Eigen::VectorXd> &_path );

    //-- End Effector auxiliar functions
    Eigen::VectorXd GetEE_XYZ( const Eigen::VectorXd &_q );
    Eigen::VectorXd GetEE_RPY( const Eigen::VectorXd &_q );
    Eigen::VectorXd GetEE_Pose( const Eigen::VectorXd &_q );

    //-- Difference (error) related auxiliar functions
    Eigen::VectorXd GetDiff_XYZ( const Eigen::VectorXd &_xyz1, const Eigen::VectorXd &_xyz2 );
    Eigen::VectorXd GetDiff_RPY( const Eigen::VectorXd &_rpy1, const Eigen::VectorXd &_rpy2 ); 
    Eigen::VectorXd GetDiff_Pose( const Eigen::VectorXd &_pose1, const Eigen::VectorXd &_pose2 );

    //-- Path-related auxiliar functions
    void GetTestPath( const Eigen::VectorXd &_startConfig, const Eigen::VectorXd &_targetPose, std::list< Eigen::VectorXd > &_path );
    void GetDirectPath( const Eigen::VectorXd &_startConfig, const Eigen::VectorXd &_targetPose, std::list< Eigen::VectorXd > &_path ); 
    void GetOrientedPath( const Eigen::VectorXd &_startConfig, const Eigen::VectorXd &_targetPose, std::list< Eigen::VectorXd > &_path );

    //-- Jacobian-related auxiliar function
    Eigen::MatrixXd GetEELinearJ();
    Eigen::MatrixXd GetEEAngularJ();
    Eigen::MatrixXd GetEECompleteJ();

    //-- Collision-related auxiliar functions
    void GetCollisionWaypoints( std::list<Eigen::VectorXd> _path, std::list<Eigen::VectorXd> &_collisionPath );
    bool CheckCollisions( const Eigen::VectorXd &_config ); 

	//-- Voxelization functions
    void calculateInfo3D( VoxelGrid<int> &_voxel, PFDistanceField &_df );

    //-- Plotting functions
    void PointcloudViewer( std::vector<string> _PCDFilenames );
    string PointcloudWriter( char* _filename, int _index );
    int line_num( char* _filename );

private:
    //-- Member variables
    bool mCopyWorld;

    //-- Random value between two given min and max
    inline int randomInRange( int _min, int _max) {
        return (int) ( (double)_min + (double) ( (_max - _min) * ( (double)rand() / ((double)RAND_MAX + 1) ) ) );
    }

};

#endif /** _HOPE_PLANNER_H_ */

