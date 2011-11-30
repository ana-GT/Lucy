/**
 * @file HopePlanner.cpp
 * @brief Read the .h heading for details :)
 * @author A.H.Q.
 * @date November 21th, 2011
 */

#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "HopePlanner.h"

/**
 * @function HopePlanner
 * @brief Constructor
 */
HopePlanner::HopePlanner() {
    mCopyWorld = false;
    mWorld = NULL;
}

/**
 * @function HopePlanner
 * @brief Constructor
 */
HopePlanner::HopePlanner( planning::World &_world, 
                          Collision *_collision,
                          bool _copyWorld, double _stepSize ) {

    mCopyWorld = _copyWorld;

    if( mCopyWorld ) {
       printf( "Not implemented yet. Sorry -- achq \n" );
    } else {
        mWorld = &_world;
    }

    mCollision = _collision;
    mStepSize = _stepSize;

    mStep_XYZ = 0.02;
    mThresh_XYZ = 0.02;
    mMaxIter = 1000;
    mThresh_RPY = 0.1;
    mStep_RPY = 0.02; // 2 deg +/-

}

/**
 * @function ~HopePlanner
 * @brief Destructor
 */
HopePlanner::~HopePlanner() {

    if( mCopyWorld ) {
        delete mWorld;
    }
}

/**
 * @function planPath
 * @brief Main function
 */
bool HopePlanner::planPath( int _robotId, 
                            const Eigen::VectorXi &_links, 
                            const Eigen::VectorXd &_start,  // Config (nx1)
                            const Eigen::VectorXd &_target, // Pose (6x1)
                            const string &_EEName, 
                            bool _smooth, 
                            unsigned int _maxNodes ) {

    if( _maxNodes <= 0 ) {
        printf("--(x) Set maxNodes to some value, you moron! (x)--\n"); return false;
    } 

    /// Store information
    mRobotId = _robotId;
    mEENode = mWorld->mRobots[mRobotId]->getNode( _EEName.c_str() );
    mEEId = mEENode->getSkelIndex();
    mLinks = _links;
 
    /// Check start position is not in collision
    mWorld->mRobots[_robotId]->setDofs( _start, mLinks );

    if( mCollision->CheckCollisions() ) {   
      printf(" --(!) Initial status is in collision. I am NOT proceeding. Exiting \n");
      return false; 
    }

    /// Get a direct path 
   GetDirectPath( _start, _target, mPath );
   //GetOrientedPath( _start, _target, mPath );
   //GetTestPath( _start, _target, mPath );

    bool result = (mPath.size() != 0);
    return result;
}


/**
 * @function checkPathSegment
 * @brief True iff collision-free
 */
bool HopePlanner::checkPathSegment( int _robotId, 
                                    const Eigen::VectorXi &_links, 
                                    const Eigen::VectorXd &_config1, 
                                    const Eigen::VectorXd &_config2 ) const {

    int n = (int)((_config2 - _config1).norm() / mStepSize );

    for( int i = 0; i < n; i++ ) {
        Eigen::VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
        mWorld->mRobots[_robotId]->setDofs( conf, _links );
	      if( mCollision->CheckCollisions() ) {
	          return false;
	      }
    }

    return true;
}

/**
 * @function smoothPath
 */
void HopePlanner::smoothPath( int _robotId, 
                             const Eigen::VectorXi &_links, 
                             std::list<Eigen::VectorXd> &_path ) {

    std::list<Eigen::VectorXd>::iterator config1, config2;
    std::list<Eigen::VectorXd>::iterator temp = _path.begin();

    if( temp == _path.end() ) return;

    while(true) {
        config1 = temp;
	      temp++;
 	      if(temp == _path.end()) return;
	      config2 = temp;
	      config2++;
	      if(config2 == _path.end()) return;
		
	      while( checkPathSegment( _robotId, _links, *config1, *config2) ) {
	          _path.erase( temp );
	          temp = config2;
	          config2++;
	          if( config2 == _path.end() ) return;
	      }
    }

}

/**
 * @function GetEE_XYZ
 */
Eigen::VectorXd HopePlanner::GetEE_XYZ( const Eigen::VectorXd &_q ) {

    mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );
    mWorld->mRobots[mRobotId]->update();
    Eigen::MatrixXd pose = mEENode->getWorldTransform(); 
    Eigen::VectorXd xyz(3); xyz << pose(0,3), pose(1,3), pose(2,3);

    return xyz;
}

/**
 * @function GetEE_RPY
 */
Eigen::VectorXd HopePlanner::GetEE_RPY( const Eigen::VectorXd &_q ) {

  mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );
  mWorld->mRobots[mRobotId]->update();

  double roll, pitch, yaw;

  Eigen::Matrix4d tf = mEENode->getWorldTransform();

	roll = atan2( tf(2,1), tf(2,2) );
	pitch = -asin( tf(2,0) );
	yaw = atan2( tf(1,0), tf(0,0) );  

  Eigen::VectorXd rpy(3); rpy << roll, pitch, yaw;

    return rpy;
}

/**
 * @function GetEE_Pose
 */
Eigen::VectorXd HopePlanner::GetEE_Pose( const Eigen::VectorXd &_q ) {

    Eigen::VectorXd EEpose(6);

    mWorld->mRobots[mRobotId]->setDofs( _q, mLinks );
    mWorld->mRobots[mRobotId]->update();
    Eigen::MatrixXd tf = mEENode->getWorldTransform(); 

   	double roll = atan2( tf(2,1), tf(2,2) );
	  double pitch = -asin( tf(2,0) );
	  double yaw = atan2( tf(1,0), tf(0,0) );  

    EEpose(0) = tf(0,3); EEpose(1) = tf(1,3); EEpose(2) = tf(2,3);
    EEpose(3) = roll; EEpose(4) = pitch; EEpose(5) = yaw;

    return EEpose;
}

/**
 * @function GetDiff_XYZ
 */
Eigen::VectorXd HopePlanner::GetDiff_XYZ( const Eigen::VectorXd &_xyz1, 
																					const Eigen::VectorXd &_xyz2 ) {
    return ( _xyz2 - _xyz1 );
}

/**
 * @function GetDiff_RPY
 */
Eigen::VectorXd HopePlanner::GetDiff_RPY( const Eigen::VectorXd &_rpy1, 
																					const Eigen::VectorXd &_rpy2 ) {

    return ( _rpy2 - _rpy1 );
}

/**
 * @function GetDiff_Pose
 */
Eigen::VectorXd HopePlanner::GetDiff_Pose( const Eigen::VectorXd &_pose1, 
																					 const Eigen::VectorXd &_pose2 ) {

    return ( _pose2 - _pose1 );
}

/**
 * @function GetTestPath
 * @brief Testing ideas
 */
void HopePlanner::GetTestPath( const Eigen::VectorXd &_startConfig, 
															 const Eigen::VectorXd &_targetPose, 
															 std::list< Eigen::VectorXd > &_path ) {

    /// 1. Start FK
    Eigen::VectorXd curr_pose = GetEE_Pose( _startConfig );
    Eigen::VectorXd gap_pose = GetDiff_Pose( curr_pose, _targetPose );   
    Eigen::VectorXd gap_xyz = gap_pose.head(3);
    Eigen::VectorXd gap_rpy = gap_pose.tail(3);

    /// 2. Loop
    Eigen::VectorXd dpose(6), drpy, dxyz;
    Eigen::VectorXd q; 
    Eigen::VectorXd dq;

    q = _startConfig;
    _path.push_back(q);
    mWorld->mRobots[mRobotId]->setDofs( q, mLinks );
    mWorld->mRobots[mRobotId]->update();

    int iters = 0;
    while( gap_xyz.norm() > mThresh_XYZ &&  iters < mMaxIter ) {

        dxyz = ( gap_xyz / gap_xyz.norm() )*mStep_XYZ;
        drpy = ( gap_rpy / gap_rpy.norm() )*mStep_RPY;
        dpose.head(3) = dxyz; dpose.tail(3) = drpy;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd( GetEECompleteJ(), Eigen::ComputeThinU | Eigen::ComputeThinV );
        dq = svd.solve( dpose ); 

        q = q + dq;
        std::cout << q.transpose() << std::endl;
        mWorld->mRobots[mRobotId]->setDofs( q, mLinks );
        mWorld->mRobots[mRobotId]->update();

        _path.push_back(q);

        curr_pose = GetEE_Pose( q );
        gap_pose = GetDiff_Pose( curr_pose, _targetPose );  
        gap_xyz = gap_pose.head(3);
        gap_rpy = gap_pose.tail(3);

        iters++;   
        if( iters % 100 == 0 )   
        { printf("iters: %d gap: %f \n", iters, gap_xyz.norm());}
    }

    if( iters == mMaxIter ) {
        printf( "--> Did not reach the goal. Last gap: %.3f \n", gap_xyz.norm() );

        if( _path.size() != 0 ) { 
          _path.clear(); 
        }
    }
    else {
        printf("Found path! Last gap: %.3f Size of path: %d  \n", gap_xyz.norm(), _path.size() );
    }

}

/**
 * @function GetDirectPath
 */
void HopePlanner::GetDirectPath( const Eigen::VectorXd &_startConfig, const Eigen::VectorXd &_targetPose, std::list< Eigen::VectorXd > &_path ) {
  
    /// 0. Get translation part of _targetPose (x,y,z,r,p,y)
    Eigen::VectorXd target_xyz(3); target_xyz << _targetPose(0), _targetPose(1), _targetPose(2);   

    /// 1. Start FK
    Eigen::VectorXd curr_xyz = GetEE_XYZ( _startConfig );
    Eigen::VectorXd gap_xyz = GetDiff_XYZ( curr_xyz, target_xyz );   

    /// 2. Loop
    Eigen::VectorXd dxyz;
    Eigen::VectorXd q; 
    Eigen::VectorXd dq;

    q = _startConfig;
    mWorld->mRobots[mRobotId]->setDofs( q, mLinks );
    mWorld->mRobots[mRobotId]->update();

    int iters = 0;
    printf( "--> Start gap: %.3f \n", gap_xyz.norm() );
    while( gap_xyz.norm() > mThresh_XYZ &&  iters < mMaxIter ) {

        dxyz = ( gap_xyz / gap_xyz.norm() )*mStep_XYZ;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd( GetEELinearJ(), Eigen::ComputeThinU | Eigen::ComputeThinV );
        dq = svd.solve( dxyz ); 

        q = q + dq; 
        mWorld->mRobots[mRobotId]->setDofs( q, mLinks );
        mWorld->mRobots[mRobotId]->update();

        _path.push_back(q);

        curr_xyz = GetEE_XYZ( q );
        gap_xyz = GetDiff_XYZ( curr_xyz, target_xyz );  
  
        iters++;   
        if( iters % 100 == 0 )   
        { printf("iters: %d gap: %f \n", iters, gap_xyz.norm());}
    }

    if( iters == mMaxIter ) {
        printf( "--> Did not reach the goal. Last gap: %.3f \n", gap_xyz.norm() );

        if( _path.size() != 0 ) { 
          _path.clear(); 
        }
    }

    else {
        printf("Yeah you made it baby! Last gap: %.3f Size of path: %d  \n", gap_xyz.norm(), _path.size() );
    }

}


/**
 * @function GetOrientedPath
 */
void HopePlanner::GetOrientedPath( const Eigen::VectorXd &_startConfig, 
                                   const Eigen::VectorXd &_targetPose, 
                                   std::list< Eigen::VectorXd > &_path ) {
  
    /// 0. Get translation part of _targetPose (x,y,z,r,p,y)
    Eigen::VectorXd target_rpy(3); target_rpy << _targetPose(3), _targetPose(4), _targetPose(5);   

    /// 1. Start FK
    Eigen::VectorXd curr_rpy = GetEE_RPY( _startConfig );
    Eigen::VectorXd gap_rpy = GetDiff_RPY( curr_rpy, target_rpy );   

    /// 2. Loop
    Eigen::VectorXd drpy;
    Eigen::VectorXd q; Eigen::VectorXd dq;

    q = _startConfig;
    mWorld->mRobots[mRobotId]->setDofs( q, mLinks );
    mWorld->mRobots[mRobotId]->update();

    int iters = 0;
    printf( "--> Start gap: %.3f \n", gap_rpy.norm() );
    while( gap_rpy.norm() > mThresh_RPY &&  iters < mMaxIter ) {
        drpy = ( gap_rpy / gap_rpy.norm() )*mStep_RPY;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd( GetEEAngularJ(), Eigen::ComputeThinU | Eigen::ComputeThinV );
        dq = svd.solve( drpy ); 
        q = q + dq; 
        mWorld->mRobots[mRobotId]->setDofs( q, mLinks );
        mWorld->mRobots[mRobotId]->update();

        _path.push_back(q);

        curr_rpy = GetEE_RPY( q );
        gap_rpy = GetDiff_RPY( curr_rpy, target_rpy );  
  
        iters++;   
        if( iters % 100 == 0 )   
        { printf("iters: %d gap rpy: %f, %f, %f \n", iters, gap_rpy(0), gap_rpy(1), gap_rpy(2) ); }
    }

    if( iters == mMaxIter ) {
        printf( "--> Did not reach the goal. Last gap: %f, %f, %f \n", gap_rpy(0), gap_rpy(1), gap_rpy(2) );

        if( _path.size() != 0 ) { 
          _path.clear(); 
        }
    }

    else {
        printf("Yeah you made it baby! Last gap:  %f, %f, %f Size of path: %d  \n",gap_rpy(0), gap_rpy(1), gap_rpy(2), _path.size() );
    }

}


/**
 * @function GetEELinearJ
 */
Eigen::MatrixXd HopePlanner::GetEELinearJ() {
    return mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
}

/**
 * @function GetEEAngularJ
 * @brief Looks like the Jacobian for roll pitch and yaw. Interesting
 */
Eigen::MatrixXd HopePlanner::GetEEAngularJ() {
    return mEENode->getJacobianAngular().topRightCorner( 3, mLinks.size() );
}

/**
 * @function GetEECompleteJ
 * @brief Get the Jacobian (angular and linear)
 */
Eigen::MatrixXd HopePlanner::GetEECompleteJ() {
 
    Eigen::MatrixXd completeJacobian( 6, mLinks.size() );   
    completeJacobian.topLeftCorner( 3, mLinks.size() ) = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
    completeJacobian.bottomLeftCorner( 3, mLinks.size() ) = mEENode->getJacobianAngular().topRightCorner( 3, mLinks.size() );
    return completeJacobian;
}

/**
 * @function GetCollisionWaypoints
 */
void HopePlanner::GetCollisionWaypoints( std::list<Eigen::VectorXd> _path, 
																				 std::list<Eigen::VectorXd> &_collisionPath ) {

    _collisionPath.clear();

    for( std::list<Eigen::VectorXd>::iterator it = _path.begin(); it != _path.end(); it++ ) {
        Eigen::VectorXd config = *it;
        if( CheckCollisions( config ) == true ) {
           _collisionPath.push_back( config );  
        }
    }
   
}

/**
 * @function checkCollisions
 */
bool HopePlanner::CheckCollisions( const Eigen::VectorXd &_config ) {
    mWorld->mRobots[mRobotId]->setDofs( _config, mLinks );
    mWorld->mRobots[mRobotId]->update();
    return mCollision->CheckCollisions();
}
