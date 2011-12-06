/**
 * @file search.h
 */

#ifndef _SEARCH_
#define _SEARCH_

#include <vector>
#include "armState.h"

class Search
{
  public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Search();
  ~Search();
  static void setPrimitives();
  void initialize( Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                   Eigen::Transform<double, 3, Eigen::Affine> _Tee  );
  bool plan( Eigen::VectorXd _start_joints, 
	     Eigen::Transform<double, 3, Eigen::Affine> _goal_pose,
             std::vector< Eigen::VectorXd > & path );

  static const double sThreshold;
  static const int sNumJoints;
  static const int sNumPrimitives;
  static const double sDeltaJoint;
  static const int sMaxIter;
  static std::vector< Eigen::VectorXd >sMotionPrimitives;

  private:

  Eigen::Vector3d mGoalPos;
  Eigen::Transform<double, 3, Eigen::Affine> mGoalPose;
  std::vector< armState > states;
  std::vector< int > openSet;

  //-- Utility functions
  void cleanup();
  std::vector< armState > getNextStates( const armState &x );
  double edgeCost( const int &key1, const int &key2 );
  double getWSError( Eigen::Vector3d nodePos, Eigen::Vector3d goalPos ); 
  double costHeuristic( const int &key, const Eigen::Transform<double, 3, Eigen::Affine> &goalPose );
  bool tracePath( const int &key, std::vector< Eigen::VectorXd> & path );

  //-- Heap functions
  void pushOpenSet( int _key );
  int popOpenSet();
  void updateLowerOpenSet( int key );
};


/*--------------------------------------------*/
/**
 * @function setPrimitives
 */
inline void Search::setPrimitives()
{
  Search::sMotionPrimitives.resize(0);

  Eigen::VectorXd mp( sNumJoints );
  mp = Eigen::VectorXd::Zero( sNumJoints );

  for( int i = 0; i < sNumJoints-1; i++ ) 
  {  for( int j = -1; j <= 1; j+= 2 )
     { mp(i) = j*sDeltaJoint;
       if( i > 0 ) 
       { mp( i - 1 ) = 0; }

       Search::sMotionPrimitives.push_back( mp );
     }  
  } 
}

/**
 * @function getWSError
 * @brief Calculate the workspace error (distance 3D) between two 3D positions
 */
inline double Search::getWSError( Eigen::Vector3d nodePos, Eigen::Vector3d goalPos )
{
   return ( goalPos - nodePos ).norm();
}

/**
 * @function costHeuristic
 * @brief Approximate the cost to the goal pose
 */
inline double Search::costHeuristic( const int &key, const Eigen::Transform<double, 3, Eigen::Affine> &goalPose )
{
   return( (goalPose.translation() - states[key].xyz ).norm() );
}


/**
 * @function edgeCost
 * @brief Calculate the edge cost between two consecutive states
 */      
inline double Search::edgeCost( const int &key1, const int &key2 )
{
   double distWS = ( states[key1].xyz - states[key2].xyz ).norm();
   return distWS;
}

#endif /** _SEARCH_ */



