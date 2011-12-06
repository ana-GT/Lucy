/**
 * @file armState
 */
#include "armState.h"

/** Static member variables */
Eigen::Transform< double, 3, Eigen::Affine > armState::Tee_ ;
Eigen::Transform< double, 3, Eigen::Affine > armState::TWBase_;
double armState::MAX_COST = DBL_MAX;

/**
 * @function Constructor
 */
armState::armState()
{
}

/**
 * @function Destructor
 */
armState::~armState()
{
}

/**
 * @function initialize
 */
void armState::initialize( Eigen::VectorXd _joints, int _key )
{
   //-- Set joints
   joints = _joints;
   //-- Set key
   key = _key;
   //-- Calculate xyz and rpy
   calculate_pose();
   //-- Set default parent
   parent = -1;
   //-- Set status
   status = IN_NO_SET;
   //-- Set costs
   costF = MAX_COST;
   costG = MAX_COST;
   costH = MAX_COST;
   
}

/**
 * @function calculate_pose
 */
void armState::calculate_pose()
{
  Eigen::Transform< double, 3, Eigen::Affine > T;
  robinaLeftArm_fk( joints, armState::TWBase_, armState::Tee_, T );  

  xyz = T.translation();
  rpy = get_rpy( T ); 
}



