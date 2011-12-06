/**
 * @file goWSPos.cpp
 * @brief 
 */

#include "goWSPos.h"

/**
 * @function goWSPos
 * @brief Constructor
 */
goWSPos::goWSPos()
{
  this->stepSize = 0.0025; // 0.1 degrees
  this->activeNode = 0; 
  this->maxTrials = 1000;
  this->goalThresh = 0.0025;
}


/**
 * @function initialize
 * @brief Initialize some parameters
 */
void goWSPos::initialize( World* _world,
                          int _robot_ID,
                          std::vector<int> _links_ID,
                          Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                          Eigen::Transform<double, 3, Eigen::Affine> _Tee )
{
  this->world = _world;
  this->robot_ID = _robot_ID;
  this->links_ID = _links_ID;
  this->ndim = _links_ID.size();
  this->TWBase = _TWBase;
  this->Tee = _Tee;

  cleanup();
}

/**
 * @function cleanup
 * @brief Cleanup
 */
void goWSPos::cleanup()
{
  configVector.clear(); 
  parentVector.clear(); 

  startConfig.resize(0);
  goalPosition.resize(0);
}
  
  

/**
 * @function straightPath
 * @brief Builds a straight path -- easy
 */
bool goWSPos::straightPath( const Eigen::VectorXd &_startConfig, 
		     	    const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose,
	          	    std::vector<Eigen::VectorXd> &path )
{
  //-- Copy input 
  startConfig = _startConfig;
  goalPose = _goalPose;
  goalPosition = _goalPose.translation();

  //-- Create needed variables
  Eigen::VectorXd q; Eigen::VectorXd wsPos;
  Eigen::Transform<double, 3, Eigen::Affine> T;
  Eigen::MatrixXd J(3, 7); Eigen::MatrixXd Jinv(7, 3);
  Eigen::MatrixXd delta_x; Eigen::MatrixXd delta_q;
  double ws_dist;

  //-- Initialize variables
  q = startConfig; path.push_back( q );
  ws_dist = DBL_MAX;

  //-- Loop
  int trials = 0;
  while( ws_dist > goalThresh )
  {
    //-- Get ws position
    robinaLeftArm_fk( q, TWBase, Tee, T );
    wsPos = T.translation();

    //-- Get the Jacobian
    robinaLeftArm_j( q, TWBase, Tee, J );

    //-- Get the pseudo-inverse(easy way)
    pseudoInv( 3, 7, J, Jinv );

    //-- Get delta_x (workspace)
    delta_x = ( goalPosition - wsPos );
    
    //-- Get delta_q (jointspace)   
    delta_q = Jinv*delta_x;

    //-- Scale
    double scal = stepSize/delta_q.norm();
    delta_q = delta_q*scal;

    //-- Add to q
    q = q + delta_q; 

    //-- Push it 
    path.push_back( q );

    //-- Check distance to goal
    ws_dist = wsDistance( q );
    trials++;      
    
    if( trials > maxTrials )
     { break; }
  }  

  if( ws_dist < goalThresh )
   { printf("Got the goal after %d trials - wsdist: %.3f \n", trials, ws_dist); 
     return true;
   }
  else
   { path.clear();
     printf(" (!)-- Did not get to the goal after %d trials - wsdist: %.3f \n", trials, ws_dist );
     return false; 
   }
   
}

/**
 * @function getWSConfig
 * @brief Gets a jointspace config from a near one and a workspace goal location. 
 */
Eigen::VectorXd goWSPos::getWSConfig( const Eigen::VectorXd &_nearConfig, 
	  	     	              const Eigen::VectorXd &_goalConfig,
	          	              double delta )
{
  //-- Copy input 
  this->goalThresh = delta;
  this->startConfig = _nearConfig;
  this->goalPosition = _goalConfig;

  //-- Create needed variables
  Eigen::VectorXd q; Eigen::VectorXd wsPos;
  Eigen::Transform<double, 3, Eigen::Affine> T;
  Eigen::MatrixXd J(3, 7); Eigen::MatrixXd Jinv(7, 3);
  Eigen::MatrixXd delta_x; Eigen::MatrixXd delta_q;
  double ws_dist;

  //-- Initialize variables
  Eigen::VectorXd def( startConfig.size() );
  for( int i = 0; i < startConfig.size(); i++ )
   { def[i] = -10;}

  q = startConfig;
  ws_dist = DBL_MAX;

  //-- Loop
  int trials = 0;
  while( ws_dist > goalThresh )
  {
    //-- Get ws position
    robinaLeftArm_fk( q, TWBase, Tee, T );
    wsPos = T.translation();

    //-- Get the Jacobian
    robinaLeftArm_j( q, TWBase, Tee, J );

    //-- Get the pseudo-inverse(easy way)
    pseudoInv( 3, 7, J, Jinv );
  
    //-- Get delta_x (workspace)
    delta_x = ( goalPosition - wsPos );
    
    //-- Get delta_q (jointspace)   
    delta_q = Jinv*delta_x;

    //-- Scale
    double scal = stepSize/delta_q.norm();
    delta_q = delta_q*scal;

    //-- Add to q
    q = q + delta_q; 

    //-- Check distance to goal
    ws_dist = wsDistance( q );
    trials++;      
    if( trials > maxTrials )
     { break; }
  }  

  if( ws_dist < goalThresh )
   { return q;}
  else
   { printf( "(!)--getWSConfig : Did not find direct point \n");return def; }
   
}


/**
 * @function wsDistance
 */
double goWSPos::wsDistance( Eigen::VectorXd q )
{
  Eigen::Transform<double, 3, Eigen::Affine> T;
  robinaLeftArm_fk( q, TWBase, Tee, T );
  Eigen::VectorXd ws_diff = ( goalPosition - T.translation() );
  double ws_dist = ws_diff.norm();
    
  return ws_dist;
}



