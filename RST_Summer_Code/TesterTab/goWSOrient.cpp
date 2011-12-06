/**
 * @file goWSOrient.cpp
 * @brief 
 */

#include "goWSOrient.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>


/**
 * @function goWSOrient
 * @brief Constructor
 */
goWSOrient::goWSOrient()
{
  this->stepSize = 0.05; // 2*sqrt(7)
  this->activeNode = 0; 
  this->maxTrials = 500;
  this->goalThresh = 0.1;
}


/**
 * @function initialize
 * @brief Initialize some parameters
 */
void goWSOrient::initialize( World* _world,
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
void goWSOrient::cleanup()
{
  configVector.clear(); 
  parentVector.clear(); 

  startConfig.resize(0);
  goalPos.resize(0);
}
  
  

/**
 * @function straightPath
 * @brief Builds a straight path -- easy
 */
bool goWSOrient::straightPath( const Eigen::VectorXd &_startConfig, 
		     	       const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose,
	          	       std::vector<Eigen::VectorXd> &path )
{
  //-- Copy input 
  startConfig = _startConfig;
  goalPose = _goalPose;
  goalPos = _goalPose.translation();

  //-- Create needed variables
  Eigen::VectorXd q; Eigen::VectorXd wsPos;
  Eigen::Transform<double, 3, Eigen::Affine> T;
  Eigen::MatrixXd Jc(6, 7); Eigen::MatrixXd Jcinv(7, 6);

  Eigen::MatrixXd delta_q;
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

    Eigen::VectorXd wsOrient(3); 
    getRPY( T, wsOrient(0), wsOrient(1), wsOrient(2) );

    //-- Get the Jacobian

    robinaLeftArm_jc( q, TWBase, Tee, Jc );

    std::cout<< " JC: " << std::endl << Jc << std::endl;

    for( int i = 3; i < 6;i++ )
    { for( int j = 0; j < 7; j++ )
      {
        Jc(i,j) = Jc(i,j)*0.001;
      }
    }

    std::cout<< " JC switch: " << std::endl << Jc << std::endl;

    //-- Get the pseudo-inverse(easy way)
    pseudoInv( 6, 7, Jc, Jcinv );

    std::cout<< " JCW Inv: " << std::endl << Jcinv << std::endl;

    Eigen::VectorXd goalOrient(3);
    getRPY( goalPose, goalOrient(0), goalOrient(1), goalOrient(2) ); 

    //-- Get delta (orientation + position)
    Eigen::VectorXd delta(6);
    delta.head(3) = (goalPos - wsPos); delta.tail(3) = (goalOrient - wsOrient);

    //-- Get delta_q (jointspace)   
    delta_q = Jcinv*delta;


    //-- Scale
    double scal = stepSize/delta_q.norm();
    delta_q = delta_q*scal;

    //-- Add to q
    q = q + delta_q; 

    //-- Push it 
    path.push_back( q );

    //-- Check distance to goal
    ws_dist = (goalPos - wsPos).norm();
    printf(" -- Ws dist: %.3f \n", ws_dist );
    trials++;      
    
    if( trials > maxTrials )
     { break; }
  }  


  if( trials >= maxTrials )
   { path.clear();
     printf(" --(!) Did not get to the goal . Thresh: %.3f Trials: %d \n", ws_dist, trials ); 
     return false;}
  else
   { printf(" -- Got to the goal . Thresh: %.3f Trials: %d \n", ws_dist, trials ); 
     return true; }

}

/**
 * @function balancePath
 * @brief Builds a straight path -- easy
 */
bool goWSOrient::balancePath( const Eigen::VectorXd &_startConfig, 
		     	      const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose,
	          	      std::vector<Eigen::VectorXd> &path )
{

  srand( time(NULL) );
  //-- Copy input 
  startConfig = _startConfig;
  goalPose = _goalPose;
  goalPos = _goalPose.translation();

  //-- Create needed variables
  Eigen::VectorXd q; Eigen::VectorXd wsPos;
  Eigen::Transform<double, 3, Eigen::Affine> T;
  Eigen::MatrixXd Jc(3, 7); Eigen::MatrixXd Jcinv(7, 3);

  Eigen::MatrixXd delta_q;
  double ws_dist;

  //-- Initialize variables
  q = startConfig; path.push_back( q );
  ws_dist = DBL_MAX;

  //-- Loop
  int trials = 0;
    //-- Get ws position
    robinaLeftArm_fk( q, TWBase, Tee, T );
    wsPos = T.translation();

    Eigen::VectorXd wsOrient(3); 
    getRPY( T, wsOrient(0), wsOrient(1), wsOrient(2) );


    Eigen::VectorXd phi(7);
    Eigen::VectorXd dq(7);   
    Eigen::VectorXd qorig = q;   

      //-- Get the Jacobian
      robinaLeftArm_j( q, TWBase, Tee, Jc );

      //-- Get the pseudo-inverse(easy way)
      pseudoInv( 3, 7, Jc, Jcinv );
    int count = 0;
    for( int i = 0; i < 1000; i++ )
    {

     for( int j = 0; j < 7; j++ )
     { phi[j] = rand()%6400 / 10000.0 ; }

      //-- Get a null space projection displacement that does not affect the thing
      dq = ( Eigen::MatrixXd::Identity(7,7) - Jcinv*Jc )*phi;

      //-- Add to q 
      //q = qorig + dq;
      Eigen::VectorXd newq;
      newq = qorig + dq;

     //-- Calculate distance
     Eigen::Transform<double, 3, Eigen::Affine> Torig;
     Eigen::Transform<double, 3, Eigen::Affine> Tphi;
     robinaLeftArm_fk( newq, TWBase, Tee, Tphi );
     robinaLeftArm_fk( qorig, TWBase, Tee, Torig );
     double dist = ( Tphi.translation() - Torig.translation() ).norm();

     if( dist < 0.015 )
     { 
       count++;
       q = newq;
       printf("--> [%d] Distance to the original xyz is: %.3f \n", count, dist );
       //-- Push it 
       path.push_back( q );
     }

    }

  return true;
}

/**
 * @function wsDistance
 */
double goWSOrient::wsDistance( Eigen::VectorXd q )
{
  Eigen::Transform<double, 3, Eigen::Affine> T;
  robinaLeftArm_fk( q, TWBase, Tee, T );
  Eigen::VectorXd ws_diff = ( goalPos - T.translation() );
  double ws_dist = ws_diff.norm();
    
  return ws_dist;
}



