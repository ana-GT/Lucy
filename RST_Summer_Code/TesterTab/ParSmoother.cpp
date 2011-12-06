/**
 * @file ParSmoother.cpp
 * @date 09-03-2011
 * @author ahq
 */

#include "ParSmoother.h"


/**
 * @function Constructor
 */
MyFeasibilityChecker::MyFeasibilityChecker( )
{
}

void MyFeasibilityChecker::initialize( World* _world, int _robot, std::vector<int> _links, double _stepsize )
{
   world = _world;
   robot = _robot;
   links = _links;  
   ndim = _links.size(); 
   stepsize = _stepsize;
}

/**
 * @function Destructor
 */
MyFeasibilityChecker::~MyFeasibilityChecker()
{}

/**
 * @function ConfigFeasible
 * @brief Check if there is not collisions in the given arm configuration
 */
bool MyFeasibilityChecker::ConfigFeasible( const Vector& x )
{
   Eigen::VectorXd c(ndim);

   for( int i = 0; i < ndim; i++ )
   { c(i) = x[i]; }
   world->robots[ robot ]->setConf( links, c );
   return world->checkCollisions();
}

/**
 * @function SegmentFeasible
 * @brief Check if the segment between these two configurations is collision-free
 */
bool MyFeasibilityChecker::SegmentFeasible( const Vector & a, const Vector & b )
{ 
   Eigen::VectorXd config1(ndim), config2(ndim);

   for( int i = 0; i < ndim; i++ )
   { config1(i) = a[i]; 
     config2(i) = b[i];
   }

   int n = (int)( (config2 - config1).norm() / stepsize );
   for(int i = 0; i < n; i++) 
   {
      Eigen::VectorXd conf = (double)(n - i)/(double)n * config1 + (double)(i)/(double)n * config2;
      world->robots[robot]->setConf(links, conf, true);
      if(world->checkCollisions()) 
      { return false; }
   }

   return true;
}


/******************************************/
/**
 * @function smoothPath
 */

ParSmoother::ParSmoother( World* _world, int _robot, std::vector<int> _links, double _stepsize ) 
{
   mfc.initialize( _world, _robot, _links, _stepsize );
   ndim = _links.size();
}

ParSmoother::~ParSmoother()
{
}

std::vector<Eigen::VectorXd> ParSmoother::smoothPath( double _maxVelArray[], double _maxAcelArray[], int _numIter, double _tol, std::vector<Eigen::VectorXd> &jerkyPath ) 
{
   numIter = _numIter;
   tol = _tol;

   maxVel.resize(0); maxAcel.resize(0);

   for( int i = 0; i < ndim; i++ )
   {
      maxVel.push_back( _maxVelArray[i] );
      maxAcel.push_back( _maxAcelArray[i] );
   }

   std::vector< Vector >  inputPath;
   inputPath.resize(0);
   Vector wayPoint( ndim );

   //-- 1. Convert to a format we can use
   for( int i = 0; i < jerkyPath.size(); i++ )
   {  
      for( int j = 0; j < ndim; j++ )
      { wayPoint[j] = jerkyPath[i](j); }
      inputPath.push_back( wayPoint ); 
   }    
 
 //  RampFeasibilityChecker checker( &mfc, tol );

   DynamicPath traj;

   traj.Init( maxVel, maxAcel );
   traj.SetMilestones( inputPath );
   printf("Initial path duration: %g\n",traj.GetTotalTime());
   printf("Where is \n"); 
   int res = traj.Shortcut( numIter, &mfc, tol );
   printf("After shortcutting: %d shortcuts taken, duration %g\n", res, traj.GetTotalTime());

   std::vector<Eigen::VectorXd>  smoothedPath;
   return smoothedPath;
   
}
