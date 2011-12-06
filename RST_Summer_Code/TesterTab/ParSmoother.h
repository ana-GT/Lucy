/**
 * @file ParSmoother.h
 * @brief Interface for the Parabolic Smoother
 * @date September 3rd, 2011
 * @author ahq
 */

#ifndef _PAR_SMOOTHER_
#define _PAR_SMOOTHER_

#include <Tools/World.h>
#include <Tools/Robot.h>
#include <Tools/ParabolicPathSmooth/DynamicPath.h>


/**
 * @class MyFeasibilityChecker
 */
class MyFeasibilityChecker : public FeasibilityCheckerBase
{
   public:

     MyFeasibilityChecker(); 
     void initialize( World* _world, int _robot, std::vector<int> _links, double _stepsize = 0.05 );
     virtual  ~MyFeasibilityChecker();
     virtual bool ConfigFeasible( const Vector& x );
     virtual bool SegmentFeasible( const Vector & a, const Vector & b );

   private:

     World* world;
     int robot;
     std::vector<int> links;
     int ndim;
     double stepsize;
};

/**
 * @function ParSmoother
 */
class ParSmoother
{
   public:
   ParSmoother( World* _world, int _robot, std::vector<int> _links, double _stepsize ); 
   ~ParSmoother();
   std::vector<Eigen::VectorXd> smoothPath( double _maxVelArray[], double _maxAcelArray[], int _numIter, double _tol, std::vector<Eigen::VectorXd> &jerkyPath ); 

   MyFeasibilityChecker mfc;
   int ndim;
   Vector maxVel; 
   Vector maxAcel;
   
   int numIter;
   Real tol;
};


#endif /** _PAR_SMOOTHER_ */
