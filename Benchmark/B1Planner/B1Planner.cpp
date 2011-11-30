/**
 * @file B1Planner.cpp
 * @brief Read the .h heading for details :)
 * @author A.H.Q.
 * @date November 14th, 2011
 */

#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "B1Planner.h"

/**
 * @function B1Planner
 * @brief Constructor
 */
B1Planner::B1Planner() {
    mCopyWorld = false;
    mWorld = NULL;
}

/**
 * @function B1Planner
 * @brief Constructor
 */
B1Planner::B1Planner( planning::World &_world, 
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
}

/**
 * @function ~B1Planner
 * @brief Destructor
 */
B1Planner::~B1Planner() {

    if( mCopyWorld ) {
        delete mWorld;
    }
}

/**
 * @function planPath
 * @brief Main function
 */
bool B1Planner::planPath( int _robotId, 
                          const Eigen::VectorXi &_links, 
                          const Eigen::VectorXd &_start, 
                          const Eigen::VectorXd &_target, 
                          bool _smooth, 
                          unsigned int _maxNodes ) {

    if( _maxNodes <= 0 ) {
        printf("--(x) Set maxNodes to some value, you moron! (x)--\n"); return false;
    } 

    mWorld->mRobots[_robotId]->setDofs( _start, _links );
    if( mCollision->CheckCollisions() )
        return false;
 
    // 1. Create a RRT entity
    mRrt = new B1RRT( mWorld, mCollision, _robotId, _links, _start, _target, mStepSize );
    B1RRT::StepResult result = B1RRT::STEP_PROGRESS;

    double smallestGap = DBL_MAX;
    double gap;

    int p = 50;     /**< Probability of directing to goal (0-100) */

    mThreshold = 0.05;

    /// 2. Search loop
    while ( smallestGap > mThreshold ) {

        if( randomInRange(0, 100) < p ) {
            mRrt->ExtendHeuristic(); 
        } else {
            mRrt->connect();  
        }

        /// Check if nodes are not max yet
        if( mRrt->getSize() > _maxNodes ) {
            printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
            return false;
        }

        gap = mRrt->GoalDist( mRrt->activeNode, _target );
        if( gap < smallestGap ) {
            smallestGap = gap;
            cout << "--> [B1Planner] Gap: " << smallestGap << "  Tree size: " << mRrt->configVector.size() << endl;
        }
    } // End of while

    /// Save path  
    printf(" --> Reached goal! : Gap: %.3f \n", gap );
    mRrt->tracePath( mRrt->activeNode, mPath, false );
   
    return true;


    if( result && _smooth ) {
        smoothPath( _robotId, _links, mPath );
    }

    return result;
}


/**
 * @function checkPathSegment
 * @brief True iff collision-free
 */
bool B1Planner::checkPathSegment( int _robotId, 
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
void B1Planner::smoothPath( int _robotId, 
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

