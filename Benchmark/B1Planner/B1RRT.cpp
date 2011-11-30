/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 * @file RRT.cpp
 *
 * Authors: Tobias Kunz <tobias@gatech.edu>, Ana Huaman <ahuaman3@gatech.edu>
 * Date: 10/2011
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 */

#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "B1RRT.h"

/**
 * @function B1RRT
 * @brief Constructor
 */
B1RRT::B1RRT( planning::World *_world, 
              Collision *_collision,
              int _robotId, 
              const Eigen::VectorXi &_links, 
              const Eigen::VectorXd &_root, 
              const Eigen::VectorXd &_targetPose,
              double _stepSize ) {

    /// Initialize some member variables
    world = _world; 
    collision = _collision;
    robotId = _robotId;
    links = _links;
    targetPose= _targetPose; 
    ndim = links.size();
    stepSize = _stepSize;
    EEId = world->mRobots[robotId]->getDof( links[ links.size()-1 ] )->getJoint()->getChildNode()->getSkelIndex();
    ranking.resize(0);

    /// Initialize random generator   
    srand( time(NULL) );

    /// Create kdtree and add the first node (start) 
    kdTree = kd_create( ndim );
    addNode( _root, -1 ); 
}

/**
 * @function ~B1RRT()
 * @brief Destructor
 */
B1RRT::~B1RRT() {
    kd_free( kdTree );
}

/**
 * @function connect
 * @brief Connect the closest node with random target, stop until it is reached or until it collides
 */
bool B1RRT::connect() {
    Eigen::VectorXd qtry = getRandomConfig();
    return connect( qtry );
   
}

/**
 * @function connect
 * @brief Connect the closest node with _target, stop until it is reached or until it collides
 */
bool B1RRT::connect( const Eigen::VectorXd &_target ) {

    int NNIdx = getNearestNeighbor( _target );
    StepResult result = STEP_PROGRESS;
    int i = 0;
    while( result == STEP_PROGRESS ) {

        result = tryStepFromNode( _target, NNIdx );
        NNIdx = configVector.size() -1;
        i++; 
    }
    return ( result == STEP_REACHED );
     

}

/**
 * @function tryStep
 * @brief Try to advance one stepSize towards a random target
 */
B1RRT::StepResult B1RRT::tryStep() {
    Eigen::VectorXd qtry = getRandomConfig();
    return tryStep( qtry );
}

/**
 * @function tryStep
 * @brief Try to advance one stepSize towards _qtry
 */
B1RRT::StepResult B1RRT::tryStep( const Eigen::VectorXd &_qtry ) {
    int NNIdx = getNearestNeighbor( _qtry );
    return tryStepFromNode( _qtry, NNIdx );
}

/**
 * @function tryStepFromNode
 * @brief Tries to extend tree towards provided sample (must be overridden for MBP ) 
 */
B1RRT::StepResult B1RRT::tryStepFromNode( const Eigen::VectorXd &_qtry, int _NNIdx ) {
    
    /// Calculates a new node to grow towards _qtry, check for collisions and adds to tree 
    Eigen::VectorXd qnear = configVector[_NNIdx];

    /// Compute direction and magnitude
    Eigen::VectorXd diff = _qtry - qnear;
    double dist = diff.norm();
 
    if( dist < stepSize ) { 
        return STEP_REACHED; 
    }

    /// Scale this vector to stepSize and add to end of qnear
    Eigen::VectorXd qnew = qnear + diff*(stepSize/dist);

    if( !checkCollisions(qnew) ) {
        addNode( qnew, _NNIdx );
        return STEP_PROGRESS;
    } else {
        return STEP_COLLISION;
    }

}

/**
 * @function addNode
 * @brief Add _qnew to tree with parent _parentId
 * @return id of node added
 */
int B1RRT::addNode( const Eigen::VectorXd &_qnew, int _parentId )
{
    /// Update graph vectors -- what does this mean?
    configVector.push_back( _qnew );
    parentVector.push_back( _parentId );

    uintptr_t id = configVector.size() - 1;
    kd_insert( kdTree, _qnew.data(), (void*) id ); //&idVector[id];  /// WTH? -- ahq

    activeNode = id;

    /// Add node to ranking
    Eigen::VectorXd entry(3);
    entry << id, HeuristicCost( id, targetPose ), 0;
    pushRanking( entry );    

    return id;
}  

/**
 * @function HeuristicCost
 */
double B1RRT::HeuristicCost( int _id, Eigen::VectorXd _targetPose ) {
    
    return GoalDist( _id, _targetPose );
}

/**
 * @function getRandomConfig
 * @brief Samples a random point for qtmp in the configuration space,
 * bounded by the provided configuration vectors (and returns ref to it)
 */
Eigen::VectorXd B1RRT::getRandomConfig() {
    Eigen::VectorXd config( ndim );
    for( unsigned int i = 0; i < ndim; i++ ) {
        double minVal = world->mRobots[robotId]->getDof(links[i])->getMin();
        double maxVal = world->mRobots[robotId]->getDof(links[i])->getMax();
        config[i] = randomInRange( minVal, maxVal ); 
    }

    return config;       
}

/**
 * @function getNearestNeighbor
 * @brief Returns Nearest Neighbor index to query point
 */
int B1RRT::getNearestNeighbor( const Eigen::VectorXd &_qsamp ) {

    struct kdres* result = kd_nearest( kdTree, _qsamp.data() );
    uintptr_t nearest = (uintptr_t)kd_res_item_data(result);

    activeNode = nearest;
    return nearest;
   
}

/**
 * @function getGap
 * @brief Get the gap (Distance) between the closest node in the tree to the _target
 */
double B1RRT::getGap( const Eigen::VectorXd &_target ) {
    return ( _target - configVector[activeNode] ).norm();
}

/**
 * @function tracePath
 * @brief Traces the path from some node to the initConfig node
 */
void B1RRT::tracePath( int _node, 
                     std::list<Eigen::VectorXd> &_path, 
                     bool _reverse ) {
  
    int x = _node;
    
    while( x != -1 ) {
        if( !_reverse ) {
            _path.push_front( configVector[x] );
        } else {
            _path.push_back( configVector[x] );
        }
        x = parentVector[x];
    }
}

/**
 * @function checkCollisions
 * @brief Returns true if collisions, false if okay? Hum, not sure yet
 * @TODO IMPLEMENT ME!!!
 */
bool B1RRT::checkCollisions( const Eigen::VectorXd &_config ) {
    world->mRobots[robotId]->setDofs( _config, links );
    world->mRobots[robotId]->update();
    return collision->CheckCollisions();
}

/**
 * @function getSize
 * @brief Returns size of the tree
 */
unsigned int B1RRT::getSize() {
    return configVector.size();
}

/**
 * @function randomInRange
 * @brief Get random number between min and max
 */
double B1RRT::randomInRange( double _min, double _max ) {

    return _min + ((_max - _min) * ((double)rand() / ((double)RAND_MAX + 1))); 
}

/**
 * @function ExtendHeuristic
 */
bool B1RRT::ExtendHeuristic() {
    int nearId;
    nearId = ranking[0][0];

    connectHeuristic( nearId, getRandomConfig() ); 
    // Get a new configuration

    return false;
}

/**
 * @function connectHeuristic
 * @brief Connect the closest node with _target, stop until it is reached or until it collides
 */
bool B1RRT::connectHeuristic( int _nearId, const Eigen::VectorXd &_target ) {

    int nearId = _nearId;
    int oldId;

    StepResult result = STEP_PROGRESS;
    int i = 0;
    do {
        result = tryStepFromNode( _target, nearId );
        oldId = nearId;
        nearId = configVector.size() -1;
        i++; 
    } while( result == STEP_PROGRESS && 
             GoalDist(nearId, targetPose) < GoalDist(oldId,targetPose) );

    if( result == STEP_COLLISION ) {
        popRanking();
    }

    return ( result == STEP_REACHED );
     

}


/**
 * @function GoalDist
 * @brief Calculate geometric distance from the goal to the End Effector Body Node
 */
double B1RRT::GoalDist( int _nodeId, Eigen::VectorXd _targetPose ) {
    return GoalDist( configVector[_nodeId], _targetPose );
}

/**
 * @function GoalDist
 * @brief Calculate geometric distance from the goal to the End Effector Body Node
 */
double B1RRT::GoalDist( Eigen::VectorXd _nodeConfig, Eigen::VectorXd _targetPose ) {

    world->mRobots[robotId]->setDofs( _nodeConfig, links );
    Eigen::MatrixXd pose = world->mRobots[robotId]->getNode( EEId )->getWorldTransform(); 
    double dist;

    Eigen::VectorXd pos(3); pos << pose(0,3), pose(1,3), pose(2,3);
    Eigen::VectorXd tpos(3); tpos << _targetPose(0), _targetPose(1), _targetPose(2);

    dist = ( pos - tpos ).norm();

    return dist;
}
