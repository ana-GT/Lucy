/**
 * @file B_RRT.cpp
 * @author achq
 * @brief Test bench 1: Bertram ( Level: 1 )
 */

#include "B_RRT.h" 
#include <Tools/Robot.h>

using namespace std;
using namespace Eigen;

/**
 * @function ~B_RRT
 * @brief Destructor
 */
B_RRT::~B_RRT()
{
  cleanup();
}

/**
 * @function initialize
 * @brief Initialize RRT tree
 */
void B_RRT::initialize( World* _world, 
		    	int _robot_ID, 
                    	std::vector<int> _links_ID, 
                    	Eigen::VectorXd &_startConfig, 
                    	double _stepSize )
{
  // Cleanup from previous
  cleanup();
  this->world = _world;   
  this->robot_ID = _robot_ID;
  this->links_ID = _links_ID;
  this->ndim = _links_ID.size();
  this->stepSize = _stepSize;

  srand( time(NULL) );
  kdTree = kd_create( ndim );
  // addNode( _startConfig, -1 );
}

/**
 * @function cleanup
 * @brief Take out the trash for a fresh new planner
 */
void B_RRT::cleanup()
{
  links_ID.clear(); links_ID.resize(0);
  
  parentVector.clear(); parentVector.resize(0);
  configVector.clear(); configVector.resize(0);
  rankingVector.clear(); rankingVector.resize(0);
  heuristicVector.clear(); heuristicVector.resize(0);
  failureVector.clear(); failureVector.resize(0);
}

/**
 * @function connect
 * @brief Slightly enhanced connect version for Bertram
 */
bool B_RRT::connect()
{
  VectorXd qtry = getRandomConfig();
  return connect( qtry ); 
}

/**
 * @function connect
 * @brief 
 */
bool B_RRT::connect( const Eigen::VectorXd &target )
{
  int NNidx = getNearestNeighbor(target);
  StepResult result = STEP_PROGRESS;

  while(result == STEP_PROGRESS) 
  {
    result = tryStep(target, NNidx);
    NNidx = configVector.size() - 1;	
  }

 return (result == STEP_REACHED);
}


/**
 * @function tryStepGoal
 */
B_RRT::StepResult B_RRT::tryStepGoal( const Eigen::VectorXd &qtry, int NNidx )
{
  VectorXd qnear( ndim );
  VectorXd qnew( ndim );
  qnear = configVector[NNidx];

  Eigen::VectorXd diff = ( qtry - qnear );
  double edist = diff.norm();

  // If the new node is nearer than the stepSize, don't add it

  if( edist < stepSize )
   { return STEP_REACHED; } 

  // Scale it in order to add it
  double scale = stepSize / edist;
  for( int i = 0; i < ndim; i++ )
  { qnew[i] = qnear[i] + diff[i]*scale; }

  double qnearCost = heuristicCost( qnear, goalPosition );
  double qnewCost = heuristicCost(  qnew, goalPosition );

  //-- Check if there are not collisions are if the heuristic distance decreases
  if( checkCollisions(qnew) )
    { return STEP_COLLISION; }
  else if( qnewCost > qnearCost )
    { return STEP_NO_NEARER;}
  else  
    { addNode( qnew, NNidx );
      return STEP_PROGRESS; }
}
	

/**
 * @function connectGoal
 * @brief connect with Heuristic goal into account
 */
bool B_RRT::connectGoal()
{
  //-- Get the node with smallest heuristic
  int NNidx = rankingVector[0];

  //-- Generate a random node 
  Eigen::VectorXd qrand = getRandomConfig();

  StepResult result = STEP_PROGRESS;
  int i = 0;

  while( result == STEP_PROGRESS )
  { 
    result = tryStepGoal( qrand, NNidx );
    NNidx = rankingVector[0];
  i++;
  }


  if( result == STEP_COLLISION  || result == STEP_NO_NEARER )
    { int badNode = rankingVector[0];
      failureVector[badNode]++; 
      
      if( failureVector[badNode] >failureThresh )
        {
          pop_Ranking();
          int badNodeDad = parentVector[badNode];
          failureVector[badNodeDad] = failureThresh;
        }
    }

  return (result == STEP_REACHED);    
}


/**
 * @function tryStep
 */
B_RRT::StepResult B_RRT::tryStep()
{
  VectorXd qtry = getRandomConfig();
  return tryStep(qtry);
}

/**
 * @function tryStep
 */
B_RRT::StepResult B_RRT::tryStep( const Eigen::VectorXd &qtry )
{
  int NNidx = getNearestNeighbor( qtry );
  return tryStep( qtry, NNidx );
}

/**
 * @function tryStep
 */
B_RRT::StepResult B_RRT::tryStep( const Eigen::VectorXd &qtry, int NNidx )
{
  VectorXd qnear( ndim );
  VectorXd qnew( ndim );
  qnear = configVector[NNidx];

  Eigen::VectorXd diff = ( qtry - qnear );
  double edist = diff.norm();

  // If the new node is nearer than the stepSize, don't add it

  if( edist < stepSize )
   { return STEP_REACHED; } 

  // Scale it in order to add it
  double scale = stepSize / edist;
  for( int i = 0; i < ndim; i++ )
  { qnew[i] = qnear[i] + diff[i]*scale; }
  
  if( !checkCollisions(qnew) )
  {
    addNode( qnew, NNidx );
    return STEP_PROGRESS;
  }
  else
  { return STEP_COLLISION; }
}
   
/**
 * @function addNode
 */
int B_RRT::addNode( const Eigen::VectorXd &qnew, int parent_ID )
{
  double heuristic = heuristicCost( qnew, goalPosition );

  configVector.push_back( qnew );
  parentVector.push_back( parent_ID );
  heuristicVector.push_back( heuristic );  
  failureVector.push_back(0); 

  uintptr_t ID = configVector.size() - 1;
  kd_insert( kdTree, qnew.data(), (void*)ID );
  add_Ranking( ID );  

  activeNode = ID;
  return ID;
}

/**
 * @function getNearestNeighbor
 */
int B_RRT::getNearestNeighbor( const Eigen::VectorXd &qsample )
{
  struct kdres* result = kd_nearest( kdTree, qsample.data() );
  uintptr_t nearest = (uintptr_t) kd_res_item_data( result );
  
  activeNode = nearest;
  return nearest;
}
  
/**
 * @function getRandomConfig
 */ 
Eigen::VectorXd B_RRT::getRandomConfig()
{
  VectorXd config( ndim );
  for( int i = 0; i < ndim; i++ )
     {
       config[i] = RANDNM( world->robots[robot_ID]->links[links_ID[i]]->jMin, world->robots[robot_ID]->links[links_ID[i]]->jMax );
     }

  return config;
}


/**
 * @function tracePath
 */   
void B_RRT::tracePath( int node_ID, std::vector<Eigen::VectorXd> &path, bool reverse )
{
  int x = node_ID;
  std::vector<Eigen::VectorXd> back_path; 
 
  while( x!= -1 )
  {
    if( !reverse )
      { back_path.push_back( configVector[x] ); }
    else
      { path.push_back( configVector[x] ); }

    x = parentVector[x];
  }

  if( !reverse )
  {
    int n = back_path.size();
    for( int i = 0; i < n; i++ )
       { path.push_back( back_path[n - i - 1] ); }
  }

}

/**
 * @function checkCollisions
 * @brief Check if config smashes something...
 */
bool B_RRT::checkCollisions( const Eigen::VectorXd &config )
{
  world->robots[robot_ID]->setConf( links_ID, config );
  return world->checkCollisions();
}

/**
 * @function getSize
 * @brief Returns the size of the tree
 */
const unsigned int B_RRT::getSize()
{
  return configVector.size();
}

/**
 * @function Basic_B_RRT
 * @brief
 */
bool B_RRT::BasicPlan( std::vector<Eigen::VectorXd> &path, 
			Eigen::VectorXd &_startConfig, 
			Eigen::Transform<double, 3, Eigen::Affine> &_goalPose, 
                        Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                        Eigen::Transform<double, 3, Eigen::Affine> _Tee,
			double _p_goal, 
			double _distThreshold, 
			int _maxNodes,
                        int _failureThresh )
{
  printf("Basic Plan \n");

  /** Store information */
  this->startConfig = _startConfig;
  this->goalPose = _goalPose;
  this->goalPosition = _goalPose.translation();
  this->TWBase = _TWBase;
  this->Tee = _Tee;
  this->p_goal = _p_goal;
  this->distThreshold = _distThreshold;  
  this->maxNodes = _maxNodes;
  this->failureThresh = _failureThresh;

  //-- Initialize the search tree
  addNode( startConfig, -1 );

  //-- Calculate the heuristicThreshold
  heuristicThreshold = w1*distThreshold + w2*abs( cos( alignxThreshold ) - 1 ) +w3*abs( cos(alignyThreshold) );

  //-- Create variables you may need
  double p;
  int ws_NN_ID = rankingVector[0];
  double heuristic = heuristicVector[ws_NN_ID];

  p = 0;
  while( heuristic > heuristicThreshold )
   { 
     p = rand()%100; 
     if( p < p_goal )
       { extendTowardsGoal(); }
     else
       { extendRandomly();}

     //-- Check if in limits 
     if( maxNodes > 0 && getSize() > maxNodes )
       { cout<<"** Exceeded "<<maxNodes<<" allowed - Heuristic: "<<heuristicVector[rankingVector[0]]<<endl;
         return false; }
     ws_NN_ID = rankingVector[0];
     heuristic = heuristicVector[ws_NN_ID];
   }

  printf("Done! \n");
  tracePath( activeNode, path );
  return true;
}


/**
 * @function workspaceDist
 * @brief
 */
Eigen::VectorXd B_RRT::workspaceDist( Eigen::VectorXd node, Eigen::VectorXd ws_target )
{
  Eigen::Transform<double, 3, Eigen::Affine> T;
  Eigen::VectorXd diff;

  // Calculate the EE position
  robinaLeftArm_fk( node, TWBase, Tee, T );
  Eigen::VectorXd ws_node = T.translation();

  // Calculate the workspace distance to goal
  diff = ( ws_target - ws_node );

  return diff;
}


/**
 * @function extendTowardsGoal
 * @brief 
 */
B_RRT::StepResult B_RRT::extendTowardsGoal()
{
  int a = rankingVector.size() ;
     connectGoal();
  int b = rankingVector.size();
}


/**
 * @function extendRandomly
 * @brief
 */
B_RRT::StepResult B_RRT::extendRandomly()
{
  int a = rankingVector.size() ;
  connect();
  int b = rankingVector.size();
}

/**
 */
double B_RRT::heuristicCost( Eigen::VectorXd node, Eigen::VectorXd ws_target)
{

  Eigen::Transform<double, 3, Eigen::Affine> T;

  // Calculate the EE position
  robinaLeftArm_fk( node, TWBase, Tee, T );

  Eigen::VectorXd trans_ee = T.translation();
  Eigen::VectorXd x_ee = T.rotation().col(0);
  Eigen::VectorXd y_ee = T.rotation().col(1);
  Eigen::VectorXd z_ee = T.rotation().col(2);

  Eigen::VectorXd GH = ( ws_target - trans_ee );

  double fx1 = GH.norm() ;

  GH = GH/GH.norm();

  double fx2 = abs( GH.dot( x_ee ) - 1 );

  double fx3 = abs( GH.dot( z_ee ) );

  double heuristic = w1*fx1 + w2*fx2 + w3*fx3;     
  //cout<<"Factor 1: "<<fx1<<" Factor 2: "<<fx2<<endl;
  return heuristic;
}

/**
 * @function add_Ranking
 * @brief Add node to RankingVector
 */
void B_RRT::add_Ranking( int node_ID )
{
  int n; 
  int node; int parent;
  int temp;

  rankingVector.push_back( node_ID );
  n = rankingVector.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { return; }

  // If not, start on the bottom and go up
  node = n;

  while( node != 0 )
  {
    parent = floor( (node - 1)/2 );
    // Always try to put new nodes up
    if( heuristicVector[ rankingVector[parent] ] >= heuristicVector[ rankingVector[node] ] )
      {
        temp = rankingVector[parent];
        rankingVector[parent] = rankingVector[node];
        rankingVector[node] = temp; 
        node = parent; 
      }  
    else
     { break; }
  }   

}

/**
 * @function pop_Ranking
 * @brief Pop out the top node from RankingVector
 */
int B_RRT::pop_Ranking()
{
  if( rankingVector.size() == 0 )
    { cout<<"Out, no nodes!"<<endl; return -1;}
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  // Save the pop-out element
  first = rankingVector[0];
  
  // Reorder your binary heap
  bottom = rankingVector.size() - 1;

  rankingVector[0] = rankingVector[bottom];
  rankingVector.pop_back();
  n = rankingVector.size();

  int u = 0;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( heuristicVector[ rankingVector[node] ] >= heuristicVector[ rankingVector[child_1] ] )
        { u = child_1;  }
       if( heuristicVector[ rankingVector[u] ]  >= heuristicVector[ rankingVector[child_2] ] )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( heuristicVector[ rankingVector[node] ] >= heuristicVector[ rankingVector[child_1] ] )
         { u = child_1; }
     }
    
    if( node != u )
     { temp = rankingVector[node];
       rankingVector[node] = rankingVector[u];
       rankingVector[u] = temp; 
     }

    else
     { break; } 
  }

  return first;
}
