/**
 * @file M_RRT.cpp
 * @author achq
 * @brief Test bench 1: Bertram ( Level: 1 )
 */

#include "M_RRT.h" 
#include <Tools/Robot.h>

using namespace std;
using namespace Eigen;

/**
 * @function M_RRT
 * @brief Constructor
 */
M_RRT::M_RRT()
{
  stepSize = 0.1;  // ~2*sqrt(7)
  maxNodes = 5000;
  pGoal = 50;

  distThresh = 0.05;
  xAlignThresh = 10/180.0*3.1416;
  yAlignThresh = 10/180.0*3.1416; 
  failureThresh = 5;
  w1 = 1; w2 = 0.0; w3 = 0.0;

}

/**
 * @function ~M_RRT
 * @brief Destructor
 */
M_RRT::~M_RRT()
{
  cleanup();
}

/**
 * @function initialize
 * @brief Initialize RRT tree
 */
void M_RRT::initialize( World* _world, 
		    	int _robot_ID, 
                    	std::vector<int> _links_ID, 
                    	Eigen::VectorXd &_startConfig )
{
  // Cleanup from previous
  cleanup();

  //-- Save input
  this->world = _world;   
  this->robot_ID = _robot_ID;
  this->links_ID = _links_ID;
  this->ndim = _links_ID.size();

  //-- Initialize random seed
  srand( time(NULL) );

  //-- Create the kdtree for nodes
  kdTree = kd_create( ndim );

  // addNode( _startConfig, -1 );
}

/**
 * @function cleanup
 * @brief Take out the trash for a fresh new planner
 */
void M_RRT::cleanup()
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
bool M_RRT::connect()
{
  VectorXd qtry = getRandomConfig();
  return connect( qtry ); 
}

/**
 * @function connect
 * @brief 
 */
bool M_RRT::connect( const Eigen::VectorXd &target )
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
 * @function tryStep
 */
M_RRT::StepResult M_RRT::tryStep()
{
  VectorXd qtry = getRandomConfig();
  return tryStep(qtry);
}

/**
 * @function tryStep
 */
M_RRT::StepResult M_RRT::tryStep( const Eigen::VectorXd &qtry )
{
  int NNidx = getNearestNeighbor( qtry );
  return tryStep( qtry, NNidx );
}

/**
 * @function tryStep
 */
M_RRT::StepResult M_RRT::tryStep( const Eigen::VectorXd &qtry, int NNidx )
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
 * @function tryStepGoal
 */
M_RRT::StepResult M_RRT::tryStepGoal( const Eigen::VectorXd &qtry, int NNidx )
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

  double qnearCost = heuristicCost( qnear );
  double qnewCost = heuristicCost(  qnew );

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
bool M_RRT::connectGoal()
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
 * @function addNode
 */
int M_RRT::addNode( const Eigen::VectorXd &qnew, int parent_ID )
{
  double heuristic = heuristicCost( qnew );

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
int M_RRT::getNearestNeighbor( const Eigen::VectorXd &qsample )
{
  struct kdres* result = kd_nearest( kdTree, qsample.data() );
  uintptr_t nearest = (uintptr_t) kd_res_item_data( result );
  
  activeNode = nearest;
  return nearest;
}
  
/**
 * @function getRandomConfig
 */ 
Eigen::VectorXd M_RRT::getRandomConfig()
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
void M_RRT::tracePath( int node_ID, std::vector<Eigen::VectorXd> &path, bool reverse )
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
bool M_RRT::checkCollisions( const Eigen::VectorXd &config )
{
  world->robots[robot_ID]->setConf( links_ID, config );
  return world->checkCollisions();
}


/**
 * @function Basic_M_RRT
 * @brief
 */
bool M_RRT::BasicPlan( std::vector<Eigen::VectorXd> &path, 
			Eigen::VectorXd &_startConfig, 
			Eigen::Transform<double, 3, Eigen::Affine> &_goalPose, 
                        Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                        Eigen::Transform<double, 3, Eigen::Affine> _Tee )
{
  printf("Basic Plan \n");

  /** Store information */
  this->startConfig = _startConfig;
  this->goalPose = _goalPose;
  this->goalPosition = _goalPose.translation();
  this->TWBase = _TWBase;
  this->Tee = _Tee;

  //-- Initialize the search tree
  addNode( startConfig, -1 );

  //-- Calculate the heuristicThreshold
  heuristicThresh = w1*distThresh + w2*abs( cos( xAlignThresh ) - 1 ) +w3*abs( cos( yAlignThresh ) );

  //-- Let's start the loop
  double p;
  double heuristic = heuristicVector[0];
  int gc = 0; int rc = 0;
  while( heuristic > heuristicThresh )
   { 
     //-- Probability
     p = rand()%100; 

     //-- Either extends towards goal or random
     if( p < pGoal )
       { printf("Goal \n");extendGoal(); gc++; }
     else
       { printf("Random \n"); extendRandom(); rc++;}

     //-- If bigger than maxNodes, get out loop 
     if( maxNodes > 0 && configVector.size() > maxNodes )
       { 
         cout<<"** Exceeded "<<maxNodes<<" MinCost: "<<heuristicVector[minHeuristicIndex()]<<"MinRankingCost: "<<heuristicVector[rankingVector[0]]<<endl;
         printf("Goal counter: %d, random counter: %d \n", gc, rc );
         return false; }

     heuristic = heuristicVector[ rankingVector[0] ];
   }
  printf("Goal counter: %d, random counter: %d \n", gc, rc );
  printf( "-- Plan successfully generated with %d nodes \n", configVector.size() );
  tracePath( activeNode, path );
  return true;
}


/**
 * @function workspaceDist
 * @brief
 */
Eigen::VectorXd M_RRT::workspaceDist( Eigen::VectorXd node, Eigen::VectorXd ws_target )
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
 * @function extendGoal
 * @brief 
 */
M_RRT::StepResult M_RRT::extendGoal()
{
  connectGoal();
}


/**
 * @function extendRandom
 * @brief
 */
M_RRT::StepResult M_RRT::extendRandom()
{
  connect();
}

/**
 * @function heuristicCost
 * @brief
 */
double M_RRT::heuristicCost( Eigen::VectorXd node )
{

  Eigen::Transform<double, 3, Eigen::Affine> T;

  // Calculate the EE position
  robinaLeftArm_fk( node, TWBase, Tee, T );

  Eigen::VectorXd trans_ee = T.translation();
  Eigen::VectorXd x_ee = T.rotation().col(0);
  Eigen::VectorXd y_ee = T.rotation().col(1);
  Eigen::VectorXd z_ee = T.rotation().col(2);

  Eigen::VectorXd GH = ( goalPosition - trans_ee );

  double fx1 = GH.norm() ;

  GH = GH/GH.norm();

  double fx2 = abs( GH.dot( x_ee ) - 1 );

  double fx3 = abs( GH.dot( z_ee ) );

  double heuristic = w1*fx1 + w2*fx2 + w3*fx3;     

  return heuristic;
}

/**
 * @function add_Ranking
 * @brief Add node to RankingVector
 */
void M_RRT::add_Ranking( int node_ID )
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
int M_RRT::pop_Ranking()
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
  printf("Popped node %d  - left size: %d \n", first, rankingVector.size() );
  return first;
}

int M_RRT::minHeuristicIndex()
{
  int minInd = -1;
  double minVal = DBL_MAX;
  for( int i = 0 ; i < heuristicVector.size(); i++ )
    {
     if ( heuristicVector[i] < minVal )
        { minInd = i; minVal = heuristicVector[i]; }
    } 
  return minInd;
}
