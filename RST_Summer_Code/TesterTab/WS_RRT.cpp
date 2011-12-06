/**
 * @file WS_RRT.cpp
 * @author achq
 * @brief Test bench 2: Jacobian guy ( Level: 1 )
 */
#include "WS_RRT.h"

/**
 * @function WS_RRT
 * @brief Constructor
 */
WS_RRT::WS_RRT()
{
  this->p_goal = 0.5;
  this->ws_distThresh = 0.03;
  this->js_stepSize = 0.1;
  this->maxNodes = 6000;
  this->activeNode = 0;
  this->ws_win_low.resize(3); ws_win_low<<1.0, 1.0, 1.0;
  this->ws_win_high.resize(3); ws_win_high<<0.5, 1.0, 0.2;
  this->max_calls = 5;
}

/**
 * @function ~WS_RRT
 * @brief Destructor
 */
WS_RRT::~WS_RRT()
{
  cleanup();
}

/**
 * @function initialize
 */
void WS_RRT::initialize( World* _world, 
		         const int &_robot_ID, 
                         const std::vector<int> &_links_ID,
                         const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase,
                         const Eigen::Transform<double, 3, Eigen::Affine> &_Tee )
{
  cout<<"...Initialize..."<<endl;
  //-- Clean up from previous searches
  cleanup();

  //-- Copy input
  this->world = _world;   
  this->robot_ID = _robot_ID;
  this->links_ID = _links_ID;
  this->ndim = _links_ID.size();
  this->TWBase = _TWBase;
  this->Tee = _Tee;
  this->kdTree = kd_create( 3 );

  //-- Seed the random generator
  srand( time(NULL) );
}

/**
 * @function cleanup
 * @brief Take out the trash for a fresh new planner
 */
void WS_RRT::cleanup()
{
  links_ID.clear(); links_ID.resize(0);
  
  parentVector.clear(); parentVector.resize(0);
  configVector.clear(); configVector.resize(0);
  wsVector.clear(); wsVector.resize(0);
}

/**
 * @function plan
 */
bool WS_RRT::plan( const Eigen::VectorXd &_startConfig, 
	           const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose, 
                   std::vector<Eigen::VectorXd> &path )
{
  cout<<"...Plan..."<<endl;
  //-- Store information
  this->startConfig = _startConfig;
  this->goalPose = _goalPose;
  this->goalPosition = _goalPose.translation();


  //-- Initialize the search tree
  addNode( startConfig, -1 );
  //-- Initiate the search
  double p;
  double result = false;

  while( wsDist( activeNode, goalPosition ) > ws_distThresh )
  {
    p = RANDNM( 0.0, 1.0 );

    if( p < p_goal )
     { if( connect_goal() ) 
         { cout<<"Goal connected!"<<endl;
           result = true;  
           break; }
     } 
    else
     { connect(); }

    if( configVector.size() > maxNodes )
      { cout<<"-- Exceeded max nodes. Last ws_dist: "<<distGoalVector[activeNode]<<endl; break; }
  }
  
  if( result == true || wsDist( activeNode, goalPosition ) <= ws_distThresh )
    { tracePath( activeNode, path );
      cout<<"-- Found a path with "<<path.size()<<" nodes -- Tree size: "<<configVector.size()<<endl;       
      return true; }

  else
    { cout<<"~~ No path found"<<endl;
      return false; }
 
}

/**
 * @function addNode
 */  
int WS_RRT::addNode( const Eigen::VectorXd &js_node, int parent_ID )
{
  Eigen::VectorXd wsPos = js_wsPos( js_node );

  configVector.push_back( js_node );
  parentVector.push_back( parent_ID );
  wsVector.push_back( wsPos );
  distGoalVector.push_back( (goalPosition - wsPos).norm() );

  uintptr_t ID = configVector.size() - 1;
  kd_insert( kdTree, wsPos.data(), (void*) ID );
  add_Ranking( ID );

  activeNode = ID;
  return ID;
}

/**
 * @function js_wsPos
 */
Eigen::VectorXd WS_RRT::js_wsPos( Eigen::VectorXd js_node )
{
  Eigen::Transform<double, 3, Eigen::Affine> T;
  robinaLeftArm_fk( js_node, TWBase, Tee, T );
  Eigen::VectorXd ws_pos = T.translation();
  return ws_pos;
}

/**
 * @function wsDist
 */
double WS_RRT::wsDist( int ID, Eigen::VectorXd ws_pos )
{
  Eigen::VectorXd ws_diff = ws_pos - wsVector[ID];
  double ws_dist = ws_diff.norm();
  return ws_dist; 
}


/**
 * @function wsDiff
 */
Eigen::VectorXd WS_RRT::wsDiff( int ID, Eigen::VectorXd ws_pos )
{
  Eigen::VectorXd ws_diff = ws_pos - wsVector[ID];
  return ws_diff; 
}

/**
 * @function connect
 */  
bool WS_RRT::connect( )
{
  Eigen::VectorXd ws_pos = getRandom_wsPos();
  return connect( ws_pos );
}

/**
 * @function connect
 */
bool WS_RRT::connect( const Eigen::VectorXd &ws_target )
{
  int ID = getNearestNeighbor( ws_target );
  return connect( ws_target, ID );
}

/**
 * @function connect
 */
bool WS_RRT::connect( const Eigen::VectorXd &ws_target, int NN_ID )
{
  StepResult result = STEP_PROGRESS;

  int i = 0;
  while( result == STEP_PROGRESS )
   {
     result = tryStep( ws_target, NN_ID );
     NN_ID = wsVector.size() - 1;
     i++;  

     if( i > max_calls )
       { break; } 
   }
  cout<<"Connect: Iter: "<<i<<" -- active Node: "<<activeNode<<" dist: "<<wsDist( activeNode, goalPosition )<<" result: "<<result<<endl;
  return (result == STEP_REACHED );
}

/**
 * @function connect_goal
 */
bool WS_RRT::connect_goal()
{
  if( rankingVector.size() <= 0 ){ cout<<"Empty ranking vector "<<endl; return false; }

  StepResult result = STEP_PROGRESS;
  
  int i = 0; int NN_ID;
  while( true )
   {
     NN_ID = pop_Ranking();
     result = tryStep( goalPosition, NN_ID );
     if( result != STEP_PROGRESS )
       { break; } 
     i++;
   }  

  cout<<" Goal:  Iter: "<<i<<" node["<<activeNode<<"]: ("<<wsVector[activeNode].transpose()<<" dist: "<<wsDist( activeNode, goalPosition )<<") result: "<<result<<endl;
  return (result == STEP_REACHED );
}

/**
 * @function tryStep
 */
WS_RRT::StepResult WS_RRT::tryStep( const Eigen::VectorXd &ws_target, int NN_ID )
{
  Eigen::VectorXd ws_delta = wsDiff( NN_ID, ws_target );
  if( ws_delta.norm() < ws_distThresh )
    { return STEP_REACHED; }

  Eigen::MatrixXd J(3,7); Eigen::MatrixXd Jt(7,3);
  robinaLeftArm_j( configVector[NN_ID], TWBase, Tee, J );
  Jt = J.transpose();  

  Eigen::VectorXd js_delta = Jt*ws_delta;

  Eigen::VectorXd js_new = configVector[NN_ID] + js_delta;
  
  if( checkCollisions(js_new) )
    { return STEP_COLLISION; }
  else
    {
      if( ( ws_target - js_wsPos(js_new) ).norm() > wsDist( NN_ID, ws_target) )
        { return STEP_NO_NEARER; }
      else
        { addNode( js_new, NN_ID );
          return STEP_PROGRESS; } 
    }
} 

/**
 * @function getRandom_wsPos
 */
Eigen::VectorXd WS_RRT::getRandom_wsPos()
{
  Eigen::VectorXd wsPos(3);
  for (int i = 0; i < 3; ++i) 
      { double a = wsVector[0](i) - ws_win_low(i);
        double b = wsVector[0](i) + ws_win_high(i);
        
        wsPos(i) = RANDNM( a, b );
      }
  return wsPos;
}

/**
 * @function getRandom_wsPos
 */
Eigen::VectorXd WS_RRT::getRandom_wsPos( int center_ID, double delta )
{
  Eigen::VectorXd wsPos(3);
  for (int i = 0; i < 3; ++i) 
      { double a = configVector[center_ID](i) - delta;
        double b = configVector[center_ID](i) + delta;
        
        wsPos(i) = RANDNM( a, b );
      }
  return wsPos;
}


/**
 * @function getNearestNeighbor
 */
int WS_RRT::getNearestNeighbor( const Eigen::VectorXd &ws_sample )
{
  struct kdres* result = kd_nearest( kdTree, ws_sample.data() );
  uintptr_t nearest = (uintptr_t) kd_res_item_data( result );
  
  activeNode = nearest;
  return nearest;  
}

/**
 * @function tracePath
 */
void WS_RRT::tracePath( int node_ID, std::vector<Eigen::VectorXd> &path, bool reverse )
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
 */
bool WS_RRT::checkCollisions( const Eigen::VectorXd &q )
{
  world->robots[robot_ID]->setConf( links_ID, q );
  world->updateAllCollisionModels();
  return world->checkCollisions();
}

/**
 * @function add_Ranking
 * @brief Add node to RankingVector
 */
void WS_RRT::add_Ranking( int node_ID )
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
    if( distGoalVector[ rankingVector[parent] ] >= distGoalVector[ rankingVector[node] ] )
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
int WS_RRT::pop_Ranking()
{
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( rankingVector.size() == 0 )
    { cout<<"NO POP ELEMENT!"<<endl; return -1; }

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
       if( distGoalVector[ rankingVector[node] ] >= distGoalVector[ rankingVector[child_1] ] )
        { u = child_1;  }
       if( distGoalVector[ rankingVector[u] ]  >= distGoalVector[ rankingVector[child_2] ] )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( distGoalVector[ rankingVector[node] ] >= distGoalVector[ rankingVector[child_1] ] )
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
