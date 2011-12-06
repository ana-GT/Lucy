/**
 * @file JG_RRT.cpp
 * @author achq
 * @brief Test bench 2: Jacobian guy ( Level: 1 )
 */
#include "JG_RRT.h"

bool JG_RRT::checkPathSegment( Eigen::VectorXd config1, Eigen::VectorXd config2) const {
	int n = (int)((config2 - config1).norm() / stepSize);
	for(int i = 0; i < n; i++) {
		Eigen::VectorXd conf = (double)(n - i)/(double)n * config1 + (double)(i)/(double)n * config2;
		world->robots[robot_ID]->setConf(links_ID, conf, true);
		if(world->checkCollisions()) {
			return false;
		}
	}
	return true;
}

void JG_RRT::smoothPath( std::vector<Eigen::VectorXd> &path ) const {
	std::vector<Eigen::VectorXd>::iterator temp, config1;
	std::vector<Eigen::VectorXd>::iterator config2 = path.begin();

	while(true) {
		config1 = config2;
		config2++;
		if(config2 == path.end()) return;
		temp = config2;
		config2++;
		if(config2 == path.end()) return;
		
		while(checkPathSegment( *config1, *config2) ) {
			path.erase(temp);                        
			temp = config2;
			config2++;
			if(config2 == path.end()) return;
		}
	}
}



/**
 * @function JG_RRT
 * @brief Constructor
 */
JG_RRT::JG_RRT()
{
  this->stepSize = 0.05;
  this->p_goal = 0.5;
  this->distanceThresh = 0.02;  
  this->maxNodes = 10000;
  this->activeNode = 0;
}

/**
 * @function ~JG_RRT
 * @brief Destructor
 */
JG_RRT::~JG_RRT()
{
  cleanup();
}

/**
 * @function initialize
 * @brief Unneeded explanation here
 */
void JG_RRT::initialize( World* _world, 
		    	const int &_robot_ID, 
                    	const std::vector<int> &_links_ID,
                        const Eigen::Transform<double, 3, Eigen::Affine> &_TWBase,
                        const Eigen::Transform<double, 3, Eigen::Affine> &_Tee )
{
  //-- Clean up from previous searches
  cleanup();

  //-- Copy input
  this->world = _world;   
  this->robot_ID = _robot_ID;
  this->links_ID = _links_ID;
  this->ndim = _links_ID.size();
  this->TWBase = _TWBase;
  this->Tee = _Tee;
  this->kdTree = kd_create( ndim );
  printf( "ndim: %d \n", ndim );
  printf(" robot ID: %d \n", robot_ID );
  cout<<"link ID: "<<links_ID[0]<<","<<links_ID[6]<<endl;

  //-- Seed the random generator
  srand( time(NULL) );

  // addNode( _startConfig, -1 );
}

/**
 * @function cleanup
 * @brief Take out the trash for a fresh new planner
 */
void JG_RRT::cleanup()
{
  links_ID.clear(); links_ID.resize(0);
  
  parentVector.clear(); parentVector.resize(0);
  configVector.clear(); configVector.resize(0);
  rankingVector.clear(); rankingVector.resize(0); 
  goalDistVector.clear(); goalDistVector.resize(0);
}

/**
 * @function plan
 * @brief
 */
bool JG_RRT::plan( const Eigen::VectorXd &_startConfig, 
	           const Eigen::Transform<double, 3, Eigen::Affine> &_goalPose,
                   const std::vector< Eigen::VectorXd > &_guidingNodes, 
                   std::vector<Eigen::VectorXd> &path )
{
  /** Store information */
  this->startConfig = _startConfig;
  this->goalPose = _goalPose;
  this->goalPosition = _goalPose.translation();


  //-- Initialize the search tree
  addNode( startConfig, -1 );

  //-- Add the guiding nodes
  addGuidingNodes( _guidingNodes );

  double p;
  while( goalDistVector[activeNode] > distanceThresh )
   {
     //-- Generate the probability
     p = RANDNM( 0.0, 1.0 );

     //-- Apply either extension to goal (J) or random connection 
     if( p < p_goal )
       { 
         if( extendGoal() == true )
          { break;} 
       }
     else
       { tryStep(); /*extendRandom();*/ }

     // Check if we are still inside 
     if( configVector.size() > maxNodes )
       { cout<<"-- Exceeded "<<maxNodes<<" allowed - ws_dist: "<<goalDistVector[rankingVector[0]]<<endl;
         break; }
   }

  //-- If a path is found
  if( goalDistVector[activeNode] < distanceThresh ) 
    { tracePath( activeNode, path );
      cout<<"JG - Got a path! - nodes: "<<path.size()<<" tree size: "<<configVector.size()<<endl; 
      return true; }
  else
    { cout<<"--(!) JG :No successful path found! "<<endl;
      return false; }
}

/**
 * @function wsDistance
 */
double JG_RRT::wsDistance( Eigen::VectorXd q )
{
  Eigen::Transform<double, 3, Eigen::Affine> T;
  robinaLeftArm_fk( q, TWBase, Tee, T );
  Eigen::VectorXd ws_diff = ( goalPosition - T.translation() );
  double ws_dist = ws_diff.norm();
    
  return ws_dist; 
}

/**
 * @function wsDiff
 */
Eigen::VectorXd JG_RRT::wsDiff( Eigen::VectorXd q )
{
  Eigen::Transform<double, 3, Eigen::Affine> T;
  robinaLeftArm_fk( q, TWBase, Tee, T );
  Eigen::VectorXd ws_diff = ( goalPosition - T.translation() );
    
  return ws_diff; 
}

/**
 * @function connect
 * @brief 
 */
bool JG_RRT::connect()
{
  Eigen::VectorXd qtry = getRandomConfig();
  return connect( qtry ); 
}

/**
 * @function connect
 * @brief 
 */
bool JG_RRT::connect( const Eigen::VectorXd &target )
{
  int NNidx = getNearestNeighbor(target);
  StepResult result = STEP_PROGRESS;
  int i = 0;
  while(result == STEP_PROGRESS) 
  {
    result = tryStep(target, NNidx);
    NNidx = configVector.size() - 1;
    i++;
  }

 return (result == STEP_REACHED);
}

/**
 * @function tryStep
 */
JG_RRT::StepResult JG_RRT::tryStep()
{
  Eigen::VectorXd qtry = getRandomConfig();
  return tryStep(qtry);
}

/**
 * @function tryStep
 */
JG_RRT::StepResult JG_RRT::tryStep( const Eigen::VectorXd &qtry )
{
  int NNidx = getNearestNeighbor( qtry );
  return tryStep( qtry, NNidx );
}

/**
 * @function tryStep
 */
JG_RRT::StepResult JG_RRT::tryStep( const Eigen::VectorXd &qtry, int NNidx )
{
  Eigen::VectorXd qnear( ndim );
  Eigen::VectorXd qnew( ndim );
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
int JG_RRT::addNode( const Eigen::VectorXd &qnew, int parent_ID)
{
  double goalDist = wsDistance( qnew );

  configVector.push_back( qnew );
  parentVector.push_back( parent_ID );
  goalDistVector.push_back( goalDist );   

  uintptr_t ID = configVector.size() - 1;
  kd_insert( kdTree, qnew.data(), (void*)ID );
  add_Ranking( ID );  

  activeNode = ID;
  return ID;
}

/**
 * @function connectGoal
 */
bool JG_RRT::connectGoal()
{
  Eigen::MatrixXd J(3,7); Eigen::MatrixXd Jinv(7,3);
  Eigen::MatrixXd delta_x; 
  Eigen::MatrixXd delta_q;
  Eigen::MatrixXd q_new;
  JG_RRT::StepResult result;
  double dist_new; double dist_old;

  result = STEP_PROGRESS;
  dist_new = 0; dist_old = DBL_MAX;

  int i = 0;
  while( result == STEP_PROGRESS && (dist_new < dist_old) )
  { //-- Get the closest node to the goal ( workspace metric )
    int NNidx = pop_Ranking();
    if( NNidx == -1 )
      { break; }
    Eigen::VectorXd q_closest = configVector[NNidx];
    dist_old = wsDistance( q_closest );   

    //-- Get the Jacobian
    robinaLeftArm_j( q_closest, TWBase, Tee, J );

    //-- Get the pseudo-inverse (easy way)
    pseudoInv( 3, 7, J, Jinv );

    //-- Get the workspace
    delta_x = wsDiff( q_closest );

    delta_q = Jinv*delta_x;
    //-- Scale -- with this 2 actually result will always be in PROGRESS, if not COLLISION
    double scal = 2*stepSize/delta_q.norm();
    delta_q = delta_q*scal;

    //-- Get q_new
    q_new = q_closest + delta_q;

    //-- Attempt a new step towards J
    result = tryStep( q_new, NNidx );
    i++;
    //-- Check
    if( result == STEP_PROGRESS )
      { dist_new = wsDistance( configVector[configVector.size() - 1] );
        if( dist_new < distanceThresh )
          { result = STEP_REACHED; break; } 
      }

  }
 return( result == STEP_REACHED );
}


/**
 * @function getNearestNeighbor
 */
int JG_RRT::getNearestNeighbor( const Eigen::VectorXd &qsample )
{
  struct kdres* result = kd_nearest( kdTree, qsample.data() );
  uintptr_t nearest = (uintptr_t) kd_res_item_data( result );
  
  activeNode = nearest;
  return nearest;
}
  
/**
 * @function getRandomConfig
 */ 
Eigen::VectorXd JG_RRT::getRandomConfig()
{
  Eigen::VectorXd config(ndim);
  for (int i = 0; i < ndim; ++i) 
     {
       config[i] = RANDNM(world->robots[robot_ID]->links[links_ID[i]]->jMin, world->robots[robot_ID]->links[links_ID[i]]->jMax);
     }
  return config;

}


/**
 * @function tracePath
 */   
void JG_RRT::tracePath( int node_ID, std::vector<Eigen::VectorXd> &path, bool reverse )
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
bool JG_RRT::checkCollisions( const Eigen::VectorXd &config )
{
  world->robots[robot_ID]->setConf( links_ID, config );
  return world->checkCollisions();
}

/**
 * @function extendGoal
 * @brief 
 */
bool JG_RRT::extendGoal()
{
  return  connectGoal();
}

/**
 * @function extendRandom
 * @brief
 */
bool JG_RRT::extendRandom()
{
  return connect();
  
}

/**
 * @function addGuidingNodes
 * @brief Add Guiding Nodes
 */
void JG_RRT::addGuidingNodes( std::vector<Eigen::VectorXd> guidingNodes )
{  
  for( int i = 0; i < guidingNodes.size(); i++ )
  {
    connect( guidingNodes[i] );
    printf("-- Added guide node [%d] tree size: %d \n", i, configVector.size() );

  } 

}

/**
 * @function add_Ranking
 * @brief Add node to RankingVector
 */
void JG_RRT::add_Ranking( int node_ID )
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
    if( goalDistVector[ rankingVector[parent] ] >= goalDistVector[ rankingVector[node] ] )
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
int JG_RRT::pop_Ranking()
{
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( rankingVector.size() == 0 )
    { return -1; }

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
       if( goalDistVector[ rankingVector[node] ] >= goalDistVector[ rankingVector[child_1] ] )
        { u = child_1;  }
       if( goalDistVector[ rankingVector[u] ]  >= goalDistVector[ rankingVector[child_2] ] )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( goalDistVector[ rankingVector[node] ] >= goalDistVector[ rankingVector[child_1] ] )
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
