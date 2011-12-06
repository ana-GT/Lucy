/**
 * @file search.cpp
 * @brief Searching guy
 */
#include "search.h"

/** Static variables of Search class */
std::vector< Eigen::VectorXd >Search::sMotionPrimitives;
const double Search::sThreshold = 0.05;
const int Search::sNumJoints = 7;
const int Search::sNumPrimitives = 12;
const double Search::sDeltaJoint = 0.09;
const int Search::sMaxIter = 500;

/**
 * @brief constructor
 */
Search::Search()
{
   setPrimitives();
}

/**
 * @brief Destructor
 */
Search::~Search()
{
   
}

/**
 * @function cleanup
 * @brief clean the leftovers (if any) of previous searches
 */
void Search::cleanup()
{
  states.resize(0);
  openSet.clear();
}


/**
 * @function initialize
 */
void Search::initialize( Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                         Eigen::Transform<double, 3, Eigen::Affine> _Tee  )
{
  cleanup();
  armState::set_Transforms( _TWBase, _Tee );
}


/**
 * @function plan
 */
bool Search::plan( Eigen::VectorXd startJoints, 
	           Eigen::Transform<double, 3, Eigen::Affine> goalPose,
                   std::vector< Eigen::VectorXd > & path )
{
   printf(" Starting plan \n");
   int x;

   //-- Save info
   mGoalPose = goalPose; 
   mGoalPos = goalPose.translation();

   //-- Create the first state
   armState startState;
   startState.initialize( startJoints, (int)states.size() );
   //-- Push it in the states vector
   states.push_back( startState );
   states[0].costG = 0;
   states[0].costH = costHeuristic( 0, mGoalPose );
   states[0].costF = states[0].costG + states[0].costH;

   //-- Push it in the OpenSet
   pushOpenSet( states[0].key ); 

   //-- Loop
   int count = 0;
   while( openSet.size() > 0  && count < sMaxIter )
   {
      count++;
      //-- Remove top node in openSet
      x = popOpenSet();
      printf("count: %d WS error: %f  x: %d \n", count, getWSError( states[x].xyz, mGoalPos ), x  );
      std::cout << states[x].joints.transpose() << std::endl;
      std::cout << states[x].xyz.transpose() << std::endl;
      std::cout << states[x].rpy.transpose() << std::endl;

      if( getWSError( states[x].xyz, mGoalPos ) < sThreshold )
      {  printf( "--(v) Found path -- Tracing! \n" );
         tracePath( x, path );
         printf(" End tracing \n");
	 return true;
      }   

      //-- Add node to closed set
      states[x].status = armState::IN_CLOSED_SET;
      
      std::vector< armState > neighbors = getNextStates( states[x] ); 
     
      //-- 
      for( int i = 0; i < neighbors.size(); i++ )
      {
         if( neighbors[i].status == armState::IN_CLOSED_SET )
         { continue; }
      
         int y = neighbors[i].key;
         double tentative_g_score = states[x].costG + edgeCost( x, y );
         
         if( states[y].status != armState::IN_OPEN_SET )
         {
            states[y].parent = x;
            states[y].costG = tentative_g_score;
            states[y].costH = costHeuristic( y, mGoalPose );
            states[y].costF = states[y].costG + states[y].costH;
            pushOpenSet( y );
         }
         else
         {
            if( tentative_g_score < states[y].costG )
            {
              states[y].parent = x;
              states[y].costG = tentative_g_score;
              states[y].costH = costHeuristic( y, mGoalPose );
              states[y].costF = states[y].costG + states[y].costH;
              //-- Reorder your openSet
              updateLowerOpenSet( y );
            } 
         }
       //std::cout<< "Neighbor: " << states[y].joints << std::endl;
       std::cout<< " F: " << states[y].costF << " G: " << states[y].costG << " H: " << states[y].costH << std::endl;
      } //-- End for every neighbor

   } //-- End big while

  printf( "--(!) iter: %d -- OpenList size: %d \n", count, openSet.size() );
  return false;

}


/**
 * @push_openSet
 * @brief push the node ID into the openSet binary heap
 */
void Search::pushOpenSet( int _key )
{
  int n; 
  int node; int parent;
  int temp;

  //-- Sign the flag
  states[_key].status = armState::IN_OPEN_SET;

  openSet.push_back( _key );
  n = openSet.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { return; }

  // If not, start on the bottom and go up
  node = n;

  while( node != 0 )
  {
    parent = floor( (node - 1)/2 );
    // Always try to put new nodes up
    if( states[ openSet[parent] ].costF >= states[ openSet[node] ].costF )
      {
        temp = openSet[parent];
        openSet[parent] = openSet[node];
        openSet[node] = temp; 
        node = parent; 
      }  
    else
     { break; }
  }   

}

/**
 * @function popOpenSet
 */
int Search::popOpenSet()
{
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( openSet.size() == 0 )
    { printf("(!)-- ERROR ! No more elements left \n"); return -1; }

  // Save the pop-out element
  first = openSet[0];
  
  // Reorder your binary heap
  bottom = openSet.size() - 1;

  openSet[0] = openSet[bottom];
  openSet.pop_back();
  n = openSet.size();

  int u = 0;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( states[ openSet[node] ].costF >= states[ openSet[child_1] ].costF )
        { u = child_1;  }
       if( states[ openSet[u] ].costF  >= states[ openSet[child_2] ].costF )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( states[ openSet[node] ].costF >= states[ openSet[child_1] ].costF )
         { u = child_1; }
     }
    
    if( node != u )
     { temp = openSet[node];
       openSet[node] = openSet[u];
       openSet[u] = temp; 
     }

    else
     { break; } 
  }

  return first;

}

/**
 * @function updateLowerOpenSet
 * @brief Update a node with a lower value
 */
void Search::updateLowerOpenSet( int key )
{ 
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  for( int i = 0; i < openSet.size(); i++ )
  {
    if( openSet[i] == key )
    { n = i; break; }
  }

  //printf(" Start pos: %d f: %f \n ", n, nodes_[ openSet_[n] ].f_score );
  //-- If it happens to be the first element
  if( n == 0 )
    { return; }

  //-- If not, start on the bottom and go up
  node = n;

  while( node != 0 )
  { 
    parent = floor( (node - 1)/2 );
    // Always try to put new nodes up
    if( states[ openSet[parent] ].costF > states[ openSet[node] ].costF )
      {
        //printf(" Parent pos: %d f: %f \n ", parent, nodes_[ openSet_[parent] ].f_score );
        temp = openSet[parent];
        openSet[parent] = openSet[node];
        openSet[node] = temp; 
        node = parent; 
      }  
    else
     {  // printf(" End pos: %d f: %f \n ", node, nodes_[ openSet_[node] ].f_score ); 
        break; }
  }   

}


/**
 * @function getNextStates
 * @brief Get the "neighboring states"
 */
std::vector< armState > Search::getNextStates( const armState &x )
{
   bool exists;
   Eigen::VectorXd oldJoints( sNumJoints ); oldJoints = x.joints;
   Eigen::VectorXd newJoints( sNumJoints );
   std::vector< armState > nextStates;

   //-- 1. Generate the newly joints
   for( int i = 0; i < sNumPrimitives; i++ )
   {
      exists = false;
      newJoints = oldJoints + Search::sMotionPrimitives[i];


      //-- 2. Check if there is not a state with the joint configuration 
      for( int j = 0; j < states.size(); j++ )
      {
         if( states[j].joints == newJoints )
         {  nextStates.push_back( states[j] );
            exists = true; break; 
         }
      }   

      //-- If it does not exist, create a new guy and push it
      if( !exists )
      {
         armState newState;
         newState.initialize( newJoints, states.size() );
         nextStates.push_back( newState ); 
         states.push_back( newState );
      }  

   }

   return nextStates;
}


/**
 * @function tracePath
 * @brief Trace path, literally
 */
bool Search::tracePath( const int &key, std::vector< Eigen::VectorXd> & path )
{
  printf("Trace path \n");
  std::vector< Eigen::VectorXd > backPath;
  path.resize(0);
  backPath.resize(0);

  int n = key;
  Eigen::VectorXi wp; wp.resize(3);

  while( n != -1 ) 
  {
    backPath.push_back( states[key].joints );
    n = states[n].parent;
  }

  int b = backPath.size();

  for( int i = 0; i < b; i++ )
     { path.push_back( backPath[ b - 1- i] ); }

  if( path.size() > 0 )
    { return true; }

  return false;  
}  

