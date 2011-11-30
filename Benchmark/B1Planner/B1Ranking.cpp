/**
 * @file B1Ranking.cpp
 */
#include <iostream>
#include "B1RRT.h"

/**
 * @function pushRanking
 * @brief Add node to Ranking
 */
void B1RRT::pushRanking( const Eigen::Vector3d &_entry )
{
  int n; 
  int node; int parent;
  Eigen::Vector3d temp;

  ranking.push_back( _entry );
  n = ranking.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { return; }

  // If not, start on the bottom and go up
  node = n;

  while( node != 0 )
  {
    parent = (int) floor( ( (node - 1)/2.0 ) );
    // Always try to put new nodes up
    if( ranking[parent](1) >= ranking[node](1) ) // If cost is lower than that of a parent, go up
      {
        temp = ranking[parent];
        ranking[parent] = ranking[node];
        ranking[node] = temp; 
        node = parent; 
      }  
    else
     { break; }
  }   
}

/**
 * @function popRanking
 * @brief Pop out the top node from ranking. Return its index
 */
int B1RRT::popRanking()
{
  if( ranking.size() == 0 )
    { std::cout<<" ------xx (!) Out, no nodes to pop out! xx------ "<<std::endl; return -1; }
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  Eigen::Vector3d temp;

  // Save the pop-out element
  first = ranking[0](0);
  
  // Reorder your binary heap
  bottom = ranking.size() - 1;

  ranking[0] = ranking[bottom];
  ranking.pop_back();
  n = ranking.size();

  int u = 0;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( ranking[node](1) >= ranking[child_1](1) )
        { u = child_1;  }
       if( ranking[u](1) >= ranking[child_2](1) )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( ranking[node](1) >= ranking[child_1](1) )
         { u = child_1; }
     }
    
    if( node != u )
     { temp = ranking[node];
       ranking[node] = ranking[u];
       ranking[u] = temp; 
     }

    else
     { break; } 
  }

  return first;
}


/**
 * @function removeRanking
 * @brief Remove ranking[ind] from Ranking
 */
void B1RRT::removeRanking( int _ind )
{
  int n; 
  int node; int parent;
  int child_1; int child_2;
  Eigen::Vector2d temp;

  //-- Locate the bottom node in the position to be removed
  ranking[_ind] = ranking[ ranking.size() - 1 ];

  //-- Delete the bottom node ( which is now copied in the removed position )
  ranking.pop_back();
  
  // Reorder your binary heap
  n = ranking.size();

  int u = _ind;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( ranking[node](1) >= ranking[child_1](1) )
        { u = child_1;  }
       if( ranking[u](1) >= ranking[child_2](1) )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( ranking[node](1) >= ranking[child_1](1) )
         { u = child_1; }
     }
    
    if( node != u )
     { temp = ranking[node];
       ranking[node] = ranking[u];
       ranking[u] = temp; 
     }

    else
     { break; } 
  } /** while end */

}

