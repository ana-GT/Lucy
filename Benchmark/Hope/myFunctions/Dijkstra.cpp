/**
 * @file Dijkstra.cpp
 * @brief Implements methods of Dijkstra class, which performs a all-to-one search
 * @author achq
 */
#include "Dijkstra.h"

const int Dijkstra3D::mNX[26] = { -1,-1,-1, -1,-1,-1, -1,-1,-1,   0, 0, 0,  0,   0,  0, 0, 0,   1, 1, 1, 1, 1, 1, 1, 1, 1 };
const int Dijkstra3D::mNY[26] = { -1, 0, 1, -1, 0, 1, -1, 0, 1,  -1, 0, 1, -1,   1, -1, 0, 1,  -1, 0, 1,-1, 0, 1,-1, 0, 1 };
const int Dijkstra3D::mNZ[26] = { -1,-1,-1,  0, 0, 0,  1, 1, 1,  -1,-1,-1,  0,   0,  1, 1, 1,  -1,-1,-1, 0, 0, 0, 1, 1, 1 };
const float Dijkstra3D::smSqr2 = sqrt(2);
const float Dijkstra3D::smSqr3 = sqrt(3);

/**
 * @function Constructor of Node3D
 */
Node3D::Node3D()
{}

/**
 * @function Init
 * @brief Initialize our Node3D object
 */
void Node3D::Init( int x, int y, int z )
{
   this->x = x;
   this->y = y;
   this->z = z;
   this->dist = INF_VAL;
   this->parent = -1;
   this->color = 2; // WHITE: Nowhere
}

/**
 * @function Destructor Node3D
 */
Node3D::~Node3D()
{
}

/*---------------------------------------*/

/**
 * @function Constructor
 */
Dijkstra3D::Dijkstra3D()
{
   this->mBaseCost = 1.0;
   this->mMinObstDist = 0.05;
}

/**
 * @function Destructor
 */
Dijkstra3D::~Dijkstra3D()
{
   if( mHT != NULL )
   { delete [] mHT; }
   if( mV != NULL )
   { delete [] mV; };
}

/**
 * @function Cleanup
 */
void Dijkstra3D::Cleanup()
{}

/**
 * @function SetGrid3D
 * @brief Save information of the grid used to build the graph and info about it.
 */
void Dijkstra3D::SetGrid3D( VoxelGrid<int>* grid )
{
   mGrid = grid;
   mDimX = grid->getNumCells( VoxelGrid<int>::DIM_X );
   mDimY = grid->getNumCells( VoxelGrid<int>::DIM_Y );
   mDimZ = grid->getNumCells( VoxelGrid<int>::DIM_Z );

   mTotalCells = mDimX*mDimY*mDimZ;
   mStride1 = mDimY*mDimZ;
   mStride2 = mDimZ;

   //-- Initialize my Hash Table for my binary heap Q
   mHT = new int[mTotalCells];
   std::fill( mHT, mHT + mTotalCells , -1 );
   //-- Allocate space for V (total nodes)
   mV = new Node3D[mTotalCells];
}

/**
 * @function SetDF
 */
void Dijkstra3D::SetDF( PFDistanceField* df )
{
   mDistField = df;
}

/**
 * @function SetGoal
 */
void Dijkstra3D::SetGoal( int goalX, int goalY, int goalZ )
{
   mGoalX = goalX;
   mGoalY = goalY;
   mGoalZ = goalZ;
}

/**
 * @function Search
 */
void Dijkstra3D::Search()
{
   //-- Initialize your states
   for( int i = 0; i < mDimX; i++ )
   {  for( int j = 0; j < mDimY; j++ )
      {  for( int k = 0; k < mDimZ; k++ )
         {  
            mV[ GetRef(i,j,k) ].Init( i, j, k ) ;
            if( mGrid->getCell( i, j, k ) != 0 ) //-- If it is an obstacle
	      {  mV[ GetRef(i,j,k) ].color = 3; } //--Mark as Obstacle (3)     
         }
      }    
   } 

  //-- Initialize the goal state
  int s = GetRef( mGoalX, mGoalY, mGoalZ ) ;
  mV[ s ].color = 1; // gray: In Open List
  mV[ s ].dist = 0;  // start: 0 distance of itself
 
  //-- Insert to Q (unvisited nodes)
  InsertOpenSet( s );
  
  //-- Iterate
  int u; int v;
  float new_dist;

  while( mQ.size() != 0 )
  {
     u = PopOpenSet( );

     //-- All neighbors
     std::vector< int > ng = GetNeighbors( u );
 
     for( int i = 0; i < ng.size(); i++ )
     {
        v = ng[i];
        new_dist = ( GetEdgeCost( u, v ) + mV[ u ].dist );

        if( new_dist < mV[ v ].dist )
        {
           mV[ v ].dist = new_dist ;
           mV[ v ].parent = u;      
        }  
  
        if( mV[ v ].color == 2 )  //-- Nowhere
        { 
          mV[ v ].color = 1;  //-- Add it to the OpenList (queue)
          InsertOpenSet( v );
        }
        else if( mV[ v ].color == 1 )
        {
           LowerKeyOpenSet( v );
        }
        else
        { /** Pretty much nothing by now */ }
  
     }  //-- End for  
     
     mV[u].color = 0; //-- Closed List
  } //-- End while
  
}


/**
 * @function InsertOpenSet
 */
void Dijkstra3D::InsertOpenSet( int ID )
{
  int n; 
  int node; int parent;
  int temp;

  mQ.push_back( ID );
  n = mQ.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { mHT[ID] = n; return; }

  // If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  {
    //parent = floor( (node - 1)/2 );
    parent = (node - 1)/2 ;
    qp = mQ[parent]; qn = mQ[node];   

    if( mV[ qp ].dist >= mV[ qn ].dist )
      {
        temp = qp;
        mQ[parent] = qn; mHT[qn] = parent; 
        mQ[node] = temp; mHT[temp] = node;
        node = parent; 
      }  
    else
     { break; }
  }   

  mHT[ID] = node;
}

/**
 * @function PopOpenSet 
 */
int Dijkstra3D::PopOpenSet( )
{
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( mQ.size() == 0 )
    { printf("(!)-- ERROR ! No more elements left \n");return -1; }

  // Save the pop-out element
  first = mQ[0];
  
  // Reorder your binary heap
  bottom = mQ.size() - 1;

  mQ[0] = mQ[bottom]; mHT[mQ[bottom]] = 0;
  mQ.pop_back();
  n = mQ.size();

  int u = 0;

  int qu;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( mV[ mQ[node] ].dist >= mV[ mQ[child_1] ].dist )
        { u = child_1;  }
       if( mV[ mQ[u] ].dist  >= mV[ mQ[child_2] ].dist )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( mV[ mQ[node] ].dist >= mV[ mQ[child_1] ].dist )
         { u = child_1; }
     }
     
    qu = mQ[u];    
    if( node != u )
     { temp = mQ[node]; 
       mQ[node] = qu; mHT[qu] = node;
       mQ[u] = temp; mHT[temp] = u;
     }

    else
     { break; } 
  }

  return first;
}

/**
 * @function LowerKeyOpenSet
 */
void Dijkstra3D::LowerKeyOpenSet( int ID )
{ 
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  n = mHT[ID];

  //-- If it happens to be the first element
  if( n == 0 )
    { return; } //-- HT[ID] = 0, the same

  //-- If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  { 
    //parent = floor( (node - 1)/2 );
    parent = floor( (node - 1)/2 );

    qp = mQ[parent]; qn = mQ[node];
    // Always try to put new nodes up
    if( mV[ qp ].dist > mV[ qn ].dist )
      {
        temp = qp;
        mQ[parent] = qn; mHT[qn] = parent;
        mQ[node] = temp; mHT[temp] = node;
        node = parent; 
      }  
    else
     {  break; }
  }   

}

/**
 * @function GetNeighbors
 * @brief Returns the IDs of the neighboring states of the input
 */
std::vector<int> Dijkstra3D::GetNeighbors( int id )
{
   std::vector<int> neighbors;

   int idx = mV[id].x;  
   int idy = mV[id].y; 
   int idz = mV[id].z;
 
   int nx; int ny; int nz;

   for( int i = 0; i < 26; i++ )  
   {
      nx = idx + mNX[i]; 
      ny = idy + mNY[i];
      nz = idz + mNZ[i];

      if( nx < 0 || nx > mDimX -1 || ny < 0 || ny > mDimY -1 || nz < 0 || nz > mDimZ -1  )
      { continue; }
      int n_id = GetRef(nx,ny,nz);
      if( mV[ n_id ].color == 0 || mGrid->getCell(nx,ny,nz) == 1 || mV[ n_id ].color == 3 )
      { continue; }  
      else
      { neighbors.push_back( n_id ); }
   }

   return neighbors;
}


/**
 * @function GetEdgeCost
 */
float Dijkstra3D::GetEdgeCost( int id1, int id2 )
{
   float cost;

   int diff = abs( mV[id1].x - mV[id2].x ) + abs( mV[id1].y - mV[id2].y ) + abs( mV[id1].z - mV[id2].z );
   if( diff == 1 )
   { cost = 1*mBaseCost; }
   else if( diff == 2 )
   { cost = smSqr2*mBaseCost; }
   else if( diff == 3 )
   { cost = smSqr3*mBaseCost; } 
   else
   { printf("This case is not possible in this configuration, you moron! \n");} 

   return cost;
}


/**
 * @function GetPath
 * @breif Returns a path from the current cell to the goal cell
 */
std::vector<Eigen::Vector3i> Dijkstra3D::GetPath( int x, int y, int z )
{
   return GetPath( GetRef(x,y,z) );
}

/**
 * @function GetPath
 * @brief Get a cell Path from the cell denoted by the id to the goal
 */
std::vector<Eigen::Vector3i> Dijkstra3D::GetPath( int id )
{
   std::vector<Eigen::Vector3i> path;

   while( mV[id].parent > 0 )
   {
      path.push_back( Eigen::Vector3i( mV[id].x, mV[id].y, mV[id].z ) );
      id = mV[id].parent;
   }
 
  path.push_back( Eigen::Vector3i( mV[id].x, mV[id].y, mV[id].z ) );

   return path;
}

