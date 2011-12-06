/**
 * @file dijkstra.cpp
 */
#include "dijkstra.h"

const int Dijkstra3D::mNX[26] = { -1,-1,-1, -1,-1,-1, -1,-1,-1,   0, 0, 0,  0,   0,  0, 0, 0,   1, 1, 1, 1, 1, 1, 1, 1, 1 };
const int Dijkstra3D::mNY[26] = { -1, 0, 1, -1, 0, 1, -1, 0, 1,  -1, 0, 1, -1,   1, -1, 0, 1,  -1, 0, 1,-1, 0, 1,-1, 0, 1 };
const int Dijkstra3D::mNZ[26] = { -1,-1,-1,  0, 0, 0,  1, 1, 1,  -1,-1,-1,  0,   0,  1, 1, 1,  -1,-1,-1, 0, 0, 0, 1, 1, 1 };
const float Dijkstra3D::smSqr2 = sqrt(2);
const float Dijkstra3D::smSqr3 = sqrt(3);

/**
 * @function State3D
 */
Node3D::Node3D()
{}

void Node3D::init( int _x, int _y, int _z )
{
   this->x = _x;
   this->y = _y;
   this->z = _z;
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
   if( HT != NULL )
   { delete [] HT; }
   if( V != NULL )
   { delete [] V; };
}

/**
 * @function cleanup
 */
void Dijkstra3D::cleanup()
{

}

/**
 * @function setGrid3D
 */
void Dijkstra3D::setGrid3D( VoxelGrid<int>* _grid )
{
   grid = _grid;
   mDimX = grid->getNumCells( VoxelGrid<int>::DIM_X );
   mDimY = grid->getNumCells( VoxelGrid<int>::DIM_Y );
   mDimZ = grid->getNumCells( VoxelGrid<int>::DIM_Z );

   mTotalCells = mDimX*mDimY*mDimZ;
   mStride1 = mDimY*mDimZ;
   mStride2 = mDimZ;

   //-- Initialize my Hash Table for my binary heap Q
   HT = new int[mTotalCells];
   std::fill( HT, HT + mTotalCells , -1 );
   //-- Allocate space for V (total nodes)
   V = new Node3D[mTotalCells];
}

/**
 * @function setDF
 */
void Dijkstra3D::setDF( PFDistanceField* _df )
{
   this->distField = _df;
}

/**
 * @function setGoal
 */
void Dijkstra3D::setGoal( int _goalX, int _goalY, int _goalZ )
{
   this->goalX = _goalX;
   this->goalY = _goalY;
   this->goalZ = _goalZ;
}

/**
 * @function search
 */
void Dijkstra3D::search()
{
  printf("Search build \n");
  ts = clock();
   //-- Initialize your states
   for( int i = 0; i < mDimX; i++ )
   {  for( int j = 0; j < mDimY; j++ )
      {  for( int k = 0; k < mDimZ; k++ )
         {  
            V[ ref(i,j,k) ].init( i, j, k ) ;
            if( grid->getCell( i, j, k ) != 0 ) //-- If it is an obstacle
            {  V[ ref(i,j,k) ].color = 3; }      
         }
      }    
   } 
  tf = clock();
  printf("Building structure in Dijkstra: %f \n", (double) (tf - ts)/CLOCKS_PER_SEC );

  //-- Initialize the goal state
  int s = ref( goalX, goalY, goalZ ) ;
  V[ s ].color = 1; // gray: In Open List
  V[ s ].dist = 0; 
 
  //-- Insert to Q (unvisited nodes)
  insertOpenSet( s );
  
  //-- Iterate
 
  int u; int v;
  float new_dist;

  while( Q.size() != 0 )
  {
     u = popOpenSet( );

     //-- All neighbors
     std::vector< int > ng = neighbors( u );
 
     for( int i = 0; i < ng.size(); i++ )
     {
        v = ng[i];
        new_dist = ( edgeCost( u, v ) + V[ u ].dist );

        if( new_dist < V[ v ].dist )
        {
           V[ v ].dist = new_dist ;
           V[ v ].parent = u;      
        }  
  
        if( V[ v ].color == 2 )  //-- Nowhere
        { 
          V[ v ].color = 1;  //-- Add it to the OpenList (queue)
          insertOpenSet( v );
        }
        else if( V[ v ].color == 1 )
        {
           lowerKeyOpenSet( v );
        }
        else
        { /** Pretty much nothing by now */ }
  
     }  //-- End for  
     
     V[u].color = 0; //-- Closed List
  } //-- End while
  
}


/**
 * @function insertOpenSet
 */
void Dijkstra3D::insertOpenSet( int ID )
{
  int n; 
  int node; int parent;
  int temp;

  Q.push_back( ID );
  n = Q.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { HT[ID] = n; return; }

  // If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  {
    //parent = floor( (node - 1)/2 );
    parent = (node - 1)/2 ;
    qp = Q[parent]; qn = Q[node];   

    if( V[ qp ].dist >= V[ qn ].dist )
      {
        temp = qp;
        Q[parent] = qn; HT[qn] = parent; 
        Q[node] = temp; HT[temp] = node;
        node = parent; 
      }  
    else
     { break; }
  }   

  HT[ID] = node;
}

/**
 * @function popOpenSet 
 */
int Dijkstra3D::popOpenSet( )
{
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( Q.size() == 0 )
    { printf("(!)-- ERROR ! No more elements left \n");return -1; }

  // Save the pop-out element
  first = Q[0];
  
  // Reorder your binary heap
  bottom = Q.size() - 1;

  Q[0] = Q[bottom]; HT[Q[bottom]] = 0;
  Q.pop_back();
  n = Q.size();

  int u = 0;

  int qu;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( V[ Q[node] ].dist >= V[ Q[child_1] ].dist )
        { u = child_1;  }
       if( V[ Q[u] ].dist  >= V[ Q[child_2] ].dist )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( V[ Q[node] ].dist >= V[ Q[child_1] ].dist )
         { u = child_1; }
     }
     
    qu = Q[u];    
    if( node != u )
     { temp = Q[node]; 
       Q[node] = qu; HT[qu] = node;
       Q[u] = temp; HT[temp] = u;
     }

    else
     { break; } 
  }

  return first;
}

/**
 * @function lowerKeyOpenSet
 */
void Dijkstra3D::lowerKeyOpenSet( int ID )
{ 
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  n = HT[ID];

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

    qp = Q[parent]; qn = Q[node];
    // Always try to put new nodes up
    if( V[ qp ].dist > V[ qn ].dist )
      {
        temp = qp;
        Q[parent] = qn; HT[qn] = parent;
        Q[node] = temp; HT[temp] = node;
        node = parent; 
      }  
    else
     {  break; }
  }   

}

/**
 * @function neighbors
 */
std::vector<int> Dijkstra3D::neighbors( int ID )
{
   std::vector<int> Ng;

   int idx = V[ID].x;  
   int idy = V[ID].y; 
   int idz = V[ID].z;
 
   int nx; int ny; int nz;

   for( int i = 0; i < 26; i++ )  
   {
      nx = idx + mNX[i]; 
      ny = idy + mNY[i];
      nz = idz + mNZ[i];

      if( nx < 0 || nx > mDimX -1 || ny < 0 || ny > mDimY -1 || nz < 0 || nz > mDimZ -1  )
      { continue; }
      int nID = ref(nx,ny,nz);
      if( V[ nID ].color == 0 || grid->getCell(nx,ny,nz) == 1 || V[ nID ].color == 3 )
      { continue; }  
      else
      { Ng.push_back( nID ); }
   }

   return Ng;
}


/**
 * @function edgeCost
 */
float Dijkstra3D::edgeCost( int ID1, int ID2 )
{
   float cost;

   int diff = abs( V[ID1].x - V[ID2].x ) + abs( V[ID1].y - V[ID2].y ) + abs( V[ID1].z - V[ID2].z );
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
 * @function getPath
 */
std::vector<Eigen::Vector3i> Dijkstra3D::getPath( int x, int y, int z )
{
   return getPath( ref(x,y,z) );
}

/**
 * @function getPath
 */
std::vector<Eigen::Vector3i> Dijkstra3D::getPath( int ID )
{
   std::vector<Eigen::Vector3i> path;

   while( V[ID].parent > 0 )
   {
      path.push_back( Eigen::Vector3i(V[ID].x, V[ID].y, V[ID].z ) );
      ID = V[ID].parent;
   }
 
  path.push_back( Eigen::Vector3i(V[ID].x, V[ID].y, V[ID].z ) );


   return path;
}

