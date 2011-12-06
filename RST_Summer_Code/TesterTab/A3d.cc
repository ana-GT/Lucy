/**
 * @file A3d.cc
 * @author achq
 */

#include "A3d.h"

const double A_3D::OBST_COST;
const double A_3D::FREE_COST;
const double A_3D::RISKY_COST;
const double A_3D::K1;
const double A_3D::K2;
const double A_3D::FREE_ISDT_COST; 
const double A_3D::RISKY_ISDT_COST; 

/** ---------------------
 * @function initialize
 * ----------------------
 */
void A_3D::initialize( BinaryVoxel* voxel,
		       World* _world,
                       int _robot_ID,
                       std::vector<int> _links_ID,
                       Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                       Eigen::Transform<double, 3, Eigen::Affine> _Tee  )
{
  //-- Clean from previous searches
  //cleanup();

  //-- Save input info
  this->voxel_ = voxel;
  this->x_dim_ = voxel_->getNumCells( BinaryVoxel::DIM_X );
  this->y_dim_ = voxel_->getNumCells( BinaryVoxel::DIM_Y );
  this->z_dim_ = voxel_->getNumCells( BinaryVoxel::DIM_Z );

  this->num_cells_ = ( x_dim_ )*( y_dim_ )*( z_dim_ );

  this->stride1_ = y_dim_*z_dim_;
  this->stride2_ = z_dim_;

  //-- Initialize nodes status
  nodes_ = new node_info[ num_cells_ ];

  node_info default_node;
  default_node.parent_ref = -1;
  default_node.g_score = 0;
  default_node.h_score = 0;
  default_node.f_score = 0;
  default_node.state = IN_NO_SET; 
  
  std::fill( nodes_, nodes_ + num_cells_, default_node );


    for( int i = 0; i <= x_dim_; i++ )
     { for( int j = 0; j <= y_dim_; j++ )
       { for( int q = 0; q <= z_dim_; q++ )
         { 
           if( voxel_->getVoxelCell( i, j, q ) == BinaryVoxel::OCCUPIED )
            { voxel_->setPadVoxelCell( i, j, q, 5, BinaryVoxel::OCCUPIED, BinaryVoxel::RISKY ); }
         } 
       }
     }

}

/**
 * @function plan
 */
bool A_3D::plan( Eigen::Vector3i start, 
	         Eigen::Vector3i goal,
                 Eigen::VectorXd startConfig,
                 std::vector< Eigen::VectorXi > & path )
{
   cleanup();
   this->start_ = start; 
   this->goal_ = goal; 

  //-- Initialize the openSet
  int x = ref( start_ );
  nodes_[x].g_score = 0;
  nodes_[x].h_score = heuristic_cost_estimate( x, goal_ );
  nodes_[x].f_score = nodes_[x].g_score + nodes_[x].h_score;
  nodes_[x].parent_ref = -1;
  push_openSet( x );

  int count = 0;
  while( openSet_.size() > 0 )
  {
    count++;
      
    //-- Remove top node in openSet
    x = pop_openSet();
    Eigen::Vector3i x_cell = ref2cell( x );

    if( x_cell == goal_ )
      { reconstruct_path( x, path ); printf(" Iterations: %d  -- path size: %d cost: %.3f \n", count, path.size(), nodes_[ ref(goal_) ].f_score ); return true; }

    //-- Add node to closeSet
    nodes_[x].state = IN_CLOSED_SET;
    
    std::vector< Eigen::Vector3i > ngb = neighbors( x_cell );
    //-- 
    for( int i = 0; i < ngb.size(); i++ )
       {
         Eigen::Vector3i y_cell = ngb[i];
         
         int y = ref( y_cell );

         if( nodes_[ y ].state == IN_CLOSED_SET )
           { continue; }

         double tentative_g_score = nodes_[x].g_score + cost_between( x_cell, y_cell );

         if( nodes_[y].state != IN_OPEN_SET ) 
           { 
             nodes_[y].parent_ref = x;
             nodes_[y].g_score = tentative_g_score;
             nodes_[y].h_score = heuristic_cost_estimate( y, goal_ );
             nodes_[y].f_score = nodes_[y].g_score + nodes_[y].h_score;
	     push_openSet( y ); 
           }
         else
          { 
            if(tentative_g_score < nodes_[y].g_score )
               { 
                 nodes_[y].parent_ref = x;
                 nodes_[y].g_score = tentative_g_score;
                 nodes_[y].h_score = heuristic_cost_estimate( y, goal_ );
                 nodes_[y].f_score = nodes_[y].g_score + nodes_[y].h_score;       
	         //-- Re-order your OpenSet
                 update_lower_openSet( y );
               }
          }      
       } /** End for */
  }

 printf(" --(!) No path found -- Iterations: %d \n", count );
 return false;
}

/**
 * @function cleanup
 * @brief clean the leftovers (if any) of previous searches
 */
void A_3D::cleanup()
{
  openSet_.clear();

  for( int i = 0; i < num_cells_; i++ )
    { nodes_[i].state = IN_NO_SET ; }
}

/**
 * @function push_openSet
 * @brief Push the new node location into nodeVector and call the fx
 */
int A_3D::push_openSet( Eigen::Vector3i node )
{  
  int ID = ref( node );

  push_openSet( ID ); 
  return ID; 
}

/**
 * @function ref
 * @brief Tell me where it is
 */
int A_3D::ref( int cell_x, int cell_y, int cell_z )
{
  return ( cell_x*stride1_ + cell_y*stride2_ + cell_z );
}

/**
 * @function ref
 */
int A_3D::ref( Eigen::Vector3i cell )
{
  return ( cell(0)*stride1_ + cell(1)*stride2_ + cell(2) );
}

/**
 * @function ref2cell
 */
void A_3D::ref2cell( const int &ref, int &cell_x, int &cell_y, int &cell_z )
{
  cell_x = ref / stride1_ ;
  cell_y = ( ref %stride1_ ) / stride2_;
  cell_z = ref % stride2_ ; 
}

/**
 * @function ref2cell
 */
Eigen::Vector3i A_3D::ref2cell( const int &ref )
{
  int cell_x = ref / stride1_ ;
  int cell_y = ( ref %stride1_ ) / stride2_;
  int cell_z = ref % stride2_ ; 

  Eigen::Vector3i cell( cell_x, cell_y, cell_z );

  return cell;
}

/**
 * @push_openSet
 * @brief push the node ID into the openSet binary heap
 */
void A_3D::push_openSet( int ID )
{
  int n; 
  int node; int parent;
  int temp;

  //-- Sign the flag
  nodes_[ ID ].state = IN_OPEN_SET;

  openSet_.push_back( ID );
  n = openSet_.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { return; }

  // If not, start on the bottom and go up
  node = n;

  while( node != 0 )
  {
    parent = floor( (node - 1)/2 );
    // Always try to put new nodes up
    if( nodes_[ openSet_[parent] ].f_score >= nodes_[ openSet_[node] ].f_score )
      {
        temp = openSet_[parent];
        openSet_[parent] = openSet_[node];
        openSet_[node] = temp; 
        node = parent; 
      }  
    else
     { break; }
  }   

}

/**
 * @update_lower_openSet
 * @brief Update a node with a lower value
 */
void A_3D::update_lower_openSet( int ID )
{ 
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  for( int i = 0; i < openSet_.size(); i++ )
  {
    if( openSet_[i] == ID )
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
    if( nodes_[ openSet_[parent] ].f_score > nodes_[ openSet_[node] ].f_score )
      {
        //printf(" Parent pos: %d f: %f \n ", parent, nodes_[ openSet_[parent] ].f_score );
        temp = openSet_[parent];
        openSet_[parent] = openSet_[node];
        openSet_[node] = temp; 
        node = parent; 
      }  
    else
     {  // printf(" End pos: %d f: %f \n ", node, nodes_[ openSet_[node] ].f_score ); 
        break; }
  }   

}


/**
 * @function pop_openSet
 */
int A_3D::pop_openSet()
{
  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( openSet_.size() == 0 )
    { printf("(!)-- ERROR ! No more elements left \n");return -1; }

  // Save the pop-out element
  first = openSet_[0];
  
  // Reorder your binary heap
  bottom = openSet_.size() - 1;

  openSet_[0] = openSet_[bottom];
  openSet_.pop_back();
  n = openSet_.size();

  int u = 0;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( nodes_[ openSet_[node] ].f_score >= nodes_[ openSet_[child_1] ].f_score )
        { u = child_1;  }
       if( nodes_[ openSet_[u] ].f_score  >= nodes_[ openSet_[child_2] ].f_score )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( nodes_[ openSet_[node] ].f_score >= nodes_[ openSet_[child_1] ].f_score )
         { u = child_1; }
     }
    
    if( node != u )
     { temp = openSet_[node];
       openSet_[node] = openSet_[u];
       openSet_[u] = temp; 
     }

    else
     { break; } 
  }

  return first;

}

/**
 * @function neighbors 
 */
std::vector< Eigen::Vector3i > A_3D::neighbors( Eigen::Vector3i n )
{   
   Eigen::Vector3i nt;

   int nx[26] = { 1, 1, 1, 0, 0, 0, -1, -1, -1, 1, 1, 1, 0, 0, -1, -1, -1, 1, 1, 1, 0, 0, 0, -1, -1, -1 };
   int ny[26] = { 1, 0, -1, 1, 0, -1, 1, 0, -1, 1, 0, -1, 1, -1, 1, 0, -1, 1, 0, -1, 1, 0, -1, 1, 0, -1 };
   int nz[26] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1 };    

   std::vector< Eigen::Vector3i > ngb; ngb.resize(0);

   for( int i = 0; i < 26; i++ )
   {
     nt<< n(0) + nx[i], n(1) + ny[i], n(2) + nz[i];
     if( (*voxel_).isVoxelCellValid( nt(0), nt(1), nt(2) ) == true && (*voxel_).getVoxelCell( nt(0), nt(1), nt(2) ) != BinaryVoxel::OCCUPIED )
     { ngb.push_back(nt); }
   }
 
   return ngb; 
}

/**
 * @function cost_between
 */
double A_3D::cost_between( Eigen::Vector3i x, Eigen::Vector3i y )
{
  double factor;

  int diff = ( abs(x(0) - y(0)) + abs(x(1) - y(1)) + abs(x(2) - y(2)) );

  if(  diff > 2 )
   { factor = 1.71; }
  else if( diff > 1 )
   { factor = 1.41; }
  else
   { factor = 1; }

  BinaryVoxel::CellState y_status = (*voxel_).getVoxelCell( y(0), y(1), y(2) ); 

  double dist = (*voxel_).getISDTCell( y(0), y(1), y(2) );
 
  if( y_status == BinaryVoxel::FREE )
  { return ( ( K1*( (*voxel_).max_ISDT_entry_ - dist ) + K2 )*FREE_COST ); }

  else if( y_status == BinaryVoxel::RISKY )
  { return ( ( K1*( (*voxel_).max_ISDT_entry_ - dist ) + K2 )*RISKY_COST ); }
}

/**
 * @function heuristic_cost_estimate
 */
double A_3D::heuristic_cost_estimate( int ID, Eigen::Vector3i target )
{
  Eigen::Vector3i here;
  ref2cell( ID, here(0), here(1), here(2) );
  Eigen::Vector3i diff = target - here;

  double dist = (*voxel_).getISDTCell( here(0), here(1), here(2) );
  
  return( K2*sqrt( diff(0)*diff(0) + diff(1)*diff(1) + diff(2)*diff(2) ) )*FREE_COST;
}

/**
 * @function reconstruct_path
 */
bool A_3D::reconstruct_path( int ID, std::vector< Eigen::VectorXi > & path )
{
  std::vector< Eigen::VectorXi > backPath;

  path.resize(0);
  backPath.resize(0);

  int n = ID;
  Eigen::VectorXi wp; wp.resize(3);

  while( n != -1 ) 
  {
    ref2cell( n, wp(0), wp(1), wp(2) );
    backPath.push_back( wp );
    n = nodes_[n].parent_ref;
  }

  int b = backPath.size();
  for( int i = 0; i < b; i++ )
     { path.push_back( backPath[ b - 1- i] ); }

  if( path.size() > 0 )
    { return true; }

  return false;
}

/**
 * @function ShowPath
 */
void A_3D::show_path( std::vector< Eigen::VectorXi > path )
{
  printf("--- Printing path --- \n");
  for( int i = 0; i<path.size();i++ )
     {
       printf("[%d] (%d,%d,%d) - State: %d \n",i, path[i](0), path[i](1), path[i](2), (*voxel_).getVoxelCell( path[i](0), path[i](1), path[i](2) ) );
     }
}
