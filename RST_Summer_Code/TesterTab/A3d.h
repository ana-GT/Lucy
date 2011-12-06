#ifndef A_3D_H
#define A_3D_H

#include <vector>
#include <list>
#include <stdlib.h>
#include <stdio.h>
#include <Eigen/Core>
#include "goWSPos.h"
#include "voxelSuite/BinaryVoxel.h"
#include <iostream>

class A_3D{

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    static const double OBST_COST = 100.0;
    static const double FREE_COST = 1.0;
    static const double RISKY_COST = 1.0;
    static const double K1 = 1.0;
    static const double K2 = 60.0;
    static const double FREE_ISDT_COST = 1.0;  // FREE_COST* factor
    static const double RISKY_ISDT_COST = 5.0; // RISKY_COST* factor

    int x_dim_;
    int y_dim_;
    int z_dim_;
    int num_cells_;
    int stride1_;
    int stride2_;

    //-- Start and End Points of the Path
    Eigen::Vector3i start_;
    Eigen::Vector3i goal_;

    typedef enum{
     IN_OPEN_SET,
     IN_CLOSED_SET,
     IN_NO_SET
    } node_state;

    struct node_info {
      int parent_ref;
      double g_score;
      double h_score;
      double f_score;
      node_state state;      
    } ;

    node_info* nodes_;
    BinaryVoxel* voxel_;

    //-- Lists
    std::vector< int > openSet_;

    //-- Functions
    void cleanup();  
    int ref( int cell_x, int cell_y, int cell_z );
    int ref( Eigen::Vector3i cell );
    void ref2cell( const int &ref, int &cell_x, int &cell_y, int &cell_z );
    Eigen::Vector3i ref2cell( const int &ref );
    void initialize( BinaryVoxel* voxel,
		     World* _world,
                     int _robot_ID,
                     std::vector<int> _links_ID,
                     Eigen::Transform<double, 3, Eigen::Affine> _TWBase,
                     Eigen::Transform<double, 3, Eigen::Affine> _Tee  );
    bool plan( Eigen::Vector3i start, 
	       Eigen::Vector3i goal,
	       Eigen::VectorXd startConfig,	
               std::vector< Eigen::VectorXi > & path );

    std::vector< Eigen::Vector3i > neighbors( Eigen::Vector3i n );

    int push_openSet( Eigen::Vector3i node );
    void push_openSet( int ID );
    int pop_openSet();
    void update_lower_openSet( int ID );

    double cost_between( Eigen::Vector3i x, Eigen::Vector3i y );
    double heuristic_cost_estimate( int ID, Eigen::Vector3i node );

    bool reconstruct_path( int ID, std::vector< Eigen::VectorXi > & path );
    void show_path( std::vector< Eigen::VectorXi > path );

};

#endif
