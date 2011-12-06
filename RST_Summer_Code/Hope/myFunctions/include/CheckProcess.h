/**
 * @function CheckProcess.h
 */
#ifndef CHECK_PROCESS_H_
#define CHECK_PROCESS_H_

#include <Eigen/Core>
#include <time.h>
#include <stdio.h>
#include <Tools/Robot.h>
#include <vector>
#include <distance_field/pf_distance_field.h>
#include <distance_field/distance_field.h>
#include "CheckObject.h"

using namespace distance_field;

class CheckProcess
{
  public:

  RAPID_model *slideBox;
  std::vector< CheckObject > objs;
  std::vector< CheckObject > links;
  int num_objs_;
  int num_links_;

  std::vector<int>bodies_ID_;

  double size_x_;
  double size_y_;
  double size_z_; 
  double origin_x_;
  double origin_y_;
  double origin_z_;
  double resolution_;

  CheckProcess( double size_x, double size_y, double size_z,
	    	double origin_x, double origin_y, double origin_z, 
	        double resolution );
  void build_slideBox();
  void getObjectsData( std::vector<Object*> objects );
  void getLinksData( Robot* robot, std::vector<int> bodies_ID );
  void reportObjects();

  //-- Voxel construction
  void build_voxel( std::vector<Object*> objects, VoxelGrid<int> &voxel );
  //-- Distance transform 
  std::vector< Eigen::Vector3i >objs_voxels;
  void build_df( PFDistanceField &df );
};

#endif /** CHECK_PROCESS_H */
