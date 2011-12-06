/**
 * @function CheckProcess.h
 */
#ifndef CHECK_PROCESS_H_
#define CHECK_PROCESS_H_

#include <Eigen/Core>
#include <time.h>
#include <stdio.h>
#include <Tools/Robot.h>

#include "CheckObject.h"
#include <vector>

class CheckProcess
{

  public:

  RAPID_model *slideBox;
  std::vector< CheckObject > objs;
  std::vector< CheckObject > links;
  int num_objs_;
  int num_links_;

  std::vector<int>bodies_ID_;

  BinaryVoxel* voxel;
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
  void initialize();
  void getObjectsData( std::vector<Object*> objects );
  void getLinksData( Robot* robot, std::vector<int> bodies_ID );
  void reportObjects();

  //-- Voxel construction
  void fillVoxel( std::vector<Object*> objects );
  void getLinksVoxels( Robot* robot );

};

#endif /** CHECK_PROCESS_H */
