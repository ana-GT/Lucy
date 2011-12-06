/**
 * @file CheckObject.h
 */

#ifndef CHECK_OBJECT_H_
#define CHECK_OBJECT_H_

#include <grip/VCollide/RAPID.H>
#include <grip/VCollide/obb.H>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <iostream>

/**
 * @class CheckObject
 */
class CheckObject 
{
  public:
  RAPID_model *rapidObject;

  double center[3];
  double radius[3];
  double boxVer[8][3];
  double current_boxVer[8][3];
  double R[3][3]; double T[3];

  double min_x; double max_x;
  double min_y; double max_y;
  double min_z; double max_z;

  int minCell_x; int maxCell_x;
  int minCell_y; int maxCell_y;
  int minCell_z; int maxCell_z;

  /**< Constructor */
  CheckObject();  
  void reset_minmax();
  void getModelData( Model3DS *_model );
  void getModelTriangles( Model3DS *_model );
  void getModelRadius();
  void getModelCenter();
  void getModelBoxVertices();
  void updateObjectData( kinematics::BodyNode *_node );

};

#endif /** CHECK_OBJECT_H */
