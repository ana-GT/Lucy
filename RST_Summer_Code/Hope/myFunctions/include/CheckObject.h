/**
 * @file CheckObject.h
 */

#ifndef CHECK_OBJECT_H_
#define CHECK_OBJECT_H_

#include <Tools/Collision/RAPID.H>
#include <Tools/Collision/obb.H>
#include <Tools/Object.h>
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
  void getModelData( Object *object );
  void getModelTriangles( Object *object );
  void getModelRadius();
  void getModelCenter();
  void getModelBoxVertices();
  void updateObjectData( Object* object );

};

#endif /** CHECK_OBJECT_H */
