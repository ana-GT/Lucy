/**
 * @file CheckObject.cpp
 */

#include "CheckObject.h"
#include <planning/Model3DS.h>
#include <kinematics/BodyNode.h>

/**
 * @function CheckObject
 * @brief Constructor
 */
CheckObject::CheckObject()
{
}

/**
 * @function reset_minmax
 */
void CheckObject::reset_minmax()
{ 
  //-- Set default min-max
  min_x = DBL_MAX ; max_x = -DBL_MAX;
  min_y = DBL_MAX ; max_y = -DBL_MAX;
  min_z = DBL_MAX ; max_z = -DBL_MAX; 
}

/**
 * @function getModelData
 * @brief Get model information in object world. Call this guy after you filled your space with triangles
 */
void CheckObject::getModelData( Model3DS *_model )
{
  getModelTriangles( _model );
  getModelCenter();
  getModelRadius();
  getModelBoxVertices();
}

/**
 * @function getModelTriangles
 * @brief Get the triangles to form the RAPID object
 */
void CheckObject::getModelTriangles( Model3DS *_model )
{
  vector<Model3DS::Triangle> *trigs = new vector<Model3DS::Triangle>;

  _model->ReportTriangles( trigs );

  //-- Save it as a RAPID object
  this->rapidObject = new RAPID_model();
  this->rapidObject->BeginModel();

  //-- Save the triangles of each object
  int count = 0;
  for( int i = 0; i < (*trigs).size() ; i++ )
  { 
    this->rapidObject->AddTri( (*trigs)[i].v1, (*trigs)[i].v2, (*trigs)[i].v3, count );
    count++; 
  }
  this->rapidObject->EndModel();

}


/**
 * @function getModelCenter
 * @brief Get model center in the object world
 */
void CheckObject::getModelCenter()
{
  center[0] = 0; center[1] = 0; center[2] = 0;

  for( int i = 0; i < rapidObject->num_tris; i++ )
  {
    center[0] = center[0] + rapidObject->tris[i].p1[0] + rapidObject->tris[i].p2[0] + rapidObject->tris[i].p3[0];
    center[1] = center[1] + rapidObject->tris[i].p1[1] + rapidObject->tris[i].p2[1] + rapidObject->tris[i].p3[1];
    center[2] = center[2] + rapidObject->tris[i].p1[2] + rapidObject->tris[i].p2[2] + rapidObject->tris[i].p3[2];     
  } 

  center[0] = center[0]/ ( 3*rapidObject->num_tris );
  center[1] = center[1]/ ( 3*rapidObject->num_tris );
  center[2] = center[2]/ ( 3*rapidObject->num_tris );
}

/**
 * @function getModelRadius
 */
void CheckObject::getModelRadius()
{
  double temp_radius[3];
  radius[0] = 0; radius[1] = 0; radius[2] = 0;

  for( int i = 0; i < rapidObject->num_tris; i++ )
  {
    temp_radius[0] = fabs( rapidObject->tris[i].p1[0] - center[0] );
    temp_radius[1] = fabs( rapidObject->tris[i].p1[1] - center[1] );
    temp_radius[2] = fabs( rapidObject->tris[i].p1[2] - center[2] );

    if( temp_radius[0] > radius[0] ) { radius[0] = temp_radius[0]; }  
    if( temp_radius[1] > radius[1] ) { radius[1] = temp_radius[1]; }
    if( temp_radius[2] > radius[2] ) { radius[2] = temp_radius[2]; }
  }
}

/**
 * @function getBoxVertices
 */
void CheckObject::getModelBoxVertices()
{
  int sx[8] = { -1, 1, -1, 1, -1, 1, -1, 1 };
  int sy[8] = { 1, 1, -1, -1, 1, 1, -1, -1 };
  int sz[8] = { 1, 1, 1, 1, -1, -1, -1, -1 };

  for( int i = 0; i < 8; i++ )
  { boxVer[i][0] = center[0] + radius[0]*( (double)sx[i] ); 
    boxVer[i][1] = center[1] + radius[1]*( (double)sy[i] ); 
    boxVer[i][2] = center[2] + radius[2]*( (double)sz[i] );      
  }

}

/**
 * @function updateObjectData
 * @brief Gives you updated min and max coordinates and curr_boxVer
 */
void CheckObject::updateObjectData( kinematics::BodyNode* _node )
{
  reset_minmax();

  //-- Get the current orientation of the objects
  Eigen::Matrix4d tf = _node->getWorldTransform();

  Eigen::Vector3d trans; 
  trans(0) = tf(0,3); trans(1) = tf(1,3); trans(2) = tf(2,3); 
  std::cout << trans << std::endl; 
  Eigen::Matrix3d rot = tf.topLeftCorner(3,3);
  std::cout << rot << std::endl;

  //-- Save it properly
  for( int j = 0; j < 3; j++ )
   { 
     this->T[j] = trans(j); 

     for( int i = 0; i < 3; i++ )
      { this->R[j][i] = rot( j, i ); }
   }

  //-- Update the position of your bounding box vertices
  for( int i = 0; i < 8; i++ )
  {  Eigen::Vector3d new_ver = rot * Eigen::Vector3d( this->boxVer[i][0], this->boxVer[i][1], this->boxVer[i][2] ) + trans ;
     this->current_boxVer[i][0] = new_ver(0); this->current_boxVer[i][1] = new_ver(1); this->current_boxVer[i][2] = new_ver(2);
  }

  for( int i = 0; i < 8; i++ )
  {
     if( this->current_boxVer[i][0] < this->min_x ) { this->min_x = this->current_boxVer[i][0]; }
     if( this->current_boxVer[i][0] > this->max_x ) { this->max_x = this->current_boxVer[i][0]; }

     if( this->current_boxVer[i][1] < this->min_y ) { this->min_y = this->current_boxVer[i][1]; }
     if( this->current_boxVer[i][1] > this->max_y ) { this->max_y = this->current_boxVer[i][1]; }

     if( this->current_boxVer[i][2] < this->min_z ) { this->min_z = this->current_boxVer[i][2]; }
     if( this->current_boxVer[i][2] > this->max_z ) { this->max_z = this->current_boxVer[i][2]; }
  }
 
}


