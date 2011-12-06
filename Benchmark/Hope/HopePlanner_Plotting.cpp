/**
 * @file pcd_write.cpp
 * @brief Write to PCD format txt files
 * @author A. Huaman
 */

#include "HopePlanner.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <vector>

/**
* @function PCL_Viewer
 */
void HopePlanner::PointcloudViewer( std::vector<string> _PCDFilenames ) {

  int num_clouds = _PCDFilenames.size();

  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds(0);


  for( int i = 0; i < num_clouds; i++ )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cc(new pcl::PointCloud<pcl::PointXYZ>);
    if( pcl::io::loadPCDFile<pcl::PointXYZ> ( _PCDFilenames[i].c_str(), *cc ) == -1 )
    {
      PCL_ERROR(" --(!) Could not read the file \n");
      return;
    }
    clouds.push_back( cc );
  }

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  for( int i = 0; i < num_clouds; i++ )
  { viewer.showCloud( clouds[i], _PCDFilenames[i].c_str() ); }

  while( !viewer.wasStopped() )
  {}

  return;
}


/**
 * @function PCL_Writer
 */
string HopePlanner::PointcloudWriter( char* _filename, int _index ) {
  
  pcl::PointCloud<pcl::PointXYZ> cloud;

  int num_points = line_num( _filename );    
  cloud.width = num_points;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize ( cloud.width*cloud.height );

  //-- Save the data
  FILE * pFile;
  pFile = fopen( _filename, "r" );
  float x, y, z;

  for( size_t k = 0; k < cloud.points.size(); k++ )
  { fscanf( pFile, "%f %f %f \n", &x, &y, &z );
    cloud.points[k].x = x;
    cloud.points[k].y = y;
    cloud.points[k].z = z;
  }  
  fclose( pFile ); 

  char pcd_name[35];
  sprintf( pcd_name, "cloud_%d.pcd", _index );
  //-- Save it into a PCD file
  pcl::io::savePCDFileASCII( pcd_name, cloud );
  std::cerr << "--> Saved " << cloud.points.size() << " data points to "<< pcd_name << std::endl;

  string name; 
  name = string(pcd_name);
  return name;

}

/**
 * @function line_num
 */
int HopePlanner::line_num( char* _filename ) {

  FILE *f;
  char c;
  int lines = 0;

  f = fopen( _filename, "r" );
  
  if( f == NULL )
  { return 0; }

  while( (c = fgetc(f)) != EOF )
  {
    if( c == '\n' )
    { lines++; } 
  } 
 
  fclose( f );

  if( c!= '\n' )
  { lines++; }

  return lines;

}

