/**
 * @file pcd_write.cpp
 * @brief Write to PCD format txt files
 * @author A. Huaman
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <vector>

using namespace std;

/**
 * Usage: ./pcd_write file1.txt file2.txt ...
 * Output: cloud0.pcd cloud1.pcd... that, if you enter a proper txt file
 */

int line_num( char* filename );

/**
* @function main
 */
int main( int argc, char** argv )
{ 
  int num_clouds = argc - 1;
  
  std::vector< char* > name_clouds;
  name_clouds.resize(0);  

  for( int i = 0; i < num_clouds; i++ )
  { name_clouds.push_back( argv[i+1] );
    printf("Cloud %d : %s \n", i, name_clouds[i]);
  }

  char name[30] = "PointCloudData.txt";
  std::vector < pcl::PointCloud<pcl::PointXYZ> > clouds( num_clouds );

  //-- For each file containing data
  for( int i = 0; i < num_clouds; i++ )
  {
    int num_points = line_num( name_clouds[i] );    
    clouds[i].width = num_points;
    clouds[i].height = 1;
    clouds[i].is_dense = false;
    clouds[i].points.resize ( clouds[i].width*clouds[i].height );

    //-- Save the data
    FILE * pFile;
    pFile = fopen( name_clouds[i], "r" );
    float x, y, z;

    for( size_t k = 0; k < clouds[i].points.size(); k++ )
    { fscanf( pFile, "%f %f %f \n", &x, &y, &z );
      clouds[i].points[k].x = x;
      clouds[i].points[k].y = y;
      clouds[i].points[k].z = z;
    }  
    fclose( pFile ); 

    char pcd_name[35];
    sprintf( pcd_name, "cloud_%d.pcd", i );
    //-- Save it into a PCD file
    pcl::io::savePCDFileASCII( pcd_name, clouds[i] );
    std::cerr << "--> Saved " << clouds[i].points.size() << " data points to "<< pcd_name << std::endl;
  }

  return 0;
}

/**
 * @function line_num
 */
int line_num( char* filename )
{
  FILE *f;
  char c;
  int lines = 0;

  f = fopen( filename, "r" );
  
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
