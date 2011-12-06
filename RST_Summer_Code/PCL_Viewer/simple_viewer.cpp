#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>

using namespace std;

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

  std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds(0);

printf("Here? aa\n");

  for( int i = 0; i < num_clouds; i++ )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cc(new pcl::PointCloud<pcl::PointXYZ>);
    if( pcl::io::loadPCDFile<pcl::PointXYZ> ( name_clouds[i], *cc ) == -1 )
    {
      PCL_ERROR(" --(!) Could not read the file \n");
      return -1;
    }
   clouds.push_back( cc );
  }
printf("Here?\n");

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  for( int i = 0; i < num_clouds; i++ )
  { viewer.showCloud( clouds[i], name_clouds[i] ); }

  while( !viewer.wasStopped() )
  {}

  return 0;
}


