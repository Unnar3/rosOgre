#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/search/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;

namespace EXX {
	namespace PCLfunctions {
		
		PointCloudT::Ptr createCloud()
		{
			PointCloudT::Ptr cloud(new PointCloudT);
			std::cout << "Creating simulated data" << std::endl;
		    // Fill in the cloud data
		    int cloud_width  = 10;
		    int cloud_height = 10;
		    cloud->width  = cloud_width*cloud_height;
		    cloud->height = 1;
		    cloud->points.resize (cloud->width * cloud->height);

		    // Generate the data
		    srand (time(NULL));
		    for (int i = 0; i < cloud_width; i++)
		    {
		    	for (int j = 0; j < cloud_height; j++)
		    	{
					cloud->points[i].x = i*10;
					cloud->points[i].y = j*10;
					cloud->points[i].z = 1.0;
					cloud->points[i].r = 255;
					cloud->points[i].g = 50 + i*10;
					cloud->points[i].b = 50;
				}
		    }

		    return cloud;
		}
	}
}