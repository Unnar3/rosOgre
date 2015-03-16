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
		    int index = 0;
		    for (int i = 0; i < cloud_width; i++)
		    {
		    	for (int j = 0; j < cloud_height; j++)
		    	{
		    		double ind_i = i+1.0;
		    		double ind_j = j+1.0;
					cloud->points[index].x = ind_i*10.0;
					cloud->points[index].y = ind_j*10.0;
					cloud->points[index].z = 1.0;
					cloud->points[index].r = 255;
					cloud->points[index].g = 50 + ind_i*10;
					cloud->points[index].b = 50;
					index++;
				}
		    }

		    return cloud;
		}

		pcl::PolygonMesh greedyProjectionTriangulation(PointCloudT::Ptr plane)
			{
			    pcl::PolygonMesh triangle;

			    // Normal estimation*
			    pcl::NormalEstimation<PointT, pcl::Normal> n;
        		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
			    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

			    // Initialize objects
			    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

			    // Set the maximum distance between connected points (maximum edge length)
			    gp3.setSearchRadius (160);

			    // Set typical values for the parameters
			    gp3.setMu (1.0);
			    gp3.setMaximumNearestNeighbors (100);
			    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			    gp3.setMinimumAngle(M_PI/18); // 10 degrees
			    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			    gp3.setNormalConsistency(false);

			    pcl::PolygonMesh triangles_planes;

			    tree->setInputCloud (plane);
			    n.setInputCloud (plane);
			    n.setSearchMethod (tree);
			    n.setKSearch (20);
			    n.compute (*normals);

			    // Concatenate the XYZ and normal fields*
			    pcl::concatenateFields (*plane, *normals, *cloud_with_normals);

			    tree2->setInputCloud (cloud_with_normals);

			    // Get result
			    gp3.setInputCloud (cloud_with_normals);
			    gp3.setSearchMethod (tree2);
			    gp3.reconstruct (triangle);

			    return triangle;
			}
	}
}