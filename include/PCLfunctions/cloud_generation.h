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
			// PointCloudT::Ptr cloud(new PointCloudT);
			// std::cout << "Creating simulated data" << std::endl;
		 //    // Fill in the cloud data
		 //    int cloud_width  = 10;
		 //    int cloud_height = 10;
		 //    cloud->width  = cloud_width*cloud_height;
		 //    cloud->height = 1;
		 //    cloud->points.resize (cloud->width * cloud->height);

		    // PointCloudT::Ptr cloud (new PointCloudT);
		    // pcl::io::loadPCDFile ("test", *cloud);
		    // return cloud;
		    PointCloudT::Ptr interior_cloud (new PointCloudT);
		    interior_cloud->width  = 8;
		    interior_cloud->height = 1;
		    interior_cloud->points.resize (interior_cloud->width * interior_cloud->height);
		    interior_cloud->points[0].x = 2.5; interior_cloud->points[0].y = 2.5;
		    interior_cloud->points[1].x = 2.5; interior_cloud->points[1].y = 5;
		    interior_cloud->points[2].x = 2.5; interior_cloud->points[2].y = 7.5;
		    interior_cloud->points[3].x = 5; interior_cloud->points[3].y = 2.5;
		    interior_cloud->points[4].x = 5; interior_cloud->points[4].y = 5;
		    interior_cloud->points[5].x = 5; interior_cloud->points[5].y = 7.5;
		    interior_cloud->points[6].x = 7.5; interior_cloud->points[6].y = 2.5;
		    interior_cloud->points[7].x = 7.5; interior_cloud->points[7].y = 5;


		    for (size_t i=0; i < interior_cloud->points.size(); i++)
		    {
		        interior_cloud->points[i].z = 1.0;
		        interior_cloud->points[i].r = 255;
		        interior_cloud->points[i].g = 255;
		        interior_cloud->points[i].b = 255;            
		    }

		    PointCloudT::Ptr boundary_cloud (new PointCloudT);
		    boundary_cloud->width  = 10;
		    boundary_cloud->height = 1;
		    boundary_cloud->points.resize (boundary_cloud->width * boundary_cloud->height);
		    boundary_cloud->points[0].x = 0; boundary_cloud->points[0].y = 0;
		    boundary_cloud->points[1].x = 2.5; boundary_cloud->points[1].y = 0;
		    boundary_cloud->points[2].x = 7.5; boundary_cloud->points[2].y = 1.0;
		    boundary_cloud->points[3].x = 10.0; boundary_cloud->points[3].y = 0.0;
		    boundary_cloud->points[4].x = 10.0; boundary_cloud->points[4].y = 5.0;
		    boundary_cloud->points[5].x = 7.5; boundary_cloud->points[5].y = 7.5;
		    boundary_cloud->points[6].x = 3.5; boundary_cloud->points[6].y = 10.0;
		    boundary_cloud->points[7].x = 0.0; boundary_cloud->points[7].y = 10.0;
		    boundary_cloud->points[8].x = 0.0; boundary_cloud->points[8].y = 7.5;
		    boundary_cloud->points[9].x = 1.0; boundary_cloud->points[9].y = 4.0;

		    for (size_t i=0; i < boundary_cloud->points.size(); i++)
		    {
		        boundary_cloud->points[i].z = 1.0;
		        boundary_cloud->points[i].r = 255;
		        boundary_cloud->points[i].g = 255;
		        boundary_cloud->points[i].b = 50;            
		    }

		  //   // Generate the data
		  //   int index = 0;
		  //   for (int i = 0; i < cloud_width; i++)
		  //   {
		  //   	for (int j = 0; j < cloud_height; j++)
		  //   	{
		  //   		double ind_i = i+1.0;
		  //   		double ind_j = j+1.0;
				// 	cloud->points[index].x = ind_i*10.0;
				// 	cloud->points[index].y = ind_j*10.0;
				// 	cloud->points[index].z = 1.0;
				// 	cloud->points[index].r = 255;
				// 	cloud->points[index].g = 50 + ind_i*10;
				// 	cloud->points[index].b = 50;
				// 	index++;
				// }
		  //   }
		    *interior_cloud += *boundary_cloud;
		    return interior_cloud;
		}

		PointCloudT::Ptr loadCloud(std::string path)
		{
			PointCloudT::Ptr cloud (new PointCloudT);
			pcl::io::loadPCDFile (path, *cloud);
			return cloud;
		}

		std::string intToType(int type) {

			if (type == -1) return "NONE";
			else if (type == 0) return "FREE";
			else if (type == 1) return "FRINGE";
			else if (type == 2) return "BOUNDARY";
			else if (type == 3) return "COMPLETED";
			else return "HMMM";
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
			    gp3.setSearchRadius (0.5);

			    // Set typical values for the parameters
			    gp3.setMu (10.0);
			    gp3.setMaximumNearestNeighbors (100);
			    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			    gp3.setMinimumAngle(M_PI/4); // 10 degrees
			    gp3.setMaximumAngle(3*M_PI/4); // 120 degrees
			    gp3.setNormalConsistency(true);

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

			    std::vector<int> parts = gp3.getPartIDs();
  				std::vector<int> states = gp3.getPointStates();

			    // std::cout << "Printing vertices and there types:" << std::endl;
			    // std::cout << "Parts size: " << parts.size();
			    // for (size_t i = 0; i < parts.size(); i++)
			    // {
			    // 	std::cout << parts.at(i) << " ";
			    // }
			    // std::cout << std::endl;
			    // std::cout << "States size: " << states.size() << std::endl;
			    // for (size_t i = 0; i < states.size(); i++)
			    // {
			    // 	std::cout << states.at(i) << " ";
			    // }
			    // std::cout << std::endl;
			    // for (size_t i = 0; i < triangle.polygons.size(); i++)
			    // {
			    // 	// std::cout << intToType(states.at(i)) << ": ";
			    // 	for (size_t j = 0; j < triangle.polygons[i].vertices.size(); j++){
			    // 		std::cout << triangle.polygons[i].vertices[j] << " ";	
			    // 	}
			    // 	std::cout << std::endl;
			    // }


			    return triangle;
			}
	}
}