/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/

#include "TutorialApplication.h"
#include <iostream>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PolygonMesh.h>

#include <OgreManualObject.h>
#include <CEGUI/CEGUI.h>
#include <CEGUI/CEGUISchemeManager.h>
#include <CEGUI/RendererModules/Ogre/CEGUIOgreRenderer.h>

#include <boost/thread/thread.hpp>
#include <PCLfunctions/cloud_generation.h>
#include <triangulation/triangulation.h>

using namespace EXX;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
    // create ManualObject
    Ogre::ManualObject* manual = mSceneMgr->createManualObject("manual");
     
    // specify the material (by name) and rendering type
    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Create the point cloud;
    PointCloudT::Ptr cloud = EXX::PCLfunctions::loadCloud();

    EXX::triangulation::cloudB new_cloud;
    new_cloud.cloud = PointCloudT::Ptr (new PointCloudT);
    pcl::io::loadPCDFile ("/home/unnar/catkin_ws/src/exx_wall_extraction/src/clouds/single_plane.pcd", *new_cloud.cloud);
    new_cloud.boundaryIndexLow  = 302;
    new_cloud.boundaryIndexHigh = new_cloud.cloud->points.size()-1;
    std::cout << "boundary Index High: " << new_cloud.boundaryIndexHigh << std::endl;
    triangulation::triangulation tri;
    tri.setInputCloud(new_cloud);
    std::vector<std::vector<int> > verts = tri.triangulate();

    pcl::PolygonMesh mesh;
    mesh = EXX::PCLfunctions::greedyProjectionTriangulation(cloud);
    pcl::io::saveVTKFile ("mesh.vtk", mesh);


    Ogre::Real r;
    Ogre::Real g;
    Ogre::Real b;
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        manual->position(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        // std::cout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
        r = (Ogre::Real)cloud->points[i].r / (Ogre::Real)255;
        g = (Ogre::Real)cloud->points[i].g / (Ogre::Real)255;
        b = (Ogre::Real)cloud->points[i].b / (Ogre::Real)255;
        manual->colour(r, g, b);

        // manual->normal(cloud->points[i].data_c[0], cloud->points[i].data_c[1], cloud->points[i].data_c[2]);
    }

    // Add indexing data to the manualobject
    // for (size_t i = 0; i < verts.size(); i++)
    // {
    //     // std::cout << "vertice size" << mesh.polygons[i].vertices.size() << std::endl;
    //     for (size_t j = 0; j < 3; j++)
    //     {
    //         int index = verts[i][j];
    //         manual->index(index);
    //     }
    //     // std::cout << "Vertices" << std::endl;
    //     // std::cout << verts[i][0];
    //     // std::cout << verts[i][1];
    //     // std::cout << verts[i][2] << std::endl;
    // }

    // Add indexing data to the manualobject
    for (size_t i = 0; i < mesh.polygons.size(); i++)
    {
        // std::cout << "vertice size" << mesh.polygons[i].vertices.size() << std::endl;
        for (size_t j = 0; j < mesh.polygons[i].vertices.size(); j++)
        {
            std::cout << "j er: " << j << std::endl;
            int index = mesh.polygons[i].vertices[j];
            manual->index(index);
        }
        std::cout << "i er: " << i << std::endl;
        // std::cout << mesh.polygons[i].vertices[0];
        // std::cout << mesh.polygons[i].vertices[1];
        // std::cout << mesh.polygons[i].vertices[2] << std::endl;
    }

    // EXX::triangulation::triangulation tri;
    // std::cout << "tri created" << std::endl;
    // tri.setInputCloud(cloud);
    // tri.setSearchRadius(0.5);
    // std::cout << "tri input cloud set" << std::endl;
    // std::cout << "cloud size is " << tri.getCloudSize() << std::endl;
     
    // define vertex position of index 0..3
    // manual->position(-100.0, -100.0, 0.0);
    // manual->position( 100.0, -100.0, 0.0);
    // manual->position( 100.0,  100.0, 0.0);
     
    // define usage of vertices by refering to the indexes
//    manual->index(0);

     
    // tell Ogre, your definition has finished
    manual->end();
     
    // add ManualObject to the RootSceneNode (so it will be visible)
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
}



#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        TutorialApplication app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
