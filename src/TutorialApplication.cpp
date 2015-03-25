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
#include <exx_compression/compression.h>
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

using namespace EXX;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
    nh = ros::NodeHandle("~");
    TutorialApplication::loadParams();
    TutorialApplication::setConfigPath(configPath_);
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

    // Create the point cloud.
    TutorialApplication::loadBaseCloud();

    // Triangulate the cloud.
    std::cout << "Triangulation started" << std::endl;
    EXX::compression cmprs;
    std::vector<EXX::cloudMesh> cmesh;
    cmprs.setInputCloud(baseCloud_);
    cmprs.setVoxelLeafSize(VoxelLeafSize_);
    cmprs.setSVVoxelResolution(SVVoxelResolution_);
    cmprs.setSVSeedResolution(SVSeedResolution_);
    cmprs.setRANSACDistanceThreshold(RANSACDistanceThreshold_);
    cmprs.triangulatePlanes();
    cmesh = cmprs.returnCloudMesh();
    pcl::io::savePCDFileASCII (savePath_+"cmesh0.pcd", *cmesh[0].cloud);
    pcl::io::savePCDFileASCII (savePath_+"cmesh1.pcd", *cmesh[1].cloud);
    pcl::io::savePCDFileASCII (savePath_+"cmesh2.pcd", *cmesh[2].cloud);
    std::cout << "Creating manual object." << std::endl;

    // Create manual Object from the triangulation
    Ogre::Real r;
    Ogre::Real g;
    Ogre::Real b;

    int pic = 0;
    for (std::vector<EXX::cloudMesh>::iterator ite = cmesh.begin(); ite < cmesh.end(); ++ite){
        for (size_t i = 0; i < (*ite).cloud->points.size(); i++){
            manual->position((*ite).cloud->points[i].x, (*ite).cloud->points[i].y, (*ite).cloud->points[i].z);

            // std::cout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
            r = (Ogre::Real)(*ite).cloud->points[i].r / (Ogre::Real)255;
            g = (Ogre::Real)(*ite).cloud->points[i].g / (Ogre::Real)255;
            b = (Ogre::Real)(*ite).cloud->points[i].b / (Ogre::Real)255;
            manual->colour(r, g, b);
        }
        for (size_t i = 0; i < (*ite).mesh.polygons.size(); ++i){
            // Add triangle facing one way
            for (size_t j = 0; j < (*ite).mesh.polygons[i].vertices.size(); ++j)
            {
                manual->index((*ite).mesh.polygons[i].vertices[j] + pic);
            }
            // Add same triangles facing the other way
            for (size_t j = (*ite).mesh.polygons[i].vertices.size(); j-- > 0; )
            {
                manual->index((*ite).mesh.polygons[i].vertices[j] + pic);
            }
        }
        pic += (*ite).cloud->points.size();
    }
    std::cout << "cmesh object size: " << cmesh.size()  << std::endl;
     
    // tell Ogre, your definition has finished
    manual->end();
     
    // add ManualObject to the RootSceneNode (so it will be visible)
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(manual);
}

void TutorialApplication::loadBaseCloud(){
    baseCloud_ = PointCloudT::Ptr (new PointCloudT ());
    if( pcl::io::loadPCDFile (cloudPath_, *baseCloud_) == -1 ){
        PCL_ERROR("Could not load base cloud, baseCloud_ is empty.");
    } else {
        std::cout << "baseCloud_ loaded;" << std::endl;
    }
}

void TutorialApplication::loadParams(){
    nh.param<std::string>("configPath", configPath_, "./");
    nh.param<std::string>("cloudPath", cloudPath_, "./");
    nh.param<std::string>("savePath", savePath_, "./");
    nh.param<double>("SVVoxelResolution", SVVoxelResolution_, 0.1);
    nh.param<double>("SVSeedResolution", SVSeedResolution_, 0.3);
    nh.param<double>("RANSACDistanceThreshold", RANSACDistanceThreshold_, 0.04);
    nh.param<double>("VoxelLeafSize", VoxelLeafSize_, 0.02);
}

#ifdef __cplusplus
extern "C" {
#endif
    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "rosOgre_node");
        
        // Create application object
        TutorialApplication app;
        ros::Rate loop_rate(10);

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
