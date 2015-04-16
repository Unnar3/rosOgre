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
    std::cout << "SVColorImportance: " << SVColorImportance_ << std::endl;
    std::cout << "SVSpatialImportance: " << SVSpatialImportance_ << std::endl;
    EXX::compression cmprs;
    cmprs.setVoxelLeafSize(VoxelLeafSize_);
    cmprs.setSVVoxelResolution(SVVoxelResolution_);
    cmprs.setSVSeedResolution(SVSeedResolution_);
    // cmprs.setSVColorImportance(SVColorImportance_);
    // cmprs.setSVSpatialImportance(SVSpatialImportance_);
    cmprs.setRANSACDistanceThreshold(RANSACDistanceThreshold_);
    cmprs.setRANSACMinInliers(RANSACMinInliers_);
    // cmprs.setSimplifyHulls(simplifyHulls_);
    cmprs.setGP3SearchRad(GP3SearchRad_);
    cmprs.setGP3Mu(GP3Mu_);
    cmprs.setGP3MaxNearestNeighbours(GP3MaxNearestNeighbours_);
    cmprs.setGP3Ksearch(GP3Ksearch_);
    cmprs.setRWHullMaxDist(RWHullMaxDist_);
    cmprs.setECClusterTolerance(ECClusterTolerance_);
    cmprs.setECMinClusterSize(ECMinClusterSize_);

    
    PointCloudT::Ptr voxel_cloud (new PointCloudT ());
    EXX::planesAndCoeffs pac;
    std::vector<PointCloudT::Ptr> c_planes;
    std::vector<PointCloudT::Ptr> hulls;
    std::vector<PointCloudT::Ptr> simplified_hulls;
    std::vector<PointCloudT::Ptr> super_planes;
    std::vector<EXX::cloudMesh> cmesh;
    clock_t t1,t2,t3,t4,t5,t6,t7,t8,t9;
    t1=clock();
    cmprs.voxelGridFilter(baseCloud_, voxel_cloud);
    t2=clock();
    cmprs.extractPlanesRANSAC(voxel_cloud, &pac);
    t3=clock();
    cmprs.projectToPlane(&pac);
    t4=clock();
    std::vector<int> normalInd;
    cmprs.euclideanClusterPlanes(&pac.cloud, &c_planes, &normalInd);
    t5=clock();
    cmprs.planeToConcaveHull(&c_planes, &hulls);
    t6=clock();
    cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls);
    t7=clock();
    cmprs.superVoxelClustering(&c_planes, &super_planes);
    t8=clock();
    cmprs.greedyProjectionTriangulationPlanes(voxel_cloud, &super_planes, &simplified_hulls, &cmesh);
    t9=clock();

    std::cout << "Total time: " << double(t9-t1) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Voxel time: " << double(t2-t1) / CLOCKS_PER_SEC << std::endl;
    std::cout << "RANSAC time: " << double(t3-t2) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Project time: " << double(t4-t3) / CLOCKS_PER_SEC << std::endl;
    std::cout << "EC time: " << double(t5-t4) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Hull time: " << double(t6-t5) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Simple Hull time: " << double(t7-t6) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Super Voxel time: " << double(t8-t7) / CLOCKS_PER_SEC << std::endl;
    std::cout << "triangulation time: " << double(t9-t8) / CLOCKS_PER_SEC << std::endl;

    // Create manual Object from the triangulation
    // Ogre::Real r;
    // Ogre::Real g;
    // Ogre::Real b;
    
    PointCloudT::Ptr tmp_cloud (new PointCloudT ());
    for (int i = 0; i < cmesh.size()-1; ++i){
        *tmp_cloud += *cmesh[i].cloud;
        std::cout << "printing" << std::endl;
    }

    pcl::io::savePCDFileASCII (savePath_ + "final_cloud.pcd", *tmp_cloud);

    int pic = 0;
    int r, g, b;
    for (std::vector<EXX::cloudMesh>::iterator ite = cmesh.begin(); ite < cmesh.end()-1; ++ite){
        r = rand () % 255;
        g = rand () % 255;
        b = rand () % 255;
        for (size_t i = 0; i < (*ite).cloud->points.size(); i++){
            manual->position((*ite).cloud->points[i].x, (*ite).cloud->points[i].y, (*ite).cloud->points[i].z);

            // std::cout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
            // r = (Ogre::Real)(*ite).cloud->points[i].r / (Ogre::Real)255;
            // g = (Ogre::Real)(*ite).cloud->points[i].g / (Ogre::Real)255;
            // b = (Ogre::Real)(*ite).cloud->points[i].b / (Ogre::Real)255;
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
    nh.param<double>("SVColorImportance", SVColorImportance_, 1.0);
    nh.param<double>("SVSpatialImportance", SVSpatialImportance_, 0.01);
    nh.param<double>("RANSACDistanceThreshold", RANSACDistanceThreshold_, 0.04);
    nh.param<int>("RANSACMinInliers", RANSACMinInliers_, 200);
    nh.param<double>("VoxelLeafSize", VoxelLeafSize_, 0.02);
    nh.param<double>("GP3SearchRad", GP3SearchRad_, 0.3); 
    nh.param<double>("GP3Mu", GP3Mu_, 2.5);
    nh.param<double>("GP3MaxNearestNeighbours", GP3MaxNearestNeighbours_, 100);
    nh.param<double>("GP3Ksearch", GP3Ksearch_, 20);
    nh.param<double>("RWHullMaxDist", RWHullMaxDist_, 0.3);
    nh.param<bool>("simplifyHulls", simplifyHulls_, true);
    nh.param<double>( "ECClusterTolerance", ECClusterTolerance_, 0.05);
    nh.param<int>( "ECMinClusterSize", ECMinClusterSize_, 100);
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
