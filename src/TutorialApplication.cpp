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
#include <params/params.h>
#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
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
    std::cout << "dafuck" << std::endl;
    PCL_ERROR("Could not load base cloud, baseCloud_ is empty.");
    TutorialApplication::loadParams();
    std::cout << "dafuck" << std::endl;
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

    std::cout << "hmmmmmmmmm" << std::endl;
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
    cmprs.setHULLAlpha(hullAlpha_);

    
    PointCloudT::Ptr voxel_cloud (new PointCloudT ());
    EXX::planesAndCoeffs pac;
    std::vector<EXX::densityDescriptor> dDesc;;
    std::vector<PointCloudT::Ptr> c_planes;
    std::vector<PointCloudT::Ptr> hulls;
    std::vector<PointCloudT::Ptr> simplified_hulls;
    std::vector<PointCloudT::Ptr> super_planes;
    std::vector<EXX::cloudMesh> cmesh;
    clock_t t1,t2,t3,t4,t5,t6,t7,t8,t9,t10;
    t1=clock();
    cmprs.voxelGridFilter(baseCloud_, voxel_cloud);
    std::cout << "voxel" << std::endl;
    t2=clock();


    primitive_params params;
    params.number_disjoint_subsets = 10;
    params.octree_res              = 1.0;
    params.normal_neigbourhood     = 0.05;
    params.inlier_threshold        = 0.04;
    params.angle_threshold         = 0.5;
    params.add_threshold           = 0.005;
    params.min_shape               = voxel_cloud->points.size()*0.000001;
    params.inlier_min              = params.min_shape;
    params.connectedness_res       = 0.2;
    params.distance_threshold      = 0.0;

    //primitive_visualizer<pcl::PointXYZRGB> viewer;
    std::vector<base_primitive*> primitives = { new plane_primitive() };
    primitive_extractor<PointT> extractor(voxel_cloud, primitives, params, NULL);
    std::vector<base_primitive*> extracted;
    extractor.extract(extracted);

    std::vector<PointCloudT::Ptr> plane_vec;
    std::vector<int> ind;
    std::vector<Eigen::Vector4d> normal;
    Eigen::VectorXd data;
    for (size_t j = 0; j < extracted.size(); ++j){
        ind = extracted[j]->supporting_inds;
        extracted.at(j)->shape_data(data); 
        normal.push_back(data.segment<4>(0));
        PointCloudT::Ptr test_cloud (new PointCloudT ());
        for (size_t i = 0; i < ind.size(); ++i){
            test_cloud->points.push_back(voxel_cloud->points[ind[i]]);
        }
        plane_vec.push_back(test_cloud);
    }


    // cmprs.extractPlanesRANSAC(voxel_cloud, &pac);
    std::cout << "ransac" << std::endl;
    t3=clock();

    for ( size_t i = 0; i < normal.size(); ++i ){
        compression::projectToPlaneS( plane_vec[i], normal[i] );
    }

    // cmprs.projectToPlane(&pac);
    t4=clock();
    std::vector<int> normalInd;
    // cmprs.euclideanClusterPlanes(&plane_vec, &c_planes, &normalInd);
    std::cout << "cluster" << std::endl;
    t5=clock();
    cmprs.planeToConcaveHull(&plane_vec, &hulls);
    // cmprs.getOBB( hulls, obb );
    // 
    cmprs.getPlaneDensity( plane_vec, hulls, dDesc);
    std::cout << "concave" << std::endl;
    t6=clock();
    cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);
    std::cout << "simpl" << std::endl;
    t7=clock();
    std::cout << "hmmm" << std::endl;
    cmprs.superVoxelClustering(&plane_vec, &super_planes, dDesc);
    std::cout << "hmm2" << std::endl;
    t8=clock();
    std::cout << "change hull color" << std::endl;
    for ( size_t i = 0; i < simplified_hulls.size(); ++i){
        for ( size_t j = 0; j < simplified_hulls.at(i)->points.size(); ++j){
            simplified_hulls.at(i)->points[j].r = 255;
            simplified_hulls.at(i)->points[j].g = 255;
            simplified_hulls.at(i)->points[j].b = 0;
        }
    }
    cmprs.greedyProjectionTriangulationPlanes(voxel_cloud, &super_planes, &simplified_hulls, &cmesh, dDesc);
    t9=clock();
    std::cout << "tri" << std::endl;
    cmprs.improveTriangulation(cmesh, super_planes, simplified_hulls);
    std::cout << "impr tri" << std::endl;
    t10 = clock();

    std::cout << "Total time: " << double(t9-t1) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Voxel time: " << double(t2-t1) / CLOCKS_PER_SEC << std::endl;
    std::cout << "RANSAC time: " << double(t3-t2) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Project time: " << double(t4-t3) / CLOCKS_PER_SEC << std::endl;
    std::cout << "EC time: " << double(t5-t4) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Hull time: " << double(t6-t5) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Simple Hull time: " << double(t7-t6) / CLOCKS_PER_SEC << std::endl;
    std::cout << "Super Voxel time: " << double(t8-t7) / CLOCKS_PER_SEC << std::endl;
    std::cout << "triangulation time: " << double(t9-t8) / CLOCKS_PER_SEC << std::endl;
    std::cout << "improve triangulation time: " << double(t10-t9) / CLOCKS_PER_SEC << std::endl;

    // Create manual Object from the triangulation
    // Ogre::Real r;
    // Ogre::Real g;
    // Ogre::Real b;
    
    PointCloudT::Ptr tmp_cloud (new PointCloudT ());
    for (int i = 0; i < cmesh.size()-1; ++i){
        *tmp_cloud += *cmesh[i].cloud;
    }

    pcl::io::savePCDFileASCII (savePath_ + "final_cloud.pcd", *tmp_cloud);

    int pic = 0;
    Ogre::Real r, g, b;
    for (std::vector<EXX::cloudMesh>::iterator ite = cmesh.begin(); ite < cmesh.end()-1; ++ite){
        // r = rand () % 255;
        // g = rand () % 255;
        // b = rand () % 255;
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
    // nh.param<std::string>("configPath", configPath_, "./");
    configPath_ = params::load<std::string>("configPath", nh);
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
    nh.param<double>("hullAlpha", hullAlpha_, 0.1);
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
