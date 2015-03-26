/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
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
#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "BaseApplication.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class TutorialApplication : public BaseApplication
{
    // Parameters
    std::string configPath_, cloudPath_, savePath_;
    double SVVoxelResolution_, SVSeedResolution_, SVColorImportance_, SVSpatialImportance_;
    double RANSACDistanceThreshold_, VoxelLeafSize_; int RANSACMinInliers_;
    double GP3SearchRad_, GP3Mu_, GP3MaxNearestNeighbours_, GP3Ksearch_;
    double RWHullMaxDist_;
    bool simplifyHulls_;
    PointCloudT::Ptr baseCloud_;

    ros::NodeHandle nh;
public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);

protected:
    virtual void createScene(void);
    virtual void loadBaseCloud();
    virtual void loadParams();

};

#endif // #ifndef __TutorialApplication_h_
