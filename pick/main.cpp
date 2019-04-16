//
// Created by cobot on 19-4-16.
//
#include<iostream>
#include "DetectorBaseScrew.h"
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
int main(){
    cVisionDL::DetectorBaseScrew detector;
    detector.initViewer();
    detector.setParam("screw_para.xml");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    detector.initRealsense();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    while(!detector.viewer->wasStopped()){
        detector.generatePointCloud(cloud, false);
        auto fliter=detector.getRoiFixed(cloud);
        detector.downsample(fliter);
        detector.getClusters(fliter,clusters);
        for (int i=0;i<clusters.size();i++){
            detector.getScrewPose(fliter);
        }
       detector.viewer->spinOnce(100);
       // cviewer.showCloud(fliter);
    }


    return 0;
}