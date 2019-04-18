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
    pcl::PointCloud<pcl::PointXYZ>::Ptr bkg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tar(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PolygonMesh::Ptr poly(new pcl::PolygonMesh);
    detector.initRealsense();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;


    pcl::PointIndices::Ptr inliersIds(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr screwcloud(new pcl::PointCloud<pcl::PointXYZ>);
    float inliersSizeRatios;

    while(!detector.viewer->wasStopped()){
        detector.generatePointCloud(cloud, false);
        auto fliter=detector.getRoiFixed(cloud);
        detector.downsample(fliter);
        detector.getClusters(fliter,clusters);
        for (int i = 0; i <clusters.size() ; ++i) {
            detector.getScrewPose(clusters[i]);

        }

        //std::cout<<clusters.size()<<endl;
       //detector.getPlaneInliers(fliter,inliersIds,inlierscloud,inliersSizeRatios);
       // detector.getScrewPose(tar);


//        detector.getClusters(fliter,clusters);
//        for (int i=0;i<clusters.size();i++){
//            detector.getScrewPose(clusters[i],i);
////            detector.reconstructPolygonMesh(clusters[i],*poly);
////            detector.poseViewer->addPolygonMesh(*poly),std::to_string(i);
//        }
        detector.viewer->spinOnce(1);
        detector.poseViewer->spinOnce(1);
       // cviewer.showCloud(fliter);
        cloud->clear();
    }


    return 0;
}