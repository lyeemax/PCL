#include <librealsense2/rs.hpp>
#include<iostream>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/io/pcd_io.h>
#include <sstream>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>

using namespace std;

class Accuracy{
public:
    void initRealsense(){
//        cfg.enable_stream(RS2_STREAM_COLOR, 1, 480, RS2_FORMAT_BGR8, 30);
//        cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
//        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);
    }

    void do_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients) {
        std::cout<<"start RANSAC "<<std::endl;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
        tree->setInputCloud(cloud);
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud);
        ne.setKSearch(500);
        ne.compute (*cloud_normals);
//        std::cout<<"1st"<<std::endl;
//
//        seg.setOptimizeCoefficients (true);
//        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//        seg.setNormalDistanceWeight (0.1);
//        seg.setMethodType (pcl::SAC_RANSAC);
//        seg.setMaxIterations (100);
//        seg.setDistanceThreshold (0.03);
//        seg.setInputCloud (cloud);
//        seg.setInputNormals (cloud_normals);
//        // Obtain the plane inliers and coefficients
//        seg.segment (*inliers_plane, *coefficients_plane);
//        std::cout<<"2nd"<<std::endl;
//        extract.setInputCloud (cloud);
//        extract.setIndices (inliers_plane);
//        extract.setNegative (true);
//        extract.filter (*cloud);
//        extract_normals.setNegative (true);
//        extract_normals.setInputCloud (cloud_normals);
//        extract_normals.setIndices (inliers_plane);
//        extract_normals.filter (*cloud_normals);
//
//        std::cout<<"3th"<<std::endl;

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations(100);
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers, *coefficients);
        std::cout<<"end RANSAC "<<std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &indices, bool negative) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(negative);
        extract.filter(*cloud_filtered);
        return cloud_filtered;
    }

    void findSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
        int total_points=cloud->points.size();
        //std::cout<<total_points<<endl;

      //  while (cloud->points.size() > 0.9 * total_points)
       // {

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            do_ransac(cloud, inliers, coefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_cloud, inliers_cloud;
            inliers_cloud = filter_cloud(cloud, inliers, true);
            outliers_cloud = filter_cloud(cloud, inliers, false);
           // float density=inliers_cloud->size()/4*3.14*coefficients->values[3]*coefficients->values[3];
//            if (inliers_cloud->points.size() < 50)
//            {
//                cloud = outliers_cloud;
//               // continue;
//            }
            coefvec.push_back(coefficients);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorin(inliers_cloud, 255, 0, 0);
            oviewer->addPointCloud(inliers_cloud,single_colorin,"cloud1");
            cout<<" radius is  "<<coefficients->values[6]<<endl;
            invec.push_back(inliers_cloud);
          //  cloud = outliers_cloud;
       // }

    };
    void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, bool bSave){
        std::cout<<"start generating "<<std::endl;
        cloud->clear();
       // viewer->removePointCloud("cloud");
        auto frames = pipe.wait_for_frames(5000);
        auto depth = frames.get_depth_frame();
        rs2::pointcloud pc;
        rs2::points points;
        points = pc.calculate(depth);
        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();
        for (auto& p : cloud->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            ptr++;
        }
        //viewer->addPointCloud(cloud,"cloud");
        //cout<<cloud->size()<<endl;
        if (bSave) {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("totalCloud.pcd", *cloud);
        }
        std::cout<<"end generating "<<std::endl;
    }
    void getRoi(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud){

        pcl::CropBox<pcl::PointXYZ> filter;
        filter.setInputCloud(oricloud);
        filter.setMin(Eigen::Vector4f(-0.1f,-0.1f,0.2f,1.0));

        filter.setMax(Eigen::Vector4f(0.1f,0.1f,0.6f,1.0));

        filter.setNegative(false);

        filter.filter(*oricloud);
        oviewer->addPointCloud(oricloud,"cloud");
    }
    bool downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud){
//        std::cout<<"start sampling "<<std::endl;
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//        sor.setInputCloud (oricloud);
//        sor.setMeanK (50);
//        sor.setStddevMulThresh (0.1);
//        sor.filter (*oricloud);
//        pcl::VoxelGrid<pcl::PointXYZ> vox;
//        vox.setInputCloud (oricloud);
//        vox.setLeafSize (0.01f, 0.01f, 0.01f);
//        vox.filter (*oricloud);
//        std::cout<<"end sampling "<<"  -size "<<oricloud->size()<<std::endl;
        return  true;
    }
    void initViewer(){
        //viewer=boost::make_shared<pcl::visualization::PCLVisualizer>("viewer");
        oviewer=boost::make_shared<pcl::visualization::PCLVisualizer>("Orignal viewer");
    }
    int getSpereSize(){
        return coefvec.size();
    }
    vector<pcl::ModelCoefficients::Ptr> getcoef(){
        return coefvec;
    }
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getinvec(){
            return invec;
    };
   // pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PCLVisualizer::Ptr oviewer;
private:
    rs2::pipeline pipe;
    rs2::config cfg;

    vector<pcl::ModelCoefficients::Ptr> coefvec;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> invec;
};

int main(){
    Accuracy ac;
    ac.initRealsense();
    ac.initViewer();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    while(!ac.oviewer->wasStopped()){
        ac.oviewer->removePointCloud("cloud");
        ac.oviewer->removePointCloud("cloud1");
//       ac.oviewer->removePointCloud("cloud");
//        for (int i = 0; i <2 ; ++i) {
//            ac.generatePointCloud(cloud, false);
//        }
        pcl::PCDReader reader;
        reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);


        //ac.getRoi(cloud);
        std::cout<<cloud->size()<<endl;
//        ac.oviewer->addPointCloud(cloud,"cloud");

        //ac.downsample(fliter_cloud);
       //ac.viewer->addPointCloud(cloud,"cloud");
            ac.findSphere(cloud);
//        for (int i = 0; i <ac.getSpereSize() ; i+=2) {
//            ac.viewer->addPointCloud(ac.getinvec()[i],std::to_string(i));
//            cout<<"the distance between two spheres with X-Aixs is"<<ac.getcoef()[i]->values[0]-ac.getcoef()[i+1]->values[0]<<endl;
//            cout<<"the distance between two spheres with Y-Aixs is"<<ac.getcoef()[i]->values[1]-ac.getcoef()[i+1]->values[1]<<endl;
//            cout<<"the distance between two spheres with Z-Aixs is"<<ac.getcoef()[i]->values[2]-ac.getcoef()[i+1]->values[2]<<endl;
//        }
       // ac.viewer->spinOnce(100);
        ac.oviewer->spinOnce(10000);
        cloud->clear();

    }


}