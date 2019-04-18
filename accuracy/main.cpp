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
        pipe.start(cfg);
    }

    void do_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setMaxIterations(100);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
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

        while (cloud->points.size() > 0.8 * total_points)
        {
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            do_ransac(cloud, inliers, coefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_cloud, inliers_cloud;
            inliers_cloud = filter_cloud(cloud, inliers, false);
            outliers_cloud = filter_cloud(cloud, inliers, true);
            float density=inliers_cloud->size()/4*3.14*coefficients->values[3]*coefficients->values[3];
            if (density < 0.8 || inliers_cloud->points.size() < 50)
            {
                cloud = outliers_cloud;
                continue;
            }
            coefvec.push_back(coefficients);
            invec.push_back(inliers_cloud);
            cloud = outliers_cloud;
        }

    };
    void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, bool bSave){
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
        cout<<cloud->size()<<endl;
        //viewer->addPointCloud(cloud);
        if (bSave) {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("totalCloud.pcd", *cloud);
        }
    }
    bool getRoi(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropBox<pcl::PointXYZ> filter;
        filter.setInputCloud(oricloud);
        filter.setMin(Eigen::Vector4f(-0.2,-0.2,0.2,1));

        filter.setMax(Eigen::Vector4f(0.2,0.2,1,1));

        filter.setNegative(false);

        filter.filter(*cloud);
        return true;
    }
    bool downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud){
//        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//        sor.setInputCloud (oricloud);
//        sor.setMeanK (50);
//        sor.setStddevMulThresh (0.1);
//        sor.filter (*oricloud);
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud (oricloud);
        vox.setLeafSize (0.01f, 0.01f, 0.01f);
        vox.filter (*oricloud);
        return  true;
    }
    void initViewer(){
        viewer=boost::make_shared<pcl::visualization::PCLVisualizer>("viewer");
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
    pcl::visualization::PCLVisualizer::Ptr viewer;
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
    while(!ac.viewer->wasStopped()){
        ac.generatePointCloud(cloud, false);
        ac.getRoi(cloud);
        ac.downsample(cloud);
        ac.findSphere(cloud);
        for (int i = 0; i <ac.getSpereSize() ; i+=2) {
            ac.viewer->addPointCloud(ac.getinvec()[i],std::to_string(i));
            cout<<"the distance between two spheres with X-Aixs is"<<ac.getcoef()[i]->values[0]-ac.getcoef()[i+1]->values[0]<<endl;
            cout<<"the distance between two spheres with Y-Aixs is"<<ac.getcoef()[i]->values[1]-ac.getcoef()[i+1]->values[1]<<endl;
            cout<<"the distance between two spheres with Z-Aixs is"<<ac.getcoef()[i]->values[2]-ac.getcoef()[i+1]->values[2]<<endl;
        }

    }


}