// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "view_helper.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include "rsconversion.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;
static bool process=true;

int main(int argc, char * argv[]) try {
    //signal( SIGINT, handler );
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
   pcl::visualization::CloudViewer cviewer ("Original Viewer");
//    pcl::visualization::RangeImageVisualizer range_image_widget ("range");
//    range_image_widget.setWindowTitle("range");
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;

    //window app(480, 480,"RGB");
    // We'll keep track for the last frame of each stream available to make the presentation persistent
    //std::map<int, rs2::frame> render_frames;


    pipe.start(cfg);

   // boost::shared_ptr<pcl::visualization::PCLVisualizer> acviewer=interactionCustomizationVis();
    while (!cviewer.wasStopped()) {
        auto frames = pipe.wait_for_frames(10000);
       // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        //auto color = frames.get_infrared_frame();
        //pc.map_to(color);
        auto depth = frames.get_depth_frame();
       auto color = frames.get_color_frame();
        pc.map_to(color);

       // render_frames[color.get_profile().unique_id()] = colorizer.process(color);
       // app.show(render_frames);
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto pcl_points = points_to_PCLXYZRGB(points,color);

//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//        pcl::PassThrough<pcl::PointXYZRGB> pass;
//        pass.setInputCloud(pcl_points);
//        pass.setFilterFieldName("z");
//        pass.setFilterLimits(0.0,1.5);
//        pass.filter(*pass_filtered);
//        cout<<pass_filtered->size()<<" --first"<<endl;
//
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
//        vox.setInputCloud (pass_filtered);
//        vox.setLeafSize (0.01f, 0.01f, 0.01f);
//        vox.filter (*voxel_filtered);
//
//
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//        sor.setInputCloud (voxel_filtered);
//        sor.setMeanK (200);
//        sor.setStddevMulThresh (0.1);
//        sor.filter (*sor_filtered);
//        cout<<sor_filtered->size()<<"  --final"<<endl;
        cviewer.showCloud(pcl_points);
//        pcl::PCDWriter writer;
//        writer.write<pcl::PointXYZRGB> ("test.pcd", *sor_filtered, false);

//        Eigen::Affine3f sensorPose;
//        sensorPose.setIdentity();
//        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//        float noise_level=0.00;
//        float min_range = 0.0f;
//        float angular_resolution = 0.5f;
//       // pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//        Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
//        pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
//        range_image_ptr->createFromPointCloud (*sor_filtered, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                          scene_sensor_pose, coordinate_frame, noise_level, min_range, 1);
//        //convert unorignized point cloud to orginized point cloud end
//
//
//        //viusalization of range image
//        range_image_widget.showRangeImage (*range_image_ptr);
//        range_image_widget.spinOnce ();



    }
//       // gl_viewer(1280,640,"test",layers);


        return EXIT_SUCCESS;

}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
