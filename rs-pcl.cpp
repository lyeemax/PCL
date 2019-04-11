// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "view_helper.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include "rsconversion.h"
using namespace std;

int main(int argc, char * argv[]) try {
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    pcl::visualization::CloudViewer viewer ("test");
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
        pipe.start();
    while (!viewer.wasStopped()) {
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto pcl_points = points_to_PCLXYZ(points);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_points);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(*cloud_filtered);
        //savePCD(cloud_filtered);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> layers;
        layers.push_back(pcl_points);
        layers.push_back(cloud_filtered);

        viewer.showCloud(cloud_filtered);
    }
       // gl_viewer(1280,640,"test",layers);


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
